#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate C files (Registers.h/.c) from the Excel table "регистры modbus.xlsx".

Usage:
    python gen_modbus_regs.py "registers_modbus.xlsx" [out_dir] [out_file_base]

The script auto-detects the header row (it searches for a cell exactly "Adress")
and reads the metadata values in the top part of the sheet:
    BaseAddress, offset per motor, offset per general reg
"""
import sys
import re
from pathlib import Path
import pandas as pd
from typing import Any, Dict, Tuple, Optional
import math

TYPE_ENUM = {"u16": "REG_T_U16", "i16": "REG_T_I16", "u32": "REG_T_U32", "i32": "REG_T_I32"}
ACCESS_ENUM = {"R": "REG_ACC_R", "W": "REG_ACC_W", "R/W": "REG_ACC_RW", "RW": "REG_ACC_RW"}


def c_ident(name: str) -> str:
    s = re.sub(r"[^a-zA-Z0-9_]", "_", (name or "").strip())
    if not s:
        return "UNNAMED"
    if s[0].isdigit():
        s = "_" + s
    return s.upper()


def c_escape(s: str) -> str:
    return (s or "").replace("\\", "\\\\").replace('"', '\\"')


def find_header_row(raw: pd.DataFrame) -> int:
    for i in range(len(raw)):
        row = raw.iloc[i].astype(str).tolist()
        if any("Address" == str(c).strip() for c in row):
            return i
    raise RuntimeError("Header row not found (cell 'Address').")


def c_define_value(v) -> str:
    if v is None:
        return "0"
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, int):
        return str(v)+"U"
    if isinstance(v, float):
        # чтобы всегда была точка, и без научной нотации при желании
        return f"{v:.6f}".rstrip("0").rstrip(".") if "." in f"{v:.6f}" else f"{v:.6f}"
    if isinstance(v, str):
        s = v.replace("\\", "\\\\").replace('"', '\\"')
        return f"\"{s}\""
    return str(v)  # fallback


def _is_empty(x: Any) -> bool:
    return x is None or (isinstance(x, float) and math.isnan(x)) or (isinstance(x, str) and not x.strip())


def _to_int_float_str(x: Any) -> Any:
    if _is_empty(x):
        return None
    if isinstance(x, (int, float)) and not isinstance(x, bool):
        if isinstance(x, float) and x.is_integer():
            return int(x)
        return x
    if isinstance(x, str):
        s = x.strip().replace(",", ".")
        try:
            if "." in s:
                f = float(s)
                return int(f) if f.is_integer() else f
            return int(s)
        except ValueError:
            return x.strip()
    return x

def extract_constants_table(raw: pd.DataFrame) -> Dict[str, Any]:
    """
    Ищет таблицу с заголовками 'Constant Name' и 'Value', затем читает строки до первой пустой.
    raw должен быть прочитан с header=None.
    """
    header_row: Optional[int] = None
    name_col: Optional[int] = None
    value_col: Optional[int] = None

    # 1) Найти заголовок
    for i in range(raw.shape[0]):
        for j in range(raw.shape[1]):
            cell = raw.iat[i, j]
            if isinstance(cell, str) and cell.strip().lower() == "constant name":
                # value обычно справа
                cell2 = raw.iat[i, j+1]
                if isinstance(cell2, str) and cell2.strip().lower() == "value":
                    header_row, name_col, value_col = i, j, j+1
                    break
            if header_row is not None:
                break
        if header_row is not None:
            break

    if header_row is None:
        raise RuntimeError("Не нашёлся заголовок таблицы: 'Constant Name' | 'Value'.")

    # 2) Читать вниз до первой пустой строки
    out: Dict[str, Any] = {}
    for r in range(header_row + 1, raw.shape[0]):
        key = raw.iat[r, name_col]
        val = raw.iat[r, value_col]

        if _is_empty(key) or key.lower()=="end":
            break  # пустая строка => конец таблицы констант

        if not isinstance(key, str):
            raise RuntimeError("ключ 'Constant Name' не может быть числом.")

        k = key.strip()
        out[k] = _to_int_float_str(val)

    return out



def load_registers_data(raw: pd.DataFrame, header_row: int):
    df = raw.iloc[header_row + 1:].copy()
    df.columns = [str(c).strip() for c in raw.iloc[header_row].tolist()]
    df = df[df["Address"].notna()]  # drop empty tail rows

    df["Register name"] = df["Register name"].astype(str).str.strip()
    df = df[df["Register name"].str.upper() != "RESERVED"].copy()

    # forward fill motor / section
    df["№ Motor"] = df["№ Motor"].ffill()
    df["Section"] = df["Section"].ffill()

    # normalize types
    df["Address"] = df["Address"].astype(int)
    df["Register index"] = df["Register index"].astype(int)
    df["Offset in motor"] = df["Offset in motor"].astype(int)
    df["Offset in Section"] = df["Offset in Section"].astype(int)
    df["Register size (bytes)"] = df["Register size (bytes)"].astype(int)
    df["Direction"] = df["Direction"].fillna("").astype(str).str.strip()
    df["Data type"] = df["Data type"].fillna("").astype(str).str.strip().str.lower()
    df["Units"] = df["Units"].fillna("").astype(str).str.strip()
    df["Description"] = df["Description"].fillna("").astype(str).str.strip()

    regs = []
    for _, r in df.iterrows():
        t = str(r["Data type"])
        a = str(r["Direction"])
        if t not in TYPE_ENUM:
            raise ValueError(f"Unknown data type '{t}' in row: {r.to_dict()}")
        if a not in ACCESS_ENUM:
            raise ValueError(f"Unknown access '{a}' in row: {r.to_dict()}")

        regs.append(
            dict(
                name=c_ident(r["Register name"]),
                idx=int(r["Register index"]),
                motorRegOffset=int(r["Offset in motor"]),
                size=int(r["Register size (bytes)"]),
                type_enum=TYPE_ENUM[t],
                access=ACCESS_ENUM[a],
                units=str(r["Units"]),
                desc=str(r["Description"]),
            )
        )

    # sort by index
    regs.sort(key=lambda x: x["idx"])
    return regs




def gen_h(regs, constansDict):
    L = []
    L.append("/* Auto-generated from Excel. Do not edit manually. Edit in *.xlsx */")
    L.append("#pragma once")
    L.append("")
    L.append("#include <stdint.h>")
    L.append("#include <stddef.h>")
    L.append("")

    for key, value in constansDict.items():
        L.append(f"#define {key} {c_define_value(value)}")
    L.append("")

    L.append("/* Register access level: read/write/read-write */")
    L.append("typedef enum {")
    L.append("    REG_ACC_R  = 1,")
    L.append("    REG_ACC_W  = 2,")
    L.append("    REG_ACC_RW = 3")
    L.append("} reg_access_t;")
    L.append("")

    L.append("/* Register data type */")
    L.append("typedef enum {")
    L.append("    REG_T_U16 = 1,")
    L.append("    REG_T_I16 = 2,")
    L.append("    REG_T_U32 = 3,")
    L.append("    REG_T_I32 = 4")
    L.append("} reg_type_t;")
    L.append("")

    L.append("typedef enum {")
    for i, r in enumerate(regs):
        L.append(f"    REG_{r['name']} = {i},")
    L.append(f"    REG__COUNT = {len(regs)}")
    L.append("} reg_id_t;")
    L.append("")
    
    L.append("/* Optional: offsets inside motor block (in 16-bit registers) */")
    for r in regs:
        L.append(f"#define REG_MOTOR_OFFSET_{r['name']:<20} ({r['motorRegOffset']}u)")
    L.append("")
    for r in regs:
        L.append(f"#define REG_INDEX_{r['name']:<20} ({r['idx']}u)")
    L.append("")
    
    L.append("typedef struct {")
    L.append("    reg_id_t     id;")
    L.append("    uint16_t     index;      /* index in motor block (16-bit registers) */")
    L.append("    uint16_t     size;       /* size in bytes */")
    L.append("    reg_type_t   type;       /* reg data type */")
    L.append("    reg_access_t access;")
    L.append("    const char*  name;")
    L.append("    const char*  units;")
    L.append("} reg_meta_t;")
    L.append("")
    
    L.append("/* Metadata for every register of the device. */")
    L.append("extern const reg_meta_t g_reg_meta[REG__COUNT];")
    L.append("")

    L.append("/* Get absolute Modbus address for motor (motor is 0..N). */")
    L.append("static inline uint32_t reg_address(uint8_t motor, reg_id_t id)")
    L.append("{")
    L.append("    /* address = BASE + motor offset + motor*STRIDE + index */")
    L.append("    return (uint32_t)REG_BASE_ADDRESS + (uint32_t)REG_MOTOR_BLOCK_START + ")
    L.append("           (uint32_t)(motor * (uint32_t)REG_MOTOR_STRIDE_WORDS) + (uint32_t)g_reg_meta[id].index;")
    L.append("}")
    L.append("")

    L.append("const reg_meta_t* reg_get_meta(reg_id_t id);")


    #L.append("int reg_find_by_index(uint16_t index, reg_id_t* out_id);")
    L.append("")

    return "\n".join(L) + "\n"



def gen_c(regs, out_file:str):
    L = []
    L.append("/* Auto-generated from Excel. Do not edit manually. */")
    L.append(f'#include "{out_file}.h"')
    L.append("#include <assert.h>")
    L.append("")
    L.append("const reg_meta_t g_reg_meta[REG__COUNT] = {")
    for r in regs:
        L.append("    {")
        L.append(f"        .id = REG_{r['name']},")
        L.append(f"        .index = {r['motorRegOffset']}u,")
        L.append(f"        .size = {r['size']}u,")
        L.append(f"        .type = {r['type_enum']},")
        L.append(f"        .access = {r['access']},")
        L.append(f'        .name = "{c_escape(r["name"])}",')
        L.append(f'        .units = "{c_escape(r["units"])}",')
        L.append("    },")
    L.append("};")
    L.append("")
    L.append("const reg_meta_t* reg_get_meta(reg_id_t id)")
    L.append("{")
    L.append("    assert((int)id >= 0 && id < REG__COUNT);")
    L.append("    return &g_reg_meta[id];")
    L.append("}")
    L.append("")
    return "\n".join(L) + "\n"


def main():
    if len(sys.argv) < 2:
        print("Usage: python gen_modbus_regs.py <xlsx_path> [out_dir] [out_file_base]")
        return 1

    xlsx_file = Path(sys.argv[1]).resolve()
    out_dir = Path(sys.argv[2]).resolve() if len(sys.argv) >= 3 else xlsx.parent
    out_file = sys.argv[3] if len(sys.argv) >= 4 else "Registers"
    

    raw_xlsx = pd.read_excel(xlsx_file, sheet_name=0, header=None)
    constansDict = extract_constants_table(raw_xlsx)

    
    header_row = find_header_row(raw_xlsx)
    regs = load_registers_data(raw_xlsx, header_row)

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / (out_file + ".h") ).write_text(gen_h(regs, constansDict), encoding="utf-8")
    (out_dir / (out_file + ".c")).write_text(gen_c(regs, out_file), encoding="utf-8")

    print(f"Generated in: '{out_dir}'; files: '{out_file}.h' '{out_file}.c'")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
