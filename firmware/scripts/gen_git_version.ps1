param(
    [Parameter(Mandatory = $true)]
    [string]$OutPath
)

$tag = ""
try {
    $tag = (git describe --tags --abbrev=0 2>$null).Trim()
} catch {
    $tag = ""
}

if ([string]::IsNullOrWhiteSpace($tag)) {
    $tag = "v0.0"
}

$versionNum = 0
if ($tag -match '^v?(\d+)\.(\d+)') {
    $major = [int]$Matches[1]
    $minor = [int]$Matches[2]
    $versionNum = $major * 10 + $minor
}

if ($versionNum -gt 255) { $versionNum = 255 }
if ($versionNum -lt 0) { $versionNum = 0 }

$content = @"
#pragma once
#define FW_GIT_TAG "$tag"
#define FW_VERSION_NUM $versionNum
"@

$writeFile = $true
if (Test-Path $OutPath) {
    try {
        $existing = Get-Content -Path $OutPath -Raw
        if ($existing -eq $content) {
            $writeFile = $false
        }
    } catch {
        $writeFile = $true
    }
}

if ($writeFile) {
    Set-Content -Path $OutPath -Value $content -Encoding ASCII
}
