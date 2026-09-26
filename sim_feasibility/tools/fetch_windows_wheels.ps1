param([string]$Manifest = 'windows_wheels.json')
$ErrorActionPreference = 'Continue'
$Project = Split-Path $PSScriptRoot -Parent
$Wheels = Get-Content "$Project\results\sources\$Manifest" -Raw | ConvertFrom-Json
New-Item -ItemType Directory -Force "$Project\assets\wheels" | Out-Null
foreach ($Wheel in $Wheels) {
    $Dest = Join-Path "$Project\assets\wheels" $Wheel.filename
    if (Test-Path $Dest) {
        $Existing = (Get-FileHash $Dest -Algorithm SHA256).Hash.ToLowerInvariant()
        if ($Existing -eq $Wheel.sha256) { continue }
        throw "Existing wheel hash mismatch: $Dest"
    }
    $Partial = "$Dest.partial"
    Write-Output "Downloading $($Wheel.filename)"
    & C:\Windows\System32\curl.exe -x http://127.0.0.1:7897 -sS -L --retry 5 --retry-all-errors --connect-timeout 20 --max-time 1800 -C - -o $Partial $Wheel.url
    if ($LASTEXITCODE -ne 0) { throw "Download failed; partial preserved: $Partial" }
    $Hash = (Get-FileHash $Partial -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($Hash -ne $Wheel.sha256) { throw "SHA256 mismatch: $Partial" }
    Move-Item $Partial $Dest
    Write-Output "Verified $($Wheel.filename)"
}
