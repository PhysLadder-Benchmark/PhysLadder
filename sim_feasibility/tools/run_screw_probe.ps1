$RunName = "screw_probe-$(Get-Date -Format 'yyyyMMdd-HHmmss')"
$ErrorActionPreference = 'Continue'
$Project = Split-Path $PSScriptRoot -Parent
. "$PSScriptRoot\runtime_env.ps1"
$Out = Join-Path $Project "results\experiments\$RunName"
New-Item -ItemType Directory -Force $Out | Out-Null
@{kind='screw_probe';started_at=(Get-Date -Format o);script_sha256=(Get-FileHash "$Project\experiments\screw_probe.py" -Algorithm SHA256).Hash} | ConvertTo-Json | Set-Content "$Out\run_manifest.json"
Write-Output "Output: $Out"
& 'D:\conda\envs\isaac51\python.exe' "$Project\experiments\screw_probe.py" --output $Out *> "$Out\run.log"
$Code = $LASTEXITCODE
if (Select-String -Path "$Out\run.log" -Pattern '\[Error\]|\[Fatal\]' -Quiet) {
    $Code = 3
    'Physics/runtime errors detected; these measurements are not valid task evidence.' | Set-Content "$Out\INVALID_RUN.txt"
}
"exit_code=$Code" | Set-Content "$Out\exit_code.txt"
exit $Code
