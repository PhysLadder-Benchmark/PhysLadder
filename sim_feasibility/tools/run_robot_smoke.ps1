$RunName = "robot_smoke-$(Get-Date -Format 'yyyyMMdd-HHmmss')"
$ErrorActionPreference = 'Continue'
$Project = Split-Path $PSScriptRoot -Parent
. "$PSScriptRoot\runtime_env.ps1"
$Out = Join-Path $Project "results\experiments\$RunName"
New-Item -ItemType Directory -Force $Out | Out-Null
@{kind='robot_smoke';started_at=(Get-Date -Format o);script_sha256=(Get-FileHash "$Project\experiments\robot_smoke.py" -Algorithm SHA256).Hash} | ConvertTo-Json | Set-Content "$Out\run_manifest.json"
Write-Output "Output: $Out"
& 'D:\conda\envs\isaac51\python.exe' "$Project\experiments\robot_smoke.py" --output $Out *> "$Out\run.log"
$Code = $LASTEXITCODE
"exit_code=$Code" | Set-Content "$Out\exit_code.txt"
exit $Code
