param([ValidateSet('insertion','friction','spring')][string]$Kind = 'insertion', [string]$RunName = '')
$ErrorActionPreference = 'Continue'
$Project = Split-Path $PSScriptRoot -Parent
. "$PSScriptRoot\runtime_env.ps1"
if (-not $RunName) { $RunName = "$Kind-$(Get-Date -Format 'yyyyMMdd-HHmmss')" }
$Out = Join-Path $Project "results\experiments\$RunName"
if (Test-Path $Out) { throw "Use a new RunName; existing evidence will be preserved: $Out" }
New-Item -ItemType Directory -Force $Out | Out-Null
@{kind=$Kind;started_at=(Get-Date -Format o);script_sha256=(Get-FileHash "$Project\experiments\rigid_mechanisms.py" -Algorithm SHA256).Hash} | ConvertTo-Json | Set-Content "$Out\run_manifest.json"
Write-Output "Output: $Out"
& 'D:\conda\envs\isaac51\python.exe' "$Project\experiments\rigid_mechanisms.py" $Kind --output $Out *> "$Out\run.log"
$Code = $LASTEXITCODE
"exit_code=$Code" | Set-Content "$Out\exit_code.txt"
exit $Code
