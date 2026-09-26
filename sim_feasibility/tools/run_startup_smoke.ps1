param([ValidateSet('d3d12','vulkan')][string]$Backend = 'd3d12')
$ErrorActionPreference = 'Continue'
$Project = Split-Path $PSScriptRoot -Parent
. "$PSScriptRoot\runtime_env.ps1"
$env:SIM_GRAPHICS_BACKEND = $Backend
$Out = Join-Path $Project "results\experiments\startup-$Backend-$(Get-Date -Format 'yyyyMMdd-HHmmss')"
New-Item -ItemType Directory -Force $Out | Out-Null
Write-Output "Output: $Out"
& 'D:\conda\envs\isaac51\python.exe' "$Project\experiments\startup_smoke.py" $Out *> "$Out\run.log"
$Code = $LASTEXITCODE
"exit_code=$Code" | Set-Content "$Out\exit_code.txt"
exit $Code
