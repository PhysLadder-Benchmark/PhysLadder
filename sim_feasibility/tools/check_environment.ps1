$ErrorActionPreference = 'Continue'
$Project = Split-Path $PSScriptRoot -Parent
. "$PSScriptRoot\runtime_env.ps1"
& "$SimEnv\python.exe" -m pip check *> "$Project\results\system\pip_check.log"
$Code = $LASTEXITCODE
& "$SimEnv\python.exe" -m pip freeze | Set-Content -Encoding utf8 "$Project\envs\isaac51_freeze.txt"
Write-Output "pip check exit code: $Code"
exit $Code
