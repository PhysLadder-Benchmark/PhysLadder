$ErrorActionPreference = 'Continue'
$Project = Split-Path $PSScriptRoot -Parent
. "$PSScriptRoot\runtime_env.ps1"
$Wheels = Get-Content "$Project\results\sources\torch_windows_wheels.json" -Raw | ConvertFrom-Json
$Paths = @($Wheels | ForEach-Object { Join-Path "$Project\assets\wheels" $_.filename })
foreach ($Path in $Paths) { if (-not (Test-Path $Path)) { throw "Missing verified wheel: $Path" } }
& "$SimEnv\python.exe" -m pip install --no-deps @Paths *> "$Project\results\system\torch_cuda_install.log"
if ($LASTEXITCODE -ne 0) { throw 'CUDA PyTorch installation failed' }
& "$SimEnv\python.exe" -m pip check *> "$Project\results\system\pip_check.log"
if ($LASTEXITCODE -ne 0) { throw 'Dependency check failed' }
& "$SimEnv\python.exe" -m pip freeze | Set-Content -Encoding utf8 "$Project\envs\isaac51_freeze.txt"
