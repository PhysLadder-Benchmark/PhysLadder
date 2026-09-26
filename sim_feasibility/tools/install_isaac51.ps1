$ErrorActionPreference = 'Stop'
$Project = Split-Path $PSScriptRoot -Parent
$Python = 'D:\conda\envs\isaac51\python.exe'
if (-not (Test-Path $Python)) { throw 'Run bootstrap_windows.ps1 first' }
$env:HTTP_PROXY = 'http://127.0.0.1:7897'
$env:HTTPS_PROXY = $env:HTTP_PROXY
$env:PIP_CACHE_DIR = 'D:\conda\pip-cache'
$env:PYTHONUTF8 = '1'
$ErrorActionPreference = 'Continue'
$LocalWheels = @(Get-ChildItem "$Project\assets\wheels\*.whl" -ErrorAction SilentlyContinue | ForEach-Object {$_.FullName})
& $Python -m pip install 'isaacsim[all,extscache]==5.1.0.0' 'wheel==0.45.1' @LocalWheels --extra-index-url https://pypi.nvidia.com --timeout 120 --progress-bar off --report "$Project\results\system\isaac51_install_report.json" *> "$Project\results\system\isaac51_install.log"
if ($LASTEXITCODE -ne 0) { throw "Isaac installation failed: $LASTEXITCODE" }
& $Python -m pip freeze > "$Project\envs\isaac51_freeze.txt"
