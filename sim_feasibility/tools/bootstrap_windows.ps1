param([string]$CondaRoot = "D:\conda\miniconda3")
$ErrorActionPreference = "Stop"
$Project = Split-Path $PSScriptRoot -Parent
$Installer = Join-Path $Project "assets\installers\Miniconda3-Windows-x86_64.exe"
if (-not (Test-Path "$CondaRoot\Scripts\conda.exe")) {
    $Signature = Get-AuthenticodeSignature $Installer
    if ($Signature.Status -ne 'Valid' -or $Signature.SignerCertificate.Subject -notmatch 'Anaconda') {
        throw 'Installer signature validation failed'
    }
    $Signature | Select-Object Status,@{Name='Signer';Expression={$_.SignerCertificate.Subject}} |
        ConvertTo-Json | Set-Content "$Project\results\system\installer_signature.json"
    $Install = Start-Process -FilePath $Installer -ArgumentList @('/InstallationType=JustMe','/RegisterPython=0','/AddToPath=0','/S',"/D=$CondaRoot") -Wait -PassThru
    if ($Install.ExitCode -ne 0) { throw "Installer failed: $($Install.ExitCode)" }
}
$env:HTTP_PROXY = 'http://127.0.0.1:7897'
$env:HTTPS_PROXY = $env:HTTP_PROXY
$env:CONDA_PKGS_DIRS = 'D:\conda\pkgs'
if (-not (Test-Path 'D:\conda\envs\isaac51\python.exe')) {
    $ErrorActionPreference = 'Continue'
    & "$CondaRoot\Scripts\conda.exe" create --prefix D:\conda\envs\isaac51 --override-channels -c conda-forge python=3.11 pip -y --json *> "$Project\results\system\conda_windows_install.log"
    if ($LASTEXITCODE -ne 0) { throw "Conda environment creation failed: $LASTEXITCODE" }
}
$ErrorActionPreference = 'Stop'
& 'D:\conda\envs\isaac51\python.exe' --version
