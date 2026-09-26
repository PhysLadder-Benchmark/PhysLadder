$Project = Split-Path $PSScriptRoot -Parent
$Url = 'https://download.pytorch.org/whl/cu128/torch-2.7.0%2Bcu128-cp311-cp311-win_amd64.whl'
foreach ($Mode in @('proxy','direct')) {
    $Out = "$Project\results\system\torch_probe_$Mode.part"
    $Args = @('-sS','-L','--range','0-1048575','--max-time','20','-o',$Out,'-w','%{http_code} %{speed_download}')
    if ($Mode -eq 'proxy') { $Args += @('-x','http://127.0.0.1:7897') }
    else { $Args += @('--noproxy','*') }
    $Args += $Url
    Write-Output $Mode
    & C:\Windows\System32\curl.exe @Args
}
