# Process-local environment setup; no system PATH or registry changes.
. "$PSScriptRoot\..\..\robot_task_framework\tools\research_storage.ps1"
$SimPreviousStorage = Set-ResearchStorageEnvironment
$SimEnv = 'D:\conda\envs\isaac51'
$env:PATH = "$SimEnv;$SimEnv\Scripts;$SimEnv\Library\bin;$env:PATH"
$env:CONDA_PREFIX = $SimEnv
$env:PYTHONNOUSERSITE = '1'
$env:OMNI_KIT_ACCEPT_EULA = 'YES'
$env:OMNI_KIT_DISABLE_TELEMETRY = '1'
$env:PYTHONUTF8 = '1'
$env:PYTHONUNBUFFERED = '1'
