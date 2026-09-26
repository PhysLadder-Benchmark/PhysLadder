param(
    [string]$TaskPython = 'D:\conda\envs\isaac51\python.exe',
    [string]$Config = 'configs/checks/r5_motion.toml',
    [switch]$Gui,
    [switch]$Debugger,
    [ValidateSet('joint', 'cartesian', 'interaction', 'gpt', 'api', 'cameras')][string]$Check = 'joint',
    [string]$DebugHost = '127.0.0.1',
    [ValidateRange(1, 65535)][int]$DebugPort = 5678
)

$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\research_storage.ps1"
$taskProject = Split-Path -Parent $PSScriptRoot
$taskEnvironment = Split-Path -Parent $TaskPython
if (-not (Test-Path $TaskPython)) { throw "Python not found: $TaskPython" }
$taskSavedEnvironment = @{}
foreach ($taskName in @('PATH', 'CONDA_PREFIX', 'PYTHONNOUSERSITE', 'PYTHONUTF8', 'PYTHONUNBUFFERED', 'OMNI_KIT_DISABLE_TELEMETRY', 'OMNI_KIT_ACCEPT_EULA')) {
    $taskSavedEnvironment[$taskName] = [Environment]::GetEnvironmentVariable($taskName, 'Process')
}
Push-Location $taskProject
try {
    $taskPreviousStorage = Set-ResearchStorageEnvironment
    $env:PATH = "$taskEnvironment;$taskEnvironment\Scripts;$taskEnvironment\Library\bin;$env:PATH"
    $env:CONDA_PREFIX = $taskEnvironment
    $env:PYTHONNOUSERSITE = '1'
    $env:PYTHONUTF8 = '1'
    $env:PYTHONUNBUFFERED = '1'
    $env:OMNI_KIT_DISABLE_TELEMETRY = '1'
    # Existing local Isaac installation already accepted its EULA during setup.
    $env:OMNI_KIT_ACCEPT_EULA = 'YES'
    $taskCommand = 'check-r5'
    if ($Check -eq 'cartesian') {
        $taskCommand = 'check-r5-cartesian'
        if (-not $PSBoundParameters.ContainsKey('Config')) { $Config = 'configs/checks/r5_cartesian.toml' }
    }
    if ($Check -eq 'interaction') {
        $taskCommand = 'check-r5-interaction'
        if (-not $PSBoundParameters.ContainsKey('Config')) { $Config = 'configs/checks/r5_interaction.toml' }
    }
    if ($Check -eq 'gpt' -or $Check -eq 'api') {
        $taskCommand = 'check-r5-gpt'
        if ($Check -eq 'api') { $taskCommand = 'check-gpt-api' }
        if (-not $PSBoundParameters.ContainsKey('Config')) { $Config = 'configs/checks/r5_gpt.toml' }
    }
    if ($Check -eq 'cameras') {
        $taskCommand = 'check-r5-cameras'
        if (-not $PSBoundParameters.ContainsKey('Config')) { $Config = 'configs/checks/r5_cameras.toml' }
    }
    $taskArguments = @('-m', 'robot_tasks', $taskCommand, '--config', $Config)
    if ($Debugger) {
        if ($Check -eq 'api') { throw 'API-only check does not use the simulator debugger' }
        if (-not $PSBoundParameters.ContainsKey('Config')) {
            $Config = 'configs/checks/r5_motion_debug.toml'
            if ($Check -eq 'cartesian') { $Config = 'configs/checks/r5_cartesian_debug.toml' }
            if ($Check -eq 'interaction') { $Config = 'configs/checks/r5_interaction_debug.toml' }
            if ($Check -eq 'gpt') { $Config = 'configs/checks/r5_gpt_debug.toml' }
            if ($Check -eq 'cameras') { $Config = 'configs/checks/r5_cameras.toml' }
        }
        $taskArguments = @('-Xfrozen_modules=off', '-m', 'robot_tasks.integrationtest.debug_motion', '--config', $Config,
                           '--listen-host', $DebugHost, '--port', "$DebugPort", '--check', $Check)
    }
    if ($Gui -and $Check -ne 'api') { $taskArguments += '--gui' }
    & $TaskPython @taskArguments
    $taskExitCode = $LASTEXITCODE
}
finally {
    Pop-Location
    if ($null -ne $taskPreviousStorage) { Restore-ResearchStorageEnvironment $taskPreviousStorage }
    foreach ($taskName in $taskSavedEnvironment.Keys) {
        [Environment]::SetEnvironmentVariable($taskName, $taskSavedEnvironment[$taskName], 'Process')
    }
}
exit $taskExitCode
