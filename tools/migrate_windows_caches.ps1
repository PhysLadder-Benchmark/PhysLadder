# Explicit one-time local migration. Copy + SHA256 verification precede switching paths.
# No system/user profile directory or unrelated application data is relocated.
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\research_storage.ps1"
$taskConfig = Get-ResearchStorageConfig
$taskStamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$taskReportDirectory = Join-Path $taskConfig.root "migration\$taskStamp"
New-Item -ItemType Directory -Path $taskReportDirectory -Force | Out-Null
$taskResults = [System.Collections.Generic.List[object]]::new()

foreach ($taskEntry in $taskConfig.junctions) {
    $taskSource = Join-Path ([Environment]::GetEnvironmentVariable($taskEntry.base, 'Process')) $taskEntry.source
    $taskTarget = [IO.Path]::GetFullPath((Join-Path $taskConfig.root $taskEntry.target))
    $taskBackup = "$taskSource.relocating-$taskStamp"
    $taskItem = [ordered]@{source=$taskSource; target=$taskTarget; status='pending'; bytes=0; files=0; backup=$null}
    try {
        Write-Output "MIGRATING $taskSource -> $taskTarget"
        if (Test-Path -LiteralPath $taskSource) {
            $taskSourceItem = Get-Item -LiteralPath $taskSource -Force
            if ($taskSourceItem.Attributes -band [IO.FileAttributes]::ReparsePoint) {
                if ($taskSourceItem.LinkType -ne 'Junction' -or $taskSourceItem.Target[0] -ne $taskTarget) {
                    throw "Existing redirect differs; leave it unchanged: $taskSource"
                }
                $taskItem.status = 'already_redirected'
                $taskResults.Add($taskItem)
                continue
            }
        }
        New-Item -ItemType Directory -Path $taskTarget -Force | Out-Null
        if (Test-Path -LiteralPath $taskSource) {
            # Never follow another redirect while recursively copying cached data.
            $taskLinks = @(Get-ChildItem -LiteralPath $taskSource -Recurse -Force | Where-Object {
                $_.Attributes -band [IO.FileAttributes]::ReparsePoint
            })
            if ($taskLinks.Count) { throw "Nested reparse point found; inspect before moving $taskSource" }
            & robocopy.exe $taskSource $taskTarget /E /COPY:DAT /DCOPY:DAT /R:0 /W:0 /XJ /NFL /NDL /NP /NJH /NJS |
                Out-File (Join-Path $taskReportDirectory ($taskEntry.target.Replace('/', '_') + '.copy.log'))
            if ($LASTEXITCODE -ge 8) { throw "Copy failed or files are locked; source preserved (robocopy $LASTEXITCODE)" }
            $taskSourceFiles = @(Get-ChildItem -LiteralPath $taskSource -File -Recurse -Force)
            $taskFileManifest = @()
            foreach ($taskFile in $taskSourceFiles) {
                $taskRelative = $taskFile.FullName.Substring($taskSource.Length).TrimStart('\')
                $taskDestination = Join-Path $taskTarget $taskRelative
                $taskHash = (Get-FileHash -LiteralPath $taskFile.FullName -Algorithm SHA256).Hash
                if ((Get-FileHash -LiteralPath $taskDestination -Algorithm SHA256).Hash -ne $taskHash) {
                    throw "Cache changed or copied bytes differ: $taskRelative"
                }
                $taskFileManifest += @{path=$taskRelative; bytes=$taskFile.Length; sha256=$taskHash}
                $taskItem.bytes += $taskFile.Length
            }
            $taskItem.files = $taskSourceFiles.Count
            $taskFileManifest | ConvertTo-Json -Depth 4 | Set-Content -Encoding UTF8 -LiteralPath (
                Join-Path $taskReportDirectory ($taskEntry.target.Replace('/', '_') + '.files.json'))
            # Keep an intact same-volume backup until the new directory entry works.
            Rename-Item -LiteralPath $taskSource -NewName ([IO.Path]::GetFileName($taskBackup))
            $taskItem.backup = $taskBackup
        } else {
            New-Item -ItemType Directory -Path (Split-Path -Parent $taskSource) -Force | Out-Null
        }
        try {
            New-Item -ItemType Junction -Path $taskSource -Target $taskTarget | Out-Null
        } catch {
            if ((Test-Path -LiteralPath $taskBackup) -and -not (Test-Path -LiteralPath $taskSource)) {
                Rename-Item -LiteralPath $taskBackup -NewName ([IO.Path]::GetFileName($taskSource))
                $taskItem.backup = $null
            }
            throw
        }
        $taskLink = Get-Item -LiteralPath $taskSource -Force
        if ($taskLink.LinkType -ne 'Junction' -or $taskLink.Target[0] -ne $taskTarget) {
            throw 'Directory junction verification failed'
        }
        $taskItem.status = 'redirected'
        if (Test-Path -LiteralPath $taskBackup) {
            try {
                Remove-Item -LiteralPath $taskBackup -Recurse -Force
                $taskItem.backup = $null
            } catch {
                $taskItem.status = 'redirected_backup_locked'
                $taskItem.error = $_.Exception.Message
            }
        }
        Write-Output "MIGRATED $($taskItem.status): $($taskItem.bytes) bytes"
    } catch {
        if ($taskItem.status -eq 'pending') { $taskItem.status = 'blocked' }
        $taskItem.error = $_.Exception.Message
        Write-Output "MIGRATION_PENDING $taskSource : $($_.Exception.Message)"
    }
    $taskResults.Add($taskItem)
    $taskResults | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $taskReportDirectory 'result.json') -Encoding UTF8
}
$taskResults | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $taskReportDirectory 'result.json') -Encoding UTF8
Write-Output "MIGRATION_REPORT $taskReportDirectory"
if (@($taskResults | Where-Object {$_.status -notin @('redirected', 'already_redirected')}).Count) { exit 2 }
