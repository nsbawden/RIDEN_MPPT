# remove_mppt_tasks.ps1
# Removes:
#  - RIDEN_MPPT_BG

$ErrorActionPreference = "Stop" # fail fast on script errors

function Test-IsAdmin {
    # returns $true if running elevated
    $id = [Security.Principal.WindowsIdentity]::GetCurrent() # current user token
    $p = New-Object Security.Principal.WindowsPrincipal($id) # principal wrapper
    return $p.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator) # admin check
}

if (-not (Test-IsAdmin)) {
    # self-elevate
    $self = $PSCommandPath # this script
    Start-Process -FilePath "powershell.exe" -ArgumentList "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "`"$self`"" -Verb RunAs # relaunch as admin
    exit 0
}

$TaskBg = "RIDEN_MPPT_BG" # task name
$TaskKill = "RIDEN_MPPT_STOP_ON_LOGON" # legacy task name (remove if present)

try {
    if (Get-ScheduledTask -TaskName $TaskBg -ErrorAction SilentlyContinue) {
        Unregister-ScheduledTask -TaskName $TaskBg -Confirm:$false # remove background task
    }

    # Remove legacy kill-on-logon task if it exists (cleanup)
    if (Get-ScheduledTask -TaskName $TaskKill -ErrorAction SilentlyContinue) {
        Unregister-ScheduledTask -TaskName $TaskKill -Confirm:$false # remove legacy task
    }
}
catch {
    Write-Host ("ERR remove tasks: " + $_.Exception.Message) # errors only
    exit 1
}
