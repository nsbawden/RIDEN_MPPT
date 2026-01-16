# install_mppt_tasks.ps1
# Creates:
#  - RIDEN_MPPT_BG (Daily 7:45 AM, WakeToRun, runs riden_mppt.py in background with --bg)

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
$TaskKill = "RIDEN_MPPT_STOP_ON_LOGON" # legacy task name (we remove it if present)

$MpptDir = "C:/Users/Quant/Python/MPPT" # working directory
$ScriptPath = "$MpptDir/riden_mppt.py" # script path

$TargetVin = "33" # arg1
$ComPort = "COM4" # arg2
$SlaveId = "1" # arg3

function Get-PythonExePath {
    # finds python.exe to run under Task Scheduler
    $candidates = @()

    try { $candidates += (Get-Command python.exe -ErrorAction SilentlyContinue).Source } catch {} # prefer python.exe
    try { $candidates += (Get-Command python -ErrorAction SilentlyContinue).Source } catch {} # sometimes no .exe in command name

    $local = "$env:LOCALAPPDATA/Programs/Python" # common per-user install location
    if (Test-Path $local) {
        $candidates += Get-ChildItem -Path $local -Filter "python.exe" -Recurse -File -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending |
        Select-Object -ExpandProperty FullName
    }

    $candidates = $candidates | Where-Object { $_ -and (Test-Path $_) } | Select-Object -Unique # keep valid unique paths

    if ($candidates.Count -gt 0) { return $candidates[0] } # take first best match
    throw "Could not find python.exe. Install Python, or ensure python.exe is discoverable via PATH for this account"
}

try {
    if (-not (Test-Path $MpptDir)) { throw "MPPT directory not found: $MpptDir" } # must exist
    if (-not (Test-Path $ScriptPath)) { throw "MPPT script not found: $ScriptPath" } # must exist

    $PyExe = Get-PythonExePath # resolve python.exe full path

    # Remove legacy kill-on-logon task if it exists
    try {
        if (Get-ScheduledTask -TaskName $TaskKill -ErrorAction SilentlyContinue) {
            Unregister-ScheduledTask -TaskName $TaskKill -Confirm:$false # remove legacy task
        }
    }
    catch {
        Write-Host ("ERR remove legacy task: " + $_.Exception.Message) # errors only
    }

    # ----- Background task -----
    $ArgsBg = "`"$ScriptPath`" $TargetVin $ComPort $SlaveId --bg" # tag background instance with --bg
    $ActionBg = New-ScheduledTaskAction -Execute $PyExe -Argument $ArgsBg -WorkingDirectory $MpptDir # background action
    $TriggerBg = New-ScheduledTaskTrigger -Daily -At 7:45AM # daily trigger
    $SettingsBg = New-ScheduledTaskSettingsSet -WakeToRun -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries # wake and run on battery

    # Run as your user (interactive token not required; still runs in background)
    $PrincipalBg = New-ScheduledTaskPrincipal -UserId "$env:USERDOMAIN\$env:USERNAME" -LogonType S4U -RunLevel Limited # no admin required

    Register-ScheduledTask -TaskName $TaskBg -Action $ActionBg -Trigger $TriggerBg -Settings $SettingsBg -Principal $PrincipalBg -Force | Out-Null # create/replace task
}
catch {
    Write-Host ("ERR install tasks: " + $_.Exception.Message) # errors only
    exit 1
}
