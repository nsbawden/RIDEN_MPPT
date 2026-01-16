# install_mppt_tasks.ps1
# Creates:
#  - RIDEN_MPPT_BG (Daily 8:00 AM, WakeToRun, runs riden_mppt.py in background with --bg)
#  - RIDEN_MPPT_STOP_ON_LOGON (At logon, kills ONLY the --bg instance so you can run it interactively)

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
$TaskKill = "RIDEN_MPPT_STOP_ON_LOGON" # task name

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

    # ----- Background task -----
    $ArgsBg = "`"$ScriptPath`" $TargetVin $ComPort $SlaveId --bg" # tag background instance with --bg
    $ActionBg = New-ScheduledTaskAction -Execute $PyExe -Argument $ArgsBg -WorkingDirectory $MpptDir # background action
    $TriggerBg = New-ScheduledTaskTrigger -Daily -At 8:00AM # 8am daily trigger
    $SettingsBg = New-ScheduledTaskSettingsSet -WakeToRun -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries # wake and run on battery

    # Run as your user (interactive token not required; still runs in background)
    $PrincipalBg = New-ScheduledTaskPrincipal -UserId "$env:USERDOMAIN\$env:USERNAME" -LogonType S4U -RunLevel Limited # no admin required

    Register-ScheduledTask -TaskName $TaskBg -Action $ActionBg -Trigger $TriggerBg -Settings $SettingsBg -Principal $PrincipalBg -Force | Out-Null # create/replace task

    # ----- Kill-on-logon task -----
    $KillCmd = @'
Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
Where-Object { $_.CommandLine -like "*riden_mppt.py*--bg*" } |
ForEach-Object { Stop-Process -Id $_.ProcessId -Force }
'@ # kill only the tagged background instance

    $KillEncoded = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($KillCmd)) # base64 for -EncodedCommand
    $ActionKill = New-ScheduledTaskAction -Execute "powershell.exe" -Argument "-NoProfile -WindowStyle Hidden -EncodedCommand $KillEncoded" # kill action
    $TriggerKill = New-ScheduledTaskTrigger -AtLogOn # when you log in
    $SettingsKill = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries # run regardless of power state

    $PrincipalKill = New-ScheduledTaskPrincipal -UserId "$env:USERDOMAIN\$env:USERNAME" -LogonType Interactive -RunLevel Limited # run in your user session

    Register-ScheduledTask -TaskName $TaskKill -Action $ActionKill -Trigger $TriggerKill -Settings $SettingsKill -Principal $PrincipalKill -Force | Out-Null # create/replace task
}
catch {
    Write-Host ("ERR install tasks: " + $_.Exception.Message) # errors only
    exit 1
}
