param(
    [string]$Distro = "Ubuntu-22.04",
    [string]$UnitreeRepo = "/home/jomo/Projects/xr_teleoperate"
)

$ErrorActionPreference = "Stop"
$backendWindowsPath = Join-Path $PSScriptRoot "g1_teleop_backend.py"
$drive = $backendWindowsPath.Substring(0, 1).ToLowerInvariant()
$relativePath = $backendWindowsPath.Substring(2).Replace('\', '/')
$backendLinuxPath = "/mnt/$drive$relativePath"

Write-Host "Starting the simulation-only Unitree G1 IK backend..." -ForegroundColor Cyan
Write-Host "Keep this window open while using BACKEND TELEOP or QUEST HAND RETARGET." -ForegroundColor Yellow

# Clean up only stale processes created by this launcher. A forcibly closed terminal can
# otherwise leave the relay holding its UDP ports and make the next launch appear broken.
Get-CimInstance Win32_Process | Where-Object {
    ($_.Name -eq 'python.exe' -and $_.CommandLine -like '*wsl_udp_relay.py*') -or
    ($_.Name -eq 'wsl.exe' -and $_.CommandLine -like '*g1_teleop_backend.py*')
} | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }

$backend = Start-Process wsl.exe -ArgumentList @(
    '-d', $Distro, '--', '/home/jomo/miniforge3/envs/tv/bin/python', $backendLinuxPath,
    '--unitree-repo', $UnitreeRepo, '--port', '7547'
) -NoNewWindow -PassThru
$relay = $null

try {
    # Do not expose the Quest-facing relay until Pinocchio/CasADi has bound the WSL UDP port.
    # Sending to WSL localhost before that happens can leave the Windows UDP forwarding socket
    # in a reset state even after the backend becomes ready.
    Start-Sleep -Seconds 8
    $backend.Refresh()
    if ($backend.HasExited) { throw "The WSL G1 solver exited during startup." }

    $relay = Start-Process python -ArgumentList (Join-Path $PSScriptRoot 'wsl_udp_relay.py') -NoNewWindow -PassThru
    Start-Sleep -Seconds 2
    $backend.Refresh()
    $relay.Refresh()
    if ($backend.HasExited) { throw "The WSL G1 solver exited during startup." }
    if ($relay.HasExited) { throw "The Windows UDP relay exited during startup (ports may already be occupied)." }

    Write-Host "READY - select BACKEND TELEOP or QUEST HAND RETARGET in the Quest HUD." -ForegroundColor Green
    Write-Host "Press Ctrl+C or close this window to stop the backend." -ForegroundColor DarkGray
    while (-not $backend.HasExited -and -not $relay.HasExited) {
        Start-Sleep -Seconds 1
        $backend.Refresh()
        $relay.Refresh()
    }
    throw "A backend process stopped unexpectedly. Review the messages above."
}
finally {
    if ($relay) { Stop-Process -Id $relay.Id -Force -ErrorAction SilentlyContinue }
    Stop-Process -Id $backend.Id -Force -ErrorAction SilentlyContinue
}
