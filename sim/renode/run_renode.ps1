<#
.SYNOPSIS
    Runs STM32 target binaries inside Antmicro Renode simulation.

.PARAMETER Target
    The simulation target: "node1", "node2", or "scanner". Default is "node1".

.PARAMETER Gui
    If specified, launches Renode with interactive graphical Monitor and serial analyzer.
    If omitted, runs headlessly in console mode.

.PARAMETER Duration
    Virtual simulation duration before automatic exit (e.g. "00:00:03"). Default is "00:00:03".
#>
param(
    [ValidateSet("node1", "node2", "scanner", "multi")]
    [string]$Target = "node1",
    [switch]$Gui,
    [string]$Duration = "00:00:03"
)

$ErrorActionPreference = "Stop"

$RenodePaths = @(
    "C:\Users\aman\tools\renode_1.16.0-dotnet_portable\renode.exe",
    "C:\Program Files\Renode\bin\renode.exe"
)

$RenodeExe = $null
foreach ($path in $RenodePaths) {
    if (Test-Path $path) {
        $RenodeExe = $path
        break
    }
}

if (-not $RenodeExe) {
    $cmd = Get-Command renode -ErrorAction SilentlyContinue
    if ($cmd) {
        $RenodeExe = $cmd.Source
    }
}

if (-not $RenodeExe) {
    Write-Error "Renode executable could not be found. Please ensure it is installed in C:\Users\aman\tools\renode_1.16.0-dotnet_portable\renode.exe"
}

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

$ScriptMap = @{
    "node1"   = "$ScriptDir\run_node1.resc"
    "node2"   = "$ScriptDir\run_node2.resc"
    "scanner" = "$ScriptDir\run_g474_scanner.resc"
    "multi"   = "$ScriptDir\multi_node_network.resc"
}

$RescFile = $ScriptMap[$Target]
if (-not (Test-Path $RescFile)) {
    Write-Error "Simulation script not found: $RescFile"
}

Write-Host "=== Launching Renode Simulation: $Target ===" -ForegroundColor Cyan
Write-Host "Engine: $RenodeExe" -ForegroundColor DarkGray
Write-Host "Script: $RescFile" -ForegroundColor DarkGray

$BaseScript = Get-Content $RescFile -Raw
$TempScript = [System.IO.Path]::GetTempFileName() + ".resc"

try {
    if ($Gui) {
        $FullScript = $BaseScript + "`nstart`n"
        [System.IO.File]::WriteAllText($TempScript, $FullScript)
        & $RenodeExe $TempScript
    } else {
        $FullScript = $BaseScript + "`nemulation RunFor `"$Duration`"`nquit`n"
        [System.IO.File]::WriteAllText($TempScript, $FullScript)
        $null | & $RenodeExe --plain --console $TempScript
    }
} finally {
    if (Test-Path $TempScript) {
        Remove-Item $TempScript -Force -ErrorAction SilentlyContinue
    }
}
