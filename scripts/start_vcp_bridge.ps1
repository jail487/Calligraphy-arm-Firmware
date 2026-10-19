<#
.SYNOPSIS
    Starts the Windows ST-Link VCP bidirectional TCP bridge and containerized micro-ROS Agent.
#>

[CmdletBinding()]
param (
    [string]$ComPort = "auto",
    [int]$BaudRate = 460800,
    [int]$TcpPort = 8888
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$BridgePy = Join-Path $ScriptDir "win_vcp_bridge.py"

Write-Host "========================================" -ForegroundColor Green
Write-Host "  ST-Link VCP TCP Bridge Entrypoint     " -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green

& python "$BridgePy" --com $ComPort --baud $BaudRate --port $TcpPort
