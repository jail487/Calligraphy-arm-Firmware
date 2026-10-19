<#
.SYNOPSIS
    Doghead Arm - 實體紙面正方形校準一鍵執行腳本
.DESCRIPTION
    自動在 doghead_ros2_dev 容器中執行已校準傾斜與幾何比例補償的正方形繪製。
#>

[CmdletBinding()]
param (
    [string]$Mode = "paper",
    [switch]$Compensate = $true,
    [switch]$AutoStart
)

$argsList = "--mode $Mode"
if ($Compensate) { $argsList += " --compensate" }
if ($AutoStart)  { $argsList += " --auto_start" }

Write-Host "=================================================" -ForegroundColor Green
Write-Host "  🔲 Doghead Arm - 正方形校準繪製                " -ForegroundColor Green
Write-Host "=================================================" -ForegroundColor Green

docker exec -it doghead_ros2_dev bash -c "source /opt/ros/humble/setup.bash && source /workspace/Software/ros2_ws/install/setup.bash && python3 /workspace/Software/ros2_ws/draw_calibration_square.py $argsList"

