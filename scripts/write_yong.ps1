<#
.SYNOPSIS
    Doghead Arm - 實體紙面「永」字書法一鍵執行腳本
.DESCRIPTION
    自動在 doghead_ros2_dev Docker 容器中呼叫 50Hz 連續平滑書法揮毫程序。
    支援整顆「永」字書寫或單筆測試。
.PARAMETER SingleStroke
    若加上此參數，僅測試第一筆「點」（快速確認落筆高度與出墨）。
.PARAMETER AutoStart
    若加上此參數，不等待按 Enter 直接揮毫。
.PARAMETER Speed
    書寫速度 (cm/s，預設 2.0)。
#>

[CmdletBinding()]
param (
    [switch]$SingleStroke,
    [switch]$AutoStart,
    [double]$Speed = 2.0
)

$argsList = ""
if ($SingleStroke) { $argsList += " --single_stroke" }
if ($AutoStart)    { $argsList += " --auto_start" }
if ($Speed -ne 2.0) { $argsList += " --speed $Speed" }

Write-Host "=================================================" -ForegroundColor Cyan
Write-Host "  🖌️  Doghead Arm - 實體紙面「永」字書法揮毫     " -ForegroundColor Cyan
Write-Host "=================================================" -ForegroundColor Cyan

docker exec -it doghead_ros2_dev bash -c "source /opt/ros/humble/setup.bash && source /workspace/Software/ros2_ws/install/setup.bash && python3 /workspace/Software/ros2_ws/draw_calligraphy_yong.py $argsList"

