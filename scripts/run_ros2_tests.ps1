[CmdletBinding()]
param(
    [switch]$KeepContainer
)

$ErrorActionPreference = 'Stop'

# Use a one-shot container for build/test work. A failed run is retained for
# inspection; a successful run is removed automatically below.
$suffix = [guid]::NewGuid().ToString('N').Substring(0, 8)
$containerName = "doghead_ros2_test_$suffix"
$testCommand = @'
set -e
source /opt/ros/humble/setup.bash
cd /workspace/Software/ros2_ws
colcon build --packages-select doghead_interfaces doghead_driver doghead_motion doghead_hil doghead_bringup
colcon test --packages-select doghead_driver doghead_motion doghead_hil doghead_bringup
colcon test-result --all --verbose
'@

$exitCode = 1
try {
    Write-Host "[INFO] Starting disposable ROS 2 test container: $containerName"
    & docker compose run --name $containerName ros2-dev bash -lc $testCommand
    $exitCode = $LASTEXITCODE
}
finally {
    $remaining = & docker ps -aq -f "name=^/$containerName$" 2>$null

    if ($remaining -and $exitCode -eq 0 -and -not $KeepContainer) {
        Write-Host "[INFO] Tests passed. Removing $containerName"
        & docker rm $containerName | Out-Null
    }
    elseif ($remaining) {
        Write-Host "[INFO] Keeping $containerName for inspection."
        Write-Host "       Remove it later with: docker rm $containerName"
    }
}

exit $exitCode
