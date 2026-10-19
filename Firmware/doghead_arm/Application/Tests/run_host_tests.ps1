$ErrorActionPreference = 'Stop'

$testRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$firmwareRoot = Split-Path -Parent (Split-Path -Parent $testRoot)
$nativeCpp = Get-Command g++ -ErrorAction SilentlyContinue
$nativeC = Get-Command gcc -ErrorAction SilentlyContinue

if (-not $nativeCpp -or -not $nativeC) {
    throw 'Native g++ and gcc are required to execute firmware host tests. The STM32 cross compiler can only perform syntax checks.'
}

$tempRoot = Join-Path ([System.IO.Path]::GetTempPath()) ('doghead-firmware-host-tests-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $tempRoot | Out-Null

try {
    $include = @(
        "-I$firmwareRoot/Application/Inc",
        "-I$firmwareRoot/Application/Src",
        "-I$firmwareRoot/Drivers/Inc",
        "-I$firmwareRoot/Drivers/STM32F4xx_HAL_Driver/Inc",
        "-I$firmwareRoot/Drivers/CMSIS/Device/ST/STM32F4xx/Include",
        "-I$firmwareRoot/Drivers/CMSIS/Include",
        "-I$firmwareRoot/Middleware/MicroROS/Inc",
        "-I$firmwareRoot/Middlewares/Third_Party/FreeRTOS/Source/include",
        "-I$firmwareRoot/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
        "-I$firmwareRoot/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F",
        "-I$firmwareRoot/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include",
        "-I$firmwareRoot/micro_ros_stm32cubemx_utils/extra_sources",
        "-I$firmwareRoot/Core/Inc"
    )

    $robotTest = Join-Path $tempRoot 'test_robot_interface.exe'
    & $nativeCpp.Source '-x' 'c++' (Join-Path $testRoot 'test_robot_interface.c') `
        (Join-Path $firmwareRoot 'Application\Src\trajectory_planner.cpp') `
        (Join-Path $firmwareRoot 'Application\Src\safety_manager.cpp') `
        (Join-Path $firmwareRoot 'Application\Src\robot_config.cpp') `
        (Join-Path $firmwareRoot 'Application\Src\test_scenarios.cpp') `
        (Join-Path $firmwareRoot 'Application\Src\motor_config.c') `
        (Join-Path $firmwareRoot 'Application\Src\motor_parameters.c') `
        (Join-Path $firmwareRoot 'Application\Src\joint_units.c') `
        (Join-Path $firmwareRoot 'Application\Src\mainpp.cpp') `
        (Join-Path $firmwareRoot 'Drivers\Src\nidec_motor_driver.c') `
        '-DHOST_TEST' '-std=c++14' '-fno-exceptions' '-fno-rtti' @include '-o' $robotTest
    if ($LASTEXITCODE -ne 0) { throw 'test_robot_interface compilation failed.' }
    & $robotTest
    if ($LASTEXITCODE -ne 0) { throw 'test_robot_interface failed.' }

    $encoderTest = Join-Path $tempRoot 'test_encoder_count.exe'
    & $nativeC.Source (Join-Path $testRoot 'test_encoder_count.c') `
        '-DHOST_TEST' '-std=c11' @include '-o' $encoderTest
    if ($LASTEXITCODE -ne 0) { throw 'test_encoder_count compilation failed.' }
    & $encoderTest
    if ($LASTEXITCODE -ne 0) { throw 'test_encoder_count failed.' }

    $jointUnitsTest = Join-Path $tempRoot 'test_joint_units.exe'
    & $nativeC.Source (Join-Path $testRoot 'test_joint_units.c') `
        (Join-Path $firmwareRoot 'Application\Src\joint_units.c') `
        (Join-Path $firmwareRoot 'Application\Src\motor_parameters.c') `
        (Join-Path $firmwareRoot 'Drivers\Src\nidec_motor_driver.c') `
        '-DHOST_TEST' '-std=c11' @include '-lm' '-o' $jointUnitsTest
    if ($LASTEXITCODE -ne 0) { throw 'test_joint_units compilation failed.' }
    & $jointUnitsTest
    if ($LASTEXITCODE -ne 0) { throw 'test_joint_units failed.' }
}
finally {
    Remove-Item -LiteralPath $tempRoot -Recurse -Force -ErrorAction SilentlyContinue
}
