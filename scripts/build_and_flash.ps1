<#
.SYNOPSIS
    一鍵命令列「自動編譯 + 燒錄」STM32 韌體腳本
.DESCRIPTION
    調用 STM32CubeIDE 的 Headless Build 編譯 doghead_arm 專案，
    編譯成功後自動透過 ST-Link 燒錄進 STM32 並自動重啟。
#>

[CmdletBinding()]
param (
    [switch]$FlashOnly = $false
)

$ErrorActionPreference = "Stop"

$ProjectDir = (Resolve-Path "Firmware\doghead_arm").Path
$ElfPath = Join-Path $ProjectDir "Debug\doghead_arm.elf"

# 1. 尋找 IDE 與 Programmer
$IdeCli = Get-ChildItem -Path "C:\ST\STM32CubeIDE*" -Filter "stm32cubeidec.exe" -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty FullName
$ProgrammerCli = Get-ChildItem -Path "C:\ST\STM32CubeIDE*" -Filter "STM32_Programmer_CLI.exe" -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty FullName

if (-not $FlashOnly -and $IdeCli -and (Test-Path $IdeCli)) {
    Write-Host "========================================" -ForegroundColor Cyan
    Write-Host "  1. 正在進行命令列無頭編譯 (Headless Build)..." -ForegroundColor Cyan
    Write-Host "========================================" -ForegroundColor Cyan
    
    $TempWorkspace = [System.IO.Path]::GetTempPath() + "stm32_build_ws"
    & "$IdeCli" --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data "$TempWorkspace" -import "$ProjectDir" -build "doghead_arm/Debug"
    
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "命令列編譯回傳代碼非 0，若已有先前編譯之 elf 檔將嘗試繼續燒錄..."
    } else {
        Write-Host "[BUILD SUCCESS] 編譯完成！" -ForegroundColor Green
    }
}

# 2. 燒錄
if (-not $ProgrammerCli -or -not (Test-Path $ProgrammerCli)) {
    Write-Error "找不到 STM32_Programmer_CLI.exe，請確認是否安裝 STM32CubeIDE。"
    exit 1
}

if (-not (Test-Path $ElfPath)) {
    Write-Error "找不到韌體檔案: '$ElfPath'，請先確認編譯是否成功。"
    exit 1
}

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "  2. 正在燒錄韌體至 STM32..." -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

& "$ProgrammerCli" -c port=SWD mode=UR -w "$ElfPath" -v -rst

if ($LASTEXITCODE -eq 0) {
    Write-Host "`n[SUCCESS] 燒錄完成！STM32 已自動重啟運行新韌體！" -ForegroundColor Green
} else {
    Write-Error "[FAIL] 燒錄失敗 (Exit Code: $LASTEXITCODE)，請檢查 ST-Link USB 連線是否正常。"
}

