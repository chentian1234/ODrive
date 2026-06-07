@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

rem ========== 配置区（根据你的环境修改） ==========
set FROMELF=D:\Users\chen\AppData\Local\Keil_v5\ARM\ARM_Compiler_5.06u7\Bin\fromelf.exe
set AXF=Example\ODrive-fw-v0.3.6\Firmware\mdk_app\output\object\project.axf
set OUT=Example\ODrive-fw-v0.3.6\Firmware\mdk_app\output
set OTA_SIZE=65536
rem =================================================

rem 切换到脚本所在目录
cd /d "%~dp0"

echo ========== STM32 OTA 固件生成 ==========
echo   脚本位置: %~dp0
echo   AXF: %AXF%
echo   输出: %OUT%
echo   分区: %OTA_SIZE% 字节
echo.

rem 检查 fromelf
if not exist "%FROMELF%" (
    echo [错误] fromelf 不存在: %FROMELF%
    pause
    exit /b 1
)

rem 检查 axf
if not exist "%AXF%" (
    echo [错误] AXF 文件不存在: %AXF%
    echo        请先编译 Keil 工程
    pause
    exit /b 1
)

rem 创建输出目录
if not exist "%OUT%" mkdir "%OUT%"

rem 获取工程名
for %%f in ("%AXF%") do set PRJ_NAME=%%~nf

rem 1. 生成原始 bin
echo [1/2] 正在生成原始 bin...
"%FROMELF%" --bincombined -o "%OUT%\%PRJ_NAME%.bin" "%AXF%"
if %errorlevel% neq 0 (
    echo [错误] fromelf 执行失败
    pause
    exit /b 1
)

rem 2. 填充 0xFF
echo [2/2] 正在生成 OTA 固件...
rem 创建临时 PowerShell 脚本
echo $r='%OUT%\%PRJ_NAME%.bin' > "%TEMP%\ota_pad.ps1"
echo $o='%OUT%\%PRJ_NAME%_ota.bin' >> "%TEMP%\ota_pad.ps1"
echo $s=%OTA_SIZE% >> "%TEMP%\ota_pad.ps1"
echo $d=[System.IO.File]::ReadAllBytes($r) >> "%TEMP%\ota_pad.ps1"
echo $len=$d.Length >> "%TEMP%\ota_pad.ps1"
echo if($len -gt $s) { Write-Host '错误: 文件过大'; exit 1 } >> "%TEMP%\ota_pad.ps1"
echo if($len -lt $s) { >> "%TEMP%\ota_pad.ps1"
echo     $pad=$s - $len >> "%TEMP%\ota_pad.ps1"
echo     $empty=[byte[]]::new($pad) >> "%TEMP%\ota_pad.ps1"
echo     for($i=0; $i -lt $pad; $i++) { $empty[$i]=0xFF } >> "%TEMP%\ota_pad.ps1"
echo     $d=$d + $empty >> "%TEMP%\ota_pad.ps1"
echo } >> "%TEMP%\ota_pad.ps1"
echo [System.IO.File]::WriteAllBytes($o, $d) >> "%TEMP%\ota_pad.ps1"
rem 执行脚本
powershell -File "%TEMP%\ota_pad.ps1"
rem 删除临时文件
del "%TEMP%\ota_pad.ps1"
if %errorlevel% neq 0 (
    echo [错误] 生成 OTA 固件失败
    pause
    exit /b 1
)

echo.
echo ========== 生成完成 ==========
echo   原始: %OUT%\%PRJ_NAME%.bin
echo   OTA:  %OUT%\%PRJ_NAME%_ota.bin
echo.