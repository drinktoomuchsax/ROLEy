@echo off
chcp 65001 > nul
echo 开始编译...
cmake --build build/Release
if %ERRORLEVEL% NEQ 0 (
    echo 编译失败！
    exit /b 1
)
echo 编译成功，开始烧录...
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program build/Release/CtrBoard-H7_DBUS.bin 0x08000000 verify reset exit"
if %ERRORLEVEL% EQU 0 (
    echo 烧录成功！
) else (
    echo 烧录失败，请检查连接！
)
