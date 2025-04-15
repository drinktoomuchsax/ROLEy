@echo off
chcp 65001 > nul
echo 开始烧录...
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program build/Release/CtrBoard-H7_DBUS.bin 0x08000000 verify reset exit"
if %ERRORLEVEL% EQU 0 (
    echo 烧录成功！
) else (
    echo 烧录失败！请检查：
    echo 1. ST-LINK 是否正确连接
    echo 2. 板子是否上电
    echo 3. 驱动是否正确安装
)
