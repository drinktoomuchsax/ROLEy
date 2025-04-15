@echo off
chcp 65001 > nul

if "%1"=="" (
    echo 使用方法：b [Debug^|Release^|RelWithDebInfo^|MinSizeRel]
    echo 默认使用 Release 配置
    set preset=Release
) else (
    set preset=%1
)

echo 开始编译 %preset% 配置...
cmake --build build/%preset%
if %ERRORLEVEL% EQU 0 (
    echo 编译成功！
) else (
    echo 编译失败！
)
