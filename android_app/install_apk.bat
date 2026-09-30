@echo off
echo =======================================================
echo  SmartBAN Android Monitor: APK Installation Pipeline
echo =======================================================
where adb >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    set ADB=adb
) else if exist "%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe" (
    set ADB="%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe"
) else (
    set ADB="C:\Users\hjafa\AppData\Local\Android\Sdk\platform-tools\adb.exe"
)

echo Listing attached Android devices:
%ADB% devices
echo.

set TARGET_APK="%~dp0SmartBAN_Monitor_v1.0.apk"
if not exist %TARGET_APK% set TARGET_APK="%~dp0app\build\outputs\apk\debug\app-debug.apk"

echo Installing %TARGET_APK% onto connected Android device...
%ADB% install -r %TARGET_APK%

if %ERRORLEVEL% EQU 0 (
    echo.
    echo [SUCCESS] SmartBAN Monitor installed successfully!
    echo Launching app on device...
    %ADB% shell am start -n com.smartban.sensor/.MainActivity
) else (
    echo.
    echo [NOTE] If device was not detected, please enable USB Debugging on your Samsung S22 Ultra and tap Allow USB Debugging.
)

pause
