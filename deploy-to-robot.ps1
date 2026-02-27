# Build FTC TeamCode and install to Robot Controller phone via USB (adb).
# Run from project root. Requires: phone connected with USB debugging on.

$ErrorActionPreference = "Stop"

# Find adb: PATH first, then Android SDK platform-tools
$adb = $null
if (Get-Command adb -ErrorAction SilentlyContinue) { $adb = "adb" }
if (-not $adb -and $env:LOCALAPPDATA) {
    $sdkAdb = Join-Path $env:LOCALAPPDATA "Android\Sdk\platform-tools\adb.exe"
    if (Test-Path $sdkAdb) { $adb = $sdkAdb }
}
if (-not $adb -and $env:ANDROID_HOME) {
    $sdkAdb = Join-Path $env:ANDROID_HOME "platform-tools\adb.exe"
    if (Test-Path $sdkAdb) { $adb = $sdkAdb }
}
if (-not $adb) {
    Write-Host "adb not found. Add Android SDK platform-tools to PATH or set ANDROID_HOME." -ForegroundColor Red
    exit 1
}

Write-Host "Building TeamCode..." -ForegroundColor Cyan
& .\gradlew.bat :TeamCode:assembleDebug
if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed." -ForegroundColor Red
    exit 1
}
$apk = "TeamCode\build\outputs\apk\debug\TeamCode-debug.apk"
if (-not (Test-Path $apk)) {
    Write-Host "APK not found: $apk" -ForegroundColor Red
    exit 1
}
Write-Host "Installing to device..." -ForegroundColor Cyan
& $adb install -r $apk
if ($LASTEXITCODE -ne 0) {
    Write-Host "Install failed. Is the phone connected and USB debugging enabled? Try: $adb devices" -ForegroundColor Red
    exit 1
}
Write-Host "Done. App installed on Robot Controller." -ForegroundColor Green
