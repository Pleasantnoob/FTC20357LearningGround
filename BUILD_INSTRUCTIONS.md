# FTC build instructions

Build the Robot Controller app (includes TeamCode and FtcRobotController):

**Windows (PowerShell or Command Prompt):**
```powershell
cd c:\Users\chris\Documents\GitHub\stuff\FTC20357LearningGround
.\gradlew.bat :TeamCode:assembleDebug
```

**Windows (cmd with `&`):**
```cmd
cd /d "c:\Users\chris\Documents\GitHub\stuff\FTC20357LearningGround" & gradlew.bat :TeamCode:assembleDebug
```

**Mac/Linux:**
```bash
cd /path/to/FTC20357LearningGround
./gradlew :TeamCode:assembleDebug
```

- The **TeamCode** module is the application module (it applies `build.common.gradle` and produces the APK).
- Output APK: `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`
- Install to the Robot Controller phone via USB or copy the APK.

To clean and rebuild:
```powershell
.\gradlew.bat clean :TeamCode:assembleDebug
```

---

## Deploy to robot (build + install to phone)

**Yes — if the Robot Controller phone is connected via USB to the same computer where Cursor runs, I can build and install the app for you.** I’d run:

1. `.\gradlew.bat :TeamCode:assembleDebug` (build)
2. `adb install -r TeamCode\build\outputs\apk\debug\TeamCode-debug.apk` (install to the connected device)

**What you need:**

1. **USB connection** — Robot Controller phone plugged into the PC with a **data** USB cable (not charge-only).
2. **USB debugging enabled** on the phone: Settings → Developer options → USB debugging (and enable Developer options if needed).
3. **`adb` on your PATH** — from the Android SDK (e.g. Android Studio’s SDK). Typical location: `%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe`. If you use Android Studio for FTC, this is usually already set up.

**One-shot deploy (PowerShell, from project root):**
```powershell
.\gradlew.bat :TeamCode:assembleDebug; if ($LASTEXITCODE -eq 0) { adb install -r TeamCode\build\outputs\apk\debug\TeamCode-debug.apk }
```

**Check that the phone is seen by ADB:**
```powershell
adb devices
```

If you plug in the phone and run the deploy command (or ask me to run it), the built app will install to the robot. **FTC note:** Android Studio / Gradle users should deploy this way (or via Android Studio Run). Do not replace the app with the stock RC APK from REV Hardware Client or the FTC releases, or you’ll lose your team code.
