# Building the project

The FTC SDK and this project’s Gradle version require **JDK 8** and the **Android SDK** to compile. If you see “invalid Java home” or “Unsupported class file major version 65”, fix JDK (below). If you see “SDK location not found”, set the Android SDK path (below).

## One-time setup: JDK 8

From the **repo root** run:

```powershell
.\scripts\setup-jdk8.ps1
```

This script will:

1. Look for an existing JDK 8 (e.g. under `C:\Program Files\Eclipse Adoptium\jdk-8*-hotspot`).
2. If none is found, install **Eclipse Temurin JDK 8** via `winget` (you may get a UAC prompt).
3. Update `gradle.properties` with `org.gradle.java.home` so Gradle uses that JDK 8.

If you prefer to install manually:

- Download JDK 8 from [Adoptium Temurin 8](https://adoptium.net/temurin/releases/?version=8&os=windows).
- Install it, then in `gradle.properties` set:
  - `org.gradle.java.home=C:\\Program Files\\Eclipse Adoptium\\jdk-8.0.xxx-hotspot`  
  (use your actual path; double backslashes required.)

## Build

From the repo root:

```powershell
.\gradlew :TeamCode:compileDebugJavaWithJavac
```

Full assemble (APK):

```powershell
.\gradlew assembleDebug
```

## Android SDK (if “SDK location not found”)

The build needs the Android SDK (usually installed with **Android Studio**). A `local.properties` file in the repo root should contain:

```properties
sdk.dir=C:\\Users\\YourUsername\\AppData\\Local\\Android\\sdk
```

Replace `YourUsername` with your Windows user name. If you use Android Studio, opening this project once will create `local.properties` automatically. Otherwise create it manually with the path where the SDK is installed. `local.properties` is in `.gitignore` (machine-specific).
