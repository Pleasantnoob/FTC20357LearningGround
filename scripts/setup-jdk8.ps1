# Setup JDK 8 for FTC build (Gradle 6.9 / FTC SDK require Java 8).
# Run from repo root:  .\scripts\setup-jdk8.ps1
# Or from scripts:   .\setup-jdk8.ps1  (then we cd to repo root)

$ErrorActionPreference = "Stop"
$repoRoot = if ($PSScriptRoot) { Join-Path $PSScriptRoot ".." } else { $PWD }
$repoRoot = (Resolve-Path $repoRoot).Path
$gradleProps = Join-Path $repoRoot "gradle.properties"

# 1) Check if JDK 8 is already in a known location
$possiblePaths = @(
    "C:\Program Files\Eclipse Adoptium\jdk-8*-hotspot",
    "C:\Program Files\Microsoft\jdk-8*",
    "C:\Program Files\Java\jdk1.8*",
    "C:\Program Files\AdoptOpenJDK\jdk-8*"
)
$jdk8Path = $null
foreach ($pattern in $possiblePaths) {
    $dirs = Get-Item -Path $pattern -ErrorAction SilentlyContinue
    if ($dirs) {
        $jdk8Path = $dirs[0].FullName
        $javaExe = Join-Path $jdk8Path "bin\java.exe"
        if (Test-Path $javaExe) {
            Write-Host "Found existing JDK 8: $jdk8Path"
            break
        }
        $jdk8Path = $null
    }
}

# 2) If not found, try winget install
if (-not $jdk8Path) {
    Write-Host "JDK 8 not found. Installing via winget (Eclipse Temurin 8)..."
    try {
        winget install -e --id EclipseAdoptium.Temurin.8.JDK --accept-package-agreements --accept-source-agreements
    } catch {
        Write-Host "winget install failed. Try installing manually from: https://adoptium.net/temurin/releases/?version=8&os=windows"
        exit 1
    }
    # After install, typical path (version may vary)
    $adoptiumBase = "C:\Program Files\Eclipse Adoptium"
    if (Test-Path $adoptiumBase) {
        $dirs = Get-ChildItem -Path $adoptiumBase -Directory -Filter "jdk-8*"
        if ($dirs) {
            $jdk8Path = $dirs[0].FullName
            Write-Host "Installed JDK 8 at: $jdk8Path"
        }
    }
}

# 3) Still no path? Try JAVA_HOME or where java is (might be JDK 8)
if (-not $jdk8Path -and $env:JAVA_HOME) {
    $ver = & "$env:JAVA_HOME\bin\java" -version 2>&1
    if ($ver -match '"1\.8') {
        $jdk8Path = $env:JAVA_HOME
        Write-Host "Using JAVA_HOME (1.8): $jdk8Path"
    }
}

if (-not $jdk8Path -or -not (Test-Path (Join-Path $jdk8Path "bin\java.exe"))) {
    Write-Host "Could not find or install JDK 8. Options:"
    Write-Host "  1. Install from https://adoptium.net/temurin/releases/?version=8&os=windows"
    Write-Host "  2. Run: winget install EclipseAdoptium.Temurin.8.JDK"
    Write-Host "  3. Then re-run this script, or set org.gradle.java.home in gradle.properties to your JDK 8 path."
    exit 1
}

# 4) Write gradle.properties (backslashes escaped for .properties file)
$pathForGradle = $jdk8Path -replace '\\', '\\\\'
$content = Get-Content $gradleProps -Raw
if ($content -match 'org\.gradle\.java\.home=.*') {
    $content = $content -replace 'org\.gradle\.java\.home=.*', "org.gradle.java.home=$pathForGradle"
} else {
    $content = $content.TrimEnd() + "`norg.gradle.java.home=$pathForGradle`n"
}
Set-Content -Path $gradleProps -Value $content -NoNewline
Write-Host "Updated gradle.properties with org.gradle.java.home=$jdk8Path"
Write-Host "You can now run: .\gradlew :TeamCode:compileDebugJavaWithJavac"