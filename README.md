# 3Dify

Android application for 3D reconstruction of real-world environments using depth camera data, ARCore, and the Point Cloud Library (PCL).

The app captures raw depth frames via ARCore, processes the resulting point cloud through a native C++ pipeline (PCL), and exports the result as a `.obj` mesh file that can be viewed in the built-in 3D viewer.

---

## Architecture

```
app/                  — Main Android application (Kotlin, ARCore, UI)
├── arcore/           — ARCore session management and depth rendering
├── rawdepth/         — Raw depth frame acquisition (DepthData)
├── ui/               — Camera, Gallery, Settings, Home fragments
└── helpers/          — Point clustering, navigation, permissions

nativelib/            — Native C++ module (JNI bridge to PCL)
├── NativeLib.kt      — JNI declarations
├── Model3DCreator.kt — Kotlin wrapper for point cloud processing
└── cpp/
    ├── PointCloudTo3DModel.cpp  — PCL pipeline (voxel filter → OBJ export)
    └── CMakeLists.txt           — CMake build using Prefab PCL package

engine/               — 3D rendering engine
viewerapp/            — Standalone OBJ viewer module
```

---

## Tech Stack

| Layer | Technology |
|---|---|
| Language | Kotlin + C++ (NDK) |
| Build | Gradle 8.14.5, AGP 8.13.2 |
| AR / Depth | ARCore 1.54.0 |
| 3D rendering | Google Sceneform 1.17.1 |
| Point Cloud | PCL 1.9.1 via [pcl-android-arm64](https://github.com/V-Serghei/pcl-binaries-android-armv8) |
| Python scripting | Chaquopy 17.0.0 (Python 3.10) |
| Camera | CameraX 1.6.1 |
| Min SDK | 29 (Android 10) |
| Target SDK | 37 |
| ABI | arm64-v8a only |

---

## PCL Native Dependency

The C++ code uses **Point Cloud Library (PCL)** for 3D processing. Instead of vendoring source, the project pulls a prebuilt AAR from GitHub Packages:

- **Package:** `io.github.vserghei:pcl-android-arm64:1.0.2`
- **Repository:** https://github.com/V-Serghei/pcl-binaries-android-armv8
- **Format:** Android AAR with Prefab metadata
- **Bundled libraries:** PCL 1.9.1, Boost 1.70, Eigen 3.3.7, FLANN 1.9.1, LZ4 1.9.1

GitHub Packages requires authentication even for public packages. Every developer must configure a GitHub Personal Access Token locally.

---

## Prerequisites

- **Android Studio** Hedgehog or newer (bundles JDK 21)
- **Android NDK 26.1.10909125** — exact version required (see note below)
- **CMake** 3.22.1 (install via SDK Manager)
- **Python 3.10** — required on the build machine for Chaquopy (see note below)
- **GitHub account** with access to read packages

---

### Installing NDK 26.1.10909125

> **Why NDK 26 specifically?**
> The PCL package bundles FLANN headers that use `std::binary_function`, removed in C++17 under NDK 27+.
> NDK 26 is the version the package was built and tested against. NDK 27+ causes a compile error in `flann/util/heap.h`.

1. Open **Android Studio → Tools → SDK Manager**
2. Go to the **SDK Tools** tab
3. Check **Show Package Details** (bottom right corner)
4. Expand **NDK (Side by side)**
5. Check **26.1.10909125**
6. Click **Apply** and wait for the download to finish

You can have multiple NDK versions installed — `nativelib/build.gradle` pins `ndkVersion "26.1.10909125"` so Gradle picks the correct one automatically.

---

### Installing Python 3.10

> **Why Python on the build machine?**
> [Chaquopy](https://chaquo.com/chaquopy/) cross-compiles Python packages (`numpy`, `trimesh`, `requests`)
> for Android at build time. It needs a matching Python 3.10 interpreter on your machine to do this.
> This is a **build-time** requirement — Python is not bundled in the APK directly.

**Windows (recommended):**
```powershell
winget install Python.Python.3.10
```

**macOS:**
```bash
brew install python@3.10
```

**Linux:**
```bash
sudo apt install python3.10
```

After installing, **restart Android Studio** (or your terminal) so Chaquopy can find the new interpreter.

---

## Setup

### 1. Clone the repository

```bash
git clone https://github.com/YOUR_ORG/3Dify.git
cd 3Dify
```

### 2. Create a GitHub Personal Access Token

GitHub Packages requires a token with `read:packages` scope.

1. Go to **GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic)**
2. Click **Generate new token (classic)**
3. Give it a name, e.g. `3Dify PCL read`
4. Select scope: `read:packages`
5. Click **Generate token**
6. Copy the token immediately — it is shown only once

### 3. Add the token to your local Gradle properties

Create or edit the file `~/.gradle/gradle.properties` (on Windows: `C:\Users\YOUR_USERNAME\.gradle\gradle.properties`):

```properties
gpr.user=YOUR_GITHUB_USERNAME
gpr.key=YOUR_GITHUB_TOKEN
```

> **Never commit this file or tokens to any repository.**
> The `~/.gradle/gradle.properties` file is outside the project directory and is never tracked by git.

### 4. Open in Android Studio

Open the `3Dify` folder in Android Studio and wait for Gradle sync to complete. Gradle will automatically download the PCL package from GitHub Packages.

### 5. Build and run

Connect a physical Android device (arm64, API 29+, depth camera required for full functionality) and press **Run**.

Or build from the command line:

```bash
# Windows
.\gradlew.bat :app:assembleDebug

# macOS / Linux
./gradlew :app:assembleDebug
```

---

## Environment Variables (CI / GitHub Actions)

For automated builds, set these secrets in your repository settings:

| Secret | Value |
|---|---|
| `GITHUB_ACTOR` | GitHub username of the token owner |
| `GITHUB_TOKEN` | PAT with `read:packages` scope |

The `settings.gradle` reads them automatically:

```groovy
username = providers.gradleProperty("gpr.user").orNull ?: System.getenv("GITHUB_ACTOR")
password = providers.gradleProperty("gpr.key").orNull ?: System.getenv("GITHUB_TOKEN")
```

---

## PCL Package Repository

The prebuilt PCL binaries are maintained in a separate repository:

| Field | Value |
|---|---|
| Repository | https://github.com/V-Serghei/pcl-binaries-android-armv8 |
| Maven group | `io.github.vserghei` |
| Artifact | `pcl-android-arm64` |
| Current version | `1.0.2` |
| ABI | `arm64-v8a` only |

To use a newer version of the package, update `nativelib/build.gradle`:

```groovy
implementation "io.github.vserghei:pcl-android-arm64:NEW_VERSION"
```

---

## Project Modules

### `:app`
Main application module. Handles ARCore session lifecycle, captures raw depth frames, triggers the native processing pipeline, and displays results in the gallery.

### `:nativelib`
Native C++ module. Receives a point cloud (array of XYZ coordinates) from Kotlin via JNI, runs it through PCL (voxel grid filter, surface reconstruction), and writes the result as an OBJ file.

### `:engine`
3D rendering engine shared between the main app and the viewer.

### `:viewerapp`
Standalone viewer that loads and renders OBJ files using Sceneform.

---

## Troubleshooting

**Gradle sync fails with 401 Unauthorized**

Your GitHub token is missing or incorrect. Check `~/.gradle/gradle.properties`:
```properties
gpr.user=YOUR_GITHUB_USERNAME
gpr.key=YOUR_TOKEN_WITH_READ_PACKAGES
```

**`find_package(pclibrary REQUIRED CONFIG)` fails in CMake**

Make sure `prefab true` is enabled in `nativelib/build.gradle`:
```groovy
buildFeatures {
    prefab true
}
```

**App crashes on launch — no depth data**

ARCore Depth API requires a device with a hardware depth sensor (Time-of-Flight camera). The app will not work on emulators or devices without depth support.

**Compile error: `no template named 'binary_function' in namespace 'std'`**

The FLANN headers bundled in the PCL package use `std::binary_function` which was removed in C++17.
The fix is already applied in `CMakeLists.txt` via `_LIBCPP_ENABLE_CXX17_REMOVED_FEATURES` which
re-enables the removed features in libc++. If you see this error, make sure you have a clean build:

```bash
.\gradlew.bat clean :nativelib:assembleDebug
```

**Build fails for x86 / x86_64 / armeabi-v7a**

The PCL package is `arm64-v8a` only. The project ABI is already restricted to `arm64-v8a` in both `app/build.gradle` and `nativelib/build.gradle`. Do not add other ABIs.

**Windows: locked build files**

```bash
.\gradlew.bat --stop
.\gradlew.bat :app:assembleDebug --no-daemon
```

---

## Security

- Never put GitHub tokens in any Gradle file inside the project
- Never commit `~/.gradle/gradle.properties`
- Use GitHub Actions `GITHUB_TOKEN` or repository secrets for CI
- The `.gitignore` already excludes `local.properties` and all machine-specific files
