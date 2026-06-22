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

## PCL Android Dependency

The C++ code uses **Point Cloud Library (PCL)** via a prebuilt AAR with Prefab metadata — source is not vendored in the repository.

### Primary: Maven Central (recommended, no token required)

```groovy
// nativelib/build.gradle
dependencies {
    implementation "io.github.v-serghei:pcl-android-arm64:1.0.4"
}
```

```groovy
// settings.gradle — no special repository configuration needed
dependencyResolutionManagement {
    repositories {
        google()
        mavenCentral()   // PCL is published here
    }
}
```

No GitHub account and no personal access token needed. Works out of the box for all contributors and CI.

### Fallback: GitHub Packages (requires authentication)

If you need a version not yet published on Maven Central, the package is also available on GitHub Packages:

| Field | Value |
|---|---|
| Repository | https://github.com/V-Serghei/pcl-binaries-android-armv8 |
| Maven group | `io.github.v-serghei` |
| Artifact | `pcl-android-arm64` |
| Current version | `1.0.4` |

GitHub Packages requires a token with `read:packages` scope even for public packages.

**Step 1** — Create a GitHub Personal Access Token with `read:packages` scope:
1. Go to **GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic)**
2. Click **Generate new token (classic)**, select scope `read:packages`
3. Copy the token immediately — it is shown only once

**Step 2** — Add credentials to `~/.gradle/gradle.properties` (never commit this file):
```properties
gpr.user=YOUR_GITHUB_USERNAME
gpr.key=YOUR_GITHUB_TOKEN
```

**Step 3** — Add the GitHub Packages repository to `settings.gradle`:
```groovy
maven {
    url = uri("https://maven.pkg.github.com/V-Serghei/pcl-binaries-android-armv8")
    credentials {
        username = providers.gradleProperty("gpr.user").orNull ?: System.getenv("GITHUB_ACTOR")
        password = providers.gradleProperty("gpr.key").orNull ?: System.getenv("GITHUB_TOKEN")
    }
}
```

> The project currently uses **Maven Central** as the only source. The GitHub Packages steps above are for reference only.

### Bundled libraries

| Library | Version |
|---|---|
| PCL | 1.9.1 |
| Boost | 1.70 |
| Eigen | 3.3.7 |
| FLANN | 1.9.1 |
| LZ4 | 1.9.1 |

---

## Prerequisites

- **Android Studio** Hedgehog or newer (bundles JDK 21)
- **Android NDK 26.1.10909125** — exact version required (see note below)
- **CMake** 3.22.1 (install via SDK Manager)
- **Python 3.10** — required on the build machine for Chaquopy (see note below)

No GitHub account or token required for a standard build.

---

### 16KB page size (Android 15+)

Android 15 requires native libraries to have LOAD segments aligned to 16KB. The flag is already applied in `nativelib/src/main/cpp/CMakeLists.txt`:

```cmake
target_link_options(nativelib PRIVATE -Wl,-z,max-page-size=16384)
```

This fixes `libnativelib.so`. Other libraries in the warning dialog are third-party and cannot be fixed from this project:

| Library | Owner | Status |
|---|---|---|
| `libpclibrary.so` | `pcl-binaries-android-armv8` (your repo) | Fixed in AAR 1.0.4 |
| `libarcore_sdk_jni.so` | Google ARCore | Wait for ARCore update |
| `libchaquopy_java.so` / Python libs | Chaquopy | Wait for Chaquopy update |
| `libarsceneview_jni.so` / Filament libs | Sceneform 1.17.1 (abandoned) | No fix available |

The warning is informational only — the app runs normally on all Android versions.

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

### 2. Install NDK 26 and Python 3.10

See the prerequisite sections above.

### 3. Open in Android Studio

Open the `3Dify` folder and wait for Gradle sync to complete. Gradle downloads the PCL package from Maven Central automatically — no token needed.

### 4. Build and run

Connect a physical Android device (arm64, API 29+, depth camera required for full functionality) and press **Run**.

Or build from the command line:

```bash
# Windows
.\gradlew.bat :app:assembleDebug

# macOS / Linux
./gradlew :app:assembleDebug
```

---

## CI / GitHub Actions

No special secrets are needed — the PCL package is pulled from Maven Central. Just run the standard Gradle build step.

If you switch to the GitHub Packages fallback, add these repository secrets:

| Secret | Value |
|---|---|
| `GITHUB_ACTOR` | GitHub username of the token owner |
| `GITHUB_TOKEN` | PAT with `read:packages` scope |

---

## Updating the PCL Version

To use a newer version, update the dependency in [nativelib/build.gradle](nativelib/build.gradle):

```groovy
implementation "io.github.v-serghei:pcl-android-arm64:NEW_VERSION"
```

The package source repository is: https://github.com/V-Serghei/pcl-binaries-android-armv8

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

**Gradle sync fails with 401 Unauthorized**

This only happens if the GitHub Packages fallback is configured. Check `~/.gradle/gradle.properties`:
```properties
gpr.user=YOUR_GITHUB_USERNAME
gpr.key=YOUR_TOKEN_WITH_READ_PACKAGES
```

---

## Security

- Never put GitHub tokens in any Gradle file inside the project
- Never commit `~/.gradle/gradle.properties`
- Use repository secrets for CI — no secrets needed for the Maven Central path
- The `.gitignore` already excludes `local.properties` and all machine-specific files
