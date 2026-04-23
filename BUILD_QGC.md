# QGC Build & Launch (macOS, Stream Cloud Dev)

Quick reference for rebuilding and launching QGroundControl during
stream-cloud development on macOS.

## One-liner (what I actually use)

```bash
pkill -f "Release/QGroundControl" 2>/dev/null; sleep 1 \
  && cmake --build build --target QGroundControl -j8 \
  && ./build/Release/QGroundControl.app/Contents/MacOS/QGroundControl > /tmp/qgc.log 2>&1 &
```

- Kills any running QGC instance first (so the rebuild can replace the bundle).
- Rebuilds just the `QGroundControl` target with 8 parallel jobs — skips the
  full tree and stays under ~25 s on a warm cache.
- Launches the freshly-built `.app`, piping stdout/stderr to `/tmp/qgc.log`,
  detached (`&`) so you keep the shell.

## Step-by-step

### 1. Kill any running instance

```bash
pkill -f "Release/QGroundControl" 2>/dev/null; sleep 1
```

### 2. Rebuild

```bash
cmake --build build --target QGroundControl -j8
```

- The `build/` directory was created once via `cmake -S . -B build …`
  (see CMakePresets below).
- `-j8` can be tuned to your core count.

### 3. Launch

```bash
./build/Release/QGroundControl.app/Contents/MacOS/QGroundControl
```

If you want the log piped somewhere:

```bash
./build/Release/QGroundControl.app/Contents/MacOS/QGroundControl \
  > /tmp/qgc.log 2>&1 &
```

### 4. Tail the log (new terminal)

```bash
tail -f /tmp/qgc.log
```

Filter for stream-cloud messages:

```bash
tail -f /tmp/qgc.log | grep -i streamcloud
```

## One-time setup

If `build/` doesn't exist (fresh checkout, clean build), configure first:

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DQGC_MACOS_UNIVERSAL_BUILD=OFF \
  -DCMAKE_OSX_ARCHITECTURES=arm64
```

The `QGC_MACOS_UNIVERSAL_BUILD=OFF` + `CMAKE_OSX_ARCHITECTURES=arm64` combo
skips the x86_64h slice on Apple silicon — it was the fix for the earlier
linker error (`ld: symbol(s) not found for architecture x86_64h`).

## Clean rebuild (rare)

When QML/QRC changes aren't picked up:

```bash
rm -rf build/Release/QGroundControl.app
cmake --build build --target QGroundControl -j8
```

Full nuke (last resort):

```bash
rm -rf build build-android
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DQGC_MACOS_UNIVERSAL_BUILD=OFF -DCMAKE_OSX_ARCHITECTURES=arm64
cmake --build build --target QGroundControl -j8
```

## Stream Cloud quick checks

- Default WebSocket endpoint: `ws://192.168.144.100:9091` (set in
  `src/FlyView/FlyViewStreamCloud.qml`, `companionIp` + `wsPort`).
- Open the viewer from the Fly View toolbar ("Stream Cloud" button).
- Point-cloud frames are tag `0x01`, odometry is `0x02` (see
  `src/FlyView/FlyViewStreamCloudData.js`).

## Useful keyboard shortcuts (inside the stream cloud viewer)

| Key          | Action                                      |
|--------------|---------------------------------------------|
| `1` / `2` / `3` | Top / Side / Perspective view presets    |
| `V`          | Toggle FPV                                   |
| `Shift+V`    | Cycle FPV heading offset (90°)              |
| `T`          | Toggle flight trace                          |
| `L`          | Re-level world to drone                      |
| `Shift+L`    | Disable level                                |
| `Shift+↑↓←→` | Nudge level trim 1° at a time               |
| `[` / `]`    | Shrink / grow point size                     |
| `;` / `'`    | Shrink / grow density-cap cell               |
| `Shift+D`    | Toggle density cap                           |
| `H`          | Toggle server-RGB / height colour           |
| `C`          | Center on drone                              |
| `F`          | Fit all                                      |
