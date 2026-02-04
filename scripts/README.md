# Build Scripts

## `build_trilateration.sh`

Compiles the trilateration C library for different platforms.

### Usage

**Native compilation (builds for current platform):**
```bash
./scripts/build_trilateration.sh
```

**Cross-compilation for Raspberry Pi (ARM64):**
```bash
./scripts/build_trilateration.sh arm64
```

### Requirements

**For native x86_64 compilation:**
- `gcc`
- Standard build tools

**For ARM64 cross-compilation (Ubuntu/Debian):**
```bash
sudo apt-get install gcc-aarch64-linux-gnu
```

### Output

Built libraries are placed in `lib/`:
- `lib/libtrilateration_x86_64.so` - x86_64 Linux
- `lib/libtrilateration_arm64.so` - ARM64 Linux (Raspberry Pi)
- `lib/libtrilateration.so` - Symlink to current arch

### Testing on Raspberry Pi

After cross-compiling, copy to RPi and verify:
```bash
scp lib/libtrilateration_arm64.so pi@raspberrypi:/tmp/
ssh pi@raspberrypi "file /tmp/libtrilateration_arm64.so"
# Should show: ELF 64-bit LSB shared object, ARM aarch64
```
