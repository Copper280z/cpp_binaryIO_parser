Serial port can be specified with the -p flag at runtime, default is /dev/ttyACM0

Registers are also hard coded as:
```cpp
uint8_t telemetry_registers[] = {REG_TARGET, REG_ANGLE, REG_VELOCITY, REG_CURRENT_Q, REG_CURRENT_D};
```

To build, from the project root directory:
```bash
meson build
cd build
ninja
```

Requires meson/ninja

Ubuntu:
```bash
sudo apt install meson ninja-build
```
MacOS with Homebrew:
```bash
brew install meson ninja
```
