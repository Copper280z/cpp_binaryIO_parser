Serial port is hard coded as /dev/ttyACM0 at top of serial_parser.cpp for the moment

Registers are also hard coded as:
```cpp
uint8_t telemetry_registers[] = { REG_TARGET, REG_ANGLE, REG_VELOCITY, REG_CURRENT_Q, REG_CURRENT_D};
```

To build:
```bash
meson build
cd build
ninja
```

Requires meson/ninja
Ubuntu
```bash
sudo apt install meson ninja-build
```
