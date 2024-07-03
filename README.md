Serial port is hard coded as /dev/ttyACM0 at top of serial_parser.cpp for the moment

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
