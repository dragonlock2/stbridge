# stbridge

Python wrapper for STLINK-V3-BRIDGE. Tested on macOS, Windows, and Linux. Uses `libusb` for straightforward cross-platform support.

This project is semi-abandoned due to being tied to proprietary hardware. See https://github.com/dragonlock2/JABI for a true cross-hardware solution.

## Supported features
- SPI controller
- I2C controller
- GPIO
- CAN

## Setup

First let's install a few dependencies.

- macOS
    - `brew install git cmake libusb`
- Linux
    - `apt install git cmake libusb-1.0-0-dev`
- Windows
    - Install [Visual Studio C++](https://visualstudio.microsoft.com/vs/features/cplusplus/) for its C++ compiler. It's also an IDE.
    - Use [winget](https://docs.microsoft.com/en-us/windows/package-manager/winget/) to install [git](https://winget.run/pkg/Git/Git) and [CMake](https://winget.run/pkg/Kitware/CMake).
    - Use [vcpkg](https://github.com/microsoft/vcpkg) to install `libusb` under the right triplet (e.g. `x64-windows`).
    - Use [Zadig](https://zadig.akeo.ie) to install the [WinUSB](https://github.com/libusb/libusb/wiki/Windows#driver-installation) driver on any `STLINK-V3` devices.

Make sure to clone with submodules.

	git clone --recursive https://github.com/dragonlock2/stbridge.git

Install locally using `pip`!

	pip install ./stbridge

## Use

Optionally run the tests. There's a 5 second timeout for commands in case they're unsuccessful like if you try CAN without other nodes. If you don't want to use `sudo` on Linux, you can add [`50-stlinkv3.rules`](50-stlinkv3.rules) to `/etc/udev/rules.d`.

	python3 tests/all.py

Check out [`stbridge.cpp`](stbridge.cpp) and [`stbridge.h`](stbridge.h) as well as [tests/](tests/) for function syntax.

## Known Issues
- Not reading CAN messages fast enough can crash USB comms on the next CAN operation, even init. Should be detected as an overrun but isn't. Also looks like more messages than can be stored in the STLINK's FIFO's can be read back. I have a feeling what's happening is messages are stored on a separate FIFO on the STLINK and once it overruns, it crashes. This appears to be an ST issue.
- Received remote CAN frames are misidentified as data frames. Garbage data is returned in remote frames due to ST's driver. This appears to be an ST issue.
- There's some memory leaks in the [backend driver](https://github.com/dragonlock2/STLINK-V3-BRIDGE_libusb/) which has to do with device enumeration. Should only affect device construction/destruction and enumeration. I'm too lazy to fix this.
- Setting CAN filters for specific extended IDs throws a parameter error. Letting everything through still works. I'm too lazy to fix this.

## License
This software is under ST's [Ultimate Liberty License](https://www.st.com/content/ccc/resource/legal/legal_agreement/license_agreement/group0/87/0c/3d/ad/0a/ba/44/26/DM00216740/files/DM00216740.pdf/jcr:content/translations/en.DM00216740.pdf).
