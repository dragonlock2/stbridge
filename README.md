# stbridge

Python wrapper for STLINK-V3-BRIDGE. Tested on macOS, Windows, and Linux. Uses `libusb` for straightforward cross-platform support. It's deployed on [PyPI](https://pypi.org/project/stbridge) so you can install it as follows.

```
pip install stbridge
```

Development on this project will only be for bug fixes and PRs due to being tied to proprietary hardware and ST's license. See [dragonlock2/JABI](https://github.com/dragonlock2/JABI) for a true cross-hardware solution.

## Supported features

- SPI controller
- I2C controller
- GPIO
- CAN

## Setup

To install from source, you'll need a few dependencies.

- macOS
    - `brew install autoconf automake libtool`
- Linux
    - `apt install autotools-dev autoconf libtool`
- Windows
    - Install [Visual Studio C++](https://visualstudio.microsoft.com/vs/features/cplusplus/) for its C++ compiler.
- Windows (MSYS2/MinGW) (experimental)
    - Install [MSYS2](https://www.msys2.org/) to install the following packages.
    - `pacman -S mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-autotools`
        - If you're not on `x86_64`, your exact package names may be different.

Then run the following.

```
git clone --recursive https://github.com/dragonlock2/stbridge.git
pip install ./stbridge
```

## Use

Check out [`stbridge.cpp`](src/stbridge.cpp) and [`stbridge.h`](src/stbridge.h) as well as [examples](examples) for function syntax. There's a 5 second timeout for commands in case they're unsuccessful (e.g. using CAN without other nodes).

```
python3 examples/all.py
```

In case no devices are available, you may need to do the following.
- On Linux, add [`50-stlinkv3.rules`](examples/50-stlinkv3.rules) to `/etc/udev/rules.d`.
- On Windows, manually install the WinUSB driver on any `STLINK-V3` devices in Device Manager.

## Known Issues

- Not reading CAN messages fast enough can crash USB comms on the next CAN operation, even init. Should be detected as an overrun but isn't. Also looks like more messages than can be stored in the STLINK's FIFO's can be read back. I have a feeling what's happening is messages are stored on a separate FIFO on the STLINK and once it overruns, it crashes. This appears to be an ST issue.
- Received remote CAN frames are misidentified as data frames. Garbage data is returned in remote frames due to ST's driver. This appears to be an ST issue.
- Setting CAN filters for specific extended IDs throws a parameter error. Letting everything through still works.
- There's some (fixed size) libusb memory leaks on Windows.

## License

This software is under ST's [Ultimate Liberty License](https://www.st.com/content/ccc/resource/legal/legal_agreement/license_agreement/group0/87/0c/3d/ad/0a/ba/44/26/DM00216740/files/DM00216740.pdf/jcr:content/translations/en.DM00216740.pdf).
