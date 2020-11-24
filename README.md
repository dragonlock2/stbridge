# stbridge

Python wrapper for STLINK-V3-BRIDGE. Tested on MacOS and Linux. Uses libusb for cross-platform support.

## Supported features
* SPI master
* I2C master
* GPIO
* CAN

## Known Issues
* Not reading CAN messages fast enough can crash USB comms on the next CAN operation, even init. Should be detected as an overrun but isn't. Also looks like more messages than can be stored in the STLINK's FIFO's can be read back. I have a feeling what's happening is messages are stored on a separate FIFO on the STLINK and once it overruns, it crashes. This appears to be an ST issue.
* Received remote CAN frames are misidentified as data frames. Garbage data is returned in remote frames due to ST's driver. This appears to be an ST issue.

## How to Build

Make sure to clone with submodules

	git clone --recursive https://github.com/dragonlock2/stbridge.git

Install dependencies

	# MacOS
	brew install boost-python3
	brew install fmt
	brew install libusb

	# Linux
	sudo apt install libboost-python-dev
	sudo apt install libfmt-dev
	sudo apt install libusb1.0-0-dev

Run make

	make

If it fails because it can't find boost_python, you might need to change setup.py to match the one on your system. Using Python's find\_library from ctypes.util may help to find it.

## How to Use

Optionally run the tests. There's a 5 second timeout for commands in case they're unsuccessful like if you try CAN without other nodes. If you don't want to use sudo on Linux, you can add 50-stlinkv3.rules to /etc/udev/rules.d.

	python3 tests/all.py

Make sure stbridge.\*.so is on PYTHONPATH or in your search path.

Check out stbridge.cpp and stbridge.h as well as tests/ for function syntax.

## Aside

This is just for future reference, but the original driver (libSTLinkUSBDriver.so or .dylib) from ST appears to use libusb under the hood. However, it's compiled for x86 which was the motivation for switching to pure libusb. I wouldn't be surprised if ST releases an ARM driver soon because of Apple Silicon. Also the driver handles much more than just the bridge functions.

## License

This software is under ST's [Ultimate Liberty License](https://www.st.com/content/ccc/resource/legal/legal_agreement/license_agreement/group0/87/0c/3d/ad/0a/ba/44/26/DM00216740/files/DM00216740.pdf/jcr:content/translations/en.DM00216740.pdf).