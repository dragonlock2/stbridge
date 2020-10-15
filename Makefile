all:
	python3 setup.py build_ext --inplace
	@echo "Make sure you have the .dylib installed before running!"

clean:
	@echo "Note: doesn't remove libSTLinkUSBDriver.dylib"
	rm -rf build
	rm stbridge.cpython-38-darwin.so

install_dep:
	cp libSTLinkUSBDriver.dylib /usr/local/lib

remove_dep:
	rm /usr/local/lib/libSTLinkUSBDriver.dylib