from distutils.core import setup
from distutils.extension import Extension

ext = Extension(
    'stbridge',
    sources=['stbridge.cpp', 'src/bridge/bridge.cpp', 'src/common/criticalsectionlock.cpp', 'src/common/stlink_device.cpp', 'src/common/stlink_interface.cpp', 'src/error/ErrLog.cpp'],
    libraries=['boost_python38'],
    extra_compile_args=['-std=c++17', '-Isrc/bridge', '-Isrc/common', '-Isrc/error'],
    extra_link_args=['-L.', '-lSTLinkUSBDriver', '-lfmt']
)

setup(
    name='stbridge',
    version='0.1',
    ext_modules=[ext])