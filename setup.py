import os
import subprocess
import sys
import multiprocessing

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

# adapted from https://github.com/pybind/cmake_example/blob/master/setup.py

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)

class CMakeBuild(build_ext):
    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        debug = int(os.environ.get("DEBUG", 0)) if self.debug is None else self.debug
        cfg = "Debug" if debug else "Release"

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={cfg}",
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{cfg.upper()}={extdir}",
        ]
        if "CMAKE_TOOLCHAIN_FILE" in os.environ:
            cmake_args.append(f"-DCMAKE_TOOLCHAIN_FILE={os.environ['CMAKE_TOOLCHAIN_FILE']}")

        build_args = ["--config", cfg]

        build_temp = os.path.join(self.build_temp, ext.name)
        if not os.path.exists(build_temp):
            os.makedirs(build_temp)

        subprocess.check_call(["cmake", ext.sourcedir] + cmake_args, cwd=build_temp)
        subprocess.check_call(["cmake", "--build", ".", f"-j{multiprocessing.cpu_count()}"] + build_args, cwd=build_temp)

setup(
    name="stbridge",
    version="0.0.1",
    ext_modules=[CMakeExtension("stbridge")],
    cmdclass={"build_ext": CMakeBuild},
)
