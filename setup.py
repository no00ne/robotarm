from setuptools import setup, Extension
import sys, os, platform

# 允许通过环境变量覆盖默认 SDK 路径
LEAP_INCLUDE = os.environ.get("LEAP_INCLUDE")
LEAP_LIBDIR  = os.environ.get("LEAP_LIBDIR")

system = platform.system()
if not LEAP_INCLUDE or not LEAP_LIBDIR:
    if system == "Windows":
        # 默认 Windows SDK 路径（根据你安装路径调整）
        LEAP_INCLUDE = LEAP_INCLUDE or r"C:\Program Files\Ultraleap\LeapSDK\include"
        LEAP_LIBDIR  = LEAP_LIBDIR  or r"C:\Program Files\Ultraleap\LeapSDK\lib\x64"
    else:
        # 默认 Linux 路径（Ubuntu）
        LEAP_INCLUDE = LEAP_INCLUDE or "/usr/include"
        LEAP_LIBDIR  = LEAP_LIBDIR  or "/usr/lib/ultraleap-hand-tracking-service"

print("Building for platform:", system)
print("Leap include:", LEAP_INCLUDE)
print("Leap libdir :", LEAP_LIBDIR)

extra_link_args = []
libraries = ["LeapC"]
library_dirs = [LEAP_LIBDIR]
include_dirs = [LEAP_INCLUDE]

# Linux: set rpath so runtime finds libLeapC.so
if system != "Windows":
    extra_link_args.append(f"-Wl,-rpath,{LEAP_LIBDIR}")

# Windows notes: if using MSVC, library_dirs is enough; ensure LeapC.lib in LEAP_LIBDIR
# If using mingw you might need extra_link_args adjustments.

leap_ext = Extension(
    "leap_ext",
    sources=["src/leap_ext.c"],
    include_dirs=include_dirs,
    library_dirs=library_dirs,
    libraries=libraries,
    extra_compile_args=[],
    extra_link_args=extra_link_args,
)

setup(
    name="leap_ext",
    version="0.1.0",
    description="Ultraleap LeapC -> Python C extension (cross-platform)",
    ext_modules=[leap_ext],
)
