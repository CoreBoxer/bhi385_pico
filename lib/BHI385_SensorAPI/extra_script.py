# extra_script.py
Import("env")
import os

lib_root = Dir('.').abspath

EXCLUDE_DIRS = {
    ".git", ".github",
    "submodules",
    "load_firmware", "euler", "quaternion", "activity",
    "step_counter", "klio", "multi_tap", "head_gesture_detection",
    "common",
}

extra_include_paths = []
for root, dirs, files in os.walk(lib_root):
    dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
    if any(f.endswith(".h") for f in files):
        extra_include_paths.append(root)

# compat目录放最前面，保证 #include "coines.h" / "verbose.h"
# 优先命中我们自己写的桩文件，而不是源码树里别处可能存在的同名文件
compat_dir = os.path.join(lib_root, "compat")
env.Prepend(CPPPATH=[compat_dir])
env.Append(CPPPATH=extra_include_paths)

print("[BHI385_SensorAPI] 自动发现的头文件搜索路径：")
for p in [compat_dir] + extra_include_paths:
    print("    -", p)