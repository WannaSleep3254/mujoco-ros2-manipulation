"""A real dlopen/dlclose probe: scope pinning and preserve process-exit destructors."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from mujoco_ros2_manipulation.paths import ROOT


class PluginLifetimeTests(unittest.TestCase):
    def test_only_selected_libraries_remain_mapped_and_destructors_still_run_at_exit(
        self,
    ):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            guard = root / "guard.so"
            subprocess.run(
                [
                    "gcc",
                    "-shared",
                    "-fPIC",
                    "-O2",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    str(ROOT / "native/moveit_plugin_lifetime.c"),
                    "-ldl",
                    "-pthread",
                    "-o",
                    str(guard),
                ],
                check=True,
            )
            source = root / "plugin.c"
            source.write_text(
                """#include <stdio.h>
#include <stdlib.h>
__attribute__((destructor)) static void cleanup(void) { FILE* f=fopen(getenv("MARKER"),"w"); if(f){fputs("destroyed",f);fclose(f);} }
int plugin_value(void){return 42;}
"""
            )
            probe = root / "probe.c"
            probe.write_text(
                """#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
int main(int argc,char** argv){if(argc!=2)return 2;void* p=dlopen(argv[1],RTLD_NOW|RTLD_LOCAL);if(!p)return 3;
dlclose(p);p=dlopen(argv[1],RTLD_NOW|RTLD_NOLOAD);puts(p?"resident":"unloaded");if(p)dlclose(p);return 0;}
"""
            )
            binary = root / "probe"
            subprocess.run(["gcc", str(probe), "-ldl", "-o", str(binary)], check=True)
            for filename, flag, pinned in [
                ("libmujoco_ros2_control.so", "1", True),
                ("libjoint_trajectory_controller.so", "1", True),
                ("libjoint_state_broadcaster.so", "1", True),
                ("libmujoco_ros2_control.so", "0", False),
                ("libunrelated.so", "1", False),
                ("libmujoco_ros2_controX.so", "1", False),
                ("libmoveit_probe.so", "0", True),
            ]:
                library = root / filename
                marker = root / "destroyed"
                marker.unlink(missing_ok=True)
                subprocess.run(
                    ["gcc", "-shared", "-fPIC", str(source), "-o", str(library)],
                    check=True,
                )
                result = subprocess.check_output(
                    [str(binary), str(library)],
                    env=dict(
                        os.environ,
                        LD_PRELOAD=str(guard),
                        MANIPULATION_PIN_CONTROL_PLUGINS=flag,
                        MARKER=str(marker),
                    ),
                    text=True,
                ).strip()
                with self.subTest(library=filename, flag=flag):
                    self.assertEqual(result, "resident" if pinned else "unloaded")
                    self.assertEqual(marker.read_text(), "destroyed")
