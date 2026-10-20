/* Keep plugin code mapped until executors release callback/weak-pointer control blocks.
 * MoveIt is pinned by default; control plugins require the explicit child-process flag.
 * The operating system releases these mappings when each process exits.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>

static void *(*original_dlopen)(const char *, int);
static pthread_once_t resolution = PTHREAD_ONCE_INIT;

static void resolve_original(void) {
    original_dlopen = (void *(*)(const char *, int))dlsym(RTLD_NEXT, "dlopen");
}

void *dlopen(const char *filename, int flags) {
    pthread_once(&resolution, resolve_original);
    if (filename) {
        const char *basename = strrchr(filename, '/');
        basename = basename ? basename + 1 : filename;
        const char *control = getenv("MANIPULATION_PIN_CONTROL_PLUGINS");
        const int pin_control = control && strcmp(control, "1") == 0 &&
            (strncmp(basename, "libmujoco_ros2_control", sizeof("libmujoco_ros2_control") - 1) == 0 ||
             strcmp(basename, "libjoint_trajectory_controller.so") == 0 ||
             strcmp(basename, "libjoint_state_broadcaster.so") == 0);
        if (strncmp(basename, "libmoveit_", 10) == 0 || pin_control) {
            flags |= RTLD_NODELETE;
        }
    }
    return original_dlopen(filename, flags);
}
