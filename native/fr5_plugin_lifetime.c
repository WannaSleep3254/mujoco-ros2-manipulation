/* Keep MoveIt plugin code mapped until its ROS callbacks have been destroyed.
 * Scoped to FR5 child processes via LD_PRELOAD; other dlopen calls are unchanged.
 * The operating system releases these mappings when each process exits.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <pthread.h>
#include <string.h>

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
        if (strncmp(basename, "libmoveit_", 10) == 0) {
            flags |= RTLD_NODELETE;
        }
    }
    return original_dlopen(filename, flags);
}
