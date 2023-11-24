/* Includes ============================================================================= */

#if defined(_WIN32)
    #define NOGDI
    #define NOUSER
#endif

#define SOKOL_TIME_IMPL
#include "external/sokol_time.h"

#include "proxima.h"

/* Private Variables ==================================================================== */

static bool initialized = false;

/* Public Functions ===================================================================== */

/* Returns the current time of the monotonic clock, in seconds. */
double prGetCurrentTime(void) {
    if (!initialized) {
        initialized = true;

        stm_setup();
    }

    return stm_sec(stm_now());
}