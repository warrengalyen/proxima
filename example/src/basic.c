/* Includes ============================================================================= */

#include "proxima.h"
#include "raylib.h"

#define PROXIMA_RAYLIB_IMPLEMENTATION
#include "proxima-raylib.h"

#ifdef PLATFORM_WEB
    #include <emscripten/emscripten.h>
#endif

/* Macros =============================================================================== */

#define TARGET_FPS       60

#define SCREEN_WIDTH     1280
#define SCREEN_HEIGHT    800

#define WORLD_CELL_SIZE  4.0f

/* Constants ============================================================================ */

static const float DELTA_TIME = 1.0f / TARGET_FPS;

/* Private Variables ==================================================================== */

static prWorld *world;

static Rectangle bounds = { .width = SCREEN_WIDTH, .height = SCREEN_HEIGHT };

/* Private Function Prototypes ========================================================== */

static void InitExample(void);
static void UpdateExample(void);
static void DeinitExample(void);

/* Public Functions =================================================================== */

int main(void) {
    SetConfigFlags(FLAG_MSAA_4X_HINT);

    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "mechanika-design/proxima | basic.c");

    InitExample();

#ifdef PLATFORM_WEB
    emscripten_set_main_loop(UpdateExample, 0, 1);
#else
    SetTargetFPS(TARGET_FPS);

    while (!WindowShouldClose()) 
        UpdateExample();
#endif

    DeinitExample();

    CloseWindow();

    return 0;
}

/* Private Functions ==================================================================== */

static void InitExample(void) {
    world = prCreateWorld(PR_API_STRUCT_ZERO(prVector2), WORLD_CELL_SIZE);
}

static void UpdateExample(void) {
    prUpdateWorld(world, DELTA_TIME);

    {
        BeginDrawing();
            
        ClearBackground(PR_DRAW_COLOR_MATTEBLACK);

        prDrawGrid(bounds, WORLD_CELL_SIZE, 0.25f, ColorAlpha(DARKGRAY, 0.75f));

        DrawFPS(8, 8);

        EndDrawing();
    }
}

static void DeinitExample(void) {
    prReleaseWorld(world);
}