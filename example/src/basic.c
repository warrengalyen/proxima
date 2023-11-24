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

#define SCREEN_WIDTH     800
#define SCREEN_HEIGHT    600

#define WORLD_CELL_SIZE  4.0f

/* Constants ============================================================================ */

static const float DELTA_TIME = 1.0f / TARGET_FPS;

/* Private Variables ==================================================================== */

static prWorld *world;

static prBody *box, *ground;

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
    world = prCreateWorld(PR_WORLD_DEFAULT_GRAVITY, WORLD_CELL_SIZE);

    ground = prCreateBodyFromShape(
        PR_BODY_STATIC,
        prVector2PixelsToUnits(
            (prVector2) {
                .x = 0.5f * SCREEN_WIDTH,
                .y = 0.85f * SCREEN_HEIGHT
            }
        ),
        prCreateRectangle(
            (prMaterial) {
                .density = 1.25f,
                .friction = 0.5f
            },
            prPixelsToUnits(0.75f * SCREEN_WIDTH),
            prPixelsToUnits(0.1f * SCREEN_HEIGHT)
        )
    );

    prAddBodyToWorld(world, ground);

     box = prCreateBodyFromShape(
        PR_BODY_DYNAMIC,
        prVector2PixelsToUnits(
           (prVector2) { 
                .x = 0.5f * SCREEN_WIDTH,
                .y = 0.5f * SCREEN_HEIGHT
            }
        ),
        prCreateRectangle(
            (prMaterial) {
                .density = 1.0f,
                .friction = 0.35f
            },
            prPixelsToUnits(45.0f),
            prPixelsToUnits(45.0f)
        )
    );

    prSetBodyAngle(box, DEG2RAD * 25.0f);

    prAddBodyToWorld(world, box);
}

static void UpdateExample(void) {
    prUpdateWorld(world, DELTA_TIME);

    {
        BeginDrawing();
            
        ClearBackground(PR_DRAW_COLOR_MATTEBLACK);

        prDrawGrid(bounds, WORLD_CELL_SIZE, 0.25f, ColorAlpha(DARKGRAY, 0.75f));

        prDrawBodyLines(ground, 1.0f, GRAY);

        prDrawBodyLines(box, 1.0f, ColorAlpha(RED, 0.85f));
        // prDrawBodyAABB(box, 1.0f, ColorAlpha(GREEN, 0.25f));

        DrawFPS(8, 8);

        EndDrawing();
    }
}

static void DeinitExample(void) {
    prReleaseShape(prGetBodyShape(ground));
    prReleaseShape(prGetBodyShape(box));

    prReleaseWorld(world);
}