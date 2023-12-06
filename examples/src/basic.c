/*
    Copyright (c) 2023 Warren Galyen <wgalyen@mechanikadesign.com>

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

/* Includes ============================================================================= */

#include "proxima.h"
#include "raylib.h"

#define PROXIMA_RAYLIB_IMPLEMENTATION
#include "proxima-raylib.h"

#ifdef PLATFORM_WEB
    #include <emscripten/emscripten.h>
#endif

/* Macros =============================================================================== */

#define TARGET_FPS 60

#define SCREEN_WIDTH  800
#define SCREEN_HEIGHT 600

/* Constants ============================================================================ */

static const float CELL_SIZE = 4.0f, DELTA_TIME = 1.0f / TARGET_FPS;

static const Rectangle SCREEN_BOUNDS = { .width = SCREEN_WIDTH,
                                         .height = SCREEN_HEIGHT };

/* Private Variables ==================================================================== */

static prWorld *world;

static prBody *box, *ground;

/* Private Function Prototypes ========================================================== */

static void InitExample(void);
static void UpdateExample(void);
static void DeinitExample(void);

/* Public Functions ===================================================================== */

int main(void) {
    SetConfigFlags(FLAG_MSAA_4X_HINT);

    InitWindow(SCREEN_WIDTH,
               SCREEN_HEIGHT,
               "mechanika-design/proxima | basic.c");

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
    world = prCreateWorld(prVector2ScalarMultiply(PR_WORLD_DEFAULT_GRAVITY,
                                                  4.0f),
                          CELL_SIZE);

    ground = prCreateBodyFromShape(
        PR_BODY_STATIC,
        prVector2PixelsToUnits((prVector2) { .x = 0.5f * SCREEN_WIDTH,
                                             .y = 0.85f * SCREEN_HEIGHT }),
        prCreateRectangle((prMaterial) { .density = 1.25f, .friction = 0.5f },
                          prPixelsToUnits(0.75f * SCREEN_WIDTH),
                          prPixelsToUnits(0.1f * SCREEN_HEIGHT)));

    prAddBodyToWorld(world, ground);

    box = prCreateBodyFromShape(
        PR_BODY_DYNAMIC,
        prVector2PixelsToUnits((prVector2) { .x = 0.5f * SCREEN_WIDTH,
                                             .y = 0.35f * SCREEN_HEIGHT }),
        prCreateRectangle((prMaterial) { .density = 1.0f, .friction = 0.35f },
                          prPixelsToUnits(45.0f),
                          prPixelsToUnits(45.0f)));

    // prSetBodyAngle(box, DEG2RAD * 25.0f);

    prAddBodyToWorld(world, box);
}

static void UpdateExample(void) {
    prUpdateWorld(world, DELTA_TIME);

    {
        BeginDrawing();

        ClearBackground(PR_DRAW_COLOR_MATTEBLACK);

        prDrawGrid(SCREEN_BOUNDS,
                   CELL_SIZE,
                   0.25f,
                   ColorAlpha(DARKGRAY, 0.75f));

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