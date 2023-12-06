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

// clang-format off

#define TARGET_FPS        60

#define SCREEN_WIDTH      1280
#define SCREEN_HEIGHT     800

#define MAX_OBJECT_COUNT  128

// clang-format on

/* Constants ============================================================================ */

static const float CELL_SIZE = 4.0f, DELTA_TIME = 1.0f / TARGET_FPS;

static const Rectangle SCREEN_BOUNDS = { .width = SCREEN_WIDTH,
                                         .height = SCREEN_HEIGHT };

/* Private Variables ==================================================================== */

static prWorld *world;

static prBody *player;

/* Private Function Prototypes ========================================================== */

static void InitExample(void);
static void UpdateExample(void);
static void DeinitExample(void);

static void DrawCursor(void);

static void OnRaycastQuery(prRaycastHit raycastHit);

/* Public Functions ===================================================================== */

int main(void) {
    SetConfigFlags(FLAG_MSAA_4X_HINT);

    InitWindow(SCREEN_WIDTH,
               SCREEN_HEIGHT,
               "mechanika-design/proxima | raycast.c");

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
    HideCursor();

#ifdef PLATFORM_WEB
    // TODO: https://github.com/emscripten-core/emscripten/issues/5446
    emscripten_hide_mouse();
#endif

    SetMousePosition(0.5f * SCREEN_WIDTH, 0.5f * SCREEN_HEIGHT);

    world = prCreateWorld(PR_WORLD_DEFAULT_GRAVITY, CELL_SIZE);

    player = prCreateBodyFromShape(
        PR_BODY_KINEMATIC,
        prVector2PixelsToUnits((prVector2) { .x = 0.5f * SCREEN_WIDTH,
                                             .y = 0.5f * SCREEN_HEIGHT }),
        prCreatePolygon(PR_API_STRUCT_ZERO(prMaterial),
                        &(const prVertices) {
                            .data = { prVector2PixelsToUnits((prVector2) {
                                          .x = 0.0f, .y = -16.0f }),
                                      prVector2PixelsToUnits((prVector2) {
                                          .x = -14.0f, .y = 16.0f }),
                                      prVector2PixelsToUnits((prVector2) {
                                          .x = 14.0f, .y = 16.0f }) },
                            .count = 3 }));

    prAddBodyToWorld(world, player);

    for (int i = 0; i < MAX_OBJECT_COUNT; i++) {
        prVector2 position = {
            .x = GetRandomValue(0, 1)
                     ? GetRandomValue(0, 0.48f * SCREEN_WIDTH)
                     : GetRandomValue(0.52f * SCREEN_WIDTH, SCREEN_WIDTH),
            .y = GetRandomValue(0, 1)
                     ? GetRandomValue(0, 0.48f * SCREEN_HEIGHT)
                     : GetRandomValue(0.52f * SCREEN_HEIGHT, SCREEN_HEIGHT)
        };

        prBody *object =
            prCreateBodyFromShape(PR_BODY_STATIC,
                                  prVector2PixelsToUnits(position),
                                  prCreateCircle(PR_API_STRUCT_ZERO(prMaterial),
                                                 0.5f * GetRandomValue(2, 4)));

        prAddBodyToWorld(world, object);
    }
}

static void UpdateExample(void) {
    const Vector2 mousePosition = GetMousePosition();

    prSetBodyAngle(
        player,
        prVector2Angle((prVector2) { .y = -1.0f },
                       prVector2Subtract(prVector2PixelsToUnits((prVector2) {
                                             .x = mousePosition.x,
                                             .y = mousePosition.y }),
                                         prGetBodyPosition(player))));

    prVector2 rayOrigin = prVector2Transform(prGetPolygonVertex(prGetBodyShape(
                                                                    player),
                                                                2),
                                             prGetBodyTransform(player));

    prVector2 rayDirection = prVector2Subtract(prVector2PixelsToUnits(
                                                   (prVector2) {
                                                       .x = mousePosition.x,
                                                       .y = mousePosition.y }),
                                               rayOrigin);

    prRay ray = { .origin = prVector2Add(
                      rayOrigin,
                      prVector2ScalarMultiply(prVector2Normalize(rayDirection),
                                              0.25f)),
                  .direction = rayDirection,
                  .maxDistance = prVector2Magnitude(rayDirection) };

    prUpdateWorld(world, DELTA_TIME);

    {
        BeginDrawing();

        ClearBackground(PR_DRAW_COLOR_MATTEBLACK);

        prDrawGrid(SCREEN_BOUNDS,
                   CELL_SIZE,
                   0.25f,
                   ColorAlpha(DARKGRAY, 0.75f));

        const int bodyCount = prGetBodyCountForWorld(world);

        for (int i = 1; i < bodyCount; i++) {
            prBody *object = prGetBodyFromWorld(world, i);

            prDrawBodyLines(object, 2.0f, ColorAlpha(LIGHTGRAY, 0.95f));
        }

        prComputeRaycastForWorld(world, ray, OnRaycastQuery);

        prDrawBodyLines(player, 2.0f, ColorAlpha(GREEN, 0.85f));

        prDrawArrow(rayOrigin,
                    prVector2Add(rayOrigin, rayDirection),
                    1.0f,
                    ColorAlpha(GREEN, 0.85f));

        DrawCursor();

        DrawFPS(8, 8);

        EndDrawing();
    }
}

static void DeinitExample(void) {
    prReleaseWorld(world);
}

static void DrawCursor(void) {
    const Vector2 mousePosition = GetMousePosition();

    DrawLineEx((Vector2) { .x = mousePosition.x - 8.0f, .y = mousePosition.y },
               (Vector2) { .x = mousePosition.x + 8.0f, .y = mousePosition.y },
               2.0f,
               WHITE);

    DrawLineEx((Vector2) { .x = mousePosition.x, .y = mousePosition.y - 8.0f },
               (Vector2) { .x = mousePosition.x, .y = mousePosition.y + 8.0f },
               2.0f,
               WHITE);
}

static void OnRaycastQuery(prRaycastHit raycastHit) {
    prDrawBodyAABB(raycastHit.body, 1.0f, YELLOW);

    Vector2 center = { .x = prUnitsToPixels(raycastHit.point.x),
                       .y = prUnitsToPixels(raycastHit.point.y) };

    DrawRing(center, 6.0f, 8.0f, 0.0f, 360.0f, 16, YELLOW);
}