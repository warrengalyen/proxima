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

#define TARGET_FPS       60

#define SCREEN_WIDTH     1280
#define SCREEN_HEIGHT    800

#define MAX_WALL_COUNT   4

/* Constants ============================================================================ */

static const prMaterial MATERIAL_BRICK = { .density = 1.25f, .friction = 0.75f };
static const prMaterial MATERIAL_WALL = { .density = 1.5f, .friction = 0.85f };

static const float BRICK_WIDTH = 60.0f, BRICK_HEIGHT = 48.0f;
static const float CELL_SIZE = 4.0f, DELTA_TIME = 1.0f / TARGET_FPS;

/* Private Variables ==================================================================== */

static prWorld *world;

static prBody *cursor, *walls[MAX_WALL_COUNT];

static Rectangle bounds = { .width = SCREEN_WIDTH, .height = SCREEN_HEIGHT };

static RenderTexture2D brickTarget;

/* Private Function Prototypes ========================================================== */

static void InitExample(void);
static void UpdateExample(void);
static void DeinitExample(void);

/* Public Functions ==================================================================== */

int main(void) {
    SetConfigFlags(FLAG_MSAA_4X_HINT);

    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "mechanika-design/proxima | bricks.c");

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
    world = prCreateWorld(
        prVector2ScalarMultiply(PR_WORLD_DEFAULT_GRAVITY, 2.5f), 
        CELL_SIZE
    );

    walls[0] = prCreateBodyFromShape(
        PR_BODY_STATIC,
        prVector2PixelsToUnits(
            (prVector2) { 
                .x = -0.05f * SCREEN_WIDTH,
                .y = 0.5f * SCREEN_HEIGHT 
            }
        ),
        prCreateRectangle(
            MATERIAL_WALL,
            prPixelsToUnits(0.1f * SCREEN_WIDTH),
            prPixelsToUnits(1.1f * SCREEN_HEIGHT)
        )
    );

    walls[1] = prCreateBodyFromShape(
        PR_BODY_STATIC,
        prVector2PixelsToUnits(
            (prVector2) {
                .x = 0.5f * SCREEN_WIDTH, 
                .y = 1.05f * SCREEN_HEIGHT 
            }
        ),
        prCreateRectangle(
            MATERIAL_WALL,
            prPixelsToUnits(1.1f * SCREEN_WIDTH),
            prPixelsToUnits(0.1f * SCREEN_HEIGHT)
        )
    );

    walls[2] = prCreateBodyFromShape(
        PR_BODY_STATIC,
        prVector2PixelsToUnits(
            (prVector2) { 
                .x = 1.05f * SCREEN_WIDTH, 
                .y = 0.5f * SCREEN_HEIGHT 
            }
        ),
        prCreateRectangle(
            MATERIAL_WALL,
            prPixelsToUnits(0.1f * SCREEN_WIDTH),
            prPixelsToUnits(1.1f * SCREEN_HEIGHT)
        )
    );

    walls[3] = prCreateBodyFromShape(
        PR_BODY_STATIC,
        prVector2PixelsToUnits(
            (prVector2) {
                .x = 0.5f * SCREEN_WIDTH, 
                .y = -0.05f * SCREEN_HEIGHT 
            }
        ),
        prCreateRectangle(
            MATERIAL_WALL,
            prPixelsToUnits(1.1f * SCREEN_WIDTH),
            prPixelsToUnits(0.1f * SCREEN_HEIGHT)
        )
    );

    for (int i = 0; i < MAX_WALL_COUNT; i++)
        prAddBodyToWorld(world, walls[i]);

    cursor = prCreateBodyFromShape(
        PR_BODY_KINEMATIC,
        PR_API_STRUCT_ZERO(prVector2),
        prCreateRectangle(
            MATERIAL_BRICK, 
            prPixelsToUnits(BRICK_WIDTH), 
            prPixelsToUnits(BRICK_HEIGHT)
        )
    );

    prAddBodyToWorld(world, cursor);

    brickTarget = LoadRenderTexture(BRICK_WIDTH, BRICK_HEIGHT);

    {
        BeginTextureMode(brickTarget);

        ClearBackground(BLANK);

        DrawRectangleLinesEx(
            (Rectangle) { 
                .width = BRICK_WIDTH, 
                .height = BRICK_HEIGHT 
            }, 
            2.0f, 
            ColorAlpha(WHITE, 0.95f)
        );

        DrawCircleV(
            (Vector2) {
                .x = 0.5f * BRICK_WIDTH, 
                .y = 0.5f * BRICK_HEIGHT 
            }, 
            2.0f, 
            ColorAlpha(WHITE, 0.95f)
        );

        EndTextureMode();
    }

    SetTextureFilter(brickTarget.texture, TEXTURE_FILTER_BILINEAR);
}

static void UpdateExample(void) {
    const Vector2 mousePosition = GetMousePosition();

    prSetBodyPosition(
        cursor, 
        prVector2PixelsToUnits(
            (prVector2) {
                .x = mousePosition.x,
                .y = mousePosition.y
            }
        )
    );

    {
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            prBody *brick = prCreateBodyFromShape(
                PR_BODY_DYNAMIC,
                prVector2PixelsToUnits(
                    (prVector2) {
                        .x = mousePosition.x,
                        .y = mousePosition.y + (1.1f * BRICK_HEIGHT)
                    }
                ),
                prCreateRectangle(
                    MATERIAL_BRICK,
                    prPixelsToUnits(BRICK_WIDTH), 
                    prPixelsToUnits(BRICK_HEIGHT)
                )
            );

            prAddBodyToWorld(world, brick);
        }
    }

    prUpdateWorld(world, DELTA_TIME);

    {
        BeginDrawing();

        ClearBackground(PR_DRAW_COLOR_MATTEBLACK);

        prDrawGrid(bounds, CELL_SIZE, 0.25f, ColorAlpha(DARKGRAY, 0.75f));

        const int bodyCount = prGetBodyCountForWorld(world);

        for (int i = MAX_WALL_COUNT; i < bodyCount; i++) {
            prBody *body = prGetBodyFromWorld(world, i);

            const prVector2 bodyPosition = prGetBodyPosition(body);

            DrawTexturePro(
                brickTarget.texture,
                (Rectangle) { 
                    .width = BRICK_WIDTH, 
                    .height = BRICK_HEIGHT 
                },
                (Rectangle) {
                    .x = prUnitsToPixels(bodyPosition.x), 
                    .y = prUnitsToPixels(bodyPosition.y),
                    .width = BRICK_WIDTH,
                    .height = BRICK_HEIGHT
                },
                (Vector2) {
                    .x = 0.5f * BRICK_WIDTH, 
                    .y = 0.5f * BRICK_HEIGHT 
                },
                RAD2DEG * prGetBodyAngle(body),
                ColorAlpha(WHITE, (body == cursor) ? 0.5f : 1.0f)
            );
        }

        DrawFPS(8, 8);

        EndDrawing();
    }
}

static void DeinitExample(void) {
    prReleaseWorld(world);
}