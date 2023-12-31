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

#define TARGET_FPS             60

#define SCREEN_WIDTH           1280
#define SCREEN_HEIGHT          800

#define LOGO_WIDTH_IN_PIECES   40
#define LOGO_HEIGHT_IN_PIECES  40

// clang-format on

/* Typedefs ============================================================================= */

typedef struct _Piece {
    prBody *body;
    prVector2 offset;
} Piece;

/* Constants ============================================================================ */

static const float CELL_SIZE = 2.8f, DELTA_TIME = 1.0f / TARGET_FPS;

static const Rectangle SCREEN_BOUNDS = { .width = SCREEN_WIDTH,
                                         .height = SCREEN_HEIGHT };

/* Private Variables ==================================================================== */

static prWorld *world;
static prBody *ball;

static Texture2D raylibTexture;

static Piece pieces[LOGO_WIDTH_IN_PIECES * LOGO_HEIGHT_IN_PIECES];

static float pieceWidth, pieceHeight;
static float halfPieceWidth, halfPieceHeight;

/* Private Function Prototypes ========================================================== */

static void InitExample(void);
static void UpdateExample(void);
static void DeinitExample(void);

/* Public Functions ===================================================================== */

int main(void) {
    SetConfigFlags(FLAG_MSAA_4X_HINT);

    InitWindow(SCREEN_WIDTH,
               SCREEN_HEIGHT,
               "mechanika-design/proxima | raylib.c");

    InitExample();

#ifdef PLATFORM_WEB
    emscripten_set_main_loop(UpdateExample, 0, 1);
#else
    // SetTargetFPS(TARGET_FPS);

    while (!WindowShouldClose())
        UpdateExample();
#endif

    DeinitExample();

    CloseWindow();

    return 0;
}

/* Private Functions ==================================================================== */

static void InitExample(void) {
    world = prCreateWorld(PR_API_STRUCT_ZERO(prVector2), CELL_SIZE);

    raylibTexture = LoadTexture("../res/images/raylib-40.png");

    if (raylibTexture.id > 0) {
        pieceWidth = raylibTexture.width / LOGO_WIDTH_IN_PIECES;
        pieceHeight = raylibTexture.height / LOGO_HEIGHT_IN_PIECES;

        halfPieceWidth = 0.5f * pieceWidth,
        halfPieceHeight = 0.5f * pieceHeight;

        prShape *pieceShape = prCreateRectangle(
            (prMaterial) {
                .density = 1.25f, .friction = 0.5f, .restitution = 0.0f },
            prPixelsToUnits(pieceWidth),
            prPixelsToUnits(pieceHeight));

        const prVector2 origin = {
            0.5f * (SCREEN_WIDTH - raylibTexture.width),
            0.5f * (SCREEN_HEIGHT - raylibTexture.height)
        };

        for (int i = 0; i < LOGO_WIDTH_IN_PIECES * LOGO_HEIGHT_IN_PIECES; i++) {
            pieces[i].offset.x = (i % LOGO_WIDTH_IN_PIECES) * pieceWidth;
            pieces[i].offset.y = (i / LOGO_WIDTH_IN_PIECES) * pieceHeight;

            const prVector2 position = {
                (origin.x + pieces[i].offset.x) + halfPieceWidth,
                (origin.y + pieces[i].offset.y) + halfPieceHeight
            };

            pieces[i].body = prCreateBodyFromShape(PR_BODY_DYNAMIC,
                                                   prVector2PixelsToUnits(
                                                       position),
                                                   pieceShape);

            prAddBodyToWorld(world, pieces[i].body);
        }

        ball = prCreateBodyFromShape(
            PR_BODY_DYNAMIC,
            prVector2PixelsToUnits(
                (prVector2) { .x = -SCREEN_WIDTH, .y = 0.5f * SCREEN_HEIGHT }),
            prCreateCircle((prMaterial) { .density = 1.85f, .friction = 0.75f },
                           prPixelsToUnits(20.0f)));

        prApplyImpulseToBody(ball,
                             PR_API_STRUCT_ZERO(prVector2),
                             (prVector2) { .x = 2048.0f, .y = 0.0f });

        prAddBodyToWorld(world, ball);
    }
}

static void UpdateExample(void) {
    for (int i = 0; i < LOGO_WIDTH_IN_PIECES * LOGO_HEIGHT_IN_PIECES; i++) {
        prAABB aabb = prGetBodyAABB(pieces[i].body);

        if (CheckCollisionRecs(
                (Rectangle) { .x = prUnitsToPixels(aabb.x),
                              .y = prUnitsToPixels(aabb.y),
                              .width = prUnitsToPixels(aabb.width),
                              .height = prUnitsToPixels(aabb.height) },
                SCREEN_BOUNDS))
            continue;

        if (prRemoveBodyFromWorld(world, pieces[i].body)) pieces[i].body = NULL;
    }

    prUpdateWorld(world, DELTA_TIME);

    {
        BeginDrawing();

        ClearBackground(PR_DRAW_COLOR_MATTEBLACK);

        prDrawGrid(SCREEN_BOUNDS,
                   CELL_SIZE,
                   0.25f,
                   ColorAlpha(DARKGRAY, 0.75f));

        for (int i = 0; i < LOGO_WIDTH_IN_PIECES * LOGO_HEIGHT_IN_PIECES; i++) {
            if (pieces[i].body == NULL) continue;

            const prVector2 bodyPosition = prGetBodyPosition(pieces[i].body);

            DrawTexturePro(raylibTexture,
                           (Rectangle) { .x = pieces[i].offset.x,
                                         .y = pieces[i].offset.y,
                                         .width = pieceWidth,
                                         .height = pieceHeight },
                           (Rectangle) { .x = prUnitsToPixels(bodyPosition.x),
                                         .y = prUnitsToPixels(bodyPosition.y),
                                         .width = pieceWidth,
                                         .height = pieceHeight },
                           (Vector2) { .x = halfPieceWidth,
                                       .y = halfPieceHeight },
                           RAD2DEG * prGetBodyAngle(pieces[i].body),
                           WHITE);
        }

        prDrawBodyLines(ball, 1.0f, WHITE);

        const Font font = GetFontDefault();

        DrawTextEx(font,
                   TextFormat("%d/%d bodies",
                              prGetBodyCountForWorld(world),
                              PR_WORLD_MAX_OBJECT_COUNT),
                   (Vector2) { .x = 8.0f, .y = 32.0f },
                   font.baseSize,
                   2.0f,
                   WHITE);

        DrawFPS(8, 8);

        EndDrawing();
    }
}

static void DeinitExample(void) {
    UnloadTexture(raylibTexture);

    prReleaseShape(prGetBodyShape(pieces[0].body));
    prReleaseShape(prGetBodyShape(ball));

    prReleaseWorld(world);
}