/* Includes ============================================================================= */

#include "proxima.h"
#include "raylib.h"

#define PROXIMA_RAYLIB_IMPLEMENTATION
#include "proxima-raylib.h"

#ifdef PLATFORM_WEB
    #include <emscripten/emscripten.h>
#endif

/* Macros =============================================================================== */

#define TARGET_FPS             60

#define SCREEN_WIDTH           1280
#define SCREEN_HEIGHT          800

#define WORLD_CELL_SIZE        2.8f

#define LOGO_WIDTH_IN_PIECES   46
#define LOGO_HEIGHT_IN_PIECES  46

/* Typedefs ============================================================================= */

typedef struct _Piece {
    prBody *body;
    prVector2 offset;
} Piece;

/* Constants ============================================================================ */

static const float DELTA_TIME = 1.0f / TARGET_FPS;

/* Private Variables ==================================================================== */

static prWorld *world;
static prBody *ball;

static Rectangle bounds = { .width = SCREEN_WIDTH, .height = SCREEN_HEIGHT };

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

    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "mechanika-design/proxima | raylib.c");

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
    world = prCreateWorld(PR_API_STRUCT_ZERO(prVector2), WORLD_CELL_SIZE);

    raylibTexture = LoadTexture("../res/images/raylib.png");

    if (IsTextureReady(raylibTexture)) {
        pieceWidth = raylibTexture.width / LOGO_WIDTH_IN_PIECES;
        pieceHeight = raylibTexture.height / LOGO_HEIGHT_IN_PIECES;

        halfPieceWidth = 0.5f * pieceWidth, halfPieceHeight = 0.5f * pieceHeight;

        prShape *pieceShape = prCreateRectangle(
            (prMaterial) {
                .density = 1.75f,
                .friction = 0.5f,
                .restitution = 0.0f
            },
            prPixelsToUnits(pieceWidth),
            prPixelsToUnits(pieceHeight)
        );

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

            pieces[i].body = prCreateBodyFromShape(
                PR_BODY_DYNAMIC,
                prVector2PixelsToUnits(position),
                pieceShape
            );

            prAddBodyToWorld(world, pieces[i].body);
        }

        ball = prCreateBodyFromShape(
            PR_BODY_DYNAMIC,
            prVector2PixelsToUnits(
                (prVector2) {
                    .x = -SCREEN_WIDTH,
                    .y = 0.5f * SCREEN_HEIGHT
                }
            ),
            prCreateCircle(
                (prMaterial) {
                    .density = 1.85f,
                    .friction = 0.75f
                }, 
                prPixelsToUnits(20.0f)
            )
        );

        prApplyImpulseToBody(
            ball,
            PR_API_STRUCT_ZERO(prVector2),
            (prVector2) {
                .x = 2048.0f,
                .y = 0.0f
            }
        );

        prAddBodyToWorld(world, ball);
    }
}

static void UpdateExample(void) {
    prUpdateWorld(world, DELTA_TIME);

    {
        BeginDrawing();
            
        ClearBackground(PR_DRAW_COLOR_MATTEBLACK);

        prDrawGrid(bounds, WORLD_CELL_SIZE, 0.25f, ColorAlpha(DARKGRAY, 0.75f));

        for (int i = 0; i < LOGO_WIDTH_IN_PIECES * LOGO_HEIGHT_IN_PIECES; i++) {
            const prVector2 bodyPosition = prGetBodyPosition(pieces[i].body);

            DrawTexturePro(
                raylibTexture,
                (Rectangle) {
                    .x = pieces[i].offset.x,
                    .y = pieces[i].offset.y,
                    .width = pieceWidth,
                    .height = pieceHeight
                },
                (Rectangle) {
                    .x = prUnitsToPixels(bodyPosition.x), 
                    .y = prUnitsToPixels(bodyPosition.y),
                    .width = pieceWidth,
                    .height = pieceHeight
                },
                (Vector2) {
                    .x = halfPieceWidth,
                    .y = halfPieceHeight
                },
                RAD2DEG * prGetBodyAngle(pieces[i].body),
                WHITE
            );
        }

        prDrawBodyLines(ball, 1.0f, WHITE);

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