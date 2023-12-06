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

#define TARGET_FPS       60

#define SCREEN_WIDTH     1280
#define SCREEN_HEIGHT    800

#define MAX_ENEMY_COUNT  256

// clang-format on

/* Typedefs ============================================================================= */

typedef enum _EntityType {
    ENTITY_PLAYER,
    ENTITY_BULLET,
    ENTITY_ENEMY,
    ENTITY_COUNT_
} EntityType;

typedef struct _EntityData {
    EntityType type;
    float attackSpeed;
    float movementSpeed;
    float counter;
} EntityData;

/* Constants ============================================================================ */

static const float CELL_SIZE = 4.0f, DELTA_TIME = 1.0f / TARGET_FPS;

static const prMaterial MATERIAL_BULLET = { .density = 2.25f,
                                            .friction = 0.85f,
                                            .restitution = 0.0f };

static const prMaterial MATERIAL_ENEMY = { .density = 0.85f,
                                           .friction = 0.5f,
                                           .restitution = 0.0f };

static const prMaterial MATERIAL_PLAYER = { .density = 1.25f,
                                            .friction = 0.75f,
                                            .restitution = 0.0f };

static const Rectangle SCREEN_BOUNDS = { .width = SCREEN_WIDTH,
                                         .height = SCREEN_HEIGHT };

/* Private Variables ==================================================================== */

static EntityData entityData[ENTITY_COUNT_] = {
    { .type = ENTITY_PLAYER, .attackSpeed = 0.1f },
    { .type = ENTITY_BULLET, .movementSpeed = 64.0f },
    { .type = ENTITY_ENEMY, .movementSpeed = 4.0f }
};

static prVertices bulletVertices, playerVertices;

static prWorld *world;

static prBody *player;

static int enemyCount;

/* Private Function Prototypes ========================================================== */

static void InitExample(void);
static void UpdateExample(void);
static void DeinitExample(void);

static void DrawCursor(void);

static void UpdateBullets(void);

static void OnPreStep(prBodyPair key, prCollision *value);

/* Public Functions ===================================================================== */

int main(void) {
    SetConfigFlags(FLAG_MSAA_4X_HINT);

    InitWindow(SCREEN_WIDTH,
               SCREEN_HEIGHT,
               "mechanika-design/proxima | shoot.c");

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

    world = prCreateWorld(prVector2ScalarMultiply(PR_WORLD_DEFAULT_GRAVITY,
                                                  0.0f),
                          CELL_SIZE);

    prSetWorldCollisionHandler(world,
                               (prCollisionHandler) {
                                   .preStep = OnPreStep,
                               });

    bulletVertices = (prVertices) {
        .data = { prVector2PixelsToUnits((prVector2) { .x = 0.0f, .y = -7.2f }),
                  prVector2PixelsToUnits((prVector2) { .x = -2.8f, .y = 7.2f }),
                  prVector2PixelsToUnits(
                      (prVector2) { .x = 2.8f, .y = 7.2f }) },
        .count = 3
    };

    playerVertices = (prVertices) {
        .data = { prVector2PixelsToUnits(
                      (prVector2) { .x = 0.0f, .y = -16.0f }),
                  prVector2PixelsToUnits(
                      (prVector2) { .x = -14.0f, .y = 16.0f }),
                  prVector2PixelsToUnits(
                      (prVector2) { .x = 14.0f, .y = 16.0f }) },
        .count = 3
    };

    player = prCreateBodyFromShape(
        PR_BODY_KINEMATIC,
        prVector2PixelsToUnits((prVector2) { .x = 0.5f * SCREEN_WIDTH,
                                             .y = 0.5f * SCREEN_HEIGHT }),
        prCreatePolygon(PR_API_STRUCT_ZERO(prMaterial), &playerVertices));

    prSetBodyUserData(player, (void *) &entityData[ENTITY_PLAYER]);

    prAddBodyToWorld(world, player);
}

static void UpdateExample(void) {
    for (int i = 0; i < MAX_ENEMY_COUNT - enemyCount; i++) {
        prVector2 position = { .x = 0.5f * SCREEN_WIDTH,
                               .y = 0.5f * SCREEN_HEIGHT };

        while (position.x >= 0.35f * SCREEN_WIDTH
               && position.x <= 0.65f * SCREEN_WIDTH)
            position.x = GetRandomValue(-2.5f * SCREEN_WIDTH,
                                        2.5f * SCREEN_WIDTH);

        while (position.y >= 0.35f * SCREEN_HEIGHT
               && position.y <= 0.65f * SCREEN_HEIGHT)
            position.y = GetRandomValue(-2.5f * SCREEN_HEIGHT,
                                        2.5f * SCREEN_HEIGHT);

        prBody *enemy = prCreateBodyFromShape(
            PR_BODY_DYNAMIC,
            prVector2PixelsToUnits(position),
            prCreateCircle(MATERIAL_ENEMY, 0.5f * GetRandomValue(2, 4)));

        prSetBodyUserData(enemy, (void *) &entityData[ENTITY_ENEMY]);

        prAddBodyToWorld(world, enemy);

        enemyCount++;
    }

    const int bodyCount = prGetBodyCountForWorld(world);

    for (int i = 0; i < bodyCount; i++) {
        prBody *body = prGetBodyFromWorld(world, i);

        const EntityData *bodyData = prGetBodyUserData(body);

        if (bodyData == NULL || bodyData->type != ENTITY_BULLET) continue;

        prAABB aabb = prGetBodyAABB(body);

        if (CheckCollisionRecs(
                (Rectangle) { .x = prUnitsToPixels(aabb.x),
                              .y = prUnitsToPixels(aabb.y),
                              .width = prUnitsToPixels(aabb.width),
                              .height = prUnitsToPixels(aabb.height) },
                SCREEN_BOUNDS))
            continue;

        prRemoveBodyFromWorld(world, body);
    }

    const Vector2 mousePosition = GetMousePosition();

    prSetBodyAngle(
        player,
        prVector2Angle((prVector2) { .y = -1.0f },
                       prVector2Subtract(prVector2PixelsToUnits((prVector2) {
                                             .x = mousePosition.x,
                                             .y = mousePosition.y }),
                                         prGetBodyPosition(player))));

    UpdateBullets();

    prUpdateWorld(world, DELTA_TIME);

    {
        BeginDrawing();

        ClearBackground(PR_DRAW_COLOR_MATTEBLACK);

        prDrawGrid(SCREEN_BOUNDS,
                   CELL_SIZE,
                   0.25f,
                   ColorAlpha(DARKGRAY, 0.75f));

        const int bodyCount = prGetBodyCountForWorld(world);

        for (int i = 0; i < bodyCount; i++) {
            prBody *body = prGetBodyFromWorld(world, i);

            const EntityData *bodyData = prGetBodyUserData(body);

            if (bodyData == NULL) continue;

            const prVector2 deltaPosition = prVector2Normalize(
                prVector2Subtract(prGetBodyPosition(player),
                                  prGetBodyPosition(body)));

            switch (bodyData->type) {
                case ENTITY_BULLET:
                    prDrawBodyLines(body, 2.0f, ColorAlpha(YELLOW, 0.85f));

                    break;

                case ENTITY_ENEMY:
                    prSetBodyVelocity(
                        body,
                        prVector2ScalarMultiply(deltaPosition,
                                                bodyData->movementSpeed));

                    prDrawBodyLines(body, 2.0f, ColorAlpha(RED, 0.65f));

                    break;

                case ENTITY_PLAYER:
                    prDrawBodyLines(body, 2.0f, ColorAlpha(GREEN, 0.95f));

                    break;
            }
        }

        DrawCursor();

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

static void UpdateBullets(void) {
    EntityData *playerData = &entityData[ENTITY_PLAYER];

    if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)
        && playerData->counter >= playerData->attackSpeed) {
        prBody *bullet = prCreateBodyFromShape(
            PR_BODY_DYNAMIC,
            prVector2Transform(playerVertices.data[0],
                               prGetBodyTransform(player)),
            prCreatePolygon(MATERIAL_BULLET, &bulletVertices));

        const Vector2 mousePosition = GetMousePosition();

        const prVector2 direction =
            prVector2Subtract(prVector2PixelsToUnits((prVector2) {
                                  .x = mousePosition.x, .y = mousePosition.y }),
                              prGetBodyPosition(player));

        prSetBodyAngle(bullet,
                       prVector2Angle((prVector2) { .y = -1.0f }, direction));
        prSetBodyUserData(bullet, (void *) &entityData[ENTITY_BULLET]);

        prSetBodyVelocity(
            bullet,
            prVector2ScalarMultiply(prVector2Normalize(direction),
                                    entityData[ENTITY_BULLET].movementSpeed));

        prAddBodyToWorld(world, bullet);

        playerData->counter = 0.0f;
    }

    playerData->counter += GetFrameTime();
}

static void OnPreStep(prBodyPair key, prCollision *value) {
    const EntityData *bodyData1 = prGetBodyUserData(key.first);
    const EntityData *bodyData2 = prGetBodyUserData(key.second);

    if ((bodyData1->type == ENTITY_BULLET && bodyData2->type == ENTITY_ENEMY)
        || (bodyData1->type == ENTITY_ENEMY
            && bodyData2->type == ENTITY_BULLET)) {
        prBody *bullet = NULL, *enemy = NULL;

        if (bodyData1->type == ENTITY_BULLET)
            bullet = key.first, enemy = key.second;
        else
            bullet = key.second, enemy = key.first;

        value->count = 0;

        prRemoveBodyFromWorld(world, bullet);
        prRemoveBodyFromWorld(world, enemy);
    }
}