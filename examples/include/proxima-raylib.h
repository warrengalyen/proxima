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

#ifndef PROXIMA_RAYLIB_H
#define PROXIMA_RAYLIB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ============================================================================= */

#include "proxima.h"
#include "raylib.h"

/* Macros =============================================================================== */

// clang-format off

#define PR_DRAW_ARROW_HEAD_LENGTH    8.0f
#define PR_DRAW_CIRCLE_SEGMENT_COUNT 32

#define PR_DRAW_COLOR_MATTEBLACK \
    CLITERAL(Color) {            \
        23, 23, 23, 255          \
    }

// clang-format on

/* Public Function Prototypes =========================================================== */

/* 
    Draws an arrow that starts from `v1` to `v2` 
    with the given `thick`ness and `color`. 
*/
void prDrawArrow(prVector2 v1, prVector2 v2, float thick, Color color);

/* 
    Draws the AABB (Axis-Aligned Bounding Box) of `b` 
    with the given `thick`ness and `color`.
*/
void prDrawBodyAABB(const prBody *b, float thick, Color color);

/* Draws the outline of `b` with the given `thick`ness and `color`. */
void prDrawBodyLines(const prBody *b, float thick, Color color);

/* 
    Draws a grid within the `bounds`, 
    with the given `cellSize`, `thick`ness and `color`. 
*/
void prDrawGrid(Rectangle bounds, float cellSize, float thick, Color color);

#ifdef __cplusplus
}
#endif

#endif  // `PROXIMA_RAYLIB_H`

#ifdef PROXIMA_RAYLIB_IMPLEMENTATION

/* Public Functions ===================================================================== */

/* 
    Draws an arrow that starts from `v1` to `v2` 
    with the given `thick`ness and `color`. 
*/
void prDrawArrow(prVector2 v1, prVector2 v2, float thick, Color color) {
    if (thick <= 0.0f) return;

    v1 = prVector2UnitsToPixels(v1);
    v2 = prVector2UnitsToPixels(v2);

    prVector2 unitDiff = prVector2Normalize(prVector2Subtract(v1, v2));

    prVector2 leftNormal = prVector2LeftNormal(unitDiff);
    prVector2 rightNormal = prVector2RightNormal(unitDiff);

    prVector2 leftHead = prVector2Add(
        v2,
        prVector2ScalarMultiply(prVector2Normalize(
                                    prVector2Add(unitDiff, leftNormal)),
                                PR_DRAW_ARROW_HEAD_LENGTH));

    prVector2 rightHead = prVector2Add(
        v2,
        prVector2ScalarMultiply(prVector2Normalize(
                                    prVector2Add(unitDiff, rightNormal)),
                                PR_DRAW_ARROW_HEAD_LENGTH));

    DrawLineEx((Vector2) { .x = v1.x, .y = v1.y },
               (Vector2) { .x = v2.x, .y = v2.y },
               thick,
               color);

    DrawLineEx((Vector2) { .x = v2.x, .y = v2.y },
               (Vector2) { .x = leftHead.x, .y = leftHead.y },
               thick,
               color);

    DrawLineEx((Vector2) { .x = v2.x, .y = v2.y },
               (Vector2) { .x = rightHead.x, .y = rightHead.y },
               thick,
               color);
}

/* 
    Draws the AABB (Axis-Aligned Bounding Box) of `b` 
    with the given `thick`ness and `color`.
*/
void prDrawBodyAABB(const prBody *b, float thick, Color color) {
    if (b == NULL || thick <= 0.0f) return;

    prAABB aabb = prGetBodyAABB(b);

    DrawRectangleLinesEx((Rectangle) { .x = prUnitsToPixels(aabb.x),
                                       .y = prUnitsToPixels(aabb.y),
                                       .width = prUnitsToPixels(aabb.width),
                                       .height = prUnitsToPixels(aabb.height) },
                         thick,
                         color);

    prVector2 position = prVector2UnitsToPixels(prGetBodyPosition(b));

    DrawCircleV((Vector2) { .x = position.x, .y = position.y }, 2.0f, color);
}

/* Draws the outline of `b` with the given `thick`ness and `color`. */
void prDrawBodyLines(const prBody *b, float thick, Color color) {
    if (b == NULL || thick <= 0.0f) return;

    prShape *s = prGetBodyShape(b);

    prTransform tx = prGetBodyTransform(b);
    prVector2 position = prVector2UnitsToPixels(prGetBodyPosition(b));

    if (prGetShapeType(s) == PR_SHAPE_CIRCLE) {
        DrawRing((Vector2) { .x = position.x, .y = position.y },
                 prUnitsToPixels(prGetCircleRadius(s)) - thick,
                 prUnitsToPixels(prGetCircleRadius(s)),
                 0.0f,
                 360.0f,
                 PR_DRAW_CIRCLE_SEGMENT_COUNT,
                 color);
    } else if (prGetShapeType(s) == PR_SHAPE_POLYGON) {
        const prVertices *vertices = prGetPolygonVertices(s);

        for (int j = vertices->count - 1, i = 0; i < vertices->count;
             j = i, i++) {
            prVector2 v1 = prVector2Transform(vertices->data[j], tx);
            prVector2 v2 = prVector2Transform(vertices->data[i], tx);

            v1 = prVector2UnitsToPixels(v1);
            v2 = prVector2UnitsToPixels(v2);

            DrawLineEx((Vector2) { .x = v1.x, .y = v1.y },
                       (Vector2) { .x = v2.x, .y = v2.y },
                       thick,
                       color);
        }
    }

    DrawCircleV((Vector2) { .x = position.x, .y = position.y }, 2.0f, color);
}

/* 
    Draws a grid within the `bounds`, 
    with the given `cellSize`, `thick`ness and `color`. 
*/
void prDrawGrid(Rectangle bounds, float cellSize, float thick, Color color) {
    if (cellSize <= 0.0f || thick <= 0.0f) return;

    const float inverseCellSize = 1.0f / cellSize;

    const int vLineCount = bounds.width * inverseCellSize;
    const int hLineCount = bounds.height * inverseCellSize;

    for (int i = 0; i <= vLineCount; i++) {
        DrawLineEx((Vector2) { .x = bounds.x + prUnitsToPixels(cellSize * i),
                               .y = bounds.y },
                   (Vector2) { .x = bounds.x + prUnitsToPixels(cellSize * i),
                               .y = bounds.y + bounds.height },
                   thick,
                   color);
    }

    for (int i = 0; i <= hLineCount; i++)
        DrawLineEx((Vector2) { .x = bounds.x,
                               .y = bounds.y + prUnitsToPixels(cellSize * i) },
                   (Vector2) { .x = bounds.x + bounds.width,
                               .y = bounds.y + prUnitsToPixels(cellSize * i) },
                   thick,
                   color);

    DrawRectangleLinesEx(bounds, thick, color);
}

#endif  // `PROXIMA_RAYLIB_IMPLEMENTATION`