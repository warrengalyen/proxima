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

#include <float.h>

#include "proxima.h"

/* Typedefs ============================================================================= */

/* 
    A structure that represents a collision shape, 
    which can be attached to a rigid body.
*/
struct _prShape {
    prShapeType type;
    prMaterial material;
    float area;
    union {
        struct { float radius; } circle;
        struct { prVertices vertices, normals; } polygon;
    };
};

/* Private Function Prototypes ========================================================== */

/* 
    Computes the convex hull for the given `input` points 
    with the gift wrapping (a.k.a. Jarvis march) algorithm.
*/
static void prJarvisMarch(const prVertices *input, prVertices *output);

/* Public Functions ===================================================================== */

/* Creates a 'circle' collision shape. */
prShape *prCreateCircle(prMaterial material, float radius) {
    if (radius <= 0.0f) return NULL;

    prShape *result = calloc(1, sizeof *result);

    result->type = PR_SHAPE_CIRCLE;
    result->material = material;

    prSetCircleRadius(result, radius);

    return result;
}

/* Creates a 'rectangle' collision shape. */
prShape *prCreateRectangle(prMaterial material, float width, float height) {
    if (width <= 0.0f || height <= 0.0f) return NULL;

    prShape *result = calloc(1, sizeof *result);

    result->type = PR_SHAPE_POLYGON;
    result->material = material;

    const float halfWidth = 0.5f * width, halfHeight = 0.5f * height;

    // NOTE: https://en.cppreference.com/w/c/language/compound_literal
    prSetPolygonVertices(result, &(const prVertices) {
        .data = {
            { .x = -halfWidth, .y = -halfHeight },
            { .x = -halfWidth, .y =  halfHeight },
            { .x = halfWidth,  .y =  halfHeight },
            { .x = halfWidth,  .y = -halfHeight }
        },
        .count = 4
    });

    return result;
}

/* Creates a 'convex polygon' collision shape. */
prShape *prCreatePolygon(prMaterial material, const prVertices *vertices) {
    if (vertices == NULL || vertices->count <= 0) return NULL;

    prShape *result = calloc(1, sizeof *result);

    result->type = PR_SHAPE_POLYGON;
    result->material = material;

    prSetPolygonVertices(result, vertices);

    return result;
}

/* Releases the memory allocated by `s`. */
void prReleaseShape(prShape *s) {
    free(s);
}

/* Returns the type of `s`. */
prShapeType prGetShapeType(const prShape *s) {
    return (s != NULL) ? s->type : PR_SHAPE_UNKNOWN;
}

/* Returns the material of `s`. */
prMaterial prGetShapeMaterial(const prShape *s) {
    return (s != NULL) ? s->material : PR_API_STRUCT_ZERO(prMaterial);
}

/* Returns the density of `s`. */
float prGetShapeDensity(const prShape *s) {
    return (s != NULL) ? s->material.density : 0.0f;
}

/* Returns the coefficient of friction of `s`. */
float prGetShapeFriction(const prShape *s) {
    return (s != NULL) ? s->material.friction : 0.0f;
}

/* Returns the coefficient of restitution of `s`. */
float prGetShapeRestitution(const prShape *s) {
    return (s != NULL) ? s->material.restitution : 0.0f;
}

/* Returns the area of `s`. */
float prGetShapeArea(const prShape *s) {
    return (s != NULL) ? s->area : 0.0f;
}

/* Returns the mass of `s`. */
float prGetShapeMass(const prShape *s) {
    return (s != NULL) ? s->material.density * s->area : 0.0f;
}

/* Returns the moment of inertia of `s`. */
float prGetShapeInertia(const prShape *s) {
    if (s == NULL || s->material.density <= 0.0f) return 0.0f;

    if (s->type == PR_SHAPE_CIRCLE) {
        return 0.5f * prGetShapeMass(s) * (s->circle.radius * s->circle.radius);
    } else if (s->type == PR_SHAPE_POLYGON) {
        float numerator = 0.0f, denominator = 0.0f;

        const int vertexCount = s->polygon.vertices.count;

        // NOTE: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        for (int j = vertexCount - 1, i = 0; i < vertexCount; j = i, i++) {
            prVector2 v1 = s->polygon.vertices.data[j], v2 = s->polygon.vertices.data[i];

            const float cross = prVector2Cross(v1, v2), dotSum = (prVector2Dot(v1, v1)
                + prVector2Dot(v1, v2) + prVector2Dot(v2, v2));

            numerator += (cross * dotSum), denominator += cross;
        }

        return s->material.density * (numerator / (6.0f * denominator));
    } else {
        return 0.0f;
    }
}

/* Returns the AABB (Axis-Aligned Bounding Box) of `s`. */
prAABB prGetShapeAABB(const prShape *s, prTransform tx) {
    prAABB result = PR_API_STRUCT_ZERO(prAABB);

     if (s != NULL) {
        if (s->type == PR_SHAPE_CIRCLE) {
            result.x = tx.position.x - s->circle.radius;
            result.y = tx.position.y - s->circle.radius;
            
            result.width = result.height = 2.0f * s->circle.radius;
        } else if (s->type == PR_SHAPE_POLYGON) {
            prVector2 minVertex = { .x = FLT_MAX,  .y = FLT_MAX };
            prVector2 maxVertex = { .x = -FLT_MAX, .y = -FLT_MAX };
            
            for (int i = 0; i < s->polygon.vertices.count; i++) {
                const prVector2 v = prVector2Transform(s->polygon.vertices.data[i], tx);
                
                if (minVertex.x > v.x) minVertex.x = v.x;
                if (minVertex.y > v.y) minVertex.y = v.y;
                    
                if (maxVertex.x < v.x) maxVertex.x = v.x;
                if (maxVertex.y < v.y) maxVertex.y = v.y;
            }
            
            const float deltaX = maxVertex.x - minVertex.x;
            const float deltaY = maxVertex.y - minVertex.y;
            
            result.x = minVertex.x;
            result.y = minVertex.y;

            result.width = deltaX;
            result.height = deltaY;
        }
    }

    return result;
}

float prGetCircleRadius(const prShape *s) {
    return (prGetShapeType(s) == PR_SHAPE_CIRCLE) ? s->circle.radius : 0.0f;
}

/* 
    Returns a vertex with the given `index` of `s`, 
    assuming `s` is a 'polygon' collision shape. 
*/
prVector2 prGetPolygonVertex(const prShape *s, int index) {
    if (prGetShapeType(s) != PR_SHAPE_POLYGON
        || index < 0 || index >= s->polygon.vertices.count)
        return PR_API_STRUCT_ZERO(prVector2);

    return s->polygon.vertices.data[index];
}

/* Returns the vertices of `s`, assuming `s` is a 'polygon' collision shape. */
const prVertices *prGetPolygonVertices(const prShape *s) {
    return (prGetShapeType(s) == PR_SHAPE_POLYGON)
        ? &(s->polygon.vertices)
        : NULL;
}

/* 
    Returns a normal with the given `index` of `s`, 
    assuming `s` is a 'polygon' collision shape. 
*/
prVector2 prGetPolygonNormal(const prShape *s, int index) {
    if (prGetShapeType(s) != PR_SHAPE_POLYGON
        || index < 0 || index >= s->polygon.normals.count) 
        return PR_API_STRUCT_ZERO(prVector2);

    return s->polygon.normals.data[index];
}

/* Returns the normals of `s`, assuming `s` is a 'polygon' collision shape. */
const prVertices *prGetPolygonNormals(const prShape *s) {
    return (prGetShapeType(s) == PR_SHAPE_POLYGON) 
        ? &(s->polygon.normals)
        : NULL;
}

/* Sets the type of `s` to `type`. */
void prSetShapeType(prShape *s, prShapeType type) {
    if (s != NULL) s->type = type;
}

/* Sets the `material` of `s`. */
void prSetShapeMaterial(prShape *s, prMaterial material) {
    if (s != NULL) s->material = material;
}

/* Sets the `density` of `s`. */
void prSetShapeDensity(prShape *s, float density) {
    if (s != NULL) s->material.density = density;
}

/* Sets the coefficient of `friction` of `s`. */
void prSetShapeFriction(prShape *s, float friction) {
    if (s != NULL) s->material.friction = friction;
}

/* Sets the coefficient of `restitution` of `s`. */
void prSetShapeRestitution(prShape *s, float restitution) {
    if (s != NULL) s->material.restitution = restitution;
}

/* Sets the `radius` of `s`, assuming `s` is a 'circle' collision shape. */
void prSetCircleRadius(prShape *s, float radius) {
    if (s == NULL || s->type != PR_SHAPE_CIRCLE) return; 
    
    s->circle.radius = radius;

    s->area = M_PI * (radius * radius);
}

/* Sets the `width` and `height` of `s`, assuming `s` is a 'rectangle' collision shape. */
void prSetRectangleDimensions(prShape *s, float width, float height) {
    if (s == NULL || width <= 0.0f || height <= 0.0f) return;

    const float halfWidth = 0.5f * width, halfHeight = 0.5f * height;

    // NOTE: https://en.cppreference.com/w/c/language/compound_literal
    prSetPolygonVertices(s, &(const prVertices) {
        .data = {
            { -halfWidth, -halfHeight },
            { -halfWidth,  halfHeight },
            {  halfWidth,  halfHeight },
            {  halfWidth, -halfHeight }
        },
        .count = 4
    });
}

/* Sets the `vertices` of `s`, assuming `s` is a 'polygon' collision shape. */
void prSetPolygonVertices(prShape *s, const prVertices *vertices) {
    if (s == NULL || vertices == NULL || vertices->count <= 0) return;

    prVertices newVertices = { .count = 0 };

    prJarvisMarch(vertices, &newVertices);

    {
        s->polygon.vertices.count = s->polygon.normals.count = newVertices.count;

        for (int i = 0; i < newVertices.count; i++)
            s->polygon.vertices.data[i] = newVertices.data[i];

        for (int j = newVertices.count - 1, i = 0; i < newVertices.count; j = i, i++)
            s->polygon.normals.data[i] = prVector2LeftNormal(
                prVector2Subtract(
                    s->polygon.vertices.data[i], 
                    s->polygon.vertices.data[j]
                )
            );
    }

    float twiceAreaSum = 0.0f;

    for (int i = 0; i < s->polygon.vertices.count - 1; i++) {
        /*
            NOTE: Divides the convex hull into multiple triangles,
            then computes the area for each triangle.
        */

        const float twiceArea = prVector2Cross(
            prVector2Subtract(s->polygon.vertices.data[i], s->polygon.vertices.data[0]),
            prVector2Subtract(s->polygon.vertices.data[i + 1], s->polygon.vertices.data[0])
        );

        twiceAreaSum += twiceArea;
    }

    s->area = fabsf(0.5f * twiceAreaSum);
}

/* Private Functions ==================================================================== */

/* 
    Computes the convex hull for the given `input` points 
    with the gift wrapping (a.k.a. Jarvis march) algorithm.
*/
static void prJarvisMarch(const prVertices *input, prVertices *output) {
    if (input == NULL || output == NULL || input->count < 3) return;

    /* 
        NOTE: Since the `input` size is most likely to be small (less than 128?),
        we do not need advanced convex hull algorithms like Graham scan, Quickhull, etc.
    */

   int lowestIndex = 0;

    for (int i = 1; i < input->count; i++)
        if (input->data[lowestIndex].x > input->data[i].x)
            lowestIndex = i;

    output->count = 0, output->data[output->count++] = input->data[lowestIndex];

    int currentIndex = lowestIndex, nextIndex = currentIndex;

    for (;;) {
        for (int i = 0; i < input->count; i++) {
            if (i == currentIndex) continue;

            nextIndex = i;

            break;
        }

        for (int i = 0; i < input->count; i++) {
            if (i == currentIndex || i == nextIndex) continue;

            const int direction = prVector2CounterClockwise(
                input->data[currentIndex], input->data[i], input->data[nextIndex]
            );

            if (direction < 0) continue;

            const float toCandidate = prVector2DistanceSqr(
                input->data[currentIndex], input->data[nextIndex]
            );

            const float toNext = prVector2DistanceSqr(
                input->data[currentIndex], input->data[nextIndex]
            );

            if (direction != 0 || (direction == 0 && (toCandidate > toNext)))
                nextIndex = i;
        }

        if (nextIndex == lowestIndex) break;

        currentIndex = nextIndex;

        output->data[output->count++] = input->data[nextIndex];
    }       
}
