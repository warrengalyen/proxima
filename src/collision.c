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

/* A structure that represents an edge of a convex polygon. */
typedef struct _prEdge {
    prVector2 data[2];
    int indexes[2];
    int count;
} prEdge;

/* Private Function Prototypes ========================================================== */

/* 
    Clips `e` so that the dot product of each vertex in `e` 
    and `v` is greater than or equal to `dot`.
*/
static bool prClipEdge(prEdge *e, prVector2 v, float dot);

/* 
    Checks whether `s1` and `s2` are colliding,
    assuming `s1` and `s2` are 'circle' collision shapes,
    then stores the collision information to `collision`.
*/
static bool prComputeCollisionCircles(
    const prShape *s1, prTransform tx1, 
    const prShape *s2, prTransform tx2,
    prCollision *collision
);

/* 
    Checks whether `s1` and `s2` are colliding,
    assuming `s1` is a 'circle' collision shape and `s2` is a 'polygon' collision shape,
    then stores the collision information to `collision`.
*/
static bool prComputeCollisionCirclePoly(
    const prShape *s1, prTransform tx1, 
    const prShape *s2, prTransform tx2,
    prCollision *collision
);

/* 
    Checks whether `s1` and `s2` are colliding,
    assuming `s1` and `s2` are 'polygon' collision shapes,
    then stores the collision information to `collision`.
*/
static bool prComputeCollisionPolys(
    const prShape *s1, prTransform tx1, 
    const prShape *s2, prTransform tx2,
    prCollision *collision
);

/* Computes the intersection of a circle and a line. */
static bool prComputeIntersectionCircleLine(
    prVector2 center, float radius,
    prVector2 origin, prVector2 direction,
    float *lambda
);

/* Computes the intersection of two lines. */
static bool prComputeIntersectionLines(
    prVector2 origin1, prVector2 direction1,
    prVector2 origin2, prVector2 direction2,
    float *lambda
);

/* Returns the edge of `s` that is most perpendicular to `v`. */
static prEdge prGetContactEdge(const prShape *s, prTransform tx, prVector2 v);

/* Finds the axis of minimum penetration from `s1` to `s2`, then returns its index. */
static int prGetSeparatingAxisIndex(
    const prShape *s1, prTransform tx1, 
    const prShape *s2, prTransform tx2,
    float *depth
);

/* Finds the vertex farthest along `v`, then returns its index. */
static int prGetSupportPointIndex(
    const prVertices *vertices, 
    prTransform tx, prVector2 v
);

/* Public Functions ===================================================================== */

/* 
    Checks whether `s1` and `s2` are colliding,
    then stores the collision information to `collision`.
*/
bool prComputeCollision(
    const prShape *s1, prTransform tx1, 
    const prShape *s2, prTransform tx2,
    prCollision *collision
) {
    if (s1 == NULL || s2 == NULL) return false;

    prShapeType t1 = prGetShapeType(s1);
    prShapeType t2 = prGetShapeType(s2);

    if (t1 == PR_SHAPE_CIRCLE && t2 == PR_SHAPE_CIRCLE)
        return prComputeCollisionCircles(s1, tx1, s2, tx2, collision);
    else if ((t1 == PR_SHAPE_CIRCLE && t2 == PR_SHAPE_POLYGON) 
        || (t1 == PR_SHAPE_POLYGON && t2 == PR_SHAPE_CIRCLE))
        return prComputeCollisionCirclePoly(s1, tx1, s2, tx2, collision);
    else if (t1 == PR_SHAPE_POLYGON && t2 == PR_SHAPE_POLYGON)
        return prComputeCollisionPolys(s1, tx1, s2, tx2, collision);
    else return false;
}

/* Private Functions ==================================================================== */

/* 
    Clips `e` so that the dot product of each vertex in `e` 
    and `v` is greater than or equal to `dot`. 
*/
static bool prClipEdge(prEdge *e, prVector2 v, float dot) {
    e->count = 0;

    float dot1 = prVector2Dot(e->data[0], v) - dot;
    float dot2 = prVector2Dot(e->data[1], v) - dot;

    if (dot1 >= 0.0f && dot2 >= 0.0f) {
        e->count = 2;

        return true;
    } else {
        prVector2 edgeVector = prVector2Subtract(e->data[1], e->data[0]);
    
        prVector2 midpoint = prVector2Add(
            e->data[0], 
            prVector2ScalarMultiply(
                edgeVector, 
                (dot1 / (dot1 - dot2))
            )
        );

        if (dot1 > 0.0f && dot2 < 0.0f) {
            e->data[1] = midpoint, e->count = 2;

            return true;
        } else if (dot1 < 0.0f && dot2 > 0.0f) {
            e->data[0] = e->data[1], e->data[1] = midpoint, e->count = 2;

            return true;
        } else {
            return false;
        }
    }
}

/* 
    Checks whether `s1` and `s2` are colliding,
    assuming `s1` and `s2` are 'circle' collision shapes,
    then stores the collision information to `collision`.
*/
static bool prComputeCollisionCircles(
    const prShape *s1, prTransform tx1, 
    const prShape *s2, prTransform tx2,
    prCollision *collision
) {
    prVector2 direction = prVector2Subtract(tx2.position, tx1.position);

    float radiusSum = prGetCircleRadius(s1) + prGetCircleRadius(s2);
    float magnitudeSqr = prVector2MagnitudeSqr(direction);

    if (radiusSum * radiusSum < magnitudeSqr) return false;

    if (collision != NULL) {
        float magnitude = sqrtf(magnitudeSqr);

        collision->direction = (magnitude > 0.0f)
            ? prVector2ScalarMultiply(direction, 1.0f / magnitude)
            : (prVector2) { .x = 1.0f };

        collision->contacts[0].id = 0;

        collision->contacts[0].point = prVector2Transform(
            prVector2ScalarMultiply(collision->direction, prGetCircleRadius(s1)), tx1
        );

        collision->contacts[0].depth = (magnitude > 0.0f)
            ? radiusSum - magnitude
            : prGetCircleRadius(s1);

        collision->contacts[1] = collision->contacts[0];

        collision->count = 1;
    }

    return true;
}

/* 
    Checks whether `s1` and `s2` are colliding,
    assuming `s1` is a 'circle' collision shape and `s2` is a 'polygon' collision shape,
    then stores the collision information to `collision`.
*/
static bool prComputeCollisionCirclePoly(
    const prShape *s1, prTransform tx1, 
    const prShape *s2, prTransform tx2,
    prCollision *collision
) {
    prShape *circle, *poly;
    prTransform circleTx, polyTx;

    if (prGetShapeType(s1) == PR_SHAPE_CIRCLE) {
        circle = (prShape *) s1, poly = (prShape *) s2;
        circleTx = tx1, polyTx = tx2;
    } else {
        circle = (prShape *) s2, poly = (prShape *) s1;
        circleTx = tx2, polyTx = tx1;
    }

    const prVertices *vertices = prGetPolygonVertices(poly);
    const prVertices *normals = prGetPolygonNormals(poly);
    
    /*
        NOTE: `txCenter` refers to the center of the 'circle' collision shape
        transformed to the local space of the 'polygon' collision shape.
    */
    prVector2 txCenter = prVector2Rotate(
        prVector2Subtract(circleTx.position, polyTx.position), 
        -polyTx.angle
    );

    float radius = prGetCircleRadius(circle), maxDot = -FLT_MAX;

    int maxIndex = -1;

    /*
        NOTE: This will find the edge of the 'polygon' collision shape
        closest to the center of the 'circle' collision shape.
    */
    for (int i = 0; i < vertices->count; i++) {
        float dot = prVector2Dot(
            normals->data[i], 
            prVector2Subtract(txCenter, vertices->data[i])
        );

        if (dot > radius) return false;
        
        if (maxDot < dot) maxDot = dot, maxIndex = i;
    }

    if (maxIndex < 0) return false;

    /*
        NOTE: Is the center of the 'circle' collision shape 
        inside the 'polygon' collision shape?
    */
    if (maxDot < 0.0f) {
        if (collision != NULL) {
            collision->direction = prVector2Negate(
                prVector2RotateTx(normals->data[maxIndex], polyTx)
            );

            prVector2 deltaPosition = prVector2Subtract(tx2.position, tx1.position);

            if (prVector2Dot(deltaPosition, collision->direction) < 0.0f)
                collision->direction = prVector2Negate(collision->direction);

            collision->contacts[0].id = 0;

            collision->contacts[0].point = prVector2Add(
                circleTx.position,
                prVector2ScalarMultiply(collision->direction, radius)
            );

            collision->contacts[0].depth = radius - maxDot;

            collision->contacts[1] = collision->contacts[0];

            collision->count = 1;
        }
    } else {
        prVector2 v1 = (maxIndex > 0)
            ? vertices->data[maxIndex - 1]
            : vertices->data[vertices->count - 1];
        
        prVector2 v2 = vertices->data[maxIndex];

        prVector2 edgeVector = prVector2Subtract(v2, v1);

        prVector2 v1ToCenter = prVector2Subtract(txCenter, v1);
        prVector2 v2ToCenter = prVector2Subtract(txCenter, v2);

        float v1Dot = prVector2Dot(v1ToCenter, edgeVector);
        float v2Dot = prVector2Dot(v2ToCenter, prVector2Negate(edgeVector));

        /*
            NOTE: This means the center of the 'circle' collision shape
            does not lie on the line segment from `v1` to `v2`.
        */
        if (v1Dot <= 0.0f || v2Dot <= 0.0f) {
            prVector2 direction = (v1Dot <= 0.0f) ? v1ToCenter : v2ToCenter;

            float magnitudeSqr = prVector2MagnitudeSqr(direction);

            if (magnitudeSqr > radius * radius) return false;

            if (collision != NULL) {
                float magnitude = sqrtf(magnitudeSqr);

                collision->direction = (magnitude > 0.0f)
                    ? prVector2ScalarMultiply(
                        prVector2RotateTx(prVector2Negate(direction), polyTx), 
                        1.0f / magnitude
                    )
                    : PR_API_STRUCT_ZERO(prVector2);

                prVector2 deltaPosition = prVector2Subtract(tx2.position, tx1.position);

                if (prVector2Dot(deltaPosition, collision->direction) < 0.0f)
                    collision->direction = prVector2Negate(collision->direction);

                collision->contacts[0].id = 0;

                collision->contacts[0].point = prVector2Transform(
                    prVector2ScalarMultiply(collision->direction, radius), circleTx
                );

                collision->contacts[0].depth = (magnitude > 0.0f)
                    ? radius - magnitude
                    : radius;

                collision->contacts[1] = collision->contacts[0];

                collision->count = 1;
            }
        } else {
            if (collision != NULL) {
                collision->direction = prVector2Negate(
                    prVector2RotateTx(normals->data[maxIndex], polyTx)
                );

                prVector2 deltaPosition = prVector2Subtract(tx2.position, tx1.position);

                if (prVector2Dot(deltaPosition, collision->direction) < 0.0f)
                    collision->direction = prVector2Negate(collision->direction);

                collision->contacts[0].id = 0;

                collision->contacts[0].point = prVector2Add(
                    circleTx.position,
                    prVector2ScalarMultiply(collision->direction, radius)
                );

                collision->contacts[0].depth = radius - maxDot;

                collision->contacts[1] = collision->contacts[0];

                collision->count = 1;
            }
        }
    }

    return true;
}

/* 
    Checks whether `s1` and `s2` are colliding,
    assuming `s1` and `s2` are 'polygon' collision shapes,
    then stores the collision information to `collision`.
*/
static bool prComputeCollisionPolys(
    const prShape *s1, prTransform tx1, 
    const prShape *s2, prTransform tx2,
    prCollision *collision
) {
    float maxDepth1 = FLT_MAX, maxDepth2 = FLT_MAX;

    int index1 = prGetSeparatingAxisIndex(s1, tx1, s2, tx2, &maxDepth1);

    if (maxDepth1 >= 0.0f) return false;
    
    int index2 = prGetSeparatingAxisIndex(s2, tx2, s1, tx1, &maxDepth2);

    if (maxDepth2 >= 0.0f) return false;

    if (collision != NULL) {
        prVector2 direction = (maxDepth1 > maxDepth2)
            ? prVector2RotateTx(prGetPolygonNormal(s1, index1), tx1)
            : prVector2RotateTx(prGetPolygonNormal(s2, index2), tx2);

        prVector2 deltaPosition = prVector2Subtract(tx2.position, tx1.position);

        if (prVector2Dot(deltaPosition, direction) < 0.0f)
            direction = prVector2Negate(direction);

        prEdge edge1 = prGetContactEdge(s1, tx1, direction);
        prEdge edge2 = prGetContactEdge(s2, tx2, prVector2Negate(direction));

        prEdge refEdge = edge1, incEdge = edge2;

        prTransform refTx = tx1, incTx = tx2;

        prVector2 edgeVector1 = prVector2Subtract(edge1.data[1], edge1.data[0]);
        prVector2 edgeVector2 = prVector2Subtract(edge2.data[1], edge2.data[0]);

        const float edgeDot1 = prVector2Dot(edgeVector1, direction);
        const float edgeDot2 = prVector2Dot(edgeVector2, direction);

        bool incEdgeFlipped = false;

        if (fabsf(edgeDot1) > fabsf(edgeDot2)) {
            refEdge = edge2, incEdge = edge1;
            refTx = tx2, incTx = tx1;
            
            incEdgeFlipped = true;
        }

        prVector2 refEdgeVector = prVector2Normalize(
            prVector2Subtract(refEdge.data[1], refEdge.data[0])
        );

        const float refDot1 = prVector2Dot(refEdge.data[0], refEdgeVector);
        const float refDot2 = prVector2Dot(refEdge.data[1], refEdgeVector);

        if (!prClipEdge(&incEdge, refEdgeVector, refDot1)) return false;
        if (!prClipEdge(&incEdge, prVector2Negate(refEdgeVector), -refDot2)) return false;

        prVector2 refEdgeNormal = prVector2RightNormal(refEdgeVector);

        const float maxDepth = prVector2Dot(refEdge.data[0], refEdgeNormal);

        const float depth1 = prVector2Dot(incEdge.data[0], refEdgeNormal) - maxDepth;
        const float depth2 = prVector2Dot(incEdge.data[1], refEdgeNormal) - maxDepth;

        collision->direction = direction;

        collision->contacts[0].id = (!incEdgeFlipped) 
            ? PR_GEOMETRY_MAX_VERTEX_COUNT + incEdge.indexes[0]
            : incEdge.indexes[0];

        collision->contacts[1].id = (!incEdgeFlipped) 
            ? PR_GEOMETRY_MAX_VERTEX_COUNT + incEdge.indexes[1]
            : incEdge.indexes[1];

        if (depth1 < 0.0f) {
            collision->contacts[0].point = incEdge.data[1];
            collision->contacts[0].depth = depth2;

            collision->contacts[1].point = collision->contacts[0].point;
            collision->contacts[1].depth = collision->contacts[0].depth;

            collision->count = 1;
        } else if (depth2 < 0.0f) {
            collision->contacts[0].point = incEdge.data[0];
            collision->contacts[0].depth = depth1;

            collision->contacts[1].point = collision->contacts[0].point;
            collision->contacts[1].depth = collision->contacts[0].depth;

            collision->count = 1;
        } else {
            collision->contacts[0].point = incEdge.data[0];
            collision->contacts[1].point = incEdge.data[1];

            collision->contacts[0].depth = depth1;
            collision->contacts[1].depth = depth2;

            collision->count = 2;
        }
    }

    return true;
}

/* Returns the edge of `s` that is most perpendicular to `v`. */
static prEdge prGetContactEdge(const prShape *s, prTransform tx, prVector2 v) {
    const prVertices *vertices = prGetPolygonVertices(s);

    int supportIndex = prGetSupportPointIndex(vertices, tx, v);

    int prevIndex = (supportIndex == 0) ? vertices->count - 1 : supportIndex - 1;
    int nextIndex = (supportIndex == vertices->count - 1) ? 0 : supportIndex + 1;

    prVector2 prevEdgeVector = prVector2Normalize(
        prVector2Subtract(vertices->data[supportIndex], vertices->data[prevIndex])
    );

    prVector2 nextEdgeVector = prVector2Normalize(
        prVector2Subtract(vertices->data[supportIndex], vertices->data[nextIndex])
    );

    v = prVector2Rotate(v, -tx.angle);

    if (prVector2Dot(prevEdgeVector, v) < prVector2Dot(nextEdgeVector, v)) {
        return (prEdge) { 
            .data = {
                prVector2Transform(vertices->data[prevIndex], tx),
                prVector2Transform(vertices->data[supportIndex], tx)
            },
            .indexes = { prevIndex, supportIndex },
            .count = 2
        };
    } else {
        return (prEdge) {
            .data = {
                prVector2Transform(vertices->data[supportIndex], tx),
                prVector2Transform(vertices->data[nextIndex], tx)
            },
            .indexes = { supportIndex, nextIndex },
            .count = 2
        };
    }
}

/* Finds the axis of minimum penetration, then returns its index. */
static int prGetSeparatingAxisIndex(
    const prShape *s1, prTransform tx1, 
    const prShape *s2, prTransform tx2,
    float *depth
) {
    const prVertices *vertices1 = prGetPolygonVertices(s1);
    const prVertices *vertices2 = prGetPolygonVertices(s2);

    const prVertices *normals1 = prGetPolygonNormals(s1);

    float maxDepth = -FLT_MAX;

    int maxIndex = -1;

    for (int i = 0; i < normals1->count; i++) {
        prVector2 vertex = prVector2Transform(vertices1->data[i], tx1);
        prVector2 normal = prVector2RotateTx(normals1->data[i], tx1);

        int supportIndex = prGetSupportPointIndex(vertices2, tx2, prVector2Negate(normal));

        if (supportIndex < 0) return supportIndex;

        prVector2 supportPoint = prVector2Transform(vertices2->data[supportIndex], tx2);

        float depth = prVector2Dot(normal, prVector2Subtract(supportPoint, vertex));

        if (maxDepth < depth) maxDepth = depth, maxIndex = i;
    }

    if (depth != NULL) *depth = maxDepth;

    return maxIndex;
}

/* Returns the index of the vertex farthest along `v`. */
static int prGetSupportPointIndex(
    const prVertices *vertices, 
    prTransform tx, prVector2 v
) {
    float maxDot = -FLT_MAX;
    
    int maxIndex = -1;

    v = prVector2Rotate(v, -tx.angle);

    for (int i = 0; i < vertices->count; i++) {
        float dot = prVector2Dot(vertices->data[i], v);
        
        if (maxDot < dot) maxDot = dot, maxIndex = i;
    }

    return maxIndex;
}

/* Casta a `ray` against `b`. */
bool prComputeRaycast(const prBody *b, prRay ray, prRaycastHit *raycastHit) {
    if (b == NULL) return false;

    ray.direction = prVector2Normalize(ray.direction);

    const prShape *s = prGetBodyShape(b);
    prTransform tx = prGetBodyTransform(b);

    prShapeType type = prGetShapeType(s);

    float lambda = FLT_MAX;

    if (type == PR_SHAPE_CIRCLE) {
        bool intersects = prComputeIntersectionCircleLine(
            tx.position, prGetCircleRadius(s),
            ray.origin, ray.direction,
            &lambda
        );

        bool result = (lambda >= 0.0f) && (lambda <= ray.maxDistance);

        if (raycastHit != NULL) {
            raycastHit->body = (prBody *) b;

            raycastHit->point = prVector2Add(
                ray.origin, 
                prVector2ScalarMultiply(ray.direction, lambda)
            );

            raycastHit->normal = prVector2LeftNormal(
                prVector2Subtract(ray.origin, raycastHit->point)
            );

            raycastHit->distance = lambda;
            raycastHit->inside = (lambda < 0.0f);
        }

        return result;
    } else if (type == PR_SHAPE_POLYGON) {
        const prVertices *vertices = prGetPolygonVertices(s);

        int intersectionCount = 0;

        float minLambda = FLT_MAX;

        for (int j = vertices->count - 1, i = 0; i < vertices->count; j = i, i++) {
            prVector2 v1 = prVector2Transform(vertices->data[i], tx);
            prVector2 v2 = prVector2Transform(vertices->data[j], tx);

            prVector2 edgeVector = prVector2Subtract(v1, v2);

            bool intersects = prComputeIntersectionLines(
                ray.origin, ray.direction, 
                v2, edgeVector, 
                &lambda
            );

            if (intersects && lambda <= ray.maxDistance) {
                if (minLambda > lambda) {
                    minLambda = lambda;

                    if (raycastHit != NULL) {
                        raycastHit->point = prVector2Add(
                            ray.origin, 
                            prVector2ScalarMultiply(
                                ray.direction, 
                                minLambda
                            )
                        );

                        raycastHit->normal = prVector2LeftNormal(edgeVector);
                    }
                }

                intersectionCount++;
            }
        }

        if (raycastHit != NULL) {
            raycastHit->body = (prBody *) b;
            raycastHit->inside = (intersectionCount & 1);
        }

        return (!(raycastHit->inside) && (intersectionCount > 0));
    } else {
        return false;
    }
}

/* Computes the intersection of a circle and a line. */
static bool prComputeIntersectionCircleLine(
    prVector2 center, float radius,
    prVector2 origin, prVector2 direction,
    float *lambda
) {
    const prVector2 originToCenter = prVector2Subtract(center, origin);

    const float dot = prVector2Dot(originToCenter, direction);

    const float heightSqr = prVector2MagnitudeSqr(originToCenter) - (dot * dot);
    const float baseSqr = (radius * radius) - heightSqr;

    if (lambda != NULL) *lambda = dot - sqrtf(baseSqr);

    return (dot >= 0.0f && baseSqr >= 0.0f);
}

/* Computes the intersection of two lines. */
static bool prComputeIntersectionLines(
    prVector2 origin1, prVector2 direction1,
    prVector2 origin2, prVector2 direction2,
    float *lambda
) {
    float rXs = prVector2Cross(direction1, direction2);

    prVector2 qp = prVector2Subtract(origin2, origin1);

    float qpXs = prVector2Cross(qp, direction2);
    float qpXr = prVector2Cross(qp, direction1);

    if (rXs != 0.0f) {
        float inverseRxS = 1.0f / rXs;

        float t = qpXs * inverseRxS, u = qpXr * inverseRxS;

        if ((t >= 0.0f && t <= 1.0f) && (u >= 0.0f && u <= 1.0f)) {
            if (lambda != NULL) *lambda = t;

            return true;
        }

        return false;
    } else {
        if (qpXr != 0.0f) return 0;

        float rDr = prVector2Dot(direction1, direction1);
        float sDr = prVector2Dot(direction2, direction1);

        float inverseRdR = 1.0f / rDr;

        float qpDr = prVector2Dot(qp, direction1);

        float k, t0 = qpDr * inverseRdR, t1 = t0 + sDr * inverseRdR;

        if (sDr < 0.0f) k = t0, t0 = t1, t1 = k;

        if ((t0 < 0.0f && t1 == 0.0f) || (t0 == 1.0f && t1 > 1.0f)) {
            if (lambda != NULL) *lambda = (t0 == 1.0f);

            return 1;
        }

        return (t1 >= 0.0f && t0 <= 1.0f);
    }
}