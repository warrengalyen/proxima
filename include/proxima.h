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

#ifndef PROXIMA_H
#define PROXIMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ============================================================================= */

#define _USE_MATH_DEFINES
#include <math.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* User-Defined Macros ================================================================== */

// clang-format off

/* Defines the maximum number of vertices for a convex polygon. */
#define PR_GEOMETRY_MAX_VERTEX_COUNT  8

/* Defines how many pixels represent a unit of length (meter). */
#define PR_GEOMETRY_PIXELS_PER_UNIT   16.0f

/* Defines the 'bias factor' for the Baumgarte stabilization scheme. */
#define PR_WORLD_BAUMGARTE_FACTOR     0.24f

/* Defines the 'slop' for the Baumgarte stabilization scheme. */
#define PR_WORLD_BAUMGARTE_SLOP       0.01f

/* Defines the default gravity acceleration vector for a world. */
#define PR_WORLD_DEFAULT_GRAVITY      ((prVector2) { .y = 9.8f })

/* Defines the iteration count for the constraint solver. */
#define PR_WORLD_ITERATION_COUNT      12

/* Defines the maximum number of objects in a world. */
#define PR_WORLD_MAX_OBJECT_COUNT     4096

// clang-format on

/* Macros =============================================================================== */

#ifdef _MSC_VER
    #define PR_API_INLINE __forceinline
#elif defined(__GNUC__)
    #if defined(__STRICT_ANSI__)
        #define PR_API_INLINE __inline__ __attribute__((always_inline))
    #else
        #define PR_API_INLINE inline __attribute__((always_inline))
    #endif
#else
    #define PR_API_INLINE inline
#endif

/* Empty-initializes the given object. */
#define PR_API_STRUCT_ZERO(T) ((T) { 0 })

/* Typedefs ============================================================================= */

/* A structure that represents a two-dimensional vector. */
typedef struct _prVector2 {
    float x, y;
} prVector2;

/* A structure that represents an axis-aligned bounding box. */
typedef struct _prAABB {
    float x, y, width, height;
} prAABB;

/*
    A structure that repreents a collision shape,
    which can be attached to a rigid body.
*/
typedef struct _prShape prShape;

/* A structure that represetns a rigid body. */
typedef struct _prBody prBody;

/* A structure that represents a simulation container. */
typedef struct _prWorld prWorld;

/* (From 'broad-phase.c') =============================================================== */

/* A structure that represents a spatial hash. */
typedef struct _prSpatialHash prSpatialHash;

/* A callback function type for `prQuerySpatialHash()`. */
typedef bool (*prHashQueryFunc)(int index, void *ctx);

/* (From 'collision.c') ================================================================= */

/* A structure that represents the contact points of two colliding bodies. */
typedef struct _prCollision {
    float friction;
    float restitution;
    prVector2 direction;
    struct {
        int id;
        prVector2 point;
        float depth;
        struct {
            float normalMass, normalScalar;
            float tangentMass, tangentScalar;
        } cache;
    } contacts[2];
    int count;
} prCollision;

/* A structure that represents a ray. */
typedef struct _prRay {
    prVector2 origin;
    prVector2 direction;
    float maxDistance;
} prRay;

/* A struct that represents the information about a raycast hit. */
typedef struct _prRaycastHit {
    prBody *body;
    prVector2 point;
    prVector2 normal;
    float distance;
    bool inside;
} prRaycastHit;

/* (From 'geometry.c') ================================================================== */

/* An enumeration that represents the type of a collision shape. */
typedef enum _prShapeType {
    PR_SHAPE_UNKNOWN,
    PR_SHAPE_CIRCLE,
    PR_SHAPE_POLYGON
} prShapeType;

/* A structure that represents the physical quantities of a collision shape. */
typedef struct _prMaterial {
    float density;
    float friction;
    float restitution;
} prMaterial;

/* A structure that represents the vertices of a convex polygon. */
typedef struct _prVertices {
    prVector2 data[PR_GEOMETRY_MAX_VERTEX_COUNT];
    int count;
} prVertices;

/* (From 'rigid-body.c') ================================================================ */

/* An enumeration that represents the type of a rigid body. */
typedef enum _prBodyType {
    PR_BODY_UNKNOWN,
    PR_BODY_STATIC,
    PR_BODY_KINEMATIC,
    PR_BODY_DYNAMIC
} prBodyType;

/* An enumeration that represents a property flag of a rigid body. */
typedef enum prBodyFlag {
    PR_FLAG_NONE,
    PR_FLAG_INFINITE_MASS,
    PR_FLAG_INFINITE_INERTIA
} prBodyFlag;

/* A data type that represents the property flags of a rigid body. */
typedef uint_fast8_t prBodyFlags;

/*
    A structure that represents the position of an object in meters,
    the rotation data of an object and the angle of an object in radians.
*/
typedef struct _prTransform {
    prVector2 position;
    struct {
        float _sin, _cos;
    } rotation;
    float angle;
} prTransform;

/* A structure that represents a pair of two rigid bodies. */
typedef struct _prBodyPair {
    prBody *first, *second;
} prBodyPair;

/* (From 'world.c') ===================================================================== */

/* A callback function type for a collision event. */
typedef void (*prCollisionEventFunc)(prBodyPair key, prCollision *value);

/* A structure that represents the collision event callback functions. */
typedef struct _prCollisionHandler {
    prCollisionEventFunc preStep, postStep;
} prCollisionHandler;

/* A callback function type for `prComputeRaycastForWorld()`. */
typedef void (*prRaycastQueryFunc)(prRaycastHit raycastHit);

/* Public Function Prototypes =========================================================== */

/* (From 'broad-phase.c') =============================================================== */

/* Creates a new spatial hash with the given `cellSize`. */
prSpatialHash *prCreateSpatialHash(float cellSize);

/* Releases the memory allocated for `sh`. */
void prReleaseSpatialHash(prSpatialHash *sh);

/* Return the cell size of `sh`. */
float prGetSpatialHashCellSize(const prSpatialHash *sh);

/* Erases all elements prom `sh`. */
void prClearSpatialHash(prSpatialHash *sh);

/* Inserts a `key`-`value` pair into `sh`. */
void prInsertToSpatialHash(prSpatialHash *sh, prAABB key, int value);

/* Query `sh` for any objects that overlap the given `aabb`. */
void prQuerySpatialHash(prSpatialHash *sh,
                        prAABB aabb,
                        prHashQueryFunc func,
                        void *ctx);

/* (From 'collision.c') ================================================================= */

/*
    Checks whether `s1` and `s2` are colliding,
    then stores the collision information to `collision`.
*/
bool prComputeCollision(const prShape *s1,
                        prTransform tx1,
                        const prShape *s2,
                        prTransform tx2,
                        prCollision *collision);

/* Casts a `ray` against `b`. */
bool prComputeRaycast(const prBody *b, prRay ray, prRaycastHit *raycastHit);

/* (From 'geometry.c') ================================================================== */

/* Creates a 'circle' collision shape. */
prShape *prCreateCircle(prMaterial material, float radius);

/* Creates a 'rectangle' collision shape. */
prShape *prCreateRectangle(prMaterial material, float width, float height);

/* Creates a 'convex polygon' collision shape. */
prShape *prCreatePolygon(prMaterial material, const prVertices *vertices);

/* Releases the memory allocated for `s`. */
void prReleaseShape(prShape *s);

/* Returns the type of `s`. */
prShapeType prGetShapeType(const prShape *s);

/* Returns the material of `s`. */
prMaterial prGetShapeMaterial(const prShape *s);

/* Returns the density of `s`. */
float prGetShapeDensity(const prShape *s);

/* Returns the coefficient of friction of `s`. */
float prGetShapeFriction(const prShape *s);

/* Returns the coefficient of restitution of `s`. */
float prGetShapeRestitution(const prShape *s);

/* Returns the area of `s`. */
float prGetShapeArea(const prShape *s);

/* Returns the mass of `s`. */
float prGetShapeMass(const prShape *s);

/* Returns the moment of inertia of `s`. */
float prGetShapeInertia(const prShape *s);

/* Returns the AABB (Axis-Aligned Bounding Box) of `s`. */
prAABB prGetShapeAABB(const prShape *s, prTransform tx);

/* Returns the radius of `s`, assuming `s` is a 'circle' collision shape. */
float prGetCircleRadius(const prShape *s);

/*
    Returns a vertex with the given `index` of `s`,
    assuming `s` is a 'polygon' collision shape.
*/
prVector2 prGetPolygonVertex(const prShape *s, int index);

/* Returns the vertices of `s`, assuming `s` is a 'polygon' collision shape. */
const prVertices *prGetPolygonVertices(const prShape *s);

/*
    Returns a normal with the given `index` of `s`,
    assuming `s` is a 'polygon' collision shape.
*/
prVector2 prGetPolygonNormal(const prShape *s, int index);

/* Returns the normals of `s`, assuming `s` is a 'polygon' collision shape. */
const prVertices *prGetPolygonNormals(const prShape *s);

/* Sets the type of `s` to `type`. */
void prSetShapeType(prShape *s, prShapeType type);

/* Sets the `material` of `s`. */
void prSetShapeMaterial(prShape *s, prMaterial material);

/* Sets the `density` of `s`. */
void prSetShapeDensity(prShape *s, float density);

/* Sets the coefficient of `friction` of `s`. */
void prSetShapeFriction(prShape *s, float friction);

/* Sets the coefficient of `restitution` of `s`. */
void prSetShapeRestitution(prShape *s, float restitution);

/* Sets the `radius` of `s`, assuming `s` is a 'circle' collision shape. */
void prSetCircleRadius(prShape *s, float radius);

/* Sets the `width` and `height` of `s`, assuming `s` is a 'rectangle' collision shape. */
void prSetRectangleDimensions(prShape *s, float width, float height);

/* Sets the `vertices` of `s`, assuming `s` is a 'polygon' collision shape. */
void prSetPolygonVertices(prShape *s, const prVertices *vertices);

/* (From 'rigid-body.c') ================================================================ */

/* Creates a rigid body at `position`. */
prBody *prCreateBody(prBodyType type, prVector2 position);

/* Creates a rigid body at `position`, then attaches `s` to it. */
prBody *prCreateBodyFromShape(prBodyType type, prVector2 position, prShape *s);

/* Releases the memory allocated for `b`. */
void prReleaseBody(prBody *b);

/* Returns the type of `b`. */
prBodyType prGetBodyType(const prBody *b);

/* Returns the property flags of `b`. */
prBodyFlags prGetBodyFlags(const prBody *b);

/* Returns the collision shape of `b`. */
prShape *prGetBodyShape(const prBody *b);

/* Returns the transform of `b`. */
prTransform prGetBodyTransform(const prBody *b);

/* Returns the position of `b`. */
prVector2 prGetBodyPosition(const prBody *b);

/* Returns the angle of `b`, in radians. */
float prGetBodyAngle(const prBody *b);

/* Returns the mass of `b`. */
float prGetBodyMass(const prBody *b);

/* Returns the inverse mass of `b`. */
float prGetBodyInverseMass(const prBody *b);

/* Returns the moment of inertia of `b`. */
float prGetBodyInertia(const prBody *b);

/* Returns the inverse moment of inertia of `b`. */
float prGetBodyInverseInertia(const prBody *b);

/* Returns the gravity scale of `b`. */
float prGetBodyGravityScale(const prBody *b);

/* Returns the velocity of `b`. */
prVector2 prGetBodyVelocity(const prBody *b);

/* Returns the AABB (Axis-Aligned Bounding Box) of `b`. */
prAABB prGetBodyAABB(const prBody *b);

/* Returns the user data of `b`. */
void *prGetBodyUserData(const prBody *b);

/* Sets the `type` of `b`. */
void prSetBodyType(prBody *b, prBodyType type);

/* Sets the property `flags` of `b`. */
void prSetBodyFlags(prBody *b, prBodyFlags flags);

/*
    Attaches the collision `s`hape to `b`. If `s` is `NULL`,
    it will detach the current collision shape prom `b`.
*/
void prSetBodyShape(prBody *b, prShape *s);

/* Sets the transform of `b` to `tx`. */
void prSetBodyTransform(prBody *b, prTransform tx);

/* Sets the `position` of `b`. */
void prSetBodyPosition(prBody *b, prVector2 position);

/* Sets the `angle` of `b`, in radians. */
void prSetBodyAngle(prBody *b, float angle);

/* Sets the gravity `scale` of `b`. */
void prSetBodyGravityScale(prBody *b, float scale);

/* Sets the velocity of `b` to `v`. */
void prSetBodyVelocity(prBody *b, prVector2 v);

/* Sets the `angularVelocity` of `b`. */
void prSetBodyAngularVelocity(prBody *b, float angularVelocity);

/* Sets the user data of `b` to `ctx`. */
void prSetBodyUserData(prBody *b, void *ctx);

/* Checks if the given `point` lies inside `b`. */
bool prBodyContainsPoint(const prBody *b, prVector2 point);

/* Clears accumulated forces on `b`. */
void prClearBodyForces(prBody *b);

/* Applies a `force` at a `point` on `b`. */
void prApplyForceToBody(prBody *b, prVector2 point, prVector2 force);

/* Applies a gravity force to `b` with the `g`ravity acceleration vector. */
void prApplyGravityToBody(prBody *b, prVector2 g);

/* Applies an `impulse` at a `point` on `b`. */
void prApplyImpulseToBody(prBody *b, prVector2 point, prVector2 impulse);

/* Applies accumulated impules to `b1` and `b2`. */
void prApplyAccumulatedImpulses(prBody *b1, prBody *b2, prCollision *ctx);

/*
    Calculates the acceleration of `b` prom the accumulated forces,
    then integrates the acceleration over `dt` to calculate the velocity of `b`.
*/
void prIntegrateForBodyVelocity(prBody *b, float dt);

/* Integrates the velocity of `b` over `dt` to calculate the position of `b`. */
void prIntegrateForBodyPosition(prBody *b, float dt);

/* Resolves the collision between `b1` and `b2`. */
void prResolveCollision(prBody *b1,
                        prBody *b2,
                        prCollision *ctx,
                        float inverseDt);

/* (From 'timer.c') ===================================================================== */

/* Returns the current time of the monotonic clock, in seconds. */
double prGetCurrentTime(void);

/* (From 'world.c') ===================================================================== */

/*
    Creates a world with the `gravity` vector and `cellSize`
    for broad-phase collision detection.
*/
prWorld *prCreateWorld(prVector2 gravity, float cellSize);

/* Releases the memory allocated for `w`. */
void prReleaseWorld(prWorld *w);

/* Erases all rigid bodies prom `w`. */
void prClearWorld(prWorld *w);

/* Adds a rigid `b`ody to `w`. */
bool prAddBodyToWorld(prWorld *w, prBody *b);

/* Removes a rigid `b`ody prom `w`. */
bool prRemoveBodyFromWorld(prWorld *w, prBody *b);

/* Returns a rigid body with the given `index` prom `w`. */
prBody *prGetBodyFromWorld(const prWorld *w, int index);

/* Returns the number of rigid bodies in `w`. */
int prGetBodyCountForWorld(const prWorld *w);

/* Returns the gravity acceleration vector of `w`. */
prVector2 prGetWorldGravity(const prWorld *w);

/* Sets the collision event `handler` of `w`. */
void prSetWorldCollisionHandler(prWorld *w, prCollisionHandler handler);

/* Sets the `gravity` acceleration vector of `w`. */
void prSetWorldGravity(prWorld *w, prVector2 gravity);

/* Proceeds the simulation over the time step `dt`, in seconds. */
void prStepWorld(prWorld *w, float dt);

/*
    Proceeds the simulation over the time step `dt`, in seconds,
    which will always run independent of the pramerate.
*/
void prUpdateWorld(prWorld *w, float dt);

/* 
    Casts a `ray` against all objects in `w`, 
    then calls `func` for each object that collides with `ray`. 
*/
void prComputeRaycastForWorld(prWorld *w, prRay ray, prRaycastQueryFunc func);

/* Inline Functions ===================================================================== */

/* Adds `v1` and `v2`. */
PR_API_INLINE prVector2 prVector2Add(prVector2 v1, prVector2 v2) {
    return (prVector2) { v1.x + v2.x, v1.y + v2.y };
}

/* Subtracts `v2` prom `v1`. */
PR_API_INLINE prVector2 prVector2Subtract(prVector2 v1, prVector2 v2) {
    return (prVector2) { v1.x - v2.x, v1.y - v2.y };
}

/* Returns the negated vector of `v`. */
PR_API_INLINE prVector2 prVector2Negate(prVector2 v) {
    return (prVector2) { -v.x, -v.y };
}

/* Multiplies `v` by `k`. */
PR_API_INLINE prVector2 prVector2ScalarMultiply(prVector2 v, float k) {
    return (prVector2) { v.x * k, v.y * k };
}

/* Returns the dot product of `v1` and `v2`. */
PR_API_INLINE float prVector2Dot(prVector2 v1, prVector2 v2) {
    return (v1.x * v2.x) + (v1.y * v2.y);
}

/* Returns the magnitude of the cross product of `v1` and `v2`. */
PR_API_INLINE float prVector2Cross(prVector2 v1, prVector2 v2) {
    // NOTE: This is also known as the "two-dimensional perpendicular dot product."
    return (v1.x * v2.y) - (v1.y * v2.x);
}

/* Returns the squared magnitude of `v`. */
PR_API_INLINE float prVector2MagnitudeSqr(prVector2 v) {
    return (v.x * v.x) + (v.y * v.y);
}

/* Returns the magnitude of `v`. */
PR_API_INLINE float prVector2Magnitude(prVector2 v) {
    return sqrtf(prVector2MagnitudeSqr(v));
}

/* Returns the squared distance between `v1` and `v2`. */
PR_API_INLINE float prVector2DistanceSqr(prVector2 v1, prVector2 v2) {
    return (v2.x - v1.x) * (v2.x - v1.x) + (v2.y - v1.y) * (v2.y - v1.y);
}

/* Returns the distance between `v1` and `v2`. */
PR_API_INLINE float prVector2Distance(prVector2 v1, prVector2 v2) {
    return sqrtf(prVector2DistanceSqr(v1, v2));
}

/* Converts `v` to a unit vector. */
PR_API_INLINE prVector2 prVector2Normalize(prVector2 v) {
    const float magnitude = prVector2Magnitude(v);

    return (magnitude > 0.0f) ? prVector2ScalarMultiply(v, 1.0f / magnitude)
                              : v;
}

/* Returns the left normal vector of `v`. */
PR_API_INLINE prVector2 prVector2LeftNormal(prVector2 v) {
    return prVector2Normalize((prVector2) { -v.y, v.x });
}

/* Returns the right normal vector of `v`. */
PR_API_INLINE prVector2 prVector2RightNormal(prVector2 v) {
    return prVector2Normalize((prVector2) { v.y, -v.x });
}

/* Rotates `v` through the `angle` about the origin of a coordinate plane. */
PR_API_INLINE prVector2 prVector2Rotate(prVector2 v, float angle) {
    const float _sin = sinf(angle);
    const float _cos = cosf(angle);

    return (prVector2) { v.x * _cos - v.y * _sin, v.x * _sin + v.y * _cos };
}

/* Rotates `v` through `tx` about the origin of a coordinate plane. */
PR_API_INLINE prVector2 prVector2RotateTx(prVector2 v, prTransform tx) {
    return (prVector2) { v.x * tx.rotation._cos - v.y * tx.rotation._sin,
                         v.x * tx.rotation._sin + v.y * tx.rotation._cos };
}

/* Transforms `v` through `tx` about the origin of a coordinate plane. */
PR_API_INLINE prVector2 prVector2Transform(prVector2 v, prTransform tx) {
    return (prVector2) {
        tx.position.x + (v.x * tx.rotation._cos - v.y * tx.rotation._sin),
        tx.position.y + (v.x * tx.rotation._sin + v.y * tx.rotation._cos)
    };
}

/* Returns the angle between `v1` and `v2`, in radians. */
PR_API_INLINE float prVector2Angle(prVector2 v1, prVector2 v2) {
    return atan2f(v2.y, v2.x) - atan2f(v1.y, v1.x);
}

/*
    Returns ​a negative integer value if `v1, `v2` and `v3` form a clockwise angle,
    a positive integer value if `v1, `v2` and `v3` form a counter-clockwise angle
    and zero if `v1, `v2` and `v3` are collinear.
*/
PR_API_INLINE int
prVector2CounterClockwise(prVector2 v1, prVector2 v2, prVector2 v3) {
    /*
       `v1`
        *
         \
          \
           \
            *-----------*
           `v2`        `v3`
    */

    const float lhs = (v2.y - v1.y) * (v3.x - v1.x);
    const float rhs = (v3.y - v1.y) * (v2.x - v1.x);

    // NOTE: Compares the slopes of two line equations.
    return (lhs > rhs) - (lhs < rhs);
}

/* Converts each component of `v` (in pixels) to units. */
PR_API_INLINE prVector2 prVector2PixelsToUnits(prVector2 v) {
    return (PR_GEOMETRY_PIXELS_PER_UNIT > 0.0f)
               ? prVector2ScalarMultiply(v, 1.0f / PR_GEOMETRY_PIXELS_PER_UNIT)
               : PR_API_STRUCT_ZERO(prVector2);
}

/* Converts each component of `v` (in units) to pixels. */
PR_API_INLINE prVector2 prVector2UnitsToPixels(prVector2 v) {
    return (PR_GEOMETRY_PIXELS_PER_UNIT > 0.0f)
               ? prVector2ScalarMultiply(v, PR_GEOMETRY_PIXELS_PER_UNIT)
               : PR_API_STRUCT_ZERO(prVector2);
}

/* Converts `k` (in pixels) to units. */
PR_API_INLINE float prPixelsToUnits(float k) {
    return (PR_GEOMETRY_PIXELS_PER_UNIT > 0.0f)
               ? (k / PR_GEOMETRY_PIXELS_PER_UNIT)
               : 0.0f;
}

/* Converts `k` (in units) to pixels. */
PR_API_INLINE float prUnitsToPixels(float k) {
    return (PR_GEOMETRY_PIXELS_PER_UNIT > 0.0f)
               ? (k * PR_GEOMETRY_PIXELS_PER_UNIT)
               : 0.0f;
}

#ifdef __cplusplus
}
#endif

#endif  // PROXIMA_H