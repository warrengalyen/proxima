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

/* NOTE: `STB_DS_IMPLEMENTATION` is already defined in 'broad-phase.c' */
#include "external/stb_ds.h"

#include "proxima.h"

/* Typedefs ============================================================================= */

/* A structure that represents the key-value pair of the contact cache. */
typedef struct _prContactCacheEntry {
    prBodyPair key;
    prCollision value;
} prContactCacheEntry;

/* A structure that represents a simulation container. */
struct _prWorld {
    prVector2 gravity;
    prBody **bodies;
    prSpatialHash *hash;
    prContactCacheEntry *cache;
    float accumulator, timestamp;
};

/* A structure that represents the context data for `prPreStepHashQueryCallback()`. */
typedef struct _prPreStepHashQueryCtx {
    prWorld *world;
    int bodyIndex;
} prPreStepHashQueryCtx;

/* Private Function Prototypes ========================================================== */

/* 
    A callback function for `prQuerySpatialHash()` 
    that will be called during `prPreStepWorld()`. 
*/
static bool prPreStepHashQueryCallback(int otherIndex, void *ctx);

/* Finds all pairs of bodies in `w` that are colliding. */
static void prPreStepWorld(prWorld *w);

/* Clears the spatial hash for `w`. */
static void prPostStepWorld(prWorld *w);

/* Public Functions ===================================================================== */

/* 
    Creates a world with the `gravity` vector and `cellSize` 
    for broad-phase collision detection.
*/
prWorld *prCreateWorld(prVector2 gravity, float cellSize) {
    prWorld *result = calloc(1, sizeof *result);

    result->gravity = gravity;
    result->hash = prCreateSpatialHash(cellSize);

    arrsetcap(result->bodies, PR_WORLD_MAX_OBJECT_COUNT);

    return result;
}

/* Release the memory allocated for `w`. */
void prReleaseWorld(prWorld *w) {
    if (w == NULL) return;

    for (int i = 0; i < arrlen(w->bodies); i++)
        prReleaseBody(w->bodies[i]);

    prReleaseSpatialHash(w->hash);

    arrfree(w->bodies), hmfree(w->cache);

    free(w);
}

/* Erases all rigid bodies from `w`. */
void prClearWorld(prWorld *w) {
    if (w == NULL) return;
    
    prClearSpatialHash(w->hash);
    
    arrsetlen(w->bodies, 0);
}

/* Adds a rigid body to `w`. */
bool prAddBodyToWorld(prWorld *w, prBody *b) {
    if (w == NULL || b == NULL || arrlen(w->bodies) >= PR_WORLD_MAX_OBJECT_COUNT)
        return false;

    arrput(w->bodies, b);

    return true;
}

/* Removes a rigid body from `w`. */
bool prRemoveBodyFromWorld(prWorld *w, prBody *b) {
    if (w == NULL || b == NULL) return false;

    for (int i = 0; i < arrlen(w->bodies); i++) {
        if (w->bodies[i] == b) {
            // NOTE: `O(1)` performance!
            arrdelswap(w->bodies, i);

            return true;
        }
    }
    
    return false;
}

/* Returns a rigid body with the given `index` from `w`. */
prBody *prGetBodyFromWorld(const prWorld *w, int index) {
    if (w == NULL || index < 0 || index >= arrlen(w->bodies)) return NULL;

    return w->bodies[index];
}

/* Returns the number of rigid bodies in `w`. */
int prGetBodyCountForWorld(const prWorld *w) {
    return (w != NULL) ? arrlen(w->bodies) : 0;
}

/* Returns the gravity acceleration vector of `w`. */
prVector2 prGetWorldGravity(const prWorld *w) {
    return (w != NULL) ? w->gravity : PR_API_STRUCT_ZERO(prVector2);
}

/* Sets the `gravity` acceleration vector of `w`. */
void prSetWorldGravity(prWorld *w, prVector2 gravity) {
    if (w != NULL) w->gravity = gravity;
}

/* Proceeds the simulation over the time step `dt`, in seconds. */
void prStepWorld(prWorld *w, float dt) {
    if (w == NULL || dt <= 0.0f) return;

    prPreStepWorld(w);

    for (int i = 0; i < arrlen(w->bodies); i++) {
        prApplyGravityToBody(w->bodies[i], w->gravity);

        prIntegrateForBodyVelocity(w->bodies[i], dt);
    }

    const float inverseDt = 1.0f / dt;

    for (int i = 0; i < PR_WORLD_ITERATION_COUNT; i++)
        for (int j = 0; j < hmlen(w->cache); j++)
            prResolveCollision(
                w->cache[j].key.first,
                w->cache[j].key.second,
                &w->cache[j].value,
                inverseDt
            );

    for (int i = 0; i < arrlen(w->bodies); i++)
        prIntegrateForBodyPosition(w->bodies[i], dt);

    prPostStepWorld(w);
}

/* 
    Proceeds the simulation over the time step `dt`, in seconds,
    which will always run independent of the framerate.
*/
void prUpdateWorld(prWorld *w, float dt) {
    if (w == NULL || dt <= 0.0f) return;

    double currentTime = prGetCurrentTime();
    double elapsedTime = currentTime = w->timestamp;

    w->timestamp = currentTime, w->accumulator += elapsedTime;

    for (; w->accumulator >= dt; w->accumulator -= dt) 
        prStepWorld(w, dt);
}

/* 
    Casts a `ray` against all objects in `w`, 
    then calls `func` for each object that collides with `ray`. 
*/
void prComputeRaycastForWorld(prWorld *w, prRay ray, prRaycastQueryFunc func) {
    if (w == NULL || func == NULL) return;

    prClearSpatialHash(w->hash);

    for (int i = 0; i < arrlen(w->bodies); i++)
        prInsertToSpatialHash(w->hash, prGetBodyAABB(w->bodies[i]), i);

    prVector2 minVertex = ray.origin, maxVertex = prVector2Add(
        ray.origin, 
        prVector2ScalarMultiply(
            prVector2Normalize(ray.direction), 
            ray.maxDistance
        )
    );

    /* TODO: ... */
}

/* Private Functions ==================================================================== */

/* 
    A callback function for `prQuerySpatialHash()` 
    that will be called during `prPreStepWorld()`. 
*/
static bool prPreStepHashQueryCallback(int otherBodyIndex, void *ctx) {
    prPreStepHashQueryCtx *queryCtx = ctx;

    if (otherBodyIndex <= queryCtx->bodyIndex) return false;

    prBody *b1 = queryCtx->world->bodies[queryCtx->bodyIndex];
    prBody *b2 = queryCtx->world->bodies[otherBodyIndex];

    const prBodyPair key = { .first = b1, .second = b2 };

    prShape *s1 = prGetBodyShape(b1), *s2 = prGetBodyShape(b2);
    prTransform tx1 = prGetBodyTransform(b1), tx2 = prGetBodyTransform(b2);

    prCollision collision = { .count = 0 };

    if (!prComputeCollision(s1, tx1, s2, tx2, &collision)) {
        // NOTE: `hmdel()` returns `0` if `key` is not in `queryCtx->world->cache`!
        hmdel(queryCtx->world->cache, key);

        return false;
    }

    prContactCacheEntry *entry = hmgetp_null(queryCtx->world->cache, key);

    if (entry != NULL) {
        collision.friction = entry->value.friction;
        collision.restitution = entry->value.restitution;

        for (int i = 0; i < collision.count; i++) {
            int k = -1;

            for (int j = 0; j < entry->value.count; j++) {
                const int edgeId = entry->value.contacts[j].edgeId;

                if (collision.contacts[i].edgeId == edgeId) {
                    k = j;

                    break;
                }
            }

            if (k < 0) continue;

            const float accNormalScalar = entry->value.contacts[k].cache.normalScalar;
            const float accTangentScalar = entry->value.contacts[k].cache.tangentScalar;

            collision.contacts[i].cache.normalScalar = accNormalScalar;
            collision.contacts[i].cache.tangentScalar = accTangentScalar;
        }
    } else {
        collision.friction = prGetShapeFriction(s1) * prGetShapeFriction(s2);
        collision.restitution = fminf(prGetShapeRestitution(s1), prGetShapeRestitution(s2));

        if (collision.friction <= 0.0f) collision.friction = 0.0f;
        if (collision.restitution <= 0.0f) collision.restitution = 0.0f;
    }

    hmputs(
        queryCtx->world->cache,
        ((prContactCacheEntry) {
            .key = key, .value = collision 
        })
    );

    return true;
}

/* Finds all pairs of bodies in `w` that are colliding. */
static void prPreStepWorld(prWorld *w) {
    for (int i = 0; i < arrlen(w->bodies); i++)
        prInsertToSpatialHash(w->hash, prGetBodyAABB(w->bodies[i]), i);

    for (int i = 0; i < arrlen(w->bodies); i++)
        prQuerySpatialHash(
            w->hash, 
            prGetBodyAABB(w->bodies[i]), 
            prPreStepHashQueryCallback, 
            &(prPreStepHashQueryCtx) {
                .world = w, .bodyIndex = i
            }
        );
}

/* Clears the spatial hash for `w`. */
static void prPostStepWorld(prWorld *w) {
    for (int i = 0; i < arrlen(w->bodies); i++)
        prClearBodyForces(w->bodies[i]);

    prClearSpatialHash(w->hash);
}