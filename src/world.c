/* Includes ============================================================================= */

/* NOTE: `STB_DS_IMPLEMENTATION` is already defined in 'broad-phase.c' */
#include "external/stb_ds.h"

#include "proxima.h"

/* Typedefs ============================================================================= */

/* A structure that represents the context data for `prPreStepQueryCallback()`. */
typedef struct _prPreStepQueryContext {
    prWorld *world;
    int bodyIndex;
} prPreStepQueryContext;

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

/* Private Function Prototypes ========================================================== */

/* 
    A callback function for `prQuerySpatialHash()` 
    that will be called during `prPreStepWorld()`. 
*/
static void prPreStepQueryCallback(int otherIndex, void *ctx);

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

/* Removes a rigid `b`ody from `w`. */
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

/* Private Functions ==================================================================== */

/* 
    A callback function for `prQuerySpatialHash()` 
    that will be called during `prPreStepWorld()`. 
*/
static void prPreStepQueryCallback(int otherBodyIndex, void *ctx) {
    prPreStepQueryContext *queryCtx = ctx;

    if (otherBodyIndex <= queryCtx->bodyIndex) return;

    prBody *b1 = queryCtx->world->bodies[queryCtx->bodyIndex];
    prBody *b2 = queryCtx->world->bodies[otherBodyIndex];

    const prBodyPair key = { .first = b1, .second = b2 };

    prShape *s1 = prGetBodyShape(b1), *s2 = prGetBodyShape(b2);
    prTransform tx1 = prGetBodyTransform(b1), tx2 = prGetBodyTransform(b2);

    prCollision collision = { .count = 0 };

    if (!prComputeCollision(s1, tx1, s2, tx2, &collision)) {
        // NOTE: `hmdel()` returns `0` if `key` is not in `queryCtx->world->cache`!
        hmdel(queryCtx->world->cache, key);

        return;
    }

    prContactCacheEntry *entry = hmgetp_null(queryCtx->world->cache, key);

    if (entry != NULL) {
        collision.friction = entry->value.friction;
        collision.restitution = entry->value.restitution;

        /* TODO: ... */
    } else {
        collision.friction = prGetShapeFriction(s1) * prGetShapeFriction(s2);
        collision.restitution = fminf(prGetShapeRestitution(s1), prGetShapeRestitution(s2));

        if (collision.friction <= 0.0f) collision.friction = 0.0f;
        if (collision.restitution <= 0.0f) collision.restitution = 0.0f;
    }

    hmputs(
        queryCtx->world->cache,
        ((prContactCacheEntry) {
            .key = key,
            .value = collision
        })
    );
}

/* Finds all pairs of bodies in `w` that are colliding. */
static void prPreStepWorld(prWorld *w) {
    for (int i = 0; i < arrlen(w->bodies); i++)
        prInsertToSpatialHash(w->hash, prGetBodyAABB(w->bodies[i]), i);

    for (int i = 0; i < arrlen(w->bodies); i++)
        prQuerySpatialHash(
            w->hash, 
            prGetBodyAABB(w->bodies[i]), 
            prPreStepQueryCallback, 
            &(prPreStepQueryContext) {
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