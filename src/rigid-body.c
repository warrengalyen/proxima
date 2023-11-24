/* Includes ============================================================================= */

#include "proxima.h"

/* Typedefs ============================================================================= */

/* A structure that represents the motion data of a rigid body. */
typedef struct _prMotionData {
    float mass, inverseMass;
    float inertia, inverseInertia;
    float gravityScale;
    prVector2 velocity;
    float angularVelocity;
    prVector2 force;
    float torque;
} prMotionData;

/* A structure that represents a rigid body. */
struct _prBody {
    prBodyType type;
    prBodyFlags flags;
    prShape *shape;
    prTransform tx;
    prMotionData mtn;
    prAABB aabb;
    void *ctx;
};

/* Constants ============================================================================ */

/* Constants for `prNormalizeAngle()`. */
static const float TWO_PI = 2.0f * M_PI, INVERSE_TWO_PI = 1.0f / (2.0f * M_PI);

/* Private Function Prototypes ========================================================== */

/* Computes the mass and the moment of inertia for `b`. */
static void prComputeBodyMass(prBody *b);

/* Normalizes the `angle` to a range `[0, 2π]`. */
static PR_API_INLINE float prNormalizeAngle(float angle);

/* Public Functions ===================================================================== */

/* Creates a rigid body at `position`. */
prBody *prCreateBody(prBodyType type, prVector2 position) {
    if (type < PR_BODY_STATIC || type > PR_BODY_DYNAMIC) return NULL;

    prBody *result = calloc(1, sizeof *result);

    result->type = type;

    result->tx.position = position;

    result->tx.rotation._sin = 0.0f;
    result->tx.rotation._cos = 1.0f;

    result->mtn.gravityScale = 1.0f;

    return result;
}

/* Creates a rigid body at `position`, then attaches `s` to it. */
prBody *prCreateBodyFromShape(prBodyType type, prVector2 position, prShape *s) {
    prBody *result = prCreateBody(type, position);

    prSetBodyShape(result, s);

    return result;
}

/* Releases the memory allocated for `b`. */
void prReleaseBody(prBody *b) {
    free(b);
}

/* Returns the type of `b`. */
prBodyType prGetBodyType(const prBody *b) {
    return (b != NULL) ? b->type : PR_BODY_UNKNOWN;
}

/* Returns the property flags of `b`. */
prBodyFlags prGetBodyFlags(const prBody *b) {
    return (b != NULL) ? b->flags : 0u;
}

/* Returns the collision shape of `b`. */
prShape *prGetBodyShape(const prBody *b) {
    return (b != NULL) ? b->shape : NULL;
}

/* Returns the transform of `b`. */
prTransform prGetBodyTransform(const prBody *b) {
    return (b != NULL) ? b->tx : PR_API_STRUCT_ZERO(prTransform);
}

/* Returns the position of `b`. */
prVector2 prGetBodyPosition(const prBody *b) {
    return (b != NULL) ? b->tx.position : PR_API_STRUCT_ZERO(prVector2);
}

/* Returns the angle of `b`, in radians. */
float prGetBodyAngle(const prBody *b) {
    return (b != NULL) ? b->tx.angle : 0.0f;
}

/* Returns the mass of `b`. */
float prGetBodyMass(const prBody *b) {
    return (b != NULL) ? b->mtn.mass : 0.0f;
}

/* Returns the inverse mass of `b`. */
float prGetBodyInverseMass(const prBody *b) {
    return (b != NULL) ? b->mtn.inverseMass : 0.0f;
}

/* Returns the moment of inertia of `b`. */
float prGetBodyInertia(const prBody *b) {
    return (b != NULL) ? b->mtn.inertia : 0.0f;
}

/* Returns the inverse moment of inertia of `b`. */
float prGetBodyInverseInertia(const prBody *b) {
    return (b != NULL) ? b->mtn.inverseInertia : 0.0f;
}

/* Returns the gravity scale of `b`. */
float prGetBodyGravityScale(const prBody *b) {
    return (b != NULL) ? b->mtn.gravityScale : 0.0f;
}

/* Returns the velocity of `b`. */
prVector2 prGetBodyVelocity(const prBody *b) {
    return (b != NULL) ? b->mtn.velocity : PR_API_STRUCT_ZERO(prVector2);
}

/* Returns the AABB (Axis-Aligned Bounding Box) of `b`. */
prAABB prGetBodyAABB(const prBody *b) {
    return (b != NULL && b->shape != NULL) ? b->aabb : PR_API_STRUCT_ZERO(prAABB);
}

/* Returns the user data of `b`. */
void *prGetBodyUserData(const prBody *b) {
    return (b != NULL) ? b->ctx : NULL;
}

/* Sets the `type` of `b`. */
void prSetBodyType(prBody *b, prBodyType type) {
    if (b == NULL) return;

    b->type = type;

    prComputeBodyMass(b);
}

/* Sets the property `flags` of `b`. */
void prSetBodyFlags(prBody *b, prBodyFlags flags) {
    if (b == NULL) return;

    b->flags = flags;

    prComputeBodyMass(b);
}

/* 
    Attaches the collision `s`hape to `b`. If `s` is `NULL`, 
    it will detach the current collision shape from `b`.
*/
void prSetBodyShape(prBody *b, prShape *s) {
    if (b == NULL) return;

    b->shape = s;

    b->aabb = (s != NULL) ? prGetShapeAABB(s, b->tx) : PR_API_STRUCT_ZERO(prAABB);

    prComputeBodyMass(b);
}

/* Sets the position of `b` to `position`. */
void prSetBodyPosition(prBody *b, prVector2 position) {
    if (b == NULL) return;

    b->tx.position = position;

    b->aabb = prGetShapeAABB(b->shape, b->tx);
}

/* Sets the `angle` of `b`, in radians. */
void prSetBodyAngle(prBody *b, float angle) {
    if (b == NULL) return;

    b->tx.angle = prNormalizeAngle(angle);

    /*
        NOTE: These values must be cached in order to 
        avoid expensive computations as much as possible.
    */
    b->tx.rotation._sin = sinf(b->tx.angle);
    b->tx.rotation._cos = cosf(b->tx.angle);

    b->aabb = prGetShapeAABB(b->shape, b->tx);
}

/* Sets the gravity `scale` of `b`. */
void prSetBodyGravityScale(prBody *b, float scale) {
    if (b != NULL) b->mtn.gravityScale = scale;
}

/* Sets the `velocity` of `b`. */
void prSetBodyVelocity(prBody *b, prVector2 velocity) {
    if (b != NULL) b->mtn.velocity = velocity;
}

/* Sets the `angularVelocity` of `b`. */
void prSetBodyAngularVelocity(prBody *b, float angularVelocity) {
    if (b != NULL) b->mtn.angularVelocity = angularVelocity;
}

/* Sets the user data of `b` to `ctx`. */
void prSetBodyUserData(prBody *b, void *ctx) {
    if (b != NULL) b->ctx = ctx;
}

/* Clears accumulated forces on `b`. */
void prClearBodyForces(prBody *b) {
    if (b == NULL) return;

    b->mtn.force.x = b->mtn.force.y = b->mtn.torque = 0.0f;
}

/* Applies a `force` at a `point` on `b`. */
void prApplyForceToBody(prBody *b, prVector2 point, prVector2 force) {
    if (b == NULL || b->mtn.inverseMass <= 0.0f) return;

    b->mtn.force = prVector2Add(b->mtn.force, force);
    b->mtn.torque += prVector2Cross(point, force);
}

/* Applies a gravity force to `b` with the `g`ravity acceleration vector. */
void prApplyGravityToBody(prBody *b, prVector2 g) {
    if (b == NULL || b->mtn.mass <= 0.0f) return;

    b->mtn.force = prVector2Add(
        b->mtn.force,
        prVector2ScalarMultiply(g, b->mtn.gravityScale * b->mtn.mass)
    );
}

/* Applies an `impulse` at a `point` on `b`. */
void prApplyImpulseToBody(prBody *b, prVector2 point, prVector2 impulse) {
    if (b == NULL || b->mtn.inverseMass <= 0.0f) return;

    b->mtn.velocity = prVector2Add(
        b->mtn.velocity,
        prVector2ScalarMultiply(impulse, b->mtn.inverseMass)
    );

    b->mtn.angularVelocity += b->mtn.inverseInertia 
        * prVector2Cross(point, impulse);
}

/* 
    Calculates the acceleration of `b` from the accumulated forces,
    then integrates the acceleration over `dt` to calculate the velocity of `b`.
*/
void prIntegrateForBodyVelocity(prBody *b, float dt) {
    if (b == NULL || b->mtn.inverseMass <= 0.0f || dt <= 0.0f) return;

    b->mtn.velocity = prVector2Add(
        b->mtn.velocity,
        prVector2ScalarMultiply(b->mtn.force, b->mtn.inverseMass * dt)
    );

    b->mtn.angularVelocity += (b->mtn.torque * b->mtn.inverseInertia) * dt;
}

/* Integrates the velocity of `b` over `dt` to calculate the position of `b`. */
void prIntegrateForBodyPosition(prBody *b, float dt) {
    if (b == NULL || b->type == PR_BODY_STATIC || dt <= 0.0f) return;

    b->tx.position.x += b->mtn.velocity.x * dt;
    b->tx.position.y += b->mtn.velocity.y * dt;

    prSetBodyAngle(b, b->tx.angle + (b->mtn.angularVelocity * dt));

    b->aabb = prGetShapeAABB(b->shape, b->tx);
}

/* Resolves the collision between `b1` and `b2`. */
void frResolveCollision(prBody *b1, prBody *b2, prCollision *ctx, float inverseDt) {
    if (b1 == NULL || b2 == NULL || ctx == NULL) return;

    if (b1->mtn.inverseMass + b2->mtn.inverseMass <= 0.0f) {
        if (prGetBodyType(b1) == PR_BODY_STATIC) 
            b1->mtn.velocity.x = b1->mtn.velocity.y = b1->mtn.angularVelocity = 0.0f;

        if (prGetBodyType(b2) == PR_BODY_STATIC)
            b2->mtn.velocity.x = b2->mtn.velocity.y = b2->mtn.angularVelocity = 0.0f;

        return;
    }

    for (int i = 0; i < ctx->count; i++) {
        const prVector2 contactPoint = ctx->contacts[i].point;

        prVector2 relPosition1 = prVector2Subtract(contactPoint, prGetBodyPosition(b1));
        prVector2 relPosition2 = prVector2Subtract(contactPoint, prGetBodyPosition(b2));

        prVector2 relNormal1 = prVector2LeftNormal(relPosition1);
        prVector2 relNormal2 = prVector2LeftNormal(relPosition2);

        prVector2 relVelocity = prVector2Subtract(
            prVector2Add(
                b2->mtn.velocity,
                prVector2ScalarMultiply(relNormal2, b2->mtn.angularVelocity)
            ),
            prVector2Add(
                b1->mtn.velocity, 
                prVector2ScalarMultiply(relNormal1, b1->mtn.angularVelocity)
            )
        );

        float relVelocityDot = prVector2Dot(relVelocity, ctx->direction);

        if (relVelocityDot > 0.0f) continue;

        float relPositionCross1 = prVector2Cross(relPosition1, ctx->direction);
        float relPositionCross2 = prVector2Cross(relPosition2, ctx->direction);

        const float normalMass = (b1->mtn.inverseMass + b2->mtn.inverseMass)
            + b1->mtn.inverseInertia * (relPositionCross1 * relPositionCross1)
            + b2->mtn.inverseInertia * (relPositionCross2 * relPositionCross2);

        const float biasScalar = -(PR_WORLD_BAUMGARTE_FACTOR * inverseDt)
            * fminf(0.0f, -ctx->contacts[i].depth + PR_WORLD_BAUMGARTE_SLOP);

        float normalScalar = ((-(1.0f + ctx->restitution) * relVelocityDot) + biasScalar)
            / normalMass;

        {
            // TODO: ...
            ctx->contacts[i].cache.normalScalar = normalScalar;
        }

        prVector2 normalImpulse = prVector2ScalarMultiply(ctx->direction, normalScalar);

        // TODO: ...
        prApplyImpulseToBody(b1, relPosition1, prVector2Negate(normalImpulse));
        prApplyImpulseToBody(b2, relPosition2, normalImpulse);

        relVelocity = prVector2Subtract(
            prVector2Add(
                b2->mtn.velocity,
                prVector2ScalarMultiply(relNormal2, b2->mtn.angularVelocity)
            ),
            prVector2Add(
                b1->mtn.velocity, 
                prVector2ScalarMultiply(relNormal1, b1->mtn.angularVelocity)
            )
        );

        relVelocityDot = prVector2Dot(relVelocity, ctx->direction);

        prVector2 tangent = { .x = ctx->direction.y, .y = -ctx->direction.x };

        relPositionCross1 = prVector2Cross(relPosition1, tangent);
        relPositionCross2 = prVector2Cross(relPosition2, tangent);

        float tangentMass = (b1->mtn.inverseMass + b2->mtn.inverseMass)
            + b1->mtn.inverseInertia * (relPositionCross1 * relPositionCross1)
            + b2->mtn.inverseInertia * (relPositionCross2 * relPositionCross2);

        float tangentScalar = -prVector2Dot(relVelocity, tangent) / tangentMass;

        const float maxTangentScalar = ctx->friction * normalScalar;

        tangentScalar = fminf(fmaxf(tangentScalar, -maxTangentScalar), maxTangentScalar);

        {
            // TODO: ...
            ctx->contacts[i].cache.tangentScalar = tangentScalar;
        }

        prVector2 tangentImpulse = prVector2ScalarMultiply(tangent, tangentScalar);

        // TODO: ...
        frApplyImpulseToBody(b1, relPosition1, frVector2Negate(tangentImpulse));
        frApplyImpulseToBody(b2, relPosition2, tangentImpulse);
    }
}

/* Private Functions ==================================================================== */

/* Computes the mass and the moment of inertia for `b`. */
static void prComputeBodyMass(prBody *b) {
    b->mtn.mass = b->mtn.inverseMass = 0.0f;
    b->mtn.inertia = b->mtn.inverseInertia = 0.0f;

    switch (b->type) {
        case PR_BODY_STATIC:
            b->mtn.velocity.x = b->mtn.velocity.y = b->mtn.angularVelocity = 0.0f;

            break;

        case PR_BODY_DYNAMIC:
            if (!(b->flags & PR_FLAG_INFINITE_MASS)) {
                b->mtn.mass = prGetShapeMass(b->shape);

                if (b->mtn.mass > 0.0f) b->mtn.inverseMass = 1.0f / b->mtn.mass;
            }

            if (!(b->flags & PR_FLAG_INFINITE_INERTIA)) {
                b->mtn.inertia = prGetShapeInertia(b->shape);

                if (b->mtn.inertia > 0.0f) b->mtn.inverseInertia = 1.0f / b->mtn.inertia;
            }

            break;

        default:
            break;
    }
}

/* Normalizes the `angle` to a range `[0, 2π]`. */
static PR_API_INLINE float prNormalizeAngle(float angle) {
    return angle - (TWO_PI * floorf(angle * INVERSE_TWO_PI));
}