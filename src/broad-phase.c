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

#define STB_DS_IMPLEMENTATION
#include "external/stb_ds.h"

#include "proxima.h"

/* Typedefs ============================================================================= */

/* A structure that represents the key of a spatial hash entry. */
typedef struct _prSpatialHashKey { 
    int x, y; 
} prSpatialHashKey;

/* A structure that represents the value of a spatial hash entry. */
typedef int *prSpatialHashValue;

/* A structure that represents the key-value pair of a spatial hash.*/
typedef struct _prSpatialHashEntry { 
    prSpatialHashKey key; 
    prSpatialHashValue value;
} prSpatialHashEntry;

/* A struct that represents a spatial hash. */
struct _prSpatialHash {
    float cellSize, inverseCellSize;
    prSpatialHashValue queryResult;
    prSpatialHashEntry *entries;
};

/* Private Function Prototypes ========================================================== */

/* 
    Returns ​a negative integer value if `a` is less than `b`, ​a positive integer value 
    if `a` is greater than `b` and zero if `a` and `b` are equivalent.
*/
static int prQSortCompare(const void *a, const void *b);

/* Public Functions ===================================================================== */

/* Creates a new spatial hash with the given `cellSize`. */
prSpatialHash *prCreateSpatialHash(float cellSize) {
    if (cellSize <= 0.0f) return NULL;

    // NOTE: `sh->queryResult` and `sh->entries` must be initialized to `NULL`
    prSpatialHash *sh = calloc(1, sizeof *sh);

    sh->cellSize = cellSize;
    sh->inverseCellSize = 1.0f / cellSize;

    return sh;
}

/* Releases the memory allocated by `sh`. */
void prReleaseSpatialHash(prSpatialHash *sh) {
    if (sh == NULL) return;

    for (int i = 0; i < hmlen(sh->entries); i++)
        arrfree(sh->entries[i].value);

    hmfree(sh->entries), arrfree(sh->queryResult), free(sh);
}

/* Erases all elements from `sh`. */
void prClearSpatialHash(prSpatialHash *sh) {
    if (sh == NULL) return;

    arrsetlen(sh->queryResult, 0);

    for (int i = 0; i < hmlen(sh->entries); i++)
        arrsetlen(sh->entries[i].value, 0);
}

/* Inserts a `key`-`value` pair into `sh`. */
void prInsertToSpatialHash(prSpatialHash *sh, prAABB key, int value) {
    if (sh == NULL) return;

    const float inverseCellSize = sh->inverseCellSize;

    const int minX = key.x * inverseCellSize;
    const int minY = key.y * inverseCellSize;

    const int maxX = (key.x + key.width) * inverseCellSize;
    const int maxY = (key.y + key.height) * inverseCellSize; 

    for (int y = minY; y <= maxY; y++)
        for (int x = minX; x <= maxX; x++) {
            const prSpatialHashKey key = { x, y };

            prSpatialHashEntry *entry = hmgetp_null(sh->entries, key);

            if (entry != NULL) {
                arrput(entry->value, value);
            } else {
                prSpatialHashEntry newEntry = { .key = key };

                arrput(newEntry.value, value);

                hmputs(sh->entries, newEntry);
            }
        }
}

/* Query `sh` for any objects that are likely to overlap the given `aabb`. */
void prQuerySpatialHash(prSpatialHash *sh, prAABB aabb, prHashQueryFunc func, void *ctx) {
    if (sh == NULL) return;

    const float inverseCellSize = sh->inverseCellSize;

    const int minX = aabb.x * inverseCellSize;
    const int minY = aabb.y * inverseCellSize;

    const int maxX = (aabb.x + aabb.width) * inverseCellSize;
    const int maxY = (aabb.y + aabb.height) * inverseCellSize;

    arrsetlen(sh->queryResult, 0);

    for (int y = minY; y <= maxY; y++) {
        for (int x = minX; x <= maxX; x++) {
            const prSpatialHashKey key = { x, y };

            const prSpatialHashEntry *entry = hmgetp_null(sh->entries, key);

            if (entry == NULL) continue;

            for (int i = 0; i < arrlen(entry->value); i++)
                arrput(sh->queryResult, entry->value[i]);
        }
    }

    const size_t oldLength = arrlen(sh->queryResult);

    if (oldLength > 1) {
        // NOTE: Sort the array first, then remove duplicates!
        qsort(sh->queryResult, oldLength, sizeof *(sh->queryResult), prQSortCompare);

        size_t newLength = 0;

        for (int i = 0; i < oldLength; i++)
            if (sh->queryResult[i] != sh->queryResult[i + 1])
                sh->queryResult[newLength++] = sh->queryResult[i];
        
        if (sh->queryResult[newLength - 1] != sh->queryResult[oldLength - 1])
            sh->queryResult[newLength++] = sh->queryResult[oldLength - 1];

        arrsetlen(sh->queryResult, newLength);
    }

    /*
        NOTE: For each object in the query result, the callback function will be called
        with the user data pointer `ctx`.
    */
   for (int i = 0; i < arrlen(sh->queryResult); i++)
        func(sh->queryResult[i], ctx);
}

/* Private Functions ==================================================================== */

/* 
    Returns ​a negative integer value if `a` is less than `b`, ​a positive integer value 
    if `a` is greater than `b` and zero if `a` and `b` are equivalent.
*/
static int prQSortCompare(const void *a, const void *b) {
    const int x = *(const int *) a;
    const int y = *(const int *) b;
    
    return (x > y) - (x < y);
}