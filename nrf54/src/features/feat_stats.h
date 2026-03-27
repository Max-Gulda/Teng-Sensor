#ifndef FEAT_STATS_H
#define FEAT_STATS_H

#include <stdint.h>
#include <math.h>

/**
 * @brief Welford online mean/variance accumulator.
 *
 * Numerically stable single-pass algorithm. Use welford_update() per sample,
 * then query mean/variance/stddev at any time. O(1) per update.
 */
typedef struct {
    uint32_t n;
    float    mean;
    float    M2;   /* sum of squared deviations from running mean */
} welford_t;

static inline void welford_reset(welford_t *w)
{
    w->n    = 0;
    w->mean = 0.0f;
    w->M2   = 0.0f;
}

static inline void welford_update(welford_t *w, float x)
{
    w->n++;
    float delta  = x - w->mean;
    w->mean     += delta / (float)w->n;
    float delta2 = x - w->mean;
    w->M2       += delta * delta2;
}

static inline float welford_mean(const welford_t *w)
{
    return w->mean;
}

/* Sample variance (unbiased, n-1 denominator). Returns 0 for n < 2. */
static inline float welford_variance(const welford_t *w)
{
    if (w->n < 2) {
        return 0.0f;
    }
    return w->M2 / (float)(w->n - 1);
}

static inline float welford_stddev(const welford_t *w)
{
    return sqrtf(welford_variance(w));
}

#endif /* FEAT_STATS_H */
