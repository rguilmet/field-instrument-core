#ifndef FIC_RING_BUF_H
#define FIC_RING_BUF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t        *buf;
    size_t          cap;
    volatile size_t head;
    volatile size_t tail;
    uint32_t        drop_count;
} FicByteRing_t;

static inline void fic_ring_buf_init(FicByteRing_t *r, uint8_t *backing_buf, size_t capacity)
{
    if (!r) {
        return;
    }
    r->buf        = backing_buf;
    r->cap        = (backing_buf != NULL && capacity > 1U) ? capacity : 0U;
    r->head       = 0U;
    r->tail       = 0U;
    r->drop_count = 0U;
}

static inline void fic_ring_buf_reset(FicByteRing_t *r)
{
    if (!r) {
        return;
    }
    r->head       = 0U;
    r->tail       = 0U;
    r->drop_count = 0U;
}

static inline bool fic_ring_buf_is_empty(const FicByteRing_t *r)
{
    if (!r || r->cap == 0U) {
        return true;
    }
    return r->head == r->tail;
}

static inline size_t fic_ring_buf_available(const FicByteRing_t *r)
{
    if (!r || r->cap == 0U) {
        return 0U;
    }
    const size_t h = r->head;
    const size_t t = r->tail;
    return (h >= t) ? (h - t) : (r->cap - t + h);
}

static inline size_t fic_ring_buf_free(const FicByteRing_t *r)
{
    if (!r || r->cap <= 1U) {
        return 0U;
    }
    return (r->cap - 1U) - fic_ring_buf_available(r);
}

static inline size_t fic_ring_buf_push(FicByteRing_t *r, const uint8_t *data, size_t len, bool drop_oldest)
{
    if (!r || !r->buf || r->cap <= 1U || !data || len == 0U) {
        return 0U;
    }

    size_t h = r->head;
    size_t t = r->tail;
    const size_t cap = r->cap;
    size_t pushed = 0U;

    for (size_t i = 0U; i < len; i++) {
        size_t next_h = (h + 1U) % cap;
        if (next_h == t) {
            if (!drop_oldest) {
                r->drop_count += (uint32_t)(len - i);
                break;
            }
            t = (t + 1U) % cap;
            r->drop_count++;
        }
        r->buf[h] = data[i];
        h = next_h;
        pushed++;
    }

    r->tail = t;
    r->head = h;
    return pushed;
}

static inline size_t fic_ring_buf_pop(FicByteRing_t *r, uint8_t *out, size_t max_len)
{
    if (!r || !r->buf || r->cap <= 1U || !out || max_len == 0U) {
        return 0U;
    }

    const size_t h = r->head;
    size_t t = r->tail;
    const size_t cap = r->cap;
    size_t popped = 0U;

    while (t != h && popped < max_len) {
        out[popped++] = r->buf[t];
        t = (t + 1U) % cap;
    }

    r->tail = t;
    return popped;
}

#ifdef __cplusplus
}
#endif

#endif // FIC_RING_BUF_H
