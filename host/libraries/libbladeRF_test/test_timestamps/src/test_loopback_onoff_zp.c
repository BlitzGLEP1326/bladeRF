/*
 * This file is part of the bladeRF project: *   http://www.github.com/nuand/bladeRF
 *
 * Copyright (C) 2014 Nuand LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* This test TX's some On-Off bursts using the timestamp's zero-padding flag
 * and verifies the lenghths of gaps
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <inttypes.h>
#include <pthread.h>
#include <libbladeRF.h>
#include "test_timestamps.h"
#include "loopback.h"
#include "minmax.h"

static inline int fill_bursts(struct loopback_burst_test *t)
{
    bladerf_dev_speed speed;
    uint64_t msg_len; /* In samples */
    uint64_t prng_val, tmp, i;
    FILE *f;
    const char filename[] = "zp-bursts.txt";

    const uint64_t min_duration = 1;
    const uint64_t max_duration = 3 * t->params->buf_size;

    speed = bladerf_device_speed(t->dev);
    switch (speed) {
        case BLADERF_DEVICE_SPEED_HIGH:
            msg_len = 256;
            break;

        case BLADERF_DEVICE_SPEED_SUPER:
            msg_len = 512;
            break;

        default:
            return -1;
    }

    i = 0;
    randval_init(&t->params->prng_state, t->params->prng_seed);

    assert(t->num_bursts >= 10);

    /* Burst 1: half a buffer */
    t->bursts[i].gap = 0;
    t->bursts[i].duration = t->params->buf_size / 2;
    i++;

    /* Burst 2: Gap is 1/2 msg, burst fills remainder of buffer */
    t->bursts[i].gap = msg_len / 2;
    t->bursts[i].duration =
        t->params->buf_size - (t->bursts[i-1].duration + t->bursts[i].gap);
    i++;

    /* Burst 3: Gap is 1.5 msg, burst fills remainder of buffer */
    t->bursts[i].gap = msg_len + msg_len / 2;
    t->bursts[i].duration = t->params->buf_size - t->bursts[i].gap;
    i++;

    /* Burst 4: Gap is entire msg, burst fills remainder of buffer */
    t->bursts[i].gap = msg_len;
    t->bursts[i].duration = t->params->buf_size - t->bursts[i].gap;
    i++;

    /* Burst 5: Gap is entire msg - 1, burst fils remainder of buffer */
    t->bursts[i].gap = msg_len - 1;
    t->bursts[i].duration = t->params->buf_size - t->bursts[i].gap;
    i++;

    /* Burst 6: Gap is 1 sample, burst fills remainder of buffer */
    t->bursts[i].gap = 1;
    t->bursts[i].duration = t->params->buf_size - t->bursts[i].gap;
    i++;

    /* Burst 7: Gap is partial message, burst fills rest of message */
    t->bursts[i].gap = msg_len / 4 + 1;
    t->bursts[i].duration = msg_len - t->bursts[i].gap;
    i++;

    /* Burst 8:
     *
     * Gap spans multiple messages worth of time.
     *
     * Burst fills multiple messages and fills a partial message.
     *
     * Since we're message aligned, there should be no zero-stuffing under the
     * hood here; the sync i/f should just use the advanced timestamp
     **/
    t->bursts[i].gap = 3 * msg_len;
    t->bursts[i].duration = 5 * msg_len + msg_len / 3 + 1;
    i++;

    /* Burst 9:
     * Gap spans a buffer's worth of time.
     *
     * Again, this shouldn't actually zero a buffer -- just the remainder of the
     * current message.
     *
     */
    t->bursts[i].gap = t->params->buf_size + t->params->buf_size / 2;
    t->bursts[i].duration = 3 * msg_len + msg_len / 4;
    i++;

    /* Burst 10: Small gap, mult-buffer duration */
    t->bursts[i].gap = 2;
    t->bursts[i].duration = 4 * t->params->buf_size - 128;
    i++;

    f = fopen(filename, "w");
    if (f == NULL) {
        perror("fopen");
        return -1;
    } else {
        uint64_t j;

        printf("Burst descriptions written to %s.\n", filename);

        for (j = 0; j < i; j++) {
            fprintf(f, "Burst #%-4"PRIu64
                    " gap=%-8"PRIu64
                    " duration=%-8"PRIu64"\n",
                    j + 1, t->bursts[j].gap, t->bursts[j].duration);
        }
    }

    /* Fill the remaining bursts with pseudo-random values */
    while (i < t->num_bursts) {
        prng_val = t->params->prng_state;

        randval_update(&t->params->prng_state);
        tmp = t->params->prng_state % (max_duration - min_duration + 1);
        t->bursts[i].duration = tmp + min_duration;

        randval_update(&t->params->prng_state);
        tmp = t->params->prng_state % (max_duration - min_duration + 1);
        t->bursts[i].gap = tmp + min_duration;

        fprintf(f, "Burst #%-4"PRIu64
                   " gap=%-8"PRIu64
                   " duration=%-8"PRIu64
                   " prng=0x%016"PRIx64"\n",
                i + 1, t->bursts[i].gap, t->bursts[i].duration, prng_val);
        i++;
    }

    fclose(f);
    return 0;
}

static inline int tx_samples(struct loopback_burst_test *t,
                             int16_t *samples, struct bladerf_metadata *meta,
                             uint64_t count, unsigned int burst_num)
{
    int status = 0;
    unsigned int to_send;

    assert(count <= UINT_MAX);

    while (count != 0 && status == 0) {
        to_send = uint_min(t->params->buf_size, (unsigned int) count);

        status = bladerf_sync_tx(t->dev, samples, to_send, meta,
                                 t->params->timeout_ms);

        if (status != 0) {
            fprintf(stderr, "Failed to TX @ burst %-4u with %"PRIu64
                    " samples left: %s\n",
                    burst_num + 1, count, bladerf_strerror(status));

            return status;
        }

        count -= to_send;
        meta->flags = 0;
    }

    return 0;
}

static void * tx_task(void *args)
{
    int status;
    int16_t *samples;
    unsigned int i;
    struct bladerf_metadata meta;
    struct loopback_burst_test *t = (struct loopback_burst_test *) args;
    bool stop = false;
    int16_t zeros[] = { 0, 0, 0, 0 };

    memset(&meta, 0, sizeof(meta));

    samples = alloc_loopback_samples(t->params->buf_size);
    if (samples == NULL) {
        return NULL;
    }

    status = bladerf_get_timestamp(t->dev, BLADERF_MODULE_TX, &meta.timestamp);
    if (status != 0) {
        fprintf(stderr, "Failed to get current timestamp: %s\n",
                bladerf_strerror(status));
        goto out;
    }

    meta.flags = BLADERF_META_FLAG_TX_BURST_START;
    meta.timestamp += 400000;

    for (i = 0; i < t->num_bursts && !stop; i++) {
        status = tx_samples(t, samples, &meta, t->bursts[i].duration, i);

        if (status != 0) {
            pthread_mutex_lock(&t->lock);
            stop = t->stop = true;
            pthread_mutex_unlock(&t->lock);
        } else {
            meta.flags = BLADERF_META_FLAG_TX_ZERO_PAD;

            meta.timestamp += t->bursts[i].duration;
            if (i != (t->num_bursts - 1)) {
                meta.timestamp += t->bursts[i+1].gap;
            }

            pthread_mutex_lock(&t->lock);
            stop = t->stop;
            pthread_mutex_unlock(&t->lock);
        }
    }

    /* Flush TX samples by ensuring we have 2 zero samples at the end
     * of our burst (as required by libbladeRF) */
    if (status == 0) {
        meta.flags = BLADERF_META_FLAG_TX_BURST_END;
        status = bladerf_sync_tx(t->dev, zeros, 2, &meta,
                                 t->params->timeout_ms);

        if (status != 0) {
            fprintf(stderr, "Failed to flush TX: %s\n",
                    bladerf_strerror(status));

        }
    }

    /* Wait for samples to finish */
    printf("TX: Waiting for samples to finish.\n");
    fflush(stdout);
    status = wait_for_timestamp(t->dev, BLADERF_MODULE_TX,
                                meta.timestamp + t->bursts[i - 1].duration,
                                3000);

    if (status != 0) {
        fprintf(stderr, "Failed to wait for TX to complete: %s\n",
                bladerf_strerror(status));
    }


out:
    free(samples);

    printf("TX: Exiting task.\n");
    fflush(stdout);
    return NULL;
}

int test_fn_loopback_onoff_zp(struct bladerf *dev, struct app_params *p)
{
    int status = 0;
    struct loopback_burst_test test;
    pthread_t tx_thread;
    bool tx_started = false;

    pthread_t rx_thread;
    bool rx_started = false;
    bool rx_ready = false;

    test.dev = dev;
    test.params = p;
    test.num_bursts = 1000;
    test.stop = false;
    test.rx_ready = false;

    pthread_mutex_init(&test.lock, NULL);

    test.bursts = (struct loopback_burst *) malloc(test.num_bursts * sizeof(test.bursts[0]));
    if (test.bursts == NULL) {
        perror("malloc");
        return -1;
    } else {
        fill_bursts(&test);
    }

    status = setup_device_loopback(&test);
    if (status != 0) {
        goto out;
    }

    printf("Starting bursts...\n");

    status = pthread_create(&rx_thread, NULL, loopback_burst_rx_task, &test);
    if (status != 0) {
        fprintf(stderr, "Failed to start RX thread: %s\n", strerror(status));
        goto out;
    } else {
        rx_started = true;
    }

    while (!rx_ready) {
        usleep(10000);
        pthread_mutex_lock(&test.lock);
        rx_ready = test.rx_ready;
        pthread_mutex_unlock(&test.lock);
    }

    status = pthread_create(&tx_thread, NULL, tx_task, &test);
    if (status != 0) {
        fprintf(stderr, "Failed to start TX thread: %s\n", strerror(status));
        goto out;
    } else {
        tx_started = true;
    }

out:
    if (tx_started) {
        pthread_join(tx_thread, NULL);
    }

    if (rx_started) {
        pthread_join(rx_thread, NULL);
    }

    free(test.bursts);

    bladerf_enable_module(dev, BLADERF_MODULE_RX, false);
    bladerf_enable_module(dev, BLADERF_MODULE_TX, false);
    bladerf_set_loopback(dev, BLADERF_LB_NONE);

    return status;
}
