#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/distance_protocol.h"

static unsigned checks;
#define CHECK(e) do { ++checks; if (!(e)) { printf("FAIL line %u: %s\n", (unsigned)__LINE__, #e); exit(1); } } while (0)
static const uint8_t sample[11] = {0x7B,0x03,0xE8,0x07,0xD0,0x0B,0xB8,0x0F,0xA0,0x5B,0x7D};
static const uint8_t mixed[11] = {0x7B,0x00,0x00,0xFF,0xFF,0x80,0x01,0x7B,0x7D,0xFC,0x7D};
static const uint8_t zero[11] = {0x7B,0,0,0,0,0,0,0,0,0x7B,0x7D};

static unsigned feed(DistanceProtocol_Receiver *rx, const uint8_t *bytes, unsigned count)
{
    unsigned completed = 0;
    for (unsigned i = 0; i < count; ++i) {
        completed += DistanceProtocol_Push(rx, bytes[i]);
        CHECK(rx->length < DISTANCE_FRAME_SIZE);
    }
    return completed;
}
static void check_sample(DistanceProtocol_Receiver *rx)
{
    CHECK(rx->data.received);
    CHECK(rx->data.distance_mm[0] == 1000 && rx->data.distance_mm[1] == 2000);
    CHECK(rx->data.distance_mm[2] == 3000 && rx->data.distance_mm[3] == 4000);
}
int main(void)
{
    DistanceProtocol_Receiver rx = {0};
    uint8_t corrupt[11];
    CHECK(!rx.data.received && rx.data.frame_count == 0);
    CHECK(feed(&rx, (const uint8_t *)"noise", 5) == 0);
    CHECK(feed(&rx, sample, 10) == 0 && !rx.data.received);
    CHECK(feed(&rx, &sample[10], 1) == 1);
    check_sample(&rx); CHECK(rx.data.frame_count == 1);
    CHECK(feed(&rx, sample, 11) == 1); /* Back-to-back frames. */
    CHECK(feed(&rx, mixed, 11) == 1);
    CHECK(rx.data.frame_count == 3 && rx.data.distance_mm[0] == 0 && rx.data.distance_mm[1] == 65535U);
    CHECK(rx.data.distance_mm[2] == 0x8001 && rx.data.distance_mm[3] == 0x7B7D);
    CHECK(feed(&rx, zero, 11) == 1 && rx.data.distance_mm[3] == 0);
    CHECK(rx.data.frame_count == 4);

    memset(&rx, 0, sizeof(rx));
    memcpy(corrupt, sample, 11); corrupt[9] ^= 1;
    CHECK(feed(&rx, corrupt, 11) == 0);
    CHECK(rx.checksum_errors == 1 && rx.tail_errors == 0);
    memcpy(corrupt, sample, 11); corrupt[10] ^= 1;
    CHECK(feed(&rx, corrupt, 11) == 0);
    CHECK(rx.checksum_errors == 1 && rx.tail_errors == 1);
    CHECK(feed(&rx, sample, 11) == 1);
    CHECK(rx.checksum_errors == 1 && rx.tail_errors == 1);

    /* Every single-byte corruption must be rejected without overwriting a snapshot. */
    for (unsigned pos = 0; pos < 11; ++pos) {
        memset(&rx, 0, sizeof(rx));
        CHECK(feed(&rx, sample, 11) == 1);
        memcpy(corrupt, sample, 11); corrupt[pos] ^= 1;
        CHECK(feed(&rx, corrupt, 11) == 0);
        CHECK(rx.data.frame_count == 1); check_sample(&rx);
        CHECK(feed(&rx, sample, 11) == 1); CHECK(rx.data.frame_count == 2);
    }
    /* An incomplete old frame may consume part of the next frame; retain its header. */
    for (unsigned prefix = 1; prefix < 11; ++prefix) {
        memset(&rx, 0, sizeof(rx));
        CHECK(feed(&rx, sample, prefix) == 0);
        CHECK(feed(&rx, sample, 11) == 1);
        check_sample(&rx);
    }
    /* Drop each byte of one frame, then recover on a subsequent complete frame. */
    for (unsigned drop = 0; drop < 11; ++drop) {
        memset(&rx, 0, sizeof(rx));
        for (unsigned i = 0; i < 11; ++i) {
            if (i != drop) CHECK(DistanceProtocol_Push(&rx, sample[i]) == 0);
        }
        CHECK(feed(&rx, sample, 11) == 1); check_sample(&rx);
    }
    memset(&rx, 0, sizeof(rx));
    for (unsigned i = 0; i < 1000; ++i) {
        DistanceProtocol_Push(&rx, 0x7B); CHECK(rx.length < 11);
    }
    CHECK(!rx.data.received);
    CHECK(feed(&rx, sample, 11) == 1); check_sample(&rx);
    CHECK(feed(&rx, mixed, 11) == 1); /* Valid payload contains header and tail bytes. */
    CHECK(rx.data.distance_mm[3] == 0x7B7D);
    printf("PASS: USART3 RX four distances, XOR/tail rejection, split/back-to-back frames and resynchronization (%u checks)\n", checks);
    return 0;
}
