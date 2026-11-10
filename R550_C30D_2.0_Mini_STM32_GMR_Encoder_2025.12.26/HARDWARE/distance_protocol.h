#ifndef DISTANCE_PROTOCOL_H
#define DISTANCE_PROTOCOL_H

#include <stdint.h>

#define DISTANCE_FRAME_SIZE       11U
typedef struct {
    uint16_t distance_mm[4];
    uint32_t frame_count;
    uint8_t received; /* At least one complete, verified frame; not a freshness flag. */
} DistanceProtocol_Data;

typedef struct {
    uint8_t frame[DISTANCE_FRAME_SIZE];
    uint8_t length;
    volatile uint32_t checksum_errors;
    volatile uint32_t tail_errors;
    volatile DistanceProtocol_Data data;
} DistanceProtocol_Receiver;

/* Zero-initialize once. Push bytes from the sole USART3 ISR producer.
 * Publish only after checking 7B + eight data bytes + XOR(bytes 0..8) + 7D.
 * All uint16 values (including 0000/FFFF) are preserved exactly as received.
 * On a bad frame, keep any embedded header for resynchronization.
 */
static inline uint8_t DistanceProtocol_Push(DistanceProtocol_Receiver *receiver, uint8_t byte)
{
    uint8_t i, start, checksum = 0;
    if (receiver->length == 0U && byte != 0x7BU) return 0;
    receiver->frame[receiver->length++] = byte;
    if (receiver->length < DISTANCE_FRAME_SIZE) return 0;
    for (i = 0U; i < 9U; ++i) checksum ^= receiver->frame[i];
    if (receiver->frame[10] == 0x7DU && receiver->frame[9] == checksum) {
        for (i = 0U; i < 4U; ++i) {
            receiver->data.distance_mm[i] = (uint16_t)(((uint16_t)receiver->frame[1U + 2U * i] << 8) |
                                                      receiver->frame[2U + 2U * i]);
        }
        ++receiver->data.frame_count;
        receiver->data.received = 1;
        receiver->length = 0;
        return 1;
    }
    if (receiver->frame[9] != checksum) ++receiver->checksum_errors;
    if (receiver->frame[10] != 0x7DU) ++receiver->tail_errors;
    for (start = 1U; start < DISTANCE_FRAME_SIZE; ++start) {
        if (receiver->frame[start] == 0x7BU) break;
    }
    receiver->length = (uint8_t)(DISTANCE_FRAME_SIZE - start);
    for (i = 0U; i < receiver->length; ++i) receiver->frame[i] = receiver->frame[start + i];
    return 0;
}

#endif
