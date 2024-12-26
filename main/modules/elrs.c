#include "elrs.h"
#include <stdint.h>
#include "driver/uart.h"
#include "esp_log.h"

#define TAG "ELRS"

#define CRSF_CRC_POLY 0xd5
#define CRSF_CRC_COMMAND_POLY 0xBA

// uint8_t current_id = -1;

uint8_t get_crc8(uint8_t *buf, size_t size, uint8_t poly) {
    uint8_t crc8 = 0x00;

    for (int i = 0; i < size; i++) {
        crc8 ^= buf[i];
        for (int j = 0; j < 8; j++) {
            if (crc8 & 0x80) {
                crc8 <<= 1;
                crc8 ^= poly;
            } else {
                crc8 <<= 1;
            }
        }
    }
    return crc8;
}

void elrs_send_data(const int port, const uint8_t *data, size_t len) {
    // printf("send data %.2x \n", data[0]);
    if (uart_write_bytes(port, data, len) != len) {
        ESP_LOGE(TAG, "Send data critical failure.");
    }
}

// Function to pack CRSF channels into bytes
void pack_crsf_to_bytes(uint16_t *channels, uint8_t *result) {
    size_t result_idx = 0;
    uint32_t newVal = 0;
    int destShift = 0;

    for (int ch_idx = 0; ch_idx < 16; ch_idx++) {
        // Validate channel value (0-1984)
        if (channels[ch_idx] > MAX_CHANNEL_VALUE) {
            // Handle error - maybe set to max or min value
            channels[ch_idx] = channels[ch_idx] > MAX_CHANNEL_VALUE ? MAX_CHANNEL_VALUE : 0;
        }

        // Put the low bits in any remaining dest capacity
        newVal |= (channels[ch_idx] << destShift) & 0xff;
        result[result_idx++] = newVal & 0xff;

        // Shift the high bits down and place them into the next dest byte
        int srcBitsLeft = 11 - 8 + destShift;
        newVal = channels[ch_idx] >> (11 - srcBitsLeft);

        // When there's at least a full byte remaining, consume that as well
        if (srcBitsLeft >= 8) {
            result[result_idx++] = newVal & 0xff;
            newVal >>= 8;
            srcBitsLeft -= 8;
        }

        // Next dest should be shifted up by the bits consumed
        destShift = srcBitsLeft;
    }
}

void create_crsf_channels_packet(uint16_t *channels, uint8_t *packet) {

    // Validate input
    if (channels == NULL || packet == NULL) {
        return;
    }

    // Packet structure:
    // [0] - Sync byte
    // [1] - Packet length
    // [2] - Packet type
    // [3-24] - Channel data (22 bytes)
    // [25] - CRC8
    packet[0] = DEVICE_ADDRESS_FLIGHT_CONTROLLER;
    packet[1] = 24;  // Total packet length
    packet[2] = FRAME_TYPE_RC_CHANNELS;

    // Pack channels into bytes starting from index 3
    pack_crsf_to_bytes(channels, &packet[3]);

    // Calculate CRC8 on the packet type and channel data
    packet[25] = get_crc8(&packet[2], 23, CRSF_CRC_POLY);
}

// Model switch
void create_model_switch_packet(uint8_t id, uint8_t *packet) {
    // if (current_id == id) return;
    // current_id = id;

    packet[0] = DEVICE_ADDRESS_FLIGHT_CONTROLLER;
    packet[1] = 8;
    packet[2] = FRAME_TYPE_DIRECT_COMMANDS;
    packet[3] = DEVICE_ADDRESS_TX_MODULE;
    packet[4] = DEVICE_ADDRESS_REMOTE_CONTROL;
    packet[5] = COMMAND_CROSSFIRE;
    packet[6] = CROSSFIRE_COMMAND_MODEL_SELECT;
    packet[7] = id;
    packet[8] = get_crc8(&packet[2], 6, CRSF_CRC_COMMAND_POLY);
    packet[9] = get_crc8(&packet[2], 7, CRSF_CRC_POLY);
}