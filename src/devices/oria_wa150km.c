/** @file
    Oria WA150KM temperature sensor decoder.

    Copyright (C) 2025 Jan Niklaas Wechselberg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/*
Oria WA150KM temperature sensor decoder.

The device uses Manchester coding with G.E. Thomas convention.
The data is bit-reflected.

Data layout after decoding:

    0  1  2  3  4  5  6  7  8  9  10 11 12 13
    FF FF FF MM ?? CC DD TT II SS ?? ?? ?? BB

- FF = Preamble: 3 bytes of 0xff
- MM = Message type (unused)
- CC = Channel (upper nibble + 1)
- DD = Device ID
- TT = Temperature decimal (upper nibble)
- II = Temperature integer (BCD)
- SS = Sign bit (bit 4, 1 = negative)
- BB = Fixed value 0x65

Observations currently not affecting implemetation:
- In normal operation, the MSG_TYPE toggles between fa20 and fa28 with every send (interval is ~34 seconds)
- Forced transmissions with the TX button have a MSG_TYPE=fa21
- DEVICE_IDs stay consistent over powercycles
- The devices transmit a "battery low" signal encoded in the byte after the temperature
- Negative temperatures have another single bit set

 */

#include "decoder.h"
#include <math.h>

#define ORIA_WA150KM_BITLEN  227
#define WARMUP_LEN           3  // number of 0xff bytes at start
#define MAX_DEVICES          32  // maximum number of devices to track
#define MAX_TEMP_DELTA       12.0f  // maximum temperature change in °C between readings

// Structure to track state for each device (identified by device_id + channel)
typedef struct {
    uint8_t device_id;
    uint8_t channel;
    float last_temperature;
    int initialized;  // 0 = not initialized, 1 = has valid previous reading
} device_state_t;

// Static array to track device states
static device_state_t device_states[MAX_DEVICES];
static int device_states_initialized = 0;

static int oria_wa150km_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    int r;
    uint8_t *b;

    // Find a valid row (skipping short preamble rows)
    for (r = 0; r < bitbuffer->num_rows; r++) {
        if (bitbuffer->bits_per_row[r] == ORIA_WA150KM_BITLEN) {
            break;
        }
    }
    if (r == bitbuffer->num_rows) {
        decoder_logf(decoder, 2, __func__, "No valid row found with %d bits", ORIA_WA150KM_BITLEN);
        return 0;
    }

    // Check warmup bytes before decoding
    b = bitbuffer->bb[r];
    for (int i = 0; i < WARMUP_LEN; i++) {
        if (b[i] != 0xAA) { // Check for alternating 1/0 pattern before Manchester decoding
            decoder_logf(decoder, 2, __func__, "Warmup byte %d is not 0xaa: %02x", i, b[i]);
            return 0;
        }
    }

    // Check last byte (raw data before Manchester decoding)
    if (b[bitbuffer->bits_per_row[r]/8 - 1] != 0x69) {
        decoder_logf(decoder, 2, __func__, "Last byte is not 0x69: %02x",
                b[bitbuffer->bits_per_row[r]/8 - 1]);
        return 0;
    }

    // Invert the buffer for G.E. Thomas decoding
    bitbuffer_invert(bitbuffer);

    // Manchester decode the row
    bitbuffer_t manchester_buffer = {0};
    bitbuffer_manchester_decode(bitbuffer, r, 0, &manchester_buffer, ORIA_WA150KM_BITLEN);

    // Reflect bits in each byte
    reflect_bytes(manchester_buffer.bb[0], manchester_buffer.bits_per_row[0]/8 + 1);

    b = manchester_buffer.bb[0];

    // Sanity check: verify last byte is 0x65 (fixed value per protocol)
    if (b[13] != 0x65) {
        decoder_logf(decoder, 1, __func__, "Last byte is not 0x65: 0x%02x (might indicate corrupted data)", b[13]);
        return DECODE_FAIL_SANITY;
    }

    // Extract channel (upper nibble + 1)
    uint8_t channel = ((b[5] >> 4) & 0x0F) + 1;

    // Extract device ID
    uint8_t device_id = b[6];

    // Sanity check: validate channel range (1-16)
    if (channel < 1 || channel > 16) {
        decoder_logf(decoder, 1, __func__, "Channel out of range: %d (expected 1-16)", channel);
        return DECODE_FAIL_SANITY;
    }

    // Extract temperature nibbles for validation
    uint8_t temp_decimal_nibble = (b[7] >> 4) & 0x0F;
    uint8_t temp_tens_nibble = (b[8] >> 4) & 0x0F;
    uint8_t temp_ones_nibble = b[8] & 0x0F;

    // Sanity check: validate BCD encoding (each nibble must be 0-9)
    if (temp_decimal_nibble > 9 || temp_tens_nibble > 9 || temp_ones_nibble > 9) {
        decoder_logf(decoder, 1, __func__, "Invalid BCD encoding: decimal=%d tens=%d ones=%d",
                temp_decimal_nibble, temp_tens_nibble, temp_ones_nibble);
        return DECODE_FAIL_SANITY;
    }

    // Extract temperature
    // BCD: Convert each nibble of byte 8 to decimal (tens+ones) and add decimal from byte 7
    float temperature = (temp_tens_nibble * 10 + temp_ones_nibble) + temp_decimal_nibble * 0.1;
    // Check sign byte (bit 4)
    if (b[9] & 0x08) {
        temperature = -temperature;
    }

    // Sanity check: validate temperature range for freezer/fridge thermometer
    // Range: -40°C to 60°C (covers freezer temps down to -40°C and room temp/fridge temps up to 60°C)
    if (temperature < -40.0f || temperature > 60.0f) {
        decoder_logf(decoder, 1, __func__, "Temperature out of reasonable range: %.1f°C (expected -40°C to 60°C)", temperature);
        return DECODE_FAIL_SANITY;
    }

    // Sanity check: reject suspicious device ID patterns that might indicate corrupted data
    // All zeros or all ones are suspicious (though technically possible, they're rare)
    if (device_id == 0x00 || device_id == 0xFF) {
        decoder_logf(decoder, 2, __func__, "Suspicious device ID: 0x%02x (might indicate corrupted data)", device_id);
        // Note: This is a warning level check, we'll still accept it if other checks pass
        // If you want to reject these, change to level 1 and return DECODE_FAIL_SANITY
    }

    // Initialize device states array if not already done
    if (!device_states_initialized) {
        for (int i = 0; i < MAX_DEVICES; i++) {
            device_states[i].initialized = 0;
        }
        device_states_initialized = 1;
    }

    // Find existing device state or an empty slot
    int device_index = -1;
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (device_states[i].initialized) {
            if (device_states[i].device_id == device_id && device_states[i].channel == channel) {
                device_index = i;
                break;
            }
        } else {
            // Found empty slot, use it for new device
            if (device_index == -1) {
                device_index = i;
            }
        }
    }

    // If no slot found (array full), reject to prevent buffer overflow
    if (device_index == -1) {
        decoder_logf(decoder, 1, __func__, "Device state tracking full, cannot track device id=0x%02x channel=%d", device_id, channel);
        return DECODE_FAIL_SANITY;
    }

    // Check if this device has a previous reading
    if (device_states[device_index].initialized) {
        float last_temp = device_states[device_index].last_temperature;
        float delta = fabsf(temperature - last_temp);

        // Reject if temperature change is too large (likely radio reception issue)
        if (delta > MAX_TEMP_DELTA) {
            decoder_logf(decoder, 1, __func__,
                    "Temperature delta too large: %.1f°C -> %.1f°C (delta=%.1f°C, max=%.1f°C), rejecting",
                    last_temp, temperature, delta, MAX_TEMP_DELTA);
            return DECODE_FAIL_SANITY;
        }
    }

    // Update device state with new reading
    device_states[device_index].device_id = device_id;
    device_states[device_index].channel = channel;
    device_states[device_index].last_temperature = temperature;
    device_states[device_index].initialized = 1;

    /* clang-format off */
    data = data_make(
            "model",        "", DATA_STRING, "Oria-WA150KM",
            "id",           "", DATA_INT,    device_id,
            "channel",      "", DATA_INT,    channel,
            "temperature_C",  "", DATA_FORMAT, "%.1f C", DATA_DOUBLE, temperature,
            NULL);
    /* clang-format on */
    decoder_output_data(decoder, data);

    return 1;
}

/*
 * List of fields that may appear in the output
 *
 * Used to determine what fields will be output in what
 * order for this device when using -F csv.
 *
 */
static char const *const output_fields[] = {
        "model",
        "id",
        "channel",
        "temperature_C",
        NULL,
};

/*
 * r_device - registers device/callback. see rtl_433_devices.h
 */
r_device const oria_wa150km = {
        .name        = "Oria WA150KM freezer and fridge thermometer",
        .modulation  = OOK_PULSE_PCM,
        .short_width = 490,
        .long_width  = 490,
        .gap_limit   = 1500,
        .reset_limit = 4000,
        .decode_fn   = &oria_wa150km_decode,
        .disabled    = 0,
        .fields      = output_fields,
};
