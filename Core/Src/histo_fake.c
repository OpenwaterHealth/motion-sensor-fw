/*
 * histo_fake.c
 *
 *  Created on: Jul 17, 2025
 *      Author: gvigelet
 */


#include "histo_fake.h"
#include "usbd_histo.h"
#include <stdlib.h>
#include <string.h>

static uint32_t frame_id = 0;
static uint8_t histo_frame_buffer[HISTO_FRAME_SIZE_BYTES];

void HistoFake_Init(void)
{
    memset(histo_frame_buffer, 0, sizeof(histo_frame_buffer));
    frame_id = 0;
}

void HistoFake_GenerateAndSend(USBD_HandleTypeDef *pdev)
{
    uint32_t *p32 = (uint32_t *)histo_frame_buffer;

    p32[0] = HISTO_SOF_MARKER;
    p32[1] = frame_id++;

    for (uint32_t i = 0; i < HISTO_BIN_COUNT; i++)
    {
        // Fake histogram data: simple moving ramp to simulate dynamic scene
        p32[2 + i] = (i + frame_id) % 1024;
    }

    p32[2 + HISTO_BIN_COUNT] = HISTO_META_DATA;
    p32[3 + HISTO_BIN_COUNT] = HISTO_EOF_MARKER;

    USBD_HISTO_SetTxBuffer(pdev, histo_frame_buffer, HISTO_FRAME_SIZE_BYTES);
}
