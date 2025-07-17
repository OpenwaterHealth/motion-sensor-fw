#include "histo_fake.h"
#include "usbd_histo.h"
#include <string.h>

typedef struct {
    uint8_t frame_buffer[HISTO_FRAME_MAX_BYTES];
    uint32_t frame_size_bytes;
    uint32_t camera_count;
    uint32_t frame_id;
} HistoFakeContext;

static HistoFakeContext histo_ctx;

void HistoFake_Init(uint32_t camera_count)
{
    if (camera_count > HISTO_CAMERA_MAX_COUNT)
        camera_count = HISTO_CAMERA_MAX_COUNT;

    histo_ctx.camera_count = camera_count;
    histo_ctx.frame_id = 0;
    histo_ctx.frame_size_bytes = 8 + camera_count * 4104;  // SOF + per-cam + EOF
    memset(histo_ctx.frame_buffer, 0, HISTO_FRAME_MAX_BYTES);
}

void HistoFake_Deinit(void)
{
    memset(histo_ctx.frame_buffer, 0, HISTO_FRAME_MAX_BYTES);
    histo_ctx.frame_id = 0;
    histo_ctx.camera_count = 0;
    histo_ctx.frame_size_bytes = 0;
}

void HistoFake_GenerateAndSend(USBD_HandleTypeDef *pdev)
{
    uint32_t *p32 = (uint32_t *)histo_ctx.frame_buffer;
    uint32_t offset = 0;

    histo_ctx.frame_id++;
    p32[offset++] = HISTO_SOF_MARKER;

    for (uint32_t cam = 0; cam < histo_ctx.camera_count; cam++)
    {
        // Per-camera unique frame ID
        p32[offset++] = histo_ctx.frame_id;

        // Histogram data (fake ramp with variation per camera)
        for (uint32_t i = 0; i < HISTO_BIN_COUNT; i++)
        {
            p32[offset++] = (i + histo_ctx.frame_id + cam) % 1024;
        }

        // Per-camera metadata (camera ID)
        p32[offset++] = cam;
    }

    p32[offset++] = HISTO_EOF_MARKER;

    USBD_HISTO_SetTxBuffer(pdev, histo_ctx.frame_buffer, histo_ctx.frame_size_bytes);
}
