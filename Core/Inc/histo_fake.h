#ifndef INC_HISTO_FAKE_H_
#define INC_HISTO_FAKE_H_

#include <stdint.h>
#include "usbd_def.h"

// Markers
#define HISTO_SOF_MARKER         0xDEADBEEF
#define HISTO_EOF_MARKER         0xFEEDBEEF

// Per camera histogram
#define HISTO_BIN_COUNT          1024     // 1024 bins, 32-bit each

// Maximum cameras supported
#define HISTO_CAMERA_MAX_COUNT   16

// Per camera block (Frame ID + Histogram + Camera ID)
#define HISTO_CAMERA_BLOCK_BYTES     (4 + (HISTO_BIN_COUNT * 4) + 4)   // 4 + 4096 + 4 = 4104 bytes
#define HISTO_CAMERA_BLOCK_WORDS     (HISTO_CAMERA_BLOCK_BYTES / 4)

// Max frame size: SOF + (per camera) + EOF
#define HISTO_FRAME_MAX_BYTES    (4 + (HISTO_CAMERA_MAX_COUNT * HISTO_CAMERA_BLOCK_BYTES) + 4)
#define HISTO_FRAME_MAX_WORDS    (HISTO_FRAME_MAX_BYTES / 4)

// API
void HistoFake_Init(uint32_t camera_count);
void HistoFake_Deinit(void);
void HistoFake_GenerateAndSend(USBD_HandleTypeDef *pdev);

#endif /* INC_HISTO_FAKE_H_ */
