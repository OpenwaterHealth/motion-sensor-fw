/*
 * histo_fake.h
 *
 *  Created on: Jul 17, 2025
 *      Author: gvigelet
 */

#ifndef INC_HISTO_FAKE_H_
#define INC_HISTO_FAKE_H_

#pragma once

#include <stdint.h>
#include "usbd_def.h"

#define HISTO_FRAME_SIZE_BYTES   (4 + 4 + 1024 * 4 + 4 + 4)   // SOF, FrameID, 1024x32bit, Meta, EOF
#define HISTO_BIN_COUNT          1024
#define HISTO_META_DATA          0x12345678
#define HISTO_SOF_MARKER         0xDEADBEEF
#define HISTO_EOF_MARKER         0xFEEDBEEF

void HistoFake_Init(void);
void HistoFake_GenerateAndSend(USBD_HandleTypeDef *pdev);


#endif /* INC_HISTO_FAKE_H_ */
