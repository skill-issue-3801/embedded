#include <mfrc522.h>
#include "uart_serial_functions.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c1;

void MFRC522_HardReset() {
	HAL_GPIO_WritePin(MFRC522_RESET_GPIO_Port, MFRC522_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(MFRC522_RESET_GPIO_Port, MFRC522_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
}

void MFRC522_Init(void) {
	//MFRC522_InitPins();
	//TM_SPI_Init(MFRC522_SPI, MFRC522_SPI_PINSPACK);

	MFRC522_HardReset();
	
	MFRC522_WriteRegister(MFRC522_TxModeReg, 0x00);
	MFRC522_WriteRegister(MFRC522_RxModeReg, 0x00);

	MFRC522_WriteRegister(MFRC522_ModWidthReg, 0x26);

	MFRC522_WriteRegister(MFRC522_TModeReg, 0x80);
	MFRC522_WriteRegister(MFRC522_TPrescalerReg, 0xA9);

	MFRC522_WriteRegister(MFRC522_TReloadReg_H, 0x03);
	MFRC522_WriteRegister(MFRC522_TReloadReg_L, 0xE8);
	MFRC522_WriteRegister(MFRC522_TxASKReg, 0x40);
	MFRC522_WriteRegister(MFRC522_TxModeReg, 0x3D);

	MFRC522_WriteRegister(MFRC522_DivIEnReg, 0x80);
	MFRC522_WriteRegister(MFRC522_DivIrqReg, 0x80);
	MFRC522_WriteRegister(MFRC522_ComIEnReg, 0x2D);

	MFRC522_FlushBuffer();

	MFRC522_AntennaOn();
}

void MFRC522_WriteRegister(uint8_t addr, uint8_t val) {

	uint8_t buf[2];
	memset(buf, 0, sizeof(buf));

	buf[0] = addr;
	buf[1] = val;

	HAL_I2C_Master_Transmit(&hi2c1, MFRC522_WRITE_ADDR, buf, 2, HAL_MAX_DELAY);
}

uint8_t MFRC522_ReadRegister(uint8_t addr) {
	uint8_t val;
	uint8_t buf[1];
	memset(buf, 0, 2);

	buf[0] = addr;

	HAL_I2C_Master_Transmit(&hi2c1, MFRC522_WRITE_ADDR, buf, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, MFRC522_READ_ADDR, &val, 1, HAL_MAX_DELAY);
	return val;	
}


void MFRC522_AntennaOn() {
	uint8_t temp;

	temp = MFRC522_ReadRegister(MFRC522_TxControlReg);
	if ((temp & 0x03) != 0x03) {
		MFRC522_SetBitMask(MFRC522_TxControlReg, 0x03);
	}
}

uint16_t MFRC522_ReadBuffer(uint8_t *data, uint16_t len) {
	uint16_t ulen;
	ulen = MFRC522_ReadRegister(MFRC522_FIFOLevelReg);
    if (ulen > len) {
        return 0;
    }
	ulen &= 0x3F;
	for (uint16_t i = 0; i < ulen; i++) {
		data[i] = MFRC522_ReadRegister(MFRC522_FIFODataReg);
	}
	return ulen;
}

void MFRC522_version_dump() {
	uart_printf("PCID Reader ");
	uint8_t version = MFRC522_ReadRegister(MFRC522_VersionReg);
	uart_printf("Version: 0x%X ", version);
	switch (version) {
	case (0x88):
			uart_printf("(Clone)\r\n");
			break;
	case (0x90):
			uart_printf("(V0.0)\r\n");
			break;
	case (0x91):
			uart_printf("(V1.0)\r\n");
			break;
	case (0x12):
			uart_printf("(Counterfeit Chip)\r\n");
			break;
	default:
			uart_printf("(Unknown)\r\n");
			break;
	}
}


PICC_STATUS MFRC522_CalculateCRC(uint8_t*  inputData, uint8_t inLen, uint8_t* CRCBytes) {
	uint32_t startTime, currentTime, timeout;
	uint8_t n;


	MFRC522_DoCmd(MFRC522_CMD_IDLE);

	MFRC522_ClearBitMask(MFRC522_DivIrqReg, 0x04);

	MFRC522_FlushBuffer();

	MFRC522_WriteBuffer(inputData, inLen);

	MFRC522_DoCmd(MFRC522_CMD_CALCCRC);


    startTime = HAL_GetTick();
    currentTime = HAL_GetTick();
    timeout = 90; // miliseconds

    do {
    	n = MFRC522_ReadRegister(MFRC522_DivIrqReg);
    	if (n & 0x04) {
    		//Read CRC calculation result
    		CRCBytes[0] = MFRC522_ReadRegister(MFRC522_CRCResultReg_L);
    		CRCBytes[1] = MFRC522_ReadRegister(MFRC522_CRCResultReg_H);
    		return PICC_STATUS_OK;
    	}
    } while ((currentTime - startTime) < timeout);
	return PICC_STATUS_TIMEOUT;
}

void MFRC522_SelfTest() {
	uint8_t zeroData[25];
	uint8_t data[64];
	memset(data, 0, 64);

	MFRC522_HardReset();

	memset(zeroData, 0, 25);
	MFRC522_WriteMem(zeroData);

	MFRC522_WriteRegister(MFRC522_AutoTestReg, 0x09);
	MFRC522_WriteRegister(MFRC522_FIFODataReg, 0x00);

	/* Start Self Test CRC Calculations */
	MFRC522_DoCmd(MFRC522_CMD_CALCCRC);

	/* Wait until we have 64 bytes to read from our FIFO */
	uint8_t n;
	for (uint8_t i = 0; i < 0xFF; i++) {
		n = MFRC522_GetFIFOLevel();
		if (n >= 64) {
			break;
		}
	}

	/* Stop Self Test CRC Calculations */
	MFRC522_DoCmd(MFRC522_CMD_IDLE);

	for (uint16_t i = 0; i < 64; i++) {
		data[i] = MFRC522_ReadRegister(MFRC522_FIFODataReg);
	}
	hex_dump(data, 64);
}


PICC_STATUS MFRC522_CommunicateWithTag(MFRC522_CMD cmd, uint8_t waitIRq, uint8_t *sendData,
									uint8_t sendLen, uint8_t *backData, uint8_t *backLen,
									uint8_t *validBits, uint8_t rxAlign, bool checkCRC) {

    uint8_t txLastBits, bitFraming, len, errorRegVal, iqrFlags;
    uint8_t valid = 0;
    uint32_t startTime, currentTime, timeout;
    uint8_t controlBuff[2];
    bool completed;
    PICC_STATUS rc;

    txLastBits = validBits ? *validBits : 0;
    bitFraming = (rxAlign << 4) | txLastBits;


    MFRC522_DoCmd(MFRC522_CMD_IDLE);

    MFRC522_WriteRegister(MFRC522_ComIrqReg, 0x7F);

    MFRC522_FlushBuffer();
    MFRC522_WriteBuffer(sendData, sendLen);
    MFRC522_WriteRegister(MFRC522_BitFramingReg, bitFraming);
    MFRC522_DoCmd(cmd);

    // Start command if it's transceive
    if (cmd == MFRC522_CMD_TRANSCEIVE) {
        MFRC522_SetBitMask(MFRC522_BitFramingReg, 0x80);
    }

    //Use FreeRTOS wrappers when put into FreeRTOS stuff
    startTime = HAL_GetTick();
    currentTime = HAL_GetTick();
    timeout = 1000; // miliseconds

    do {
        iqrFlags = MFRC522_ReadRegister(MFRC522_ComIrqReg);
        if (iqrFlags & waitIRq) {
            completed = true;
            break;
        } else if (iqrFlags & 0x01) {
            return PICC_STATUS_TIMEOUT;
        }
        currentTime = HAL_GetTick();
    } while ((currentTime - startTime) < timeout);

    if (!completed) {
        return PICC_STATUS_TIMEOUT;
    }

    errorRegVal = MFRC522_ReadRegister(MFRC522_ErrorReg);
    if (errorRegVal & 0x13) {
        return PICC_STATUS_ERROR;
    }


    if (backData && backLen) {
        len = MFRC522_ReadBuffer(backData, *backLen);
        if (len == 0) {
            return PICC_STATUS_NO_ROOM;
        }
        *backLen = len;
        valid = MFRC522_ReadRegister(MFRC522_ControlReg) & 0x07;

        *validBits = valid;
    }

    if (errorRegVal & 0x08) {
        return PICC_STATUS_COLLISION;
    }

    if (backData && backLen && checkCRC) {
    	if (*backLen == 1 && valid == 4) {
    		return PICC_STATUS_MIFARE_NACK;
    	}
    	if (*backLen < 2 || valid != 0) {
    		return PICC_STATUS_CRC_WRONG;
    	}
    	rc = MFRC522_CalculateCRC(&backData[0], *backLen - 2, &controlBuff[0]);
    	if (rc != PICC_STATUS_OK) {
    		return rc;
    	}
    	if ((backData[*backLen - 2] != controlBuff[0] ) || (backData[*backLen -1] != controlBuff[1])) {
    		return PICC_STATUS_CRC_WRONG;
    	}

    }

	return PICC_STATUS_OK;
}


PICC_STATUS MFRC522_TransieveData(uint8_t *sendData, size_t sendLen, uint8_t *backData,
							uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, bool checkCRC) {
	uint8_t waitIRq = 0x30; //RxIRq and WaitIRq
	return MFRC522_CommunicateWithTag(MFRC522_CMD_TRANSCEIVE, waitIRq, sendData, sendLen,
										backData, backLen, validBits, rxAlign, checkCRC);
}


PICC_STATUS MFRC522_WupAOrReqA(PICC_CMD cmd, uint8_t *bufferATQA, uint8_t *bufferSize) {
    uint8_t validBits;
    PICC_STATUS rc;

    if (bufferATQA == NULL || *bufferSize < 2) {
        return PICC_STATUS_NO_ROOM;
    }
    MFRC522_ClearCollisions();
    validBits = 7;
    rc = MFRC522_TransieveData(&cmd, 1, bufferATQA, bufferSize, &validBits, 0, false);
    if (rc != PICC_STATUS_OK) {
        return rc;
    }
    if (*bufferSize != 2 || validBits != 0) {
        return PICC_STATUS_ERROR;
    }
    return PICC_STATUS_OK;
}

bool MFRC522_IsNewCardPresent() {
    PICC_STATUS result;
    uint8_t bufferATQA[2];
    uint8_t bufferSize = sizeof(bufferATQA);

    memset(bufferATQA, 0, 2);

    MFRC522_WriteRegister(MFRC522_TxModeReg, 0x00);
    MFRC522_WriteRegister(MFRC522_RxModeReg, 0x00);

    MFRC522_WriteRegister(MFRC522_ModWidthReg, 0x26);

    result = MFRC522_WupAOrReqA(PICC_CMD_REQA, bufferATQA, &bufferSize);
    return (result == PICC_STATUS_OK || result == PICC_STATUS_COLLISION);
}



void MFRC522_buffTest() {
	uint8_t data[64];
	uint8_t data2[64];
	for (int i = 1; i < 64; i++) {
		data[i] = ((6 * i) + 3);
	}
    MFRC522_FlushBuffer();

	MFRC522_WriteBuffer(data, 64);

	for (uint16_t i = 0; i < 64; i++) {
		data2[i] = MFRC522_ReadRegister(MFRC522_FIFODataReg);
	}
	hex_dump(data, 64);
	hex_dump(data2, 64);

}

void MFRC522_Halt(void) {
	uint8_t len;
	uint8_t buff[4];
	uint8_t validBits, rxAlign = 0;

	buff[0] = PICC_CMD_HLTA;
	buff[1] = 0;

	MFRC522_CalculateCRC(buff, 2, &buff[2]);
	MFRC522_TransieveData(buff, 4, buff, &len, &validBits, rxAlign, true);
}

PICC_STATUS MFRC522_Select(Tag *tag, uint8_t *validBits) {
	bool complete, selDone, cascade;
	uint8_t cascadeLevel = 1;
	uint8_t count, checkBit, index, idIndex;
	int8_t curLevKwnBits;
	uint8_t rxAlign, txLastBits, maxBytes, buffUsed;

	uint8_t cmdBuff[9];
	uint8_t rspnsLen;
	uint8_t *rspnsBuff;

	PICC_STATUS rc;


	if (*validBits > 80) {
		return PICC_STATUS_INVALID;
	}

	/* Clear collision flag */
	MFRC522_ClearBitMask(MFRC522_CollReg, 0x80);

	complete = false;
	while (!complete) {
		switch (cascadeLevel){
			case 1:
				cmdBuff[0] = PICC_CMD_SEL_CL1;
				idIndex = 0;
				cascade = validBits && tag->tag_id_len > 4;
				break;
			case 2:
				cmdBuff[0] = PICC_CMD_SEL_CL2;
				idIndex = 3;
				cascade = validBits && tag->tag_id_len > 7;
				break;
			case 3:
				cmdBuff[0] = PICC_CMD_SEL_CL3;
				idIndex = 7;
				cascade = false;
			default:
				return PICC_STATUS_INTERNAL_ERROR;
				break;
		}

		curLevKwnBits = *validBits - (8 * idIndex);
		if (curLevKwnBits < 0) {
			curLevKwnBits = 0;
		}

		index = 2;
		if (cascade) {
			cmdBuff[index++] = PICC_CMD_CT;
		}

		/* Divide by 8, take the ceiling */
		uint8_t bytesToCopy = curLevKwnBits / 8 + (curLevKwnBits % 8 ? 1 : 0);

		if (bytesToCopy) {
			maxBytes = cascade ? 3 : 4; //**
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (int i = 0; i < bytesToCopy; i++) {
				cmdBuff[index++] = tag->tag_id[idIndex + i];
			}
		}

		if (cascade) {
			curLevKwnBits += 8;
		}

		selDone = false;
		while (!selDone) {
			if (curLevKwnBits >= 32) { /* Setup Select Command */
				cmdBuff[1] = 0x70;
				cmdBuff[6] = cmdBuff[2] ^ cmdBuff[3] ^ cmdBuff[4] ^ cmdBuff[5]; //BCC Calc
				rc = MFRC522_CalculateCRC(cmdBuff, 7, &cmdBuff[7]); //CRC Calc
                if (rc != PICC_STATUS_OK) {
                    return rc;
                }
                txLastBits = 0;
                rxAlign = 0;
                buffUsed = 9;
                rspnsBuff = &cmdBuff[6];
                rspnsLen = 3; //SlAcK response
            }
            else { 			/* Perform ANTICOLLISION to get more bits of ID */
                txLastBits = curLevKwnBits % 8;
                rxAlign = txLastBits; //Send this many bits from last reg, start writing at same offset in same buffer
                count = curLevKwnBits / 8;
                index = 2 + count;
                cmdBuff[1] = (index << 4) | txLastBits;
                buffUsed = index + (txLastBits ? 1 : 0);
                rspnsBuff = &cmdBuff[index];
                rspnsLen = sizeof(cmdBuff) - index;
            }

            MFRC522_WriteRegister(MFRC522_BitFramingReg, (rxAlign << 4) + txLastBits);

            rc = MFRC522_TransieveData(cmdBuff, buffUsed, rspnsBuff, &rspnsLen, &txLastBits, rxAlign, true);
            if (rc == PICC_STATUS_COLLISION) {
            	uint8_t valueOfCollReg = MFRC522_ReadRegister(MFRC522_CollReg);
                if (valueOfCollReg & 0x20) { //Invalid Position of collision
                	return PICC_STATUS_COLLISION;
                }
                uint8_t collisionPos = valueOfCollReg & 0x1F;
                if (collisionPos == 0) {
                    collisionPos = 32;
                }

                if (collisionPos <= curLevKwnBits) {
                    return PICC_STATUS_INTERNAL_ERROR;
                }
                curLevKwnBits = collisionPos;
                checkBit = curLevKwnBits / 8;
                count = curLevKwnBits % 8;
                index = 1 + (curLevKwnBits / 8) + (count ? 1 : 0);
                cmdBuff[index] &= ~(1 << checkBit); //Choose the tag with bit 0

           } else if (rc != PICC_STATUS_OK) {
                    return rc;

           } else {
                if (curLevKwnBits >= 32) {
                    selDone = true;
                } else {
                    curLevKwnBits = 32;
                }
			}
		}

        index = (cmdBuff[2] == PICC_CMD_CT) ? 3 : 2;
        bytesToCopy = (cmdBuff[2] == PICC_CMD_CT) ? 3 : 4;
        for (int i = 0; i < bytesToCopy; i++) {
            tag->tag_id[idIndex + i] = cmdBuff[index++];
        }

        if (rspnsLen != 3 || txLastBits != 0) {
            return PICC_STATUS_ERROR;
        }
        rc = MFRC522_CalculateCRC(rspnsBuff, 1, &cmdBuff[2]);
        if (rc != PICC_STATUS_OK) {
            return rc;
        }
        if ((cmdBuff[2] != rspnsBuff[1] || (cmdBuff[3] != rspnsBuff[2]))) {
            return PICC_STATUS_CRC_WRONG;
        }
        if (rspnsBuff[0] & 0x04) {
            cascadeLevel++;
        } else {
            complete = true;
            tag->tag_slAck = rspnsBuff[0];
        }
	}
	tag->tag_id_len = 3 * cascadeLevel + 1;
	return PICC_STATUS_OK;
}
