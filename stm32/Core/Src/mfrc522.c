#include <mfrc522.h>
#include "uart_serial_functions.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>

#include "cmsis_os.h"
#include "event_manager.h"
#include "event_groups.h"

extern I2C_HandleTypeDef hi2c1;

void MFRC522_HardReset() {
	HAL_GPIO_WritePin(MFRC522_RESET_GPIO_Port, MFRC522_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(MFRC522_RESET_GPIO_Port, MFRC522_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
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
	MFRC522_WriteRegister(MFRC522_ModeReg, 0x3D);

	MFRC522_WriteRegister(MFRC522_DivIEnReg, 0x80);
	MFRC522_WriteRegister(MFRC522_DivIrqReg, 0x80);
	MFRC522_WriteRegister(MFRC522_ComIEnReg, 0x2D);
	MFRC522_SetBitMask(MFRC522_RFCfgReg, (0b111 < 4));

	MFRC522_FlushBuffer();

	MFRC522_AntennaOn();
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
    		MFRC522_DoCmd(MFRC522_CMD_IDLE);
    		CRCBytes[0] = MFRC522_ReadRegister(MFRC522_CRCResultReg_L);
    		CRCBytes[1] = MFRC522_ReadRegister(MFRC522_CRCResultReg_H);
    		return PICC_STATUS_OK;
    	}
    	taskYIELD();
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
									uint8_t *validBits, uint8_t rxAlign, bool checkCRC, bool test) {

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
        taskYIELD();
    } while ((currentTime - startTime) < timeout);

    if (!completed) {
        return PICC_STATUS_TIMEOUT;
    }

    errorRegVal = MFRC522_ReadRegister(MFRC522_ErrorReg);
    if (errorRegVal & 0x13) {
        return PICC_STATUS_ERROR;
    }


    if (backData && backLen) {
        len = MFRC522_ReadBuffer(backData, *backLen); //Not getting CRC bytes from tag
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
										backData, backLen, validBits, rxAlign, checkCRC, false);
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

bool MFRC522_WakeupCards() {
    PICC_STATUS result;
    uint8_t bufferATQA[2];
    uint8_t bufferSize = sizeof(bufferATQA);

    memset(bufferATQA, 0, 2);

    MFRC522_WriteRegister(MFRC522_TxModeReg, 0x00);
    MFRC522_WriteRegister(MFRC522_RxModeReg, 0x00);

    MFRC522_WriteRegister(MFRC522_ModWidthReg, 0x26);

    MFRC522_WupAOrReqA(PICC_CMD_WUPA, bufferATQA, &bufferSize);
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
	uint8_t buff[4];

	buff[0] = PICC_CMD_HLTA;
	buff[1] = 0;

	MFRC522_CalculateCRC(buff, 2, &buff[2]);
	MFRC522_TransieveData(buff, 4, NULL, NULL, NULL, 0, true);
}

PICC_STATUS MFRC522_PICC_AntiCollision(Tag *tag, uint8_t *cmdBuff, int8_t *knownBits) {
	uint8_t txLastBits, rxAlign, buffUsed, index, count, checkBit;
	uint8_t rspnsLen;
	uint8_t *rspnsBuff;

	PICC_STATUS rc;

	txLastBits = *knownBits % 8;
	rxAlign = txLastBits;
	count = *knownBits / 8;
	index = 2 + count;
	cmdBuff[1] = (index << 4) | txLastBits;
	buffUsed = index + (txLastBits ? 1 : 0);
	rspnsBuff = &cmdBuff[index];
	rspnsLen = 9 - index;

	MFRC522_WriteRegister(MFRC522_BitFramingReg, (rxAlign << 4) + txLastBits);

	rc = MFRC522_TransieveData(cmdBuff, buffUsed, rspnsBuff, &rspnsLen, &txLastBits, rxAlign, false);
	if (rc == PICC_STATUS_COLLISION) {
		uint8_t valueOfCollReg = MFRC522_ReadRegister(MFRC522_CollReg);
		if (valueOfCollReg & 0x20) {
			return PICC_STATUS_COLLISION;
		}
		uint8_t collisionPos = valueOfCollReg & 0x1F;
		if (collisionPos == 0) {
			collisionPos = 32;
		}

		if (collisionPos <= *knownBits) {
			return PICC_STATUS_INTERNAL_ERROR;
		}
		*knownBits = collisionPos;
		checkBit = (*knownBits - 1) % 8;
		count = *knownBits % 8;
		index = 1 + (*knownBits / 8) + (count ? 1 : 0);
		cmdBuff[index] &= ~(1 << checkBit); //Choose the tag with bit 0

	} else if (rc != PICC_STATUS_OK) {
		return rc;
	} else {
		*knownBits = 32;
	}
	return PICC_STATUS_OK;
}

PICC_STATUS MFRC522_PICC_Select(Tag *tag, uint8_t cmdBuff[9], uint8_t *idIndex) {
	uint8_t txLastBits = 0;
	uint8_t rspnsLen;
	uint8_t *rspnsBuff;
	uint8_t index, bytesToCopy;
	PICC_STATUS rc;

	rspnsBuff = &cmdBuff[6];
	rspnsLen = 3;

	cmdBuff[1] = 0x70;
	cmdBuff[6] = cmdBuff[2] ^ cmdBuff[3] ^ cmdBuff[4] ^ cmdBuff[5]; //BCC Calc

	rc = MFRC522_CalculateCRC(cmdBuff, 7, &cmdBuff[7]);
	if (rc != PICC_STATUS_OK) {
		return rc;
	}

	MFRC522_WriteRegister(MFRC522_BitFramingReg, 0x00);
	rc = MFRC522_TransieveData(cmdBuff, 9, rspnsBuff, &rspnsLen, &txLastBits, 0, false);
	if (rc != PICC_STATUS_OK) {
		return rc;
	}
	index = (cmdBuff[2] == PICC_CMD_CT) ? 3 : 2;
	bytesToCopy = (cmdBuff[2] == PICC_CMD_CT) ? 3 : 4;
	for (int i = 0; i < bytesToCopy; i++) {
		tag->tag_id[(*idIndex) + i] = cmdBuff[index++];
	}

	if (rspnsLen != 3 || txLastBits != 0) {
		return PICC_STATUS_ERROR;
	}
	rc = MFRC522_CalculateCRC(rspnsBuff, 1, &cmdBuff[7]);
	if (rc != PICC_STATUS_OK) {
		return rc;
	}
	if ((cmdBuff[7] != rspnsBuff[1] || (cmdBuff[8] != rspnsBuff[2]))) {
		return PICC_STATUS_CRC_WRONG;
	}
	return PICC_STATUS_OK;
}

/*
 * NOTE: IF YOU SAY YOU HAVE THE FIRST 7 BYTES OF A 10 BYTE ID, IT WILL INTERPRET IT AS A 7 BYTE ID
 * WORKS BEST WHEN YOU EITHER KNOW NOTHING, OR THE ENTIRE ID.
 */
PICC_STATUS MFRC522_SelectCascade(Tag *tag, PICC_CMD selectLevel, uint8_t *cascadeLevel, bool *complete) {
	uint8_t index, idIndex, maxBytes, bytesToCopy;
	int8_t knownBits;
	bool cascade;

	uint8_t cmdBuff[9];
	uint8_t *validBits = &tag->tag_id_known_bits;

	PICC_STATUS rc;

	memset(cmdBuff, 0, 9);
	cmdBuff[0] = selectLevel;

	idIndex = (*cascadeLevel - 1) * 3;
	knownBits = *validBits - (8 * idIndex);
	if (knownBits < 0) {
		knownBits = 0;
	}

	cascade = (validBits && tag->tag_id_len > (idIndex + 4));

	index = 2;
	if (cascade) {
		cmdBuff[index++] = PICC_CMD_CT;
	}

	bytesToCopy = knownBits / 8 + (knownBits % 8 ? 1 : 0);

	if (bytesToCopy) {
		maxBytes = cascade ? 3 : 4;
		if (bytesToCopy > maxBytes) {
			bytesToCopy = maxBytes;
		}
		for (int i = 0; i < bytesToCopy; i++) {
			cmdBuff[index++] = tag->tag_id[idIndex + i];
		}
	}
	if (cascade) {
		knownBits += 8;
	}

	while (knownBits < 32) {
		rc = MFRC522_PICC_AntiCollision(tag, cmdBuff, &knownBits);
		if (rc != PICC_STATUS_OK) {
			return rc;
		}
	}
	rc = MFRC522_PICC_Select(tag, cmdBuff, &idIndex);
	if (rc != PICC_STATUS_OK) {
		return rc;
	}
	if (cmdBuff[2] == PICC_CMD_CT) {
		*cascadeLevel += 1;
	} else {
		tag->tag_slAck = cmdBuff[6];
		*complete = true;
	}
	return PICC_STATUS_OK;

}

PICC_STATUS MFRC522_SelectStart(Tag *tag) {

	PICC_STATUS rc;

	bool complete;

	uint8_t cascadeLevel = 1;

	if ((tag->tag_id_known_bits > 80) || (tag->tag_id_len > 10)) {
		return PICC_STATUS_INVALID;
	}

	complete = false;
	while (!complete) {
		switch (cascadeLevel){
			case 1:
				rc = MFRC522_SelectCascade(tag, PICC_CMD_SEL_CL1, &cascadeLevel, &complete);
				break;
			case 2:
				rc = MFRC522_SelectCascade(tag, PICC_CMD_SEL_CL2, &cascadeLevel, &complete);
				break;
			case 3:
				rc = MFRC522_SelectCascade(tag, PICC_CMD_SEL_CL3, &cascadeLevel, &complete);
				complete = true;
				break;
			default:
				return PICC_STATUS_INTERNAL_ERROR;
				break;
		}
		if (rc != PICC_STATUS_OK) {
			return rc;
		}
	}
	tag->tag_id_len = (cascadeLevel * 3) + 1;
	tag->tag_id_known_bits = tag->tag_id_len * 8;
	return PICC_STATUS_OK;
}

PICC_STATUS MFRC522_Read(uint8_t blockAddr, uint8_t *readBuf, uint8_t *bufLen) {
	PICC_STATUS rc;
	if (readBuf == NULL || *bufLen < 18) {
		return PICC_STATUS_NO_ROOM;
	}

	readBuf[0] = PICC_CMD_MIFARE_READ;
	readBuf[1] = blockAddr;

	rc = MFRC522_CalculateCRC(readBuf, 2, & readBuf[2]);
	if (rc != PICC_STATUS_OK) {
		return rc;
	}

	return MFRC522_TransieveData(readBuf, 4, readBuf, bufLen, NULL, 0, true);
}

void MFRC522_GetType(Tag tag) {
	uint8_t SlAck = tag.tag_slAck & 0x7F;
	uart_printf("Give tag is of type: \r\n:");
	switch (SlAck) {
		case 0x04:
			uart_printf("Incomplete\r\n");
			break;
		case 0x09:
			uart_printf("MiFare mini\r\n");
			break;
		case 0x08:
			uart_printf("MiFare mini 1K\r\n");
			break;
		case 0x18:
			uart_printf("MiFare mini 4K\r\n");
			break;
		case 0x00:
			uart_printf("MiFare mini UL\r\n");
			break;
		case 0x10:
		case 0x11:
			uart_printf("MiFare mini plus\r\n");
			break;
		case 0x01:
			uart_printf("TNP3XXX\r\n");
			break;
		case 0x20:
			uart_printf("ISO_14443_4\r\n");
			break;
		case 0x40:
			uart_printf("ISO_18092\r\n");
			break;
		default:
			uart_printf("Unknown\r\n");
	}
}

uint8_t user1aid[10] = {0x04, 0x25, 0x45, 0x3A, 0x25, 0x77, 0x80, 0x0, 0x0, 0x0};
uint8_t user1bid[10] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t user1cid[10] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

uint8_t user2aid[10] = {0x04, 0xB9, 0xBf, 0x42, 0xF8, 0x73, 0x80, 0x0, 0x0, 0x0};
uint8_t user2bid[10] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t user2cid[10] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};


uint8_t user3aid[10] = {0x74, 0x01, 0xA1, 0xED, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t user3bid[10] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t user3cid[10] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};


uint8_t user4aid[10] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t user4bid[10] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t user4cid[10] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};


bool PICC_ID_Equal(uint8_t id1[10], uint8_t id2[10]) {
	for (int i = 0; i < 10; i++) {
		if(id1[i] != id2[i]) {
			return false;
		}
	}
	return true;
}

#define TAG_USER1A (1 << 0)
#define TAG_USER1B (1 << 1)
#define TAG_SER1C (1 << 2)
#define TAG_USER2A (1 << 3)
#define TAG_USER2B (1 << 4)
#define TAG_USER2C (1 << 5)
#define TAG_USER3A (1 << 6)
#define TAG_USER3B (1 << 7)
#define TAG_USER3C (1 << 8)
#define TAG_USER4A (1 << 9)
#define TAG_USER4B (1 << 10)
#define TAG_USER4D (1 << 11)
#define TOKEN_USER1 (1 << 12)
#define TOKEN_USER2 (1 << 13)
#define TOKEN_USER3 (1 << 14)
#define TOKEN_USER4 (1 << 15)

Tag tags[5];
void NFCTask(void *argument)
{

  memset(tags, 0, sizeof(Tag)*5);
  for(;;)
  {
	int i = 0;
	int i_left = 0;
	int i_entered = 0;

    uint16_t left;
	uint16_t entered;
	uint16_t current;

	while(MFRC522_IsNewCardPresent()) {
		while(tags[i].present) {
			i++;
			if (!(i < 5)) {
				i = 0;
				break;
			}
		}
		if (MFRC522_SelectStart(&tags[i]) == PICC_STATUS_OK) {

			if (PICC_ID_Equal(tags[i].tag_id, user1aid)) {
				if (!(current & TOKEN_USER1)) {
					trigger_event(EVENT_SELECT_USER1);
				}

			} else if (PICC_ID_Equal(tags[i].tag_id, user2id)) {
				if (!(current & TOKEN_USER1)) {
					current &=
					trigger_event(EVENT_SELECT_USER1);
				}

			} else if (PICC_ID_Equal(tags[i].tag_id, user2id)) {
				if (!(current & TOKEN_USER1)) {
					trigger_event(EVENT_SELECT_USER1);
				}

			} else if (PICC_ID_Equal(tags[i].tag_id, user2id)) {
				trigger_event(EVENT_SELECT_USER2);

			} else if (PICC_ID_Equal(tags[i].tag_id, user2id)) {
				trigger_event(EVENT_SELECT_USER2);

			} else if (PICC_ID_Equal(tags[i].tag_id, user2id)) {
				trigger_event(EVENT_SELECT_USER2);

			} else if (PICC_ID_Equal(tags[i].tag_id, user2id)) {
				trigger_event(EVENT_SELECT_USER2);

			} else if (PICC_ID_Equal(tags[i].tag_id, user2id)) {
				trigger_event(EVENT_SELECT_USER2);

			} else if (PICC_ID_Equal(tags[i].tag_id, user2id)) {
				trigger_event(EVENT_SELECT_USER2);

			} else if (PICC_ID_Equal(tags[i].tag_id, user2id)) {
				trigger_event(EVENT_SELECT_USER2);

			} else if (PICC_ID_Equal(tags[i].tag_id, user3id)) {
				trigger_event(EVENT_SELECT_USER3);

			} else if (PICC_ID_Equal(tags[i].tag_id, user4id)) {
				trigger_event(EVENT_SELECT_USER4);
			}

			tags[i].present = true;
			MFRC522_Halt();
		}
	}

	for (i = 0; i < 5; i ++) {
		if (tags[i].present) {
			MFRC522_WakeupCards();
			if (MFRC522_SelectStart(&tags[i]) == PICC_STATUS_TIMEOUT) {
				tags[i].present = false;

				memset(&tags[i], 0, sizeof(Tag));
			} else {
				MFRC522_Halt();
			}
		}
	}
	vTaskDelay(50);
  }

}
