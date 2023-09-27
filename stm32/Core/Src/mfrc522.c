#include <mfrc522_reg.h>
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
	
	MFRC522_WriteRegister(MFRC522_REG_TX_MODE, 0x00);
	MFRC522_WriteRegister(MFRC522_REG_RX_MODE, 0x00);

	MFRC522_WriteRegister(MFRC522_REG_MOD_WIDTH, 0x26);

	MFRC522_WriteRegister(MFRC522_REG_T_MODE, 0x80);

	MFRC522_WriteRegister(MFRC522_REG_T_PRESCALER, 0xA9);
	MFRC522_WriteRegister(MFRC522_REG_T_RELOAD_H, 0x03);
	MFRC522_WriteRegister(MFRC522_REG_T_RELOAD_L, 0xE8);
	MFRC522_WriteRegister(MFRC522_REG_TX_AUTO, 0x40);
	MFRC522_WriteRegister(MFRC522_REG_TX_MODE, 0x3D);

	MFRC522_WriteRegister(MFRC522_REG_DIV1_EN, 0x80);
	MFRC522_WriteRegister(MFRC522_REG_DIV_IRQ, 0x80);
	MFRC522_WriteRegister(MFRC522_REG_COMM_IE_N, 0x2D);

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

	temp = MFRC522_ReadRegister(MFRC522_REG_TX_CONTROL);
	if ((temp & 0x03) != 0x03) {
		MFRC522_SetBitMask(MFRC522_REG_TX_CONTROL, 0x03);
	}
}

uint16_t MFRC522_ReadBuffer(uint8_t *data, uint16_t len) {
	uint16_t ulen;
	ulen = MFRC522_ReadRegister(MFRC522_REG_FIFO_LEVEL);
    if (ulen > len) {
        return 0;
    }
	ulen &= 0x3F;
	for (uint16_t i = 0; i < ulen; i++) {
		data[i] = MFRC522_ReadRegister(MFRC522_REG_FIFO_DATA);
	}
	return ulen;
}

void MFRC522_version_dump() {
	uart_printf("PCID Reader ");
	uint8_t version = MFRC522_ReadRegister(MFRC522_REG_VERSION);
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


void MFRC522_CalculateCRC(uint8_t*  pIndata, uint8_t len, uint8_t* pOutData) {
	uint32_t startTime, currentTime, timeout;
	uint8_t n;


	MFRC522_DoCmd(PCD_IDLE);
	MFRC522_ClearBitMask(MFRC522_REG_DIV_IRQ, 0x04);
	MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);
	MFRC522_FlushBuffer();
	MFRC522_WriteBuffer(pIndata, len);
	MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_CALCCRC);


    startTime = HAL_GetTick();
    currentTime = HAL_GetTick();
    timeout = 90; // miliseconds

    do {
    	n = MFRC522_ReadRegister(MFRC522_REG_DIV_IRQ);
    	if (n & 0x04) {
    		break;
    	}
    } while ((currentTime - startTime) < timeout);

	//Read CRC calculation result
	pOutData[0] = MFRC522_ReadRegister(MFRC522_REG_CRC_RESULT_L);
	pOutData[1] = MFRC522_ReadRegister(MFRC522_REG_CRC_RESULT_M);
}

uint8_t GlobData[25];
void MFRC522_SelfTest() {
	uint8_t data[64];
	memset(data, 0, 64);

	MFRC522_HardReset();

	memset(GlobData, 0, 25);
	MFRC522_WriteMem(GlobData);

	MFRC522_WriteRegister(MFRC522_REG_AUTO_TEST, 0x09);
	MFRC522_WriteRegister(MFRC522_REG_FIFO_DATA, 0x00);

	/* Start Self Test CRC Calculations */
	MFRC522_DoCmd(PCD_CALCCRC);

	/* Wait until we have 64 bytes to read from our FIFO */
	uint8_t n;
	for (uint8_t i = 0; i < 0xFF; i++) {
		n = MFRC522_GetFIFOLevel();
		if (n >= 64) {
			break;
		}
	}

	/* Stop Self Test CRC Calculations */
	MFRC522_DoCmd(PCD_IDLE);

	for (uint16_t i = 0; i < 64; i++) {
		data[i] = MFRC522_ReadRegister(MFRC522_REG_FIFO_DATA);
	}
	hex_dump(data, 64);
}


PICC_STATUS MFRC522_CommunicateWithTag(PCD_CMD cmd, uint8_t waitIRq, uint8_t *sendData,
									uint8_t sendLen, uint8_t *backData, uint8_t *backLen,
									uint8_t *validBits, uint8_t rxAlign, bool checkCRC) {

    uint8_t txLastBits, bitFraming, len, errorRegVal, iqrFlags;
    uint8_t valid = 0;
    uint32_t startTime, currentTime, timeout;
    bool completed;

    txLastBits = validBits ? *validBits : 0;
    bitFraming = (rxAlign << 4) | txLastBits;


    MFRC522_DoCmd(PCD_IDLE);

    MFRC522_WriteRegister(MFRC522_REG_COMM_IRQ, 0x7F);

    MFRC522_FlushBuffer();
    MFRC522_WriteBuffer(sendData, sendLen);
    MFRC522_WriteRegister(MFRC522_REG_BIT_FRAMING, bitFraming);
    MFRC522_DoCmd(cmd);

    // Start command if it's transceive
    if (cmd == PCD_TRANSCEIVE) {
        MFRC522_SetBitMask(MFRC522_REG_BIT_FRAMING, 0x80);
    }

    //Use FreeRTOS wrappers when put into FreeRTOS stuff
    startTime = HAL_GetTick();
    currentTime = HAL_GetTick();
    timeout = 1000; // miliseconds

    do {
        iqrFlags = MFRC522_ReadRegister(MFRC522_REG_COMM_IRQ);
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

    errorRegVal = MFRC522_ReadRegister(MFRC522_REG_ERROR);
    if (errorRegVal & 0x13) {
        return PICC_STATUS_ERROR;
    }


    if (backData && backLen) {
        len = MFRC522_ReadBuffer(backData, *backLen);
        if (len == 0) {
            return PICC_STATUS_NO_ROOM;
        }
        *backLen = len;
        valid = MFRC522_ReadRegister(MFRC522_REG_CONTROL) & 0x07;

        *validBits = valid;
    }

    if (errorRegVal & 0x08) {
        return PICC_STATUS_COLLISION;
    }

    if (backData && backLen && checkCRC) {
        return PICC_STATUS_CRC_WRONG;
    }

	return PICC_STATUS_OK;
}


PICC_STATUS MFRC522_TransieveData(uint8_t *sendData, size_t sendLen, uint8_t *backData,
							uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, bool checkCRC) {
	uint8_t waitIRq = 0x30; //RxIRq and WaitIRq
	return MFRC522_CommunicateWithTag(PCD_TRANSCEIVE, waitIRq, sendData, sendLen,
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

    MFRC522_WriteRegister(MFRC522_REG_TX_MODE, 0x00);
    MFRC522_WriteRegister(MFRC522_REG_RX_MODE, 0x00);

    MFRC522_WriteRegister(MFRC522_REG_MOD_WIDTH, 0x26);

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
		data2[i] = MFRC522_ReadRegister(MFRC522_REG_FIFO_DATA);
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
