/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
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

	MFRC522_AntennaOn();		//Open the antenna
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

uint8_t GlobData[25];
void MFRC522_SelfTest() {
	uint8_t data[64];

	MFRC522_Reset();

	memset(GlobData, 0, 25);
	MFRC522_WriteMem(GlobData);

	MFRC522_WriteRegister(MFRC522_REG_AUTO_TEST, 0x09);
	MFRC522_WriteRegister(MFRC522_REG_FIFO_DATA, 0x00);

	/* Start Self Test CRC Calculations */
	MFRC522_DoCmd(PCD_CALCCRC);

	/* Wait until we have 64 bytes to read from our FIFO */
	uint8_t n;
	for (uint8_t i = 0; i < 0xFF; i++) {
		n = MFRC522_ReadRegister(MFRC522_REG_FIFO_LEVEL);
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

    uint8_t txLastBits, bitFraming, n, errorRegVal, valid;
    uint32_t startTime, currentTime, timeout;
    bool completed;

    txLastBits = validBits ? *validBits : 0;
    bitFraming = (rxAlign << 4) + txLastBits;

    MFRC522_DoCmd(PCD_IDLE);

    MFRC522_WriteRegister(MFRC522_REG_COMM_IE_N, 0x77 | 0x80);
    MFRC522_WriteRegister(MFRC522_REG_COMM_IRQ, 0x80);
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
    timeout = 32; // miliseconds

    do {
        n = MFRC522_ReadRegister(MFRC522_REG_COMM_IRQ);
        if (n & waitIRq) {
            completed = true;
            break;
        } else if (n & 0x01) {
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
        n = MFRC522_ReadBuffer(backData, *backLen);
        if (n == 0) {
            return PICC_STATUS_NO_ROOM;
        }
        *backLen = n;
        valid = MFRC522_ReadRegister(MFRC522_REG_CONTROL) & 0x07;

        if (valid) {
            *validBits = valid;
        }
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

    result = MFRC522_WupAOrReqA(PICC_CMD_WUPA, bufferATQA, &bufferSize);
    return (result == PICC_STATUS_OK || result == PICC_STATUS_COLLISION);
}
/*
MFRC522_Status_t MFRC522_Check(uint8_t* id) {
	MFRC522_Status_t status;
	//Find cards, return card type
	status = MFRC522_Request(PICC_REQIDL, id);
	if (status == MI_OK) {
		//Card detected
		//Anti-collision, return card serial number 4 bytes
		status = MFRC522_Anticoll(id);
	}
	MFRC522_Halt();			//Command card into hibernation

	return status;
}
*/

/*
MFRC522_Status_t MFRC522_Compare(uint8_t* CardID, uint8_t* CompareID) {
	uint8_t i;
	for (i = 0; i < 5; i++) {
		if (CardID[i] != CompareID[i]) {
			return MI_ERR;
		}
	}
	return MI_OK;
}
*/
/*
MFRC522_Status_t MFRC522_Request(uint8_t reqMode, uint8_t* TagType) {
	MFRC522_Status_t status;
	uint16_t backBits;			//The received data bits

	MFRC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x07);		//TxLastBists = BitFramingReg[2..0]	???

	TagType[0] = reqMode;
	status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

	if ((status != MI_OK) || (backBits != 0x10)) {    
		status = MI_ERR;
	}

	return status;
}
*/

/*
MFRC522_Status_t MFRC522_Anticoll(uint8_t* serNum) {
	MFRC522_Status_t status;
	uint8_t i;
	uint8_t serNumCheck = 0;
	uint16_t unLen;

	MFRC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x00);		//TxLastBists = BitFramingReg[2..0]

	serNum[0] = PICC_ANTICOLL;
	serNum[1] = 0x20;
	status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

	if (status == MI_OK) {
		//Check card serial number
		for (i = 0; i < 4; i++) {   
			serNumCheck ^= serNum[i];
		}
		if (serNumCheck != serNum[i]) {   
			status = MI_ERR;    
		}
	}
	return status;
} 
*/

/*
void MFRC522_CalculateCRC(uint8_t*  pIndata, uint8_t len, uint8_t* pOutData) {
	uint8_t i, n;

	MFRC522_ClearBitMask(MFRC522_REG_DIV_IRQ, 0x04);			//CRCIrq = 0
	MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);			//Clear the FIFO pointer
	//Write_MFRC522(CommandReg, PCD_IDLE);

	//Writing data to the FIFO	
	for (i = 0; i < len; i++) {   
		MFRC522_WriteRegister(MFRC522_REG_FIFO_DATA, *(pIndata+i));
	}
	MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_CALCCRC);

	//Wait CRC calculation is complete
	i = 0xFF;
	do {
		n = MFRC522_ReadRegister(MFRC522_REG_DIV_IRQ);
		i--;
	} while ((i!=0) && !(n&0x04));			//CRCIrq = 1

	//Read CRC calculation result
	pOutData[0] = MFRC522_ReadRegister(MFRC522_REG_CRC_RESULT_L);
	pOutData[1] = MFRC522_ReadRegister(MFRC522_REG_CRC_RESULT_M);
}
*/

/*
uint8_t MFRC522_SelectTag(uint8_t* serNum) {
	uint8_t i;
	MFRC522_Status_t status;
	uint8_t size;
	uint16_t recvBits;
	uint8_t buffer[9]; 

	buffer[0] = PICC_SElECTTAG;
	buffer[1] = 0x70;
	for (i = 0; i < 5; i++) {
		buffer[i+2] = *(serNum+i);
	}
	MFRC522_CalculateCRC(buffer, 7, &buffer[7]);		//??
	status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

	if ((status == MI_OK) && (recvBits == 0x18)) {   
		size = buffer[0]; 
	} else {   
		size = 0;    
	}

	return size;
}
*/

/*
MFRC522_Status_t MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum) {
	MFRC522_Status_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[12]; 

	//Verify the command block address + sector + password + card serial number
	buff[0] = authMode;
	buff[1] = BlockAddr;
	for (i = 0; i < 6; i++) {    
		buff[i+2] = *(Sectorkey+i);   
	}
	for (i=0; i<4; i++) {    
		buff[i+8] = *(serNum+i);   
	}
	status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

	if ((status != MI_OK) || (!(MFRC522_ReadRegister(MFRC522_REG_STATUS2) & 0x08))) {
		status = MI_ERR;   
	}

	return status;
}
*/

/*
MFRC522_Status_t MFRC522_Read(uint8_t blockAddr, uint8_t* recvData) {
	MFRC522_Status_t status;
	uint16_t unLen;

	recvData[0] = PICC_READ;
	recvData[1] = blockAddr;
	MFRC522_CalculateCRC(recvData,2, &recvData[2]);
	status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

	if ((status != MI_OK) || (unLen != 0x90)) {
		status = MI_ERR;
	}

	return status;
}
*/
/*
MFRC522_Status_t MFRC522_Write(uint8_t blockAddr, uint8_t* writeData) {
	MFRC522_Status_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[18]; 

	buff[0] = PICC_WRITE;
	buff[1] = blockAddr;
	MFRC522_CalculateCRC(buff, 2, &buff[2]);
	status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

	if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {   
		status = MI_ERR;   
	}

	if (status == MI_OK) {
		//Data to the FIFO write 16Byte
		for (i = 0; i < 16; i++) {    
			buff[i] = *(writeData+i);   
		}
		MFRC522_CalculateCRC(buff, 16, &buff[16]);
		status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {   
			status = MI_ERR;   
		}
	}

	return status;
}
*/

/*
void MFRC522_Halt(void) {
	uint16_t unLen;
	uint8_t buff[4];

	buff[0] = PICC_CMD_HLTA;
	buff[1] = 0;
	MFRC522_CalculateCRC(buff, 2, &buff[2]);

	MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}
*/
