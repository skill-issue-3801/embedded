#ifndef _MFRC522_H_
#define _MFRC522_H_

#include <stdint.h>
#include <stdbool.h>

#define MFRC522_ADDR	(0x002C << 1)
#define MFRC522_WRITE_ADDR (0x002C << 1) | 0
#define MFRC522_READ_ADDR (0x002C << 1) | 1

typedef enum {
	PICC_CMD_REQA 				= 0x26,
	PICC_CMD_WUPA 				= 0x52,
	PICC_CMD_CT					= 0x88,
	PICC_CMD_SEL_CL1			= 0x93,
	PICC_CMD_SEL_CL2			= 0x95,
	PICC_CMD_SEL_CL3			= 0x98,
	PICC_CMD_HLTA				= 0x50,
    PICC_CMD_AUTHENT1A      	= 0x60,
    PICC_CMD_AUTHENT1B      	= 0x61,
    PICC_CMD_READ           	= 0x30,
    PICC_CMD_WRITE          	= 0xA0,
    PICC_CMD_DECREMENT      	= 0xC0,
    PICC_CMD_RESTORE       		= 0xC2,
    PICC_CMD_TRANSFER      		= 0xB0,
	PICC_CMD_MIFARE_READ		= 0x30
} PICC_CMD;

typedef enum {
	PICC_STATUS_OK				= 0,
	PICC_STATUS_ERROR			= 1,
	PICC_STATUS_COLLISION		= 2,
	PICC_STATUS_TIMEOUT			= 3,
	PICC_STATUS_NO_ROOM			= 3,
	PICC_STATUS_INTERNAL_ERROR	= 5,
	PICC_STATUS_INVALID			= 6,
	PICC_STATUS_CRC_WRONG		= 7,
	PICC_STATUS_MIFARE_NACK 	= 0xff
} PICC_STATUS;

/* MFRC522 Commands */
typedef enum {
	MFRC522_CMD_IDLE			= 0x00,   //NO action; Cancel the current command
	MFRC522_CMD_MEMSET			= 0x01,   //Write FIFO buffer to internal buffer
	MFRC522_CMD_AUTHENT			= 0x0E,   //Authentication Key
	MFRC522_CMD_RECEIVE			= 0x08,   //Receive Data
	MFRC522_CMD_TRANSMIT		= 0x04,   //Transmit data
	MFRC522_CMD_TRANSCEIVE		= 0x0C,   //Transmit and receive data,
	MFRC522_CMD_RESET			= 0x0F,   //Reset
	MFRC522_CMD_CALCCRC			= 0x03   //CRC Calculate
} MFRC522_CMD;

/*
 * Information about a reader
 */
typedef struct {
	uint16_t reader_i2c_addr;
} MFRC522_Reader;

/*
 * Information about a Tag
 */
typedef struct {
	uint8_t tag_id_len;
	uint8_t tag_id[10];
	uint8_t tag_id_known_bits;
	uint8_t tag_slAck; //select acknowledge
	uint8_t tag_token_id;
	uint8_t tag_num;
	bool present;
} Tag;

typedef enum {
	MFRC522_CommandReg 		= 0x01,
	MFRC522_ComIEnReg 		= 0x02,
	MFRC522_DivIEnReg 		= 0x03,
	MFRC522_ComIrqReg 		= 0x04,
	MFRC522_DivIrqReg 		= 0x05,
	MFRC522_ErrorReg 		= 0x06,
	MFRC522_Status1Reg		= 0x07,
	MFRC522_Status2Reg 		= 0x08,
	MFRC522_FIFODataReg 	= 0x09,
	MFRC522_FIFOLevelReg 	= 0x0A,
	MFRC522_WaterLevelReg 	= 0x0B,
	MFRC522_ControlReg		= 0x0C,
	MFRC522_BitFramingReg 	= 0x0D,
	MFRC522_CollReg 		= 0x0E
} MFRC522_StatusReg;

typedef enum {
	MFRC522_ModeReg 		= 0x11,
	MFRC522_TxModeReg 		= 0x12,
	MFRC522_RxModeReg 		= 0x13,
	MFRC522_TxControlReg 	= 0x14,
	MFRC522_TxASKReg 		= 0x15,
	MFRC522_TxSelReg 		= 0x16,
	MFRC522_RxSelReg 		= 0x17,
	MFRC522_RxThresholdReg 	= 0x18,
	MFRC522_DemodReg 		= 0x19,
	MFRC522_mfTxReg 		= 0x1C,
	MFRC522_mfRxReg 		= 0x1D,
	MFRC522_SerialSpeedReg 	= 0x1F
} MFRC522_CmdReg;

typedef enum {
	MFRC522_CRCResultReg_H 	= 0x21,
	MFRC522_CRCResultReg_L 	= 0x22,
	MFRC522_ModWidthReg 	= 0x24,
	MFRC522_RFCfgReg 		= 0x26,
	MFRC522_GsNReg 			= 0x27,
	MFRC522_CWGsPReg 		= 0x28,
	MFRC522_ModGsPReg 		= 0x29,
	MFRC522_TModeReg 		= 0x2A,
	MFRC522_TPrescalerReg 	= 0x2B,
	MFRC522_TReloadReg_H 	= 0x2C,
	MFRC522_TReloadReg_L 	= 0x2D,
	MFRC522_TCounterReg_H 	= 0x2E,
	MFRC522_TCounterReg_L 	= 0x2F,
} MFRC522_ConfigReg;

typedef enum {
	MFRC522_TestSel1Reg 	= 0x31,
	MFRC522_TestSel2Reg 	= 0x32,
	MFRC522_TestPinEnReg 	= 0x33,
	MFRC522_TestPinValueReg = 0x34,
	MFRC522_TestBusReg 		= 0x35,
	MFRC522_AutoTestReg 	= 0x36,
	MFRC522_VersionReg 		= 0x37,
	MFRC522_TestAnalogTest 	= 0x38,
	MFRC522_TestDAC1Reg 	= 0x39,
	MFRC522_TestDAC2Reg 	= 0x3A,
	MFRC522_TestADCReg 		= 0x3B,
} MFRC522_TestReg;

void MFRC522_Init();
void MFRC522_AntennaOn();
void MFRC522_AntennaOff();
void MFRC522_Reset();
void MFRC522_SelfTest();
void MFRC522_Halt();
void MFRC522_version_dump();
bool MFRC522_IsNewCardPresent();
PICC_STATUS MFRC522_SelectStart(Tag *tag);
PICC_STATUS MFRC522_WupAOrReqA(PICC_CMD cmd, uint8_t *bufferATQA, uint8_t *bufferSize);
PICC_STATUS MFRC522_Read(uint8_t blockAddr, uint8_t *readBuf, uint8_t *bufLen);
void MFRC522_GetType(Tag tag);
bool MFRC522_WakeupCards();

void NFCTask(void *argument);

void MFRC522_buffTest();

void MFRC522_WriteRegister(uint8_t addr, uint8_t val);


#define MFRC522_SetBitMask(reg, mask) \
	do { \
		MFRC522_WriteRegister(reg, MFRC522_ReadRegister(reg) | mask); \
	} while (0)

#define MFRC522_ClearBitMask(reg, mask) \
	do { \
		MFRC522_WriteRegister(reg, MFRC522_ReadRegister(reg) & (~mask)); \
	} while (0)

#define MFRC522_ClearSetBitMask(reg, mask)  \
	do { \
		MFRC522_ClearBitMask(reg, mask); \
		MFRC522_SetBitMask(reg, mask); \
	} while(0)


#define MFRC522_AntennaOff() \
do { \
	MFRC522_ClearBitMask(MFRC522_TxControlReg, 0x03); \
} while(0)

#define MFRC522_Reset() \
do { \
	MFRC522_WriteRegister(MFRC522_CommandReg, MFRC522_CMD_RESET); \
} while(0)

#define MFRC522_DoCmd(cmd) \
do { \
	MFRC522_WriteRegister(MFRC522_CommandReg, cmd); \
} while (0)

#define MFRC522_FlushBuffer() \
do { \
	MFRC522_WriteRegister(MFRC522_FIFOLevelReg, 0x80); \
} while (0)

#define MFRC522_WriteBuffer(data, ulen) \
do { \
	for (uint16_t i = 0; i < ulen; i++) { \
		MFRC522_WriteRegister(MFRC522_FIFODataReg, data[i]); \
	} \
} while (0)

#define MFRC522_WriteMem(data) \
do { \
	MFRC522_FlushBuffer(); \
	MFRC522_WriteBuffer(data, 25); \
	MFRC522_DoCmd(MFRC522_CMD_MEMSET); \
} while(0)

#define MFRC522_PowerDown() \
do { \
	MFRC522_SetBitMask(MFRC522_CommandReg, (1 << 4)); \
} while(0)

#define MFRC522_PowerUp() \
do { \
	MFRC522_ClearBitMask(MFRC522_CommandReg, (1 << 4)); \
	while (MFRC522_ReadRegister(MFRC522_CommandReg) & (1 << 4)); \
} while(0)

#define MFRC522_ClearCollisions() \
do { \
    MFRC522_ClearBitMask(MFRC522_CollReg, 0x80); \
} while(0)

#define MFRC522_GetFIFOLevel() (MFRC522_ReadRegister(MFRC522_FIFOLevelReg) & 0x1F)

#define MFRC522_PrintReg(label, reg) \
do { \
	uint8_t dummy; \
    dummy = MFRC522_ReadRegister(MFRC522_ComIrqReg); \
    uart_printf("%s = 0x%0X\r\n", label, dummy); \
} while(0)

#endif

