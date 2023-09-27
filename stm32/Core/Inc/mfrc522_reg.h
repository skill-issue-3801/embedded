#ifndef _MFRC522_H_
#define _MFRC522_H_

#include <stdint.h>
#include <stdbool.h>

#define MFRC522_ADDR	(0x002C << 1)
#define MFRC522_WRITE_ADDR (0x002C << 1) | 0
#define MFRC522_READ_ADDR (0x002C << 1) | 1

typedef enum {
	PICC_CMD_REQA 		= 0x26,
	PICC_CMD_WUPA 		= 0x52,
	PICC_CMD_CT			= 0x88,
	PICC_CMD_SEL_CL1	= 0x93,
	PICC_CMD_SEL_CL2	= 0x95,
	PICC_CMD_HLTA		= 0x40,
    PICC_CMD_AUTHENT1A      = 0x60,
    PICC_CMD_AUTHENT1B      = 0x61,
    PICC_CMD_READ           = 0x30,
    PICC_CMD_WRITE          = 0xA0,
    PICC_CMD_DECREMENT      = 0xC0,
    PICC__CMD_RESTORE        = 0xC2,
    PICC_CMD_TRANSFER       = 0xB0,
} PICC_CMD;

typedef enum {
	PICC_STATUS_OK				= 0,
	PICC_STATUS_ERROR			= 1,
	PICC_STATUS_COLLISION		= 2,
	PICC_STATUS_TIMEOUT			= 3,
	PICC_STATUS_NO_ROOM			= 3,
	PICC_STATUS_INTERNAL_ERROR	= 5,
	PICC_STATUS_STATUS_INVALID	= 6,
	PICC_STATUS_CRC_WRONG		= 7,
	PICC_STATUS_MIFARE_NACK 	= 0xff
} PICC_STATUS;

//#define MFRC522_CS_LOW					MFRC522_CS_PORT->BSRRH = MFRC522_CS_PIN;
//#define MFRC522_CS_HIGH					MFRC522_CS_PORT->BSRRL = MFRC522_CS_PIN;

/* MFRC522 Commands */
typedef enum {
	PCD_IDLE			= 0x00,   //NO action; Cancel the current command
	PCD_MEMSET			= 0x01,   //Write FIFO buffer to internal buffer
	PCD_AUTHENT			= 0x0E,   //Authentication Key
	PCD_RECEIVE			= 0x08,   //Receive Data
	PCD_TRANSMIT		= 0x04,   //Transmit data
	PCD_TRANSCEIVE		= 0x0C,   //Transmit and receive data,
	PCD_RESETPHASE		= 0x0F,   //Reset
	PCD_CALCCRC			= 0x03   //CRC Calculate
} PCD_CMD;

/* MFRC522 Registers */
//Page 0: Command and Status
#define MFRC522_REG_COMMAND				0x01    
#define MFRC522_REG_COMM_IE_N			0x02    
#define MFRC522_REG_DIV1_EN				0x03    
#define MFRC522_REG_COMM_IRQ			0x04    
#define MFRC522_REG_DIV_IRQ				0x05
#define MFRC522_REG_ERROR				0x06    
#define MFRC522_REG_STATUS1				0x07    
#define MFRC522_REG_STATUS2				0x08    
#define MFRC522_REG_FIFO_DATA			0x09
#define MFRC522_REG_FIFO_LEVEL			0x0A
#define MFRC522_REG_WATER_LEVEL			0x0B
#define MFRC522_REG_CONTROL				0x0C
#define MFRC522_REG_BIT_FRAMING			0x0D
#define MFRC522_REG_COLL				0x0E

/* Command Registers */
#define MFRC522_REG_MODE				0x11
#define MFRC522_REG_TX_MODE				0x12
#define MFRC522_REG_RX_MODE				0x13
#define MFRC522_REG_TX_CONTROL			0x14
#define MFRC522_REG_TX_AUTO				0x15
#define MFRC522_REG_TX_SELL				0x16
#define MFRC522_REG_RX_SELL				0x17
#define MFRC522_REG_RX_THRESHOLD		0x18
#define MFRC522_REG_DEMOD				0x19
#define MFRC522_REG_MIFARE				0x1C
#define MFRC522_REG_SERIALSPEED			0x1F

/* Config Registers */
#define MFRC522_REG_CRC_RESULT_M		0x21
#define MFRC522_REG_CRC_RESULT_L		0x22
#define MFRC522_REG_MOD_WIDTH			0x24
#define MFRC522_REG_RF_CFG				0x26
#define MFRC522_REG_GS_N				0x27
#define MFRC522_REG_CWGS_PREG			0x28
#define MFRC522_REG__MODGS_PREG			0x29
#define MFRC522_REG_T_MODE				0x2A
#define MFRC522_REG_T_PRESCALER			0x2B
#define MFRC522_REG_T_RELOAD_H			0x2C
#define MFRC522_REG_T_RELOAD_L			0x2D
#define MFRC522_REG_T_COUNTER_VALUE_H	0x2E
#define MFRC522_REG_T_COUNTER_VALUE_L	0x2F

/* Test registers */
#define MFRC522_REG_TEST_SEL1			0x31
#define MFRC522_REG_TEST_SEL2			0x32
#define MFRC522_REG_TEST_PIN_EN			0x33
#define MFRC522_REG_TEST_PIN_VALUE		0x34
#define MFRC522_REG_TEST_BUS			0x35
#define MFRC522_REG_AUTO_TEST			0x36
#define MFRC522_REG_VERSION				0x37
#define MFRC522_REG_ANALOG_TEST			0x38
#define MFRC522_REG_TEST_ADC1			0x39  
#define MFRC522_REG_TEST_ADC2			0x3A   
#define MFRC522_REG_TEST_ADC0			0x3B   

/* Other */
#define MFRC522_DUMMY					0x00
#define MFRC522_MAX_LEN					16


void MFRC522_Init();
void MFRC522_AntennaOn();
void MFRC522_AntennaOff();
void MFRC522_Reset();
void MFRC522_SelfTest();
void MFRC522_Halt();
void MFRC522_version_dump();
bool MFRC522_IsNewCardPresent();

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
	MFRC522_ClearBitMask(MFRC522_REG_TX_CONTROL, 0x03); \
} while(0)

#define MFRC522_Reset() \
do { \
	MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_RESETPHASE); \
} while(0)

#define MFRC522_DoCmd(cmd) \
do { \
	MFRC522_WriteRegister(MFRC522_REG_COMMAND, cmd); \
} while (0)

#define MFRC522_FlushBuffer() \
do { \
	MFRC522_WriteRegister(MFRC522_REG_FIFO_LEVEL, 0x80); \
} while (0)

#define MFRC522_WriteBuffer(data, ulen) \
do { \
	for (uint16_t i = 0; i < ulen; i++) { \
		MFRC522_WriteRegister(MFRC522_REG_FIFO_DATA, data[i]); \
	} \
} while (0)

#define MFRC522_WriteMem(data) \
do { \
	MFRC522_FlushBuffer(); \
	MFRC522_WriteBuffer(data, 25); \
	MFRC522_DoCmd(PCD_MEMSET); \
} while(0)

#define MFRC522_PowerDown() \
do { \
	MFRC522_SetBitMask(MFRC522_REG_COMMAND, (1 << 4)); \
} while(0)

#define MFRC522_PowerUp() \
do { \
	MFRC522_ClearBitMask(MFRC522_REG_COMMAND, (1 << 4)); \
	while (MFRC522_ReadRegister(MFRC522_REG_COMMAND) & (1 << 4)); \
} while(0)

#define MFRC522_ClearCollisions() \
do { \
    MFRC522_ClearBitMask(MFRC522_REG_COLL, 0x80); \
} while(0)

#define MFRC522_GetFIFOLevel() (MFRC522_ReadRegister(MFRC522_REG_FIFO_LEVEL) & 0x1F)

#define MFRC522_PrintReg(label, reg) \
do { \
	uint8_t dummy; \
    dummy = MFRC522_ReadRegister(MFRC522_REG_COMM_IRQ); \
    uart_printf("%s = 0x%0X\r\n", label, dummy); \
} while(0)

#endif

