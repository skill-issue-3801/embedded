/*
 * mfrc522_reg.h
 *
 *  Created on: Sep 5, 2023
 *      Author: lachjet
 */

#ifndef INC_MFRC522_REG_H_
#define INC_MFRC522_REG_H_

#define MFRC522_ADDR	(0x2C << 1)
#define MFRC522_WRITE_ADDR (0x2C << 1) | 0
#define MFRC522_READ_ADDR (0x2C << 1) | 1

/*
 * Commands
 */
#define MFRC522_CMD_Idle		0x0
#define MFRC522_CMD_Mem 		0x1
#define MFRC522_CMD_GenRandomID 0x2
#define MFRC522_CALC_CRC_CMD 	0x3

#define MFRC522_CMD_Transmit	0x4

#define MFRC522_CMD_NoCmdChange	0x7
#define MFRC522_CMD_Recieve		0x8

#define MFRC522_CMD_Transceive	0xC

#define MFRC522_CMD_MFAuthent	0xE
#define MFRC522_CMD_SoftReset 	0xF

/*
 * Register Addresses
 */

//Command and Status Regs
#define MFRC522_CommandReg		0x00

#define MFRC522_ComIEnReg		0x01
#define MFRC522_DivIEnReg		0x03
#define MFRC522_ComIrqReg		0x04
#define MFRC522_DivIrqReg		0x05

#define MFRC522_ErrorReg		0x06
#define MFRC522_Status1Reg		0x07
#define MFRC522_Status2Reg		0x08

#define MFRC522_FIFODataReg		0x09
#define MFRC522_FIFOLevelReg	0x0A

#define MFRC522_WaterLevelReg	0x0B

#define MFRC522_ControlReg		0x0C
#define MFRC522_BitFramingReg	0x0D
#define MFRC522_CollReg			0x0E

//Command Regs
#define MFRC522_ModeReg			0x11
#define MFRC522_TxModeReg		0x12
#define MFRC522_RxModeReg		0x13

#define MFRC522_TxControlReg	0x14
#define MFRC522_TxASKReg		0x15
#define MFRC522_TxSelReg		0x16

#define MFRC522_RxSelReg		0x17
#define MFRC522_ThresholdReg	0x18

#define MFRC522_DemodReg		0x19

#define MFRC522_MfTxReg			0x1C
#define MFRC522_MfRxReg			0x1D

#define MFRC522_SerialSpeedReg	0x1F

//Config Regs
#define MFRC522_CRCResultReg_L	0x21
#define MFRC522_CRCResultReg_H	0x22

#define MFRC522_ModWidthReg		0x24

#define MFRC522_RFCfgReg		0x26
#define MFRC522_GsNReg			0x27
#define MFRC522_CWGsPReg		0x28
#define MFRC522_ModGsPReg		0x29

#define MFRC522_TModeReg		0x2A
#define MFRC522_TPrescalerReg	0x2B
#define MFRC522_TReloadReg_L	0x2C
#define MFRC522_TReloadReg_H	0x2D
#define MFRC522_TCounterValReg_L 0x2E
#define MFRC522_TCounterValReg_H 0x2F

//Test Register
#define MFRC522_TestSel1Reg		0x31
#define MFRC522_TestSel2Reg		0x32

#define MFRC522_TestPinEnReg	0x33
#define MFRC522_TestPinValueReg	0x34
#define MFRC522_TestBusReg		0x35
#define MFRC522_AutoTestReg		0x36
#define MFRC522_VersionReg		0x37
#define MFRC522_AnalogTestReg	0x38
#define MFRC522_TestDAC1Reg		0x39
#define MFRC522_TestDAC2Reg		0x3A
#define MFRC522_TestADCReg		0x3B


#endif /* INC_MFRC522_REG_H_ */
