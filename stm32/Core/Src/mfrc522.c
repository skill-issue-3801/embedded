/*
 * mfrc522.c
 *
 *  Created on: Sep 5, 2023
 *      Author: lachjet
 */
#include <string.h>
#include <stdint.h>

#include "mfrc522.h"
#include "mfrc522_reg.h"
#include "main.h"


int mfrc522_write_reg(uint8_t addr, uint8_t data) {
	int ret = HAL_OK;
	uint8_t buf[2];
	memset(buf, 0, sizeof(buf));

	buf[0] = addr;
	buf[1] = data;

	ret = HAL_I2C_Master_Transmit(&hi2c1, MFRC522_ADDR, buf, 2, HAL_MAX_DELAY);

	return ret;
}

int mfrc522_read_reg(uint8_t addr, uint8_t* readVal) {
	int ret = HAL_OK;
	uint8_t buf[2];
	memset(buf, 0, 2);

	buf[0] = addr;

	ret = HAL_I2C_Master_Transmit(&hi2c1, MFRC522_READ_ADDR, buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		return ret;
	}

	ret = HAL_I2C_Master_Receive(&hi2c1, MFRC522_WRITE_ADDR, readVal, 1, HAL_MAX_DELAY);
	return ret;
}

int mfrc522_clear_reg_mask(uint8_t addr, uint8_t mask) {
    int ret = HAL_OK;
    uint8_t data;
    ret = mfrc522_read_reg(addr, &data);
    if (ret != HAL_OK) {
        return ret;
    }

    data = data & ~mask;

    ret = mfrc522_write_reg(addr, data);
    return ret;
}

int mfrc522_set_reg_mask(uint8_t addr, uint8_t mask) {
    int ret = HAL_OK;
    uint8_t data;
    ret = mfrc522_read_reg(addr, &data);
    if (ret != HAL_OK) {
        return ret;
    }

    data = data & ~mask;
    data = data | mask;

    ret = mfrc522_write_reg(addr, data);
    return ret;
}

/*
 * MFRC522 Soft reset
 */
int mfrc522_soft_reset() {
    mfrc522_write_reg(MFRC522_CommandReg, MFRC522_CMD_SoftReset);

    uint8_t count = 0;
    do {
        HAL_delay(50);
    } while ((mfrc522_read_reg(MFRC522_CommandReg) & (1 << 4)) && (++count) < 3);
}

/*
 * Turns antenna on, enabling TX1 and TX2
 * (These pins are disabled after reset)
 */
int mfrc522_antenna_on() {
    int ret = HAL_OK;

    uint8_t value;
    ret = mfrc522_read_reg(MFRC522_TxControllReg, &value);
    if (ret != HAL_OK) {
        return ret;
    }
    if ((value & 0x03) != 0x03) {
        ret = mfrc522_write_reg(MFRC522_TxControlReg, value | 0x03);
        if (ret != HAL_OK) {
            return ret;
        }
    }
}

/*
 * Turns antenna off by disabling TX1 and TX2
 */
int mfrc522_antenna_off() {
    int ret = HAL_OK;

    uint8_t value;
    ret = mfrc_read_reg(MFRC522_TxControlReg, &value);
    if (ret != HAL_OK) {
        return ret;
    }
    ret = mfrc_write_reg(MFRC522_TxControlReg, value & ~0x03); 
    return ret;
}

uint8_t mfrc522_get_antenna_gain() {
    uint8_t gain;
    mfrc_read_reg(MFRC522_RFCfgReg, &gain);
    return gain & (0x07 << 4);
}

int mfrc552_set_antenna_gain(uint8_t mask) {
    int ret = HAL_OK;

    if (mfrc522_get_antenna_gain() != mask) {
        ret = mfrc522_set_reg_mask(MFRC522_RFCfgReg, mask & (0x07 << 4));
    }

    return ret;
}

/*
 * Initialise the mfrc522
 */
int mfrc522_init() {
    int ret = HAL_OK;

    ret = mfrc522_soft_reset();
    if (ret != HAL_OK) {
        return ret;
    }
    
    //Reset Baud rate on TX
    ret = mfrc522_write_reg(MFRC522_TxModeReg, 0x00);
     if (ret != HAL_OK) {
        return ret;
    }

    //Reset Baud rate on RX
    ret = mfrc522_write_reg(MFRC522_RxModeReg, 0x00);
    if (ret != HAL_OK) {
        return ret;
    }

    //Reset ModWidthRing
    ret = mfrc522_write_reg(MFRC522_ModWidthReg, 0x26);
    if (ret != HAL_OK) {
        return ret;
    }

    ret = mfrc522_write_reg(MFRC522_TModeReg, 0x80);
    if (ret != HAL_OK) {
        return ret;
    }

    ret = mfrc522_write_reg(MFRC522_TPrescalerReg, 0xA9);
    if (ret != HAL_OK) {
        return ret;
    }   

    ret = mfrc522_write_reg(MFRC522_TReloadReg_H, 0x03);
    if (ret != HAL_OK) {
        return ret;
    }   

    ret = mfrc522_write_reg(MFRC522_TReloadReg_L, 0xEB);
    if (ret != HAL_OK) {
        return ret;
    }

    ret = mfrc522_write_reg(MFRC522_TxASKReg, 0x40);
    if (ret != HAL_OK) {
        return ret;
    }

    ret = mfrc522_write_reg(MFRC522_ModeReg, 0x3D);
    if (ret != HAL_OK) {
        return ret;
    }

    ret = mfrc522_antenna_on();
    return ret;
}

int mfrc522_rx_init() {
	int ret = HAL_OK;
	uint8_t data;

	/* Set up settings for rx */
	data = (0x3 << 4) | (1 << 7);
	ret = mfrc522_write_reg(MFRC522_RxModeReg, data);
	return ret;
}

int mfrc522_send_cmd(int command) {
	int ret = HAL_OK;
	ret = mfrc522_write_reg(MFRC522_CommandReg, command);
	return ret;
}

int mfrc522_tag_read(uint8_t* buffer, uint8_t len) {
	int ret = HAL_OK;
	uint8_t addr = MFRC522_FIFODataReg;

	ret = mfrc522_send_cmd(MFRC522_CMD_Recieve);
	if (ret != HAL_OK) {
		buffer[0] = ret;
		return ret;
	}
	HAL_Delay(50);
	buffer[0] = addr;
	ret = HAL_I2C_Master_Transmit(&hi2c1, MFRC522_READ_ADDR, buffer, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		buffer[0] = ret;
		return ret;
	}
	ret = HAL_I2C_Master_Receive(&hi2c1, MFRC522_WRITE_ADDR, buffer, len, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		buffer[0] = ret;
		return ret;
	}
	return ret;
}

int mfrc522_tag_write() {
	return 0;
}

