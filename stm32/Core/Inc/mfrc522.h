/*
 * mfrc522.h
 *
 *  Created on: Sep 5, 2023
 *      Author: lachjet
 */

#ifndef INC_MFRC522_H_
#define INC_MFRC522_H_

int mfrc522_init();

int mfrc522_tag_read(uint8_t* buffer, uint8_t len);

int mfrc522_tag_write();

int mfrc522_rx_init();


#endif /* INC_MFRC522_H_ */
