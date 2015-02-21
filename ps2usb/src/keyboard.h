/*
 * keyboard.h
 *
 * Created: 2014-03-08 10:52:02
 *  Author: Robert Szymiec
 */ 

#ifndef KEYBOARD_H
#define KEYBOARD_H

void send_keyboard_report(void);
void vusb_transfer_keyboard(void);
void register_code(uint8_t code);
void unregister_code(uint8_t code);

void trunOffMouseRx(void);

void trunOnMouseRx(void);



#endif




