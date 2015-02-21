#ifndef MOUSE_H
#define MOUSE_H
void mouse_task(void);
void vusb_transfer_mouse(void);
void send_mouse(report_mouse_t *report);
uint8_t ps2_mouse_init(void);
#define PS2_MOUSE_BTN_MASK      0x07
#define PS2_MOUSE_BTN_LEFT      0
#define PS2_MOUSE_BTN_RIGHT     1
#define PS2_MOUSE_BTN_MIDDLE    2
#define PS2_MOUSE_X_SIGN        4
#define PS2_MOUSE_Y_SIGN        5
#define PS2_MOUSE_X_OVFLW       6
#define PS2_MOUSE_Y_OVFLW       7
#define X_IS_NEG  (mouse_report.buttons & (1<<PS2_MOUSE_X_SIGN))
#define Y_IS_NEG  (mouse_report.buttons & (1<<PS2_MOUSE_Y_SIGN))
#define X_IS_OVF  (mouse_report.buttons & (1<<PS2_MOUSE_X_OVFLW))
#define Y_IS_OVF  (mouse_report.buttons & (1<<PS2_MOUSE_Y_OVFLW))

#endif