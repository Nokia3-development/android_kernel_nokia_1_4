#ifndef __OCP2132WPAD__
#define __OCP2132WPAD__



#define OCP2132PWAD_NAME                     "ocp2132wpad"
#define OCP_2132_RETRY_COUNT 		         3
#define OCP2131_AVDD_ADDR			0x00
#define OCP2131_AVEE_ADDR			0x01

void ocp2132_lcd_resume(uint8_t voltage);
void ocp2132_lcd_suspend(void);
#endif
