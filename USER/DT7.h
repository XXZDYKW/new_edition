#ifndef __DT7_H
#define __DT7_H
#include "sys.h"

#define DBUS_BUF_SIZE    18
/* ----------------------- RC Channel Value Definition---------------------------- */
#define CH_VALUE_MIN                   ((uint16_t)364    ) 
#define CH_VALUE_OFFSET                ((uint16_t)1024   ) 
#define CH_VALUE_MAX                   ((uint16_t)1684   )  
/* ----------------------- RC Switch Value Definition----------------------------- */
#define SW_UP                          ((uint16_t)1      ) 
#define SW_MID                          ((uint16_t)3      ) 
#define SW_DOWN                        ((uint16_t)2      )  
/* ----------------------- PC Mouse Value Definition-------------------------------- */
#define MOUSE_MOVE_VALUE_MIN         ((uint16_t)-32768 ) 
#define MOUSE_MOVE_VALUE_OFFSET      ((uint16_t)0      ) 
#define MOUSE_MOVE_VALUE_MAX         ((uint16_t)32767  ) 
#define MOUSE_BTN_UP                  ((uint8_t)0       ) 
#define MOUSE_BTN_DN                  ((uint8_t)1       ) 
/* ----------------------- PC Key Value Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W         ((uint16_t)0x01<<0) 
#define KEY_PRESSED_OFFSET_S         ((uint16_t)0x01<<1) 
#define KEY_PRESSED_OFFSET_A         ((uint16_t)0x01<<2) 
#define KEY_PRESSED_OFFSET_D         ((uint16_t)0x01<<3) 
#define KEY_PRESSED_OFFSET_Q         ((uint16_t)0x01<<4) 
#define KEY_PRESSED_OFFSET_E         ((uint16_t)0x01<<5) 
#define KEY_PRESSED_OFFSET_SHIFT     ((uint16_t)0x01<<6) 
#define KEY_PRESSED_OFFSET_CTRL      ((uint16_t)0x01<<7)  

typedef struct
{
    struct	//Ò£¿Ø
    {
        uint16_t ch0;
        uint16_t ch1;
        uint16_t ch2;
        uint16_t ch3;
        uint8_t  s1;
        uint8_t  s2;
    }rc;
    
    struct	//Êó±ê
    {
        uint16_t x;
        uint16_t y;
        uint16_t z;
        uint8_t l;
        uint8_t r;
    }mouse;
    
    struct 	//¼üÅÌ
    {
        uint16_t v;
    }key;
    
    uint16_t res;
    
}DBUS;

void DT7_init(void);
void DBUS_Dec(DBUS* dbus,const unsigned char* buf);


#endif
