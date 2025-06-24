#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_epi.h"
#include "inc/hw_ints.h"
#include "driverlib/epi.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/ssi.h"
#include "driverlib/fpu.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "TFTinit/TFT_400x240_OTM4001A_16bit.h"
//#include "TFTinit/picture.h"
#include "TOUCHinit/TOUCH_TSC2046.h"
#include "EPIinit/EPIinit.h"
#include "IQmath/IQmathLib.h"
#include "inc/tm4c1294ncpdt.h"
#include <stdlib.h>
#include "pictrue.h"
#include <math.h>

#define SCREEN_W 400   // ������
#define SCREEN_H 240   // ����߶�
#define MAP_W 24
#define MAP_H 24

int worldMap[MAP_W][MAP_H]=
{
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,2,2,2,2,2,0,0,0,0,3,0,3,0,3,0,0,0,1},
  {1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,3,0,0,0,3,0,0,0,1},
  {1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,2,2,0,2,2,0,0,0,0,3,0,3,0,3,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,0,0,0,5,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

int posX = 22, posY = 11;
float dirX = -1, dirY = 0;
float planeX = 0, planeY = 0.66;
int dir_flag = 0;
float dirXset[] = {-1,0,1,0};
float dirYset[] = {0,1,0,-1};
float planeXset[] = { 0,       -1.05,      0,       1.05};
float planeYset[] = { 1.05,     0,        -1.05,     0};
int score = 0;
void drawVerticalLine(int x, int yStart, int yEnd, uint16_t color) {
    int y;
//    for (y = 0; y < yStart; y++){
//    	TFTLCD_DrawPoint(y, x, BLACK);
//    }
//    for (y = yStart; y <= yEnd; y++) {
//        TFTLCD_DrawPoint(y, x, color);
//    }
//    for (y = yEnd + 1; y < SCREEN_H; y++){
//    	TFTLCD_DrawPoint(y, x, BLACK);
//    }
    if(color == 0x001C || color == 0x000E){
    	if(x < 260 && x>140){
			TFTLCD_FillBlock(80,  120, x, x, BLACK);
			TFTLCD_FillBlock(120,  yEnd, x, x, 0x001F);
			TFTLCD_FillBlock(yEnd,  SCREEN_H, x, x, 0x9407);
    	}
		else{
			TFTLCD_FillBlock(0,  120, x, x, BLACK);
			TFTLCD_FillBlock(120,  yEnd, x, x, 0x001F);
			TFTLCD_FillBlock(yEnd,  SCREEN_H, x, x, 0x9407);
		}
	return;
    }
    if(x < 260 && x>140){
    	TFTLCD_FillBlock(80,  yStart, x, x, BLACK);
    	if(yStart < 80){
    		TFTLCD_FillBlock(80,  yEnd, x, x, color);
    	}
    	else{
    		TFTLCD_FillBlock(yStart,  yEnd, x, x, color);
    	}
    	TFTLCD_FillBlock(yEnd,  SCREEN_H, x, x, 0x9407);
    }
    else{
    	TFTLCD_FillBlock(0,  yStart, x, x, BLACK);
        TFTLCD_FillBlock(yStart,  yEnd, x, x, color);
        TFTLCD_FillBlock(yEnd,  SCREEN_H, x, x, 0x9407);
    }
}

void raycaster_draw_frame() {
    int x;
    for (x = 0; x < SCREEN_W; x++) {
//    	if(x%40 == 0){
//        	ShowGrade();
//    	}
        float cameraX = 2 * x / (float)SCREEN_W - 1;
        float rayDirX = dirXset[dir_flag] + planeXset[dir_flag] * cameraX;
        float rayDirY = dirYset[dir_flag] + planeYset[dir_flag] * cameraX;

        int mapX = (int)posX;
        int mapY = (int)posY;

        float sideDistX, sideDistY;
        float deltaDistX = rayDirX == 0 ? 1e30 : fabsf(1 / rayDirX);
        float deltaDistY = rayDirY == 0 ? 1e30 : fabsf(1 / rayDirY);
        float perpWallDist;

        int stepX, stepY;
        int hit = 0;
        int side;

        // ���� X �᷽��
        if (rayDirX < 0) {
            stepX = -1;
            sideDistX = (posX - mapX) * deltaDistX;
        } else {
            stepX = 1;
            sideDistX = (mapX + 1.0 - posX) * deltaDistX;
        }

        // ���� Y �᷽��
        if (rayDirY < 0) {
            stepY = -1;
            sideDistY = (posY - mapY) * deltaDistY;
        } else {
            stepY = 1;
            sideDistY = (mapY + 1.0 - posY) * deltaDistY;
        }

        // ִ�� DDA
        while (!hit) {
            if (sideDistX < sideDistY) {
                sideDistX += deltaDistX;
                mapX += stepX;
                side = 0;
            } else {
                sideDistY += deltaDistY;
                mapY += stepY;
                side = 1;
            }
            if (worldMap[mapX][mapY] > 0) hit = 1;
        }

        // ������ǽ�ľ���
        if (side == 0)
            perpWallDist = sideDistX - deltaDistX;
        else
            perpWallDist = sideDistY - deltaDistY;

        // ������Ƶ�ǽ�ĸ߶�
        int lineHeight = (int)(SCREEN_H / perpWallDist);

        // ������ʼ�ͽ���������λ��
        int drawStart = -lineHeight / 2 + SCREEN_H / 2;
        int drawEnd = lineHeight / 2 + SCREEN_H / 2;

        // ȷ����������Ļ��Χ
        if (drawStart < 0) drawStart = 0;
        if (drawEnd >= SCREEN_H) drawEnd = SCREEN_H - 1;

        // ����ǽ������ѡ����ɫ
        uint16_t color;
        switch (worldMap[mapX][mapY]) {
            case 1: color = 0xF800; break; // red
            case 2: color = 0x07E0; break; // green
            case 3: color = 0x001F; break; // blue
            case 4: color = 0xFFFF; break; // white
            case 6: color = 0x001C; break;
            default: color = 0xFFE0; break; // yellow
        }

        if (side == 1) color = color >> 1; // ǽ���� Y �᷽��ʱ��ɫ��΢�䰵

        // ����ǽ��
        drawVerticalLine(x, drawStart, drawEnd, color);
    }
}

// System clock rate in Hz.
//
//*****************************************************************************
uint32_t g_ui32SysClock;

extern uint32_t GetData[6];

uint32_t TouchXData[6];
uint32_t TouchYData[6];
int IfDead;
int GameTime;
int Action;
int RandX,RandY,Rand;
int i,count;
volatile uint32_t key;
volatile uint32_t ui32Loop;
//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

void delay()
{
    int ui32Loop0;
    for(ui32Loop0=0;ui32Loop0<1000;ui32Loop0++){;}  //delay
}

int identify_key()
{
    key=0;
    GPIO_PORTD_AHB_DATA_R = 0x00;
    GPIO_PORTH_AHB_DATA_R = 0x0c;
    GPIO_PORTM_DATA_R = 0x08;
    for(ui32Loop=0;ui32Loop<1000;ui32Loop++)  //delay
    {
        ;
    }
 if((GPIO_PORTP_DATA_R&0x04)==0x00)//&&(GPIO_PORTP_DATA_R==0x04)
     {
     delay();
     if((GPIO_PORTP_DATA_R&0x04)==0x00)
     {
         key=1;
         return 0;
     }
     }
 else
     if((GPIO_PORTN_DATA_R&0x08)==0x00)
     {

         delay();
         if((GPIO_PORTN_DATA_R&0x08)==0x00)
         {
             key=5;
             return 0;
         }
     }
     else
         if((GPIO_PORTN_DATA_R&0x04)==0x00)
         {
             delay();
             if((GPIO_PORTN_DATA_R&0x04)==0x00)
             {
                 key=9;
                 return 0;
             }
         }
         else
             if((GPIO_PORTD_AHB_DATA_R&0x01)==0x00)
             {
                 delay();
                 if((GPIO_PORTD_AHB_DATA_R&0x01)==0x00)
                 {
                     key=13;
                     return 0;
                 }
             }

    GPIO_PORTD_AHB_DATA_R = 0x02;
    GPIO_PORTH_AHB_DATA_R = 0x04;
    GPIO_PORTM_DATA_R = 0x08;
    for(ui32Loop=0;ui32Loop<1000;ui32Loop++)
    {
        ;
    }
    if((GPIO_PORTP_DATA_R&0x04)==0x00)//&&(GPIO_PORTP_DATA_R==0x04)
             {
                 delay();
                 if((GPIO_PORTP_DATA_R&0x04)==0x00)
                 {
                     key=2;
                     return 0;
                 }
             }
         else
             if((GPIO_PORTN_DATA_R&0x08)==0x00)
             {

                 delay();
                 if((GPIO_PORTN_DATA_R&0x08)==0x00)
                 {
                     key=6;
                     return 0;
                 }
             }
             else
                 if((GPIO_PORTN_DATA_R&0x04)==0x00)
                 {
                     delay();
                     if((GPIO_PORTN_DATA_R&0x04)==0x00)
                     {
                         key=10;
                         return 0;
                     }
                 }
                 else
                     if((GPIO_PORTD_AHB_DATA_R&0x01)==0x00)
                     {
                         delay();
                         if((GPIO_PORTD_AHB_DATA_R&0x01)==0x00)
                         {
                             key=14;
                             return 0;
                         }
                     }

    GPIO_PORTD_AHB_DATA_R = 0x02;
    GPIO_PORTH_AHB_DATA_R = 0x08;
    GPIO_PORTM_DATA_R = 0x08;
    for(ui32Loop=0;ui32Loop<1000;ui32Loop++)
    {
        ;
    }
    if((GPIO_PORTP_DATA_R&0x04)==0x00)//&&(GPIO_PORTP_DATA_R==0x04)
             {
                 delay();
                 if((GPIO_PORTP_DATA_R&0x04)==0x00)
                 {
                     key=3;
                     return 0;
                 }
             }
         else
             if((GPIO_PORTN_DATA_R&0x08)==0x00)
             {

                 delay();
                 if((GPIO_PORTN_DATA_R&0x08)==0x00)
                 {
                     key=7;
                     return 0;
                 }
             }
             else
                 if((GPIO_PORTN_DATA_R&0x04)==0x00)
                 {
                     delay();
                     if((GPIO_PORTN_DATA_R&0x04)==0x00)
                     {
                         key=11;
                         return 0;
                     }
                 }
                 else
                     if((GPIO_PORTD_AHB_DATA_R&0x01)==0x00)
                     {
                         delay();
                         if((GPIO_PORTD_AHB_DATA_R&0x01)==0x00)
                         {
                             key=15;
                             return 0;
                         }
                     }


    GPIO_PORTD_AHB_DATA_R = 0x02;
    GPIO_PORTH_AHB_DATA_R = 0x0c;
    GPIO_PORTM_DATA_R = 0x00;
    for(ui32Loop=0;ui32Loop<1000;ui32Loop++)
    {
        ;
    }
    if((GPIO_PORTP_DATA_R&0x04)==0x00)//&&(GPIO_PORTP_DATA_R==0x04)
             {
                 delay();
                 if((GPIO_PORTP_DATA_R&0x04)==0x00)
                 {
                     key=4;
                     return 0;
                 }
             }
         else
             if((GPIO_PORTN_DATA_R&0x08)==0x00)
             {

                 delay();
                 if((GPIO_PORTN_DATA_R&0x08)==0x00)
                 {
                     key=8;
                     return 0;
                 }
             }
             else
                 if((GPIO_PORTN_DATA_R&0x04)==0x00)
                 {
                     delay();
                     if((GPIO_PORTN_DATA_R&0x04)==0x00)
                     {
                         key=12;
                         return 0;
                     }
                 }
                 else
                     if((GPIO_PORTD_AHB_DATA_R&0x01)==0x00)
                     {
                         delay();
                         if((GPIO_PORTD_AHB_DATA_R&0x01)==0x00)
                         {
                             key=16;
                             return 0;
                         }
                     }
return 1;
}

void TOUCH_GetKey()
{
    int i,ui32Loop;
    uint32_t XXX=0,YYY=0;
    for(i=0;i<10;i++)
    {
        for(ui32Loop=0;ui32Loop<5;ui32Loop++)
        {
            SSIDataPut(SSI0_BASE,0x90);
            SysCtlDelay(3);
            SSIDataGet(SSI0_BASE,&TouchXData[ui32Loop]);
            XXX+=TouchXData[ui32Loop];
            SysCtlDelay(3);
            SSIDataPut(SSI0_BASE,0xd0);
            SysCtlDelay(3);
            SSIDataGet(SSI0_BASE,&TouchYData[ui32Loop]);
            YYY+=TouchYData[ui32Loop];
            SysCtlDelay(3);
        }
    }
    TouchXData[5]=XXX/50;
    TouchYData[5]=YYY/50;
    TOUCH_PointAdjust(&TouchXData[5], &TouchYData[5]);
    return;
}

void TOUCH_PressKey(uint32_t TouchXData, uint32_t TouchYData)
{
    if((TouchXData>=9)&&(TouchXData<=50)&&(TouchYData>=339)&&(TouchYData<=356))
    {
        Action=1;
    }
    if((TouchXData>=84)&&(TouchXData<=125)&&(TouchYData>=339)&&(TouchYData<=356))
    {
        Action=2;
    }
    if((TouchXData>=159)&&(TouchXData<=224)&&(TouchYData>=339)&&(TouchYData<=356))
    {
        Action=3;
    }
    return;
}

#define _NOP() _nop()

//*********************************************************************
//*********************************************************************
#define I2C0_MASTER_BASE 0x40020000
#define I2C0_SLAVE_BASE 0x40020000
//*********************************************************************
// ��ַ���Ĵ����ȶ��岿��
//*********************************************************************
//*********************************************************************
//
// �趨slave���ӣ�ģ��ĵ�ַ������һ��7-bit�ĵ�ַ����RSλ��������ʽ����:
//                      [A6:A5:A4:A3:A2:A1:A0:RS]
// RSλ��һ��ָʾλ�����RS=0����˵�������������ݣ��ӽ������ݣ�RS=1˵�������������ݣ��ӷ�������
//
//*********************************************************************
//U21����4�����ֹܺ�����ܽŵ�����
#define I2C0_ADDR_TUBE_SEL        0x30  //00110000
//U22�������ֹ�7~14�ܽŶ�Ӧ�����
#define I2C0_ADDR_TUBE_SEG_LOW    0x32  //00110010
//U23�������ֹ�15~18�ܽŶ�Ӧ�����
#define I2C0_ADDR_TUBE_SEG_HIGH  0x34   //00110100
//U24����LED����

//PCA9557�ڲ��Ĵ�����Ҳ���ӵ�ַ
#define PCA9557_REG_INPUT    0x00
#define PCA9557_REG_OUTPUT   0x01
#define PCA9557_REG_PolInver 0x02
#define PCA9557_REG_CONFIG   0x03

//*************************************************************************************
 #define NUM 0
//IIC ����������ʱ������
unsigned char I2C_RECV_DATA[] =
                {
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00
                };

/*******************************************
        ���� SDA �ź�
********************************************/
void I2C_Set_sda_high( void )
{
    GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,GPIO_PIN_3);  //����PB3
    _NOP();
    _NOP();
    return;
}

/*******************************************
        ����SDA �ź�
********************************************/
void I2C_Set_sda_low ( void )
{
    GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,0X00000000);  //����PB3
    _NOP();
    _NOP();
    return;
}

/*******************************************
        ����SCL �ź�
********************************************/
void I2C_Set_scl_high( void )
{
    GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2,GPIO_PIN_2);  //����PB2
    _NOP();
    _NOP();
    return;
}

/*******************************************
        ����SCL �ź�
********************************************/
void I2C_Set_scl_low ( void )
{
    GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2,0X00000000);  //����PB2
    _NOP();
    _NOP();
    return;
}

/*******************************************
        IIC �źŽ����źź���
********************************************/
void I2C_STOP(void)
{
    int i;
    I2C_Set_sda_low();
    for(i = NUM;i > 0;i--);
    I2C_Set_scl_low();
    for(i = NUM;i > 0;i--);
    I2C_Set_scl_high();
    for(i = NUM;i > 0;i--);
    I2C_Set_sda_high();
    for(i = NUM+1;i > 0;i--);
    return;
}

/*******************************************
        IIC �źų�ʼ��
********************************************/
void I2C_Initial( void )
{
    I2C_Set_scl_low();
    I2C_STOP();
    return;
}


/*******************************************
        IIC �ź���ʼ�źź���
********************************************/
void I2C_START(void)
{
    int i;

    I2C_Set_sda_high();
    for(i = NUM;i > 0;i--);
    I2C_Set_scl_high();
    for(i = NUM;i > 0;i--);
    I2C_Set_sda_low();
    for(i = NUM;i > 0;i--);
    I2C_Set_scl_low();
    return;
}

/*******************************************
        IIC ��ȡӦ����
********************************************/
int  I2C_GetACK(void)
{
    int j;
    _NOP();
    _NOP();
    I2C_Set_scl_low();
    for(j = NUM;j> 0;j--);
    I2C_Set_scl_high();
    for(j = NUM;j> 0;j--);
    I2C_Set_sda_low();
    for(j = NUM;j > 0;j--);
    I2C_Set_scl_low();
    return 1;
}

/*******************************************
        IIC ����Ӧ����
********************************************/
void I2C_SetNAk(void)
{
    I2C_Set_scl_low();
    I2C_Set_sda_high();
    I2C_Set_scl_high();
    I2C_Set_scl_low();
    return;
}

/*******************************************
        IIC �����ֽں���
        ����  1��Ҫ�����ֽ�ֵ
        return ���޷���
********************************************/
void I2C_TxByte(unsigned char nValue)
{
    int i;
    int j;
    for(i = 0;i < 8;i++)
    {
        if(nValue & 0x80)
            I2C_Set_sda_high();
        else
            I2C_Set_sda_low();
        for(j = NUM;j > 0;j--);
        I2C_Set_scl_high();
        nValue <<= 1;
        for(j = NUM;j > 0;j--);
        I2C_Set_scl_low();
    }

    return;
}

/*******************************************
        IIC �����ֽں���
        ����      ��
        return ���޷���
********************************************/
unsigned char  I2C_RxByte(void)
{
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_3);//����PB3Ϊ�����
    unsigned char nTemp=0 ;
    int i;

    I2C_Set_sda_high();

    _NOP();
    _NOP();
    _NOP();
    _NOP();
    for(i = 0;i < 8;i++)
    {
        I2C_Set_scl_high(); //ģ��SCL�ź�
        if(GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_3) == 0x18) //�����ж�PB3��SDA������
        {
            nTemp |= (0x01 << (7-i));  //8λSDA������һλΪ�߾���1
        }
        I2C_Set_scl_low();
//        Delay5us();
    }
    return nTemp;
}

/*******************************************
        IIC �������麯��
    ����      1 num : �����ֽ���
        2 device_addr : iicĿ���ַ
        3 *data �����������ַ
    return ���޷���
********************************************/
void i2c_write(int num, unsigned char device_addr,unsigned char *data)
{
    int i = 0;
    int count = num;
    unsigned char *send_data = data;
    unsigned char write_addr = device_addr;

    I2C_Set_scl_high();
    for(i = NUM;i > 0;i--);
    I2C_Set_sda_high();
    for(i = NUM;i > 0;i--);

    for(i = 0;i < count;i++)
    {
      I2C_START();           //ģ��I2Cд���ݵ�ʱ��
      I2C_TxByte(write_addr);
      I2C_GetACK();
      I2C_TxByte(send_data[i]);
      I2C_GetACK();
      i++;
      I2C_TxByte(send_data[i]);
      I2C_GetACK();
      I2C_STOP();
    }
}

/*******************************************
        IIC ��ȡ���麯��
    ����      1 num : �����ֽ���
        2 device_addr : iicĿ���ַ
        3 *data �����������ַ
    return ���޷���
********************************************/
void i2c_read(int num, unsigned char device_addr,unsigned char *data)
{
  int i = 0;
  int count = num;
  unsigned char *send_data = data;
  unsigned char read_addr = device_addr;

  I2C_Set_scl_high();
  for(i = NUM;i > 0;i--);
  I2C_Set_sda_high();
  for(i = NUM;i > 0;i--);

  for(i = 0; i < count;i++)
  {
    I2C_START();               //ģ��I2C������
    I2C_TxByte((read_addr - 1));
    I2C_GetACK();
    I2C_TxByte(send_data[2*i]);
    I2C_GetACK();

    I2C_START();
    I2C_TxByte(read_addr);
    I2C_GetACK();

    I2C_RECV_DATA[i] = I2C_RxByte();
    data[2*i+1]=I2C_RECV_DATA[i];
    I2C_SetNAk();
    I2C_STOP();
  }

}

//*********************************************************************

//*********************************************************************
//******����I2C0ģ���IO���ţ�**********************************************
void I2C0GPIOBEnable(void)
{   // Enable GPIO portB containing the I2C pins (PB2&PB3)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2|GPIO_PIN_3);

}

//******����PCA9557оƬ���������ֹܵĸ�����Ϊ���***********************************
void I2C0DeviceInit(void)
{
    unsigned char dataBuf[2] = {PCA9557_REG_CONFIG, 0x00};
    i2c_write(2,I2C0_ADDR_TUBE_SEL,dataBuf);
    i2c_write(2,I2C0_ADDR_TUBE_SEG_LOW,dataBuf);
    i2c_write(2,I2C0_ADDR_TUBE_SEG_HIGH,dataBuf);

}

//*******�������ֹܵĹ�ѡ�ź�**************************************************
void I2C0TubeSelSet(char data)
{   //ѡ��1��2��3��4��5�ĸ����ֹ���
    unsigned char dataBuf[2] = {PCA9557_REG_OUTPUT, data};
    i2c_write(2,I2C0_ADDR_TUBE_SEL,dataBuf);
}
//*******�������ֹܵ���Ӧ���**************************************************
void I2C0TubeLowSet(char data)
{  //����7-14�ܽŶ�Ӧ�����
    unsigned char dataBuf[2] = {PCA9557_REG_OUTPUT, data};
    i2c_write(2,I2C0_ADDR_TUBE_SEG_LOW,dataBuf);
}
void I2C0TubeHighSet(char data)
{  //����15-18�ܽŶ�Ӧ�����
    unsigned char dataBuf[2] = {PCA9557_REG_OUTPUT, data};
    i2c_write(2,I2C0_ADDR_TUBE_SEG_HIGH,dataBuf);
}

void ShowNumber(int Number,int SelSet)
{
    switch(SelSet)
    {
        case 1:I2C0TubeSelSet(~0x20);break;
        case 2:I2C0TubeSelSet(~0x02);break;
        case 3:I2C0TubeSelSet(~0x04);break;
        case 4:I2C0TubeSelSet(~0x08);break;
        default:I2C0TubeSelSet(0xFF);
    }
    switch(Number)
    {
        case 0:I2C0TubeLowSet(0x10);I2C0TubeHighSet(0x3E);break;
        case 1:I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x18);break;
        case 2:I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x2C);break;
        case 3:I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x26);break;
        case 4:I2C0TubeLowSet(0x60);I2C0TubeHighSet(0x32);break;
        case 5:I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x16);break;
        case 6:I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x1E);break;
        case 7:I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x26);break;
        case 8:I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x3E);break;
        case 9:I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x36);break;
        default:I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
    }
    if(SelSet==0)SysCtlDelay(20000);
    else SysCtlDelay(80000);

    return;
}

void ShowGrade()
{
    int a;
    a=score/1000;
    ShowNumber(a,1);
    ShowNumber(10,0);
    a=((score)%1000)/100;
    ShowNumber(a,2);
    ShowNumber(10,0);
    a=((score)%100)/10;
    ShowNumber(a,3);
    ShowNumber(10,0);
    a=((score)%10);
    ShowNumber(a,4);
    ShowNumber(10,0);

    return;
}


void ButtonInit()
{
    SYSCTL_RCGCGPIO_R |= (SYSCTL_RCGCGPIO_R13 | SYSCTL_RCGCGPIO_R12 | SYSCTL_RCGCGPIO_R11 |SYSCTL_RCGCGPIO_R10| SYSCTL_RCGCGPIO_R7 |SYSCTL_RCGCGPIO_R3 );//

    GPIO_PORTN_DIR_R = 0x03;
    GPIO_PORTM_DIR_R = 0x28;
    GPIO_PORTH_AHB_DIR_R = 0x0c;
    GPIO_PORTP_DIR_R = 0x00;
    GPIO_PORTD_AHB_DIR_R = 0x02;
    GPIO_PORTL_DIR_R = 0x0f;

    GPIO_PORTN_DEN_R = 0x0f;
    GPIO_PORTM_DEN_R = 0x28;
    GPIO_PORTH_AHB_DEN_R = 0x0c;
    GPIO_PORTP_DEN_R = 0x04;
    GPIO_PORTD_AHB_DEN_R = 0x03;
    GPIO_PORTL_DEN_R = 0x0f;

    return;
}


void raycaster_update_controls() {

    identify_key();  // ��ȡ����״̬
    int moveSpeed = 1;

    if (UARTCharsAvail(UART0_BASE))
	{
		char c = UARTCharGet(UART0_BASE);
		UARTprintf("forward.\n");
		UARTprintf("Received: %c\n", c);
		if(c == 'w'){
			UARTprintf("Received: %c\n", c);
			int nx = posX + (int)dirXset[dir_flag];
			int ny = posY + (int)dirYset[dir_flag];
			if (nx >= 0 && nx < MAP_W && ny >= 0 && ny < MAP_H && worldMap[nx][ny] == 0) {
				posX = nx;
				posY = ny;
			}
		}
		if(c == 's'){
			UARTprintf("Received: %c\n", c);
			int nx = posX - (int)dirXset[dir_flag];
			int ny = posY - (int)dirYset[dir_flag];
			if (nx >= 0 && nx < MAP_W && ny >= 0 && ny < MAP_H && worldMap[nx][ny] == 0) {
				posX = nx;
				posY = ny;
			}
		}
		if(c == 'a'){
			UARTprintf("Received: %c\n", c);
			int nx = posX - (int)dirYset[dir_flag]*pow(-1,dir_flag);
			int ny = posY + (int)dirXset[dir_flag]*pow(-1,dir_flag);
			if (nx >= 0 && nx < MAP_W && ny >= 0 && ny < MAP_H && worldMap[nx][ny] == 0) {
				posX = nx;
				posY = ny;
			}
		}
		if(c == 'd'){
			UARTprintf("Received: %c\n", c);
			int nx = posX + (int)dirYset[dir_flag]*pow(-1,dir_flag);
			int ny = posY - (int)dirXset[dir_flag]*pow(-1,dir_flag);
			if (nx >= 0 && nx < MAP_W && ny >= 0 && ny < MAP_H && worldMap[nx][ny] == 0) {
				posX = nx;
				posY = ny;
			}
		}
		if(c == 'e'){
			UARTprintf("Received: %c\n", c);
			dir_flag = (dir_flag+1)%4;
		}
		if(c == 'q'){
			UARTprintf("Received: %c\n", c);
			dir_flag = (dir_flag+3)%4;
		}
		if(c == ' '){
			UARTprintf("Received: %c\n", c);
	    	Buzzer_Gunshot_Soft();
	    	shoot();
		}
	}


    if (key == 10) { // forward
        int nx = posX + (int)dirXset[dir_flag];
        int ny = posY + (int)dirYset[dir_flag];
        if (nx >= 0 && nx < MAP_W && ny >= 0 && ny < MAP_H && worldMap[nx][ny] == 0) {
            posX = nx;
            posY = ny;
        }
    }

    if (key == 2) { // backward
        int nx = posX - (int)dirXset[dir_flag];
        int ny = posY - (int)dirYset[dir_flag];
        if (nx >= 0 && nx < MAP_W && ny >= 0 && ny < MAP_H && worldMap[nx][ny] == 0) {
            posX = nx;
            posY = ny;
        }
    }

    if (key == 5) { // right
        int nx = posX + (int)dirYset[dir_flag]*pow(-1,dir_flag);
        int ny = posY - (int)dirXset[dir_flag]*pow(-1,dir_flag);
        if (nx >= 0 && nx < MAP_W && ny >= 0 && ny < MAP_H && worldMap[nx][ny] == 0) {
            posX = nx;
            posY = ny;
        }
    }

    if (key == 7) { // left
        int nx = posX - (int)dirYset[dir_flag]*pow(-1,dir_flag);
        int ny = posY + (int)dirXset[dir_flag]*pow(-1,dir_flag);
        if (nx >= 0 && nx < MAP_W && ny >= 0 && ny < MAP_H && worldMap[nx][ny] == 0) {
            posX = nx;
            posY = ny;
        }
    }

    if (key == 9) {
    	dir_flag = (dir_flag+1)%4;
    }
    if(key == 11){
    	dir_flag = (dir_flag+3)%4;
    }
    if(key == 6){
    	Buzzer_Gunshot_Soft();
    	shoot();
    }

}

void Buzzer_PlayTone(uint32_t freq, uint32_t duration_ms)
{
    // ÿ�������ܹ�����״̬���ߵ�ƽ�͵͵�ƽ
    uint32_t half_period_cycles = (g_ui32SysClock / freq) / 2;
    uint32_t total_cycles = (freq * duration_ms) / 1000;
    int i;
    for(i = 0; i < total_cycles; i++) {
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, GPIO_PIN_5); // ����
        SysCtlDelay(half_period_cycles / 3); // ע�� /3 �� SysCtlDelay ÿ��ѭ��3����

        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0); // ����
        SysCtlDelay(half_period_cycles / 3);
    }
}

void Buzzer_Gunshot_Soft()
{
    // ����Ƶ��Ϊ�ϵ�Ƶ�ʣ����ӽ���ʵ�ı���
    int base_freq = 600;  // ǹ�����Ǽ����Ƶ�����Ƕ۱���

    int segments = 6;  // �ֳɶ��˥��
    int s,i;
    for (s = 0; s < segments; ++s) {
        int freq = base_freq + s * 50; // ������Ƶ
        int half_period = (g_ui32SysClock / freq) / 2;

        int cycles = 30 - s * 3;  // ÿ�γ���ʱ��ݼ�
        for (i = 0; i < cycles; i++) {
            GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, GPIO_PIN_5);
            SysCtlDelay(half_period / 3);
            GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0);
            SysCtlDelay(half_period / 3);
        }
    }

    // ����ͣ��ģ�⡰����β����
    SysCtlDelay(g_ui32SysClock / 1000);  // 1ms pause
    GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, GPIO_PIN_5);
    SysCtlDelay(g_ui32SysClock / 30000); // ����β��
    GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0);
}

int ti,tj;
void generate_target(){
	int i,j;
	i = rand()%24;
	j = rand()%24;
	while(worldMap[i][j]!=0){
		i = rand()%24;
		j = rand()%24;
	}
	ti = i;
	tj = j;
	worldMap[ti][tj] = 6;
}


void shoot(){
	int done_flag = 0;
	if(dir_flag == 0){
		if( tj == posY){
			int way_go = posX;
			for(;way_go>ti;way_go--){
				if(worldMap[way_go][tj]!=0){
					return;
				}
			}
			done_flag = 1;
		}
	}
	if(dir_flag == 2){
		if( tj == posY){
			int way_go = posX;
			for(;way_go<ti;way_go++){
				if(worldMap[way_go][tj]!=0){
					return;
				}
			}
			done_flag = 1;
		}
	}
	if(dir_flag == 1){
		if( ti == posX){
			int way_go = posX;
			for(;way_go<tj;way_go++){
				if(worldMap[ti][way_go]!=0){
					return;
				}
			}
			done_flag = 1;
		}
	}
	if(dir_flag == 3){
		if( ti == posX){
			int way_go = posX;
			for(;way_go>tj;way_go--){
				if(worldMap[ti][way_go]!=0){
					return;
				}
			}
			done_flag = 1;
		}
	}
	if(done_flag == 1){
		worldMap[ti][tj]=0;
		score += 100;
		generate_target();
//		ShowGrade();
	}
}

void main()

{
    volatile uint32_t ui32Loop0,ui32Loop1,ui32Loop2,ui32Loop3,ui32Loop4;

    FPUEnable();
    FPULazyStackingEnable();

    //
    // Run from the PLL at 120 MHz.
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 120000000);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_5);
    ButtonInit();
    I2C0GPIOBEnable();//����I2C0ģ���IO����
    I2C0DeviceInit();//����PCA9557оƬ���������ֹܵĸ�����Ϊ���
//    IntMasterEnable();
//    SysTickIntEnable();
//    SysTickEnable();

    ConfigureUART();
    UARTprintf("UART ready to receive commands.\n");
    EPIGPIOinit();
    TOUCH_TSC2046init(g_ui32SysClock);
    TFT_400x240_OTM4001Ainit(g_ui32SysClock);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0, GPIO_PIN_0);

    GPIOIntEnable(GPIO_PORTB_BASE,GPIO_INT_PIN_0);
    GPIOIntTypeSet(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_FALLING_EDGE);

    TFT_400x240_OTM4001Ainit(g_ui32SysClock);
    SSIDataPut(SSI0_BASE,0xd0);

    GameTime=0;

    raycaster_draw_frame();
    TFTLCD_ShowPicture(0,0,ak);
    generate_target();
    ShowGrade();
    while(1){
    	ShowGrade();
    	raycaster_update_controls();
    	ShowGrade();
    	raycaster_draw_frame();
    }
}
