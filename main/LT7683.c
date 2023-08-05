#include "LT7683.h"
#include "lcd_spi.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "common.h"

uint8_t MCLK;
uint8_t CCLK;
uint8_t SCLK;  

uint8_t tx_data[2];
uint8_t rx_data[2];

static uint8_t LT7683_StatusRead(void) {
    tx_data[0] = 0x40;
    tx_data[1] = 0x00;

    esp_err_t ret = lcd_spi_read_write(tx_data, rx_data);

    return rx_data[1];
}

static void LT7683_CmdWrite(uint8_t cmd) {   
    tx_data[0] = 0x00;
    tx_data[1] = cmd;

    esp_err_t ret = lcd_spi_read_write(tx_data, rx_data);
}

static uint8_t LT7683_DataRead(void) {
    tx_data[0] = 0xC0;
    tx_data[1] = 0x00;

    esp_err_t ret = lcd_spi_read_write(tx_data, rx_data);

    return rx_data[1];
}

static void LT7683_DataWrite(uint8_t sendData) {
    tx_data[0] = 0x80;
    tx_data[1] = sendData;

    esp_err_t ret = lcd_spi_read_write(tx_data, rx_data);
}

static void LT7683_RegisterWrite(uint8_t Cmd, uint8_t Data) {
    LT7683_CmdWrite(Cmd);
    LT7683_DataWrite(Data);
}

void Check_Mem_WR_FIFO_not_Full(void) {
/*  0: Memory Write FIFO is not full.
  1: Memory Write FIFO is full.   */
  while(LT7683_StatusRead() & 0x80);
}

void Check_Mem_WR_FIFO_Empty(void) {
/*  0: Memory Write FIFO is not empty.
  1: Memory Write FIFO is empty.    */  
  while((LT7683_StatusRead() & 0x40) == 0x00);
}

void Check_Mem_RD_FIFO_not_Full(void) {
/*  0: Memory Read FIFO is not full.
  1: Memory Read FIFO is full.    */
  while(LT7683_StatusRead() & 0x20);
}

void Check_Mem_RD_FIFO_not_Empty(void) {
/*  0: Memory Read FIFO is not empty.
  1: Memory Read FIFO is empty.
    */
  while(LT7683_StatusRead() & 0x10);
}

void Check_2D_Busy(void) {
  while(LT7683_StatusRead() & 0x08);
}

void Check_SDRAM_Ready(void) {
/*  0: SDRAM is not ready for access
  1: SDRAM is ready for access    */  
  unsigned char temp;   
  
  do {
    temp = LT7683_StatusRead();
  } while((temp&0x04) == 0x00);
}

unsigned char Power_Saving_Status(void) {
    unsigned char temp;

    if((LT7683_StatusRead() & 0x02) == 0x02)
      temp = 1;
    else
      temp = 0;

    return temp;
}

void Check_Power_is_Normal(void) {
/*  0: Normal operation state
  1: Power saving state   */  
  while(LT7683_StatusRead() & 0x02);
}

void Check_Power_is_Saving(void) {
/*  0: Normal operation state
  1: Power saving state   */  
  while((LT7683_StatusRead() & 0x02) == 0x00);
}

void Check_NO_Interrupt(void) {
/*  0: without interrupt event
  1: interrupt event occur
    */  
  while(LT7683_StatusRead() & 0x01);
}

void Check_Interrupt_Occur(void) {
/*  0: without interrupt event
  1: interrupt event occur
    */  
  while((LT7683_StatusRead() & 0x01) == 0x00);
}

void Check_Busy_Draw(void) {
  unsigned char temp;
  do {
    temp = LT7683_StatusRead();
  } while(temp&0x08);
}

static void LT7683_Init_Reset_GPIO(void) {
    //Initialize non-SPI GPIOs
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << LT7683_RST_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable pull-down resistor
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // Disable pull-up resistor
    gpio_config(&io_conf);
    gpio_set_level(LT7683_RST_PIN, 1);
}

static void LT7683_Hard_Reset(void) {
    gpio_set_level(LT7683_RST_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(LT7683_RST_PIN, 1);
    vTaskDelay(20 / portTICK_PERIOD_MS);

    //printf("LT7683_Hard_Reset\n");
    Check_2D_Busy();
}

void LT7683_Soft_Reset(void) {
  unsigned char temp;

  LT7683_CmdWrite(0x00);
  temp = LT7683_DataRead();
  temp |= 0x01;
  LT7683_DataWrite(temp);

  do {
    temp = LT7683_DataRead();
  } while(temp & 0x01);
}

static void LT7683_Init_PLL(void) {
    unsigned int  temp = 0, temp1 =0 ;
	
	unsigned short lpllOD_sclk, lpllOD_cclk, lpllOD_mclk;
	unsigned short lpllR_sclk, lpllR_cclk, lpllR_mclk;
	unsigned short lpllN_sclk, lpllN_cclk, lpllN_mclk;
	
	temp = (HBACK_PORCH + HFRONT_PORCH + HPULSE_WIDTH + LT7683_SCREEN_WIDTH) 
            * (VBACK_PORCH + VFRONT_PORCH + VPULSE_WIDTH + LT7683_SCREEN_HEIGHT) * 60; // 54721800
	
	temp1=(temp % 1000000) / 100000;
	if(temp1 >= 5) {
		 temp = temp / 1000000 + 1;
    } else {
        temp = temp / 1000000;
    }
	
    SCLK = temp;      // 54
	temp = temp * 3;  
	MCLK = temp;	    // 162
	CCLK = temp;      // 162
	
	if(CCLK > 100) {
        CCLK = 100;
    }
	
    if(MCLK > 100) {
        MCLK = 100;
    }
	
    if(SCLK > 65) {
        SCLK = 65;
    }

	lpllOD_sclk = 3;
	lpllOD_cclk = 2;
	lpllOD_mclk = 2;
	lpllR_sclk  = 5;
	lpllR_cclk  = 5;
	lpllR_mclk  = 5;
	lpllN_mclk  = MCLK;        // 100
	lpllN_cclk  = CCLK;        // 100
	lpllN_sclk  = SCLK;        // 108  
      
    LT7683_RegisterWrite(0x05, (lpllOD_sclk << 6) | (lpllR_sclk << 1) | ((lpllN_sclk >> 8) & 0x1));
    LT7683_RegisterWrite(0x07, (lpllOD_mclk << 6) | (lpllR_mclk << 1) | ((lpllN_mclk >> 8) & 0x1));
    LT7683_RegisterWrite(0x09, (lpllOD_cclk << 6) | (lpllR_cclk << 1) | ((lpllN_cclk >> 8) & 0x1));
    
    LT7683_RegisterWrite(0x06, lpllN_sclk);
    LT7683_RegisterWrite(0x08, lpllN_mclk);
    LT7683_RegisterWrite(0x0A, lpllN_cclk);
}

static void LT7683_Init_SDRAM(void) {   
    LT7683_RegisterWrite(0xE0, 0x29);
    LT7683_RegisterWrite(0xE1, 0x03);
  
    LT7683_RegisterWrite(0xE2, 0x1A);
    LT7683_RegisterWrite(0xE3, 0x06);

    LT7683_RegisterWrite(0xE4, 0x01);

    //printf("LT7683_SDRAM_Initial\n");
    
    Check_SDRAM_Ready();
}

static void checkHardResetSuccess(void) {
    uint8_t i = 0;
	uint8_t temp = 0;
	uint8_t system_ok = 0;
	
    do {
		if((LT7683_StatusRead() & 0x02) == 0x00){
			delay_us(1000);                  
			LT7683_CmdWrite(0x01);
			delay_us(1000);                  
			temp = LT7683_DataRead();
            
			if((temp & 0x80)==0x80) {
				system_ok = 1;
				i = 0;
			} else {
				delay_us(1000); 
				LT7683_CmdWrite(0x01);
				delay_us(1000); 
				LT7683_DataWrite(0x80);
			}
		} else {
			system_ok = 0;
			i++;
		}

		if(system_ok == 0 && i == 5) {
			LT7683_Hard_Reset(); 
			i = 0;
		}
	} while(system_ok == 0);
}

static void LT7683_spi4ByteAddressMode(void) {
    //LCD_RegisterWrite(0xB9, 0x30); 
    Select_nSS_drive_on_xnsfcs1();
    nSS_Active();
    Reset_CPHA(); 
    LT7683_CmdWrite(0xB8); 
    
    delay_us(1000);    
    LT7683_CmdWrite(0xB7);
    delay_us(1000);
    
    Select_nSS_drive_on_xnsfcs1();
    nSS_Active();
    Reset_CPHA(); 
    //LCD_RegisterWrite(0xB9, 0x30);
}

void LT7683_init(void) {
    LT7683_Init_Reset_GPIO();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    LT7683_Hard_Reset();
    checkHardResetSuccess();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    LT7683_Soft_Reset();
    LT7683_Init_PLL();
    LT7683_Init_SDRAM();

    //**[01h]**//
    TFT_16bit();
    Host_Bus_16bit(); //Host bus 16bit
        
    //**[02h]**//
    RGB_16b_16bpp();
    MemWrite_Left_Right_Top_Down(); 
        
    //**[03h]**//
    Graphic_Mode();
    Memory_Select_SDRAM();

    HSCAN_L_to_R();     //REG[12h]:from left to right
    VSCAN_T_to_B();       //REG[12h]:from top to bottom
    PDATA_Set_RGB();        //REG[12h]:Select RGB output

    Set_PCLK(LCD_PCLK_Falling_Rising);   //LCD_PCLK_Falling_Rising
    Set_HSYNC_Active(LCD_HSYNC_Active_Polarity);
    Set_VSYNC_Active(LCD_VSYNC_Active_Polarity);
    Set_DE_Active(LCD_DE_Active_Polarity);
    
    LT7683_HorizontalWidth_VerticalHeight(LT7683_SCREEN_WIDTH, LT7683_SCREEN_HEIGHT);
    LT7683_Horizontal_Non_Display(HBACK_PORCH);                          
    LT7683_HSYNC_Start_Position(HFRONT_PORCH);                              
    LT7683_HSYNC_Pulse_Width(HPULSE_WIDTH);                              
    LT7683_Vertical_Non_Display(VBACK_PORCH);                               
    LT7683_VSYNC_Start_Position(VFRONT_PORCH);                               
    LT7683_VSYNC_Pulse_Width(VPULSE_WIDTH);                              
        
    Select_Main_Window_16bpp();
    Memory_XY_Mode(); //Block mode (X-Y coordination addressing)
    Memory_16bpp_Mode();

    Display_ON();
    
    Select_Main_Window_16bpp();
    Main_Image_Start_Address(0);        
    Main_Image_Width(LT7683_SCREEN_WIDTH);
    Main_Window_Start_XY(0, 0);
    Canvas_Image_Start_address(0);
    Canvas_image_width(LT7683_SCREEN_WIDTH);
    Active_Window_XY(0, 0);
    Active_Window_WH(LT7683_SCREEN_WIDTH, LT7683_SCREEN_HEIGHT);

    LT7683_spi4ByteAddressMode();
}

void MPU8_8bpp_Memory_Write(unsigned short x, unsigned short y, unsigned short w, unsigned short h, const unsigned char *data) {                             
  unsigned short i,j;
  Graphic_Mode();
  Active_Window_XY(x,y);
  Active_Window_WH(w,h);          
  Goto_Pixel_XY(x,y);
  LT7683_CmdWrite(0x04); 
  
  for(i=0;i< h;i++) { 
    for(j=0;j< w;j++) {    
      Check_Mem_WR_FIFO_not_Full();
      LT7683_DataWrite(*data);
      data++;
    }
  }
  
  Check_Mem_WR_FIFO_Empty();
}  

void MPU8_16bpp_Memory_Write(unsigned short x, unsigned short y, unsigned short w, unsigned short h, const unsigned char *data) {
  unsigned short i,j;
  Graphic_Mode();
  Active_Window_XY(x,y);
  Active_Window_WH(w,h);          
  Goto_Pixel_XY(x,y);
  LT7683_CmdWrite(0x04);

  for(i=0;i< h;i++) { 
    for(j=0;j< w;j++) {
      Check_Mem_WR_FIFO_not_Full();
      LT7683_DataWrite(*data);
      data++;
      Check_Mem_WR_FIFO_not_Full();
      LT7683_DataWrite(*data);
      data++;
    }
  }
  
  Check_Mem_WR_FIFO_Empty();
}    

void MPU8_24bpp_Memory_Write(unsigned short x, unsigned short y, unsigned short w, unsigned short h, const unsigned char *data) {
  unsigned short i,j;
  Graphic_Mode();
  Active_Window_XY(x,y);
  Active_Window_WH(w,h);          
  Goto_Pixel_XY(x,y);
  LT7683_CmdWrite(0x04);

  for(i=0;i< h;i++) { 
    for(j=0;j< w;j++) {
      Check_Mem_WR_FIFO_not_Full();
      LT7683_DataWrite(*data);
      data++;
      Check_Mem_WR_FIFO_not_Full();
      LT7683_DataWrite(*data);
      data++;
      Check_Mem_WR_FIFO_not_Full();
      LT7683_DataWrite(*data);
      data++;
    }
  }
  
  Check_Mem_WR_FIFO_Empty();
}

void MPU16_16bpp_Memory_Write(unsigned short x, unsigned short y, unsigned short w, unsigned short h, const unsigned short *data) {
  unsigned short i,j;
  Graphic_Mode();
  Active_Window_XY(x,y);
  Active_Window_WH(w,h);          
  Goto_Pixel_XY(x,y);
  LT7683_CmdWrite(0x04);

  for(i=0;i< h;i++) { 
    for(j=0;j< w;j++) {
      Check_Mem_WR_FIFO_not_Full();
      LT7683_DataWrite(*data);
      data++;
    }
  }
  
  Check_Mem_WR_FIFO_Empty();
}

void MPU16_24bpp_Mode1_Memory_Write(unsigned short x, unsigned short y, unsigned short w, unsigned short h, const unsigned short *data) {
  unsigned short i,j;
  Graphic_Mode();
  Active_Window_XY(x,y);
  Active_Window_WH(w,h);          
  Goto_Pixel_XY(x,y);
  LT7683_CmdWrite(0x04);

  for(i=0;i< h;i++) { 
    for(j=0;j< w/2;j++) {
      LT7683_DataWrite(*data);
      Check_Mem_WR_FIFO_not_Full();
      data++;
      LT7683_DataWrite(*data);
      Check_Mem_WR_FIFO_not_Full();
      data++;
      LT7683_DataWrite(*data);
      Check_Mem_WR_FIFO_not_Full();
      data++;
    }
  }
  
  Check_Mem_WR_FIFO_Empty();
}

void MPU16_24bpp_Mode2_Memory_Write(unsigned short x, unsigned short y, unsigned short w, unsigned short h, const unsigned short *data) {
  unsigned short i,j;
  Graphic_Mode();
  Active_Window_XY(x,y);
  Active_Window_WH(w,h);          
  Goto_Pixel_XY(x,y);
  LT7683_CmdWrite(0x04);

  for(i=0;i< h;i++) { 
    for(j=0;j< w;j++) {
      Check_Mem_WR_FIFO_not_Full();
      LT7683_DataWrite(*data);
      data++;
      Check_Mem_WR_FIFO_not_Full();
      LT7683_DataWrite(*data);
      data++;
    }
  }
  
  Check_Mem_WR_FIFO_Empty();
}

//------------------------------------- Line -----------------------------------------
void LT7683_DrawLine(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2, unsigned long  LineColor) {
  Foreground_color_65k(LineColor);
  Line_Start_XY(X1,Y1);
  Line_End_XY(X2,Y2);
  Start_Line();
  Check_2D_Busy();
}

void LT7683_DrawLine_Width(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2, unsigned long LineColor, unsigned short Width) {
  unsigned short  i = 0;
  while(Width--) {
    LT7683_DrawLine(X1,Y1+i,X2,Y2+i,LineColor);
    i++;
  }
}

//------------------------------------- Circle -----------------------------------------
void LT7683_DrawCircle(unsigned short XCenter, unsigned short YCenter, unsigned short R, unsigned long CircleColor) {
  Foreground_color_65k(CircleColor);
  Circle_Center_XY(XCenter,YCenter);
  Circle_Radius_R(R);
  Start_Circle_or_Ellipse();
  Check_2D_Busy(); 
}

void LT7683_DrawCircle_Fill(unsigned short XCenter, unsigned short YCenter, unsigned short R, unsigned long ForegroundColor) {
  Foreground_color_65k(ForegroundColor);
  Circle_Center_XY(XCenter,YCenter);
  Circle_Radius_R(R);
  Start_Circle_or_Ellipse_Fill();
  Check_2D_Busy(); 
}

void LT7683_DrawCircle_Width(unsigned short XCenter, unsigned short YCenter, unsigned short R, unsigned long CircleColor, unsigned long ForegroundColor, unsigned short Width) {
  LT7683_DrawCircle_Fill(XCenter,YCenter,R+Width,CircleColor);
  LT7683_DrawCircle_Fill(XCenter,YCenter,R,ForegroundColor);
}

//------------------------------------- Ellipse -----------------------------------------
void LT7683_DrawEllipse(unsigned short XCenter, unsigned short YCenter, unsigned short X_R, unsigned short Y_R, unsigned long EllipseColor) {
  Foreground_color_65k(EllipseColor);
  Ellipse_Center_XY(XCenter,YCenter);
  Ellipse_Radius_RxRy(X_R,Y_R);
  Start_Circle_or_Ellipse();
  Check_2D_Busy(); 
}

void LT7683_DrawEllipse_Fill(unsigned short XCenter, unsigned short YCenter, unsigned short X_R, unsigned short Y_R, unsigned long ForegroundColor) {
  Foreground_color_65k(ForegroundColor);
  Ellipse_Center_XY(XCenter,YCenter);
  Ellipse_Radius_RxRy(X_R,Y_R);
  Start_Circle_or_Ellipse_Fill();
  Check_2D_Busy(); 
}

void LT7683_DrawEllipse_Width(unsigned short XCenter, unsigned short YCenter, unsigned short X_R, unsigned short Y_R, unsigned long EllipseColor, unsigned long ForegroundColor, unsigned short Width) {
  LT7683_DrawEllipse_Fill(XCenter,YCenter,X_R+Width,Y_R+Width,EllipseColor);
  LT7683_DrawEllipse_Fill(XCenter,YCenter,X_R,Y_R,ForegroundColor);
}

//------------------------------------- Square -----------------------------------------
void LT7683_DrawSquare(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2, unsigned long SquareColor) {
  Foreground_color_65k(SquareColor);
  Square_Start_XY(X1,Y1);
  Square_End_XY(X2,Y2);
  Start_Square();
  Check_2D_Busy(); 
}

void LT7683_DrawSquare_Fill(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2, unsigned long ForegroundColor) {
  Foreground_color_65k(ForegroundColor);
  Square_Start_XY(X1,Y1);
  Square_End_XY(X2,Y2);
  Start_Square_Fill();
  Check_2D_Busy();
}

void LT7683_DrawSquare_Width(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2,
                             unsigned long SquareColor, unsigned long ForegroundColor, unsigned short Width) {
  LT7683_DrawSquare_Fill(X1-Width,Y1-Width,X2+Width,Y2+Width,SquareColor);
  LT7683_DrawSquare_Fill(X1,Y1,X2,Y2,ForegroundColor);
}

//------------------------------------- CircleSquare -----------------------------------------
void LT7683_DrawCircleSquare(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2,
                             unsigned short X_R, unsigned short Y_R, unsigned long CircleSquareColor) {
  Foreground_color_65k(CircleSquareColor);
  Square_Start_XY(X1,Y1);
  Square_End_XY(X2,Y2); 
  Circle_Square_Radius_RxRy(X_R,Y_R);
  Start_Circle_Square();
  Check_2D_Busy();
}

void LT7683_DrawCircleSquare_Fill(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2,
                                  unsigned short X_R, unsigned short Y_R, unsigned long ForegroundColor) {
  Foreground_color_65k(ForegroundColor);
  Square_Start_XY(X1,Y1);
  Square_End_XY(X2,Y2); 
  Circle_Square_Radius_RxRy(X_R,Y_R);
  Start_Circle_Square_Fill();
  Check_2D_Busy(); 
}

void LT7683_DrawCircleSquare_Width(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2, unsigned short X_R, unsigned short Y_R, 
                                   unsigned long CircleSquareColor, unsigned long ForegroundColor, unsigned short Width) {
  LT7683_DrawCircleSquare_Fill(X1-Width,Y1-Width,X2+Width,Y2+Width,X_R,Y_R,CircleSquareColor);
  LT7683_DrawCircleSquare_Fill(X1,Y1,X2,Y2,X_R,Y_R,ForegroundColor);
}

//------------------------------------- Triangle -----------------------------------------
void LT7683_DrawTriangle(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2,
                         unsigned short X3, unsigned short Y3, unsigned long TriangleColor) {
  Foreground_color_65k(TriangleColor);
  Triangle_Point1_XY(X1,Y1);
  Triangle_Point2_XY(X2,Y2);
  Triangle_Point3_XY(X3,Y3);
  Start_Triangle();
  Check_2D_Busy(); 
}

void LT7683_DrawTriangle_Fill(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2, 
                              unsigned short X3, unsigned short Y3, unsigned long ForegroundColor) {
  Foreground_color_65k(ForegroundColor);
  Triangle_Point1_XY(X1,Y1);
  Triangle_Point2_XY(X2,Y2);
  Triangle_Point3_XY(X3,Y3);
  Start_Triangle_Fill();
  Check_2D_Busy();
}

//------------------------------------- 1/4 Curve -----------------------------------------
void LT7683_DrawLeftUpCurve(unsigned short XCenter, unsigned short YCenter, unsigned short X_R, unsigned short Y_R, unsigned long CurveColor) {
  Foreground_color_65k(CurveColor);
  Ellipse_Center_XY(XCenter,YCenter);
  Ellipse_Radius_RxRy(X_R,Y_R);
  Start_Left_Up_Curve();
  Check_2D_Busy(); 
}

void LT7683_DrawLeftDownCurve(unsigned short XCenter, unsigned short YCenter, unsigned short X_R, unsigned short Y_R, unsigned long CurveColor) {
  Foreground_color_65k(CurveColor);
  Ellipse_Center_XY(XCenter,YCenter);
  Ellipse_Radius_RxRy(X_R,Y_R);
  Start_Left_Down_Curve();
  Check_2D_Busy(); 
}

void LT7683_DrawRightUpCurve(unsigned short XCenter, unsigned short YCenter, unsigned short X_R, unsigned short Y_R, unsigned long CurveColor) {
  Foreground_color_65k(CurveColor);
  Ellipse_Center_XY(XCenter,YCenter);
  Ellipse_Radius_RxRy(X_R,Y_R);
  Start_Right_Up_Curve();
  Check_2D_Busy(); 
}

void LT7683_DrawRightDownCurve(unsigned short XCenter, unsigned short YCenter, unsigned short X_R, unsigned short Y_R, unsigned long CurveColor) {
  Foreground_color_65k(CurveColor);
  Ellipse_Center_XY(XCenter,YCenter);
  Ellipse_Radius_RxRy(X_R,Y_R);
  Start_Right_Down_Curve();
  Check_2D_Busy(); 
}

void LT7683_SelectDrawCurve(unsigned short XCenter, unsigned short YCenter, unsigned short X_R, unsigned short Y_R, unsigned long CurveColor, unsigned short Dir) {
  switch(Dir) {
    case 0:LT7683_DrawLeftDownCurve(XCenter,YCenter,X_R,Y_R,CurveColor); break;
    case 1:LT7683_DrawLeftUpCurve(XCenter,YCenter,X_R,Y_R,CurveColor); break;
    case 2:LT7683_DrawRightUpCurve(XCenter,YCenter,X_R,Y_R,CurveColor); break;
    case 3:LT7683_DrawRightDownCurve(XCenter,YCenter,X_R,Y_R,CurveColor); break;
    default: break;
  }
}

//------------------------------------- 1/4 Curve_Fill -----------------------------------------
void LT7683_DrawLeftUpCurve_Fill(unsigned short XCenter, unsigned short YCenter, unsigned short X_R, unsigned short Y_R, unsigned long ForegroundColor) {
  Foreground_color_65k(ForegroundColor);
  Ellipse_Center_XY(XCenter,YCenter);
  Ellipse_Radius_RxRy(X_R,Y_R);
  Start_Left_Up_Curve_Fill();
  Check_2D_Busy(); 
}

void LT7683_DrawLeftDownCurve_Fill(unsigned short XCenter, unsigned short YCenter, unsigned short X_R, unsigned short Y_R, unsigned long ForegroundColor) {
  Foreground_color_65k(ForegroundColor);
  Ellipse_Center_XY(XCenter,YCenter);
  Ellipse_Radius_RxRy(X_R,Y_R);
  Start_Left_Down_Curve_Fill();
  Check_2D_Busy(); 
}

void LT7683_DrawRightUpCurve_Fill(unsigned short XCenter, unsigned short YCenter, unsigned short X_R, unsigned short Y_R, unsigned long ForegroundColor) {
  Foreground_color_65k(ForegroundColor);
  Ellipse_Center_XY(XCenter,YCenter);
  Ellipse_Radius_RxRy(X_R,Y_R);
  Start_Right_Up_Curve_Fill();
  Check_2D_Busy(); 
}

void LT7683_DrawRightDownCurve_Fill(unsigned short XCenter, unsigned short YCenter, unsigned short X_R, unsigned short Y_R, unsigned long ForegroundColor) {
  Foreground_color_65k(ForegroundColor);
  Ellipse_Center_XY(XCenter,YCenter);
  Ellipse_Radius_RxRy(X_R,Y_R);
  Start_Right_Down_Curve_Fill();
  Check_2D_Busy(); 
}

void LT7683_SelectDrawCurve_Fill(unsigned short XCenter, unsigned short YCenter, unsigned short X_R, unsigned short Y_R, 
                                 unsigned long CurveColor, unsigned short Dir) {
  switch(Dir) {
    case 0:LT7683_DrawLeftDownCurve_Fill(XCenter,YCenter,X_R,Y_R,CurveColor); break;
    case 1:LT7683_DrawLeftUpCurve_Fill(XCenter,YCenter,X_R,Y_R,CurveColor); break;
    case 2:LT7683_DrawRightUpCurve_Fill(XCenter,YCenter,X_R,Y_R,CurveColor); break;
    case 3:LT7683_DrawRightDownCurve_Fill(XCenter,YCenter,X_R,Y_R,CurveColor); break;
    default: break;
  }
}

void LT7683_DrawQuadrilateral(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2, 
                              unsigned short X3, unsigned short Y3, unsigned short X4, unsigned short Y4,
                              unsigned long ForegroundColor) {
  Foreground_color_65k(ForegroundColor);
  Triangle_Point1_XY(X1,Y1);
  Triangle_Point2_XY(X2,Y2);
  Triangle_Point3_XY(X3,Y3);
  Ellipse_Radius_RxRy(X4,Y4);
  
  LT7683_CmdWrite(0x67);
  LT7683_DataWrite(0x8d);
  Check_Busy_Draw();
  
  Check_2D_Busy(); 
}

void LT7683_DrawQuadrilateral_Fill(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2, 
                                   unsigned short X3, unsigned short Y3, unsigned short X4, unsigned short Y4,
                                   unsigned long ForegroundColor) {
  Foreground_color_65k(ForegroundColor);
  Triangle_Point1_XY(X1,Y1);
  Triangle_Point2_XY(X2,Y2);
  Triangle_Point3_XY(X3,Y3);
  Ellipse_Radius_RxRy(X4,Y4);
  
  LT7683_CmdWrite(0x67);
  LT7683_DataWrite(0xa7);
  Check_Busy_Draw();
  
  Check_2D_Busy(); 
}

void LT7683_DrawPentagon(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2,
                         unsigned short X3, unsigned short Y3, unsigned short X4, unsigned short Y4, 
                         unsigned short X5, unsigned short Y5, unsigned long ForegroundColor) {
  Foreground_color_65k(ForegroundColor);
  Triangle_Point1_XY(X1,Y1);
  Triangle_Point2_XY(X2,Y2);
  Triangle_Point3_XY(X3,Y3);
  Ellipse_Radius_RxRy(X4,Y4);
  Ellipse_Center_XY(X5,Y5);
  
  LT7683_CmdWrite(0x67);
  LT7683_DataWrite(0x8F);
  Check_Busy_Draw();
  
  Check_2D_Busy(); 
}

void LT7683_DrawPentagon_Fill(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2, 
                              unsigned short X3, unsigned short Y3, unsigned short X4, unsigned short Y4,
                              unsigned short X5, unsigned short Y5, unsigned long ForegroundColor) {
  Foreground_color_65k(ForegroundColor);
  Triangle_Point1_XY(X1,Y1);
  Triangle_Point2_XY(X2,Y2);
  Triangle_Point3_XY(X3,Y3);
  Ellipse_Radius_RxRy(X4,Y4);
  Ellipse_Center_XY(X5,Y5);
  
  LT7683_CmdWrite(0x67);
  LT7683_DataWrite(0xa9);
  Check_Busy_Draw();
  
  Check_2D_Busy(); 
}

unsigned char LT7683_DrawCylinder(unsigned short XCenter, unsigned short YCenter, unsigned short X_R, unsigned short Y_R, 
                                  unsigned short H, unsigned long CylinderColor, unsigned long ForegroundColor) {
  if(YCenter < H) return 1;
  
  LT7683_DrawEllipse_Fill(XCenter,YCenter,X_R,Y_R,ForegroundColor);
  LT7683_DrawEllipse(XCenter,YCenter,X_R,Y_R,CylinderColor);

  LT7683_DrawSquare_Fill(XCenter-X_R,YCenter-H,XCenter+X_R,YCenter,ForegroundColor);

  LT7683_DrawEllipse_Fill(XCenter,YCenter-H,X_R,Y_R,ForegroundColor);
  LT7683_DrawEllipse(XCenter,YCenter-H,X_R,Y_R,CylinderColor);
  
  LT7683_DrawLine(XCenter-X_R,YCenter,XCenter-X_R,YCenter-H,CylinderColor);
  LT7683_DrawLine(XCenter+X_R,YCenter,XCenter+X_R,YCenter-H,CylinderColor);
  
  return 0;
}

void LT7683_DrawQuadrangular(unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2,
                            unsigned short X3, unsigned short Y3, unsigned short X4, unsigned short Y4, 
                            unsigned short X5, unsigned short Y5, unsigned short X6, unsigned short Y6,
                            unsigned long QuadrangularColor, unsigned long ForegroundColor) {
  LT7683_DrawSquare_Fill(X1,Y1,X5,Y5,ForegroundColor);
  LT7683_DrawSquare(X1,Y1,X5,Y5,QuadrangularColor);
  
  LT7683_DrawQuadrilateral_Fill(X1,Y1,X2,Y2,X3,Y3,X4,Y4,ForegroundColor);
  LT7683_DrawQuadrilateral(X1,Y1,X2,Y2,X3,Y3,X4,Y4,QuadrangularColor);
  
  LT7683_DrawQuadrilateral_Fill(X3,Y3,X4,Y4,X5,Y5,X6,Y6,ForegroundColor);
  LT7683_DrawQuadrilateral(X3,Y3,X4,Y4,X5,Y5,X6,Y6,QuadrangularColor);
}

void LT7683_MakeTable(unsigned short X1, unsigned short Y1, unsigned short W, unsigned short H,                   
                      unsigned short Line, unsigned short Row, unsigned long  TableColor, unsigned long  ItemColor,           
                      unsigned long  ForegroundColor, unsigned short width1, unsigned short width2, unsigned char mode) {
  unsigned short i = 0;
  unsigned short x2,y2;
  x2 = X1 + W * Row;
  y2 = Y1 + H * Line;
  
  LT7683_DrawSquare_Width(X1,Y1,x2,y2,TableColor,ForegroundColor,width2);  
  
  if(mode == 0)       LT7683_DrawSquare_Fill(X1,Y1,X1+W,y2,ItemColor);  
  else if(mode == 1)  LT7683_DrawSquare_Fill(X1,Y1,x2,Y1+H,ItemColor); 
  
  for(i = 0 ; i < Line ; i++) {
    LT7683_DrawLine_Width(X1,Y1+i*H,x2,Y1+i*H,TableColor,width1);
  }
  
  for(i = 0 ; i < Row ; i++) {
    LT7683_DrawLine_Width(X1+i*W,Y1,X1+i*W,y2,TableColor,width1);
  }
}

// TODO: SCS and Clk can be constants when only 1 SPI device connected to LT7683; 
// SCS = 0, Clk = 0
void LT7683_DMA_24bit_Linear(unsigned char SCS, unsigned char Clk, unsigned long flash_addr,
                            unsigned long memory_addr, unsigned long data_num) {
  Enable_SFlash_SPI();                               
  if(SCS == 0)    Select_SFI_0();                    // Select SPI0
  if(SCS == 1)    Select_SFI_1();                    // Select SPI1
  
  Memory_Linear_Mode();
  Select_SFI_DMA_Mode();                             // Select SPI DMA mode
  
  SPI_Clock_Period(Clk);                             // SPI speed
  SFI_DMA_Destination_Start_Address(memory_addr);    
  SFI_DMA_Transfer_Number(data_num);                 
  SFI_DMA_Source_Start_Address(flash_addr);          
  Check_Busy_SFI_DMA(); 
  Start_SFI_DMA();
  Check_Busy_SFI_DMA();
  Memory_XY_Mode();
}

// TODO: SCS and Clk can be constants when only 1 SPI device connected to LT7683; 
// SCS = 0, Clk = 0
void LT7683_DMA_32bit_Linear(unsigned char SCS, unsigned char Clk, unsigned long flash_addr,
                            unsigned long memory_addr, unsigned long data_num) {
  Enable_SFlash_SPI();                               
  if(SCS == 0)    Select_SFI_0();                    // Select SPI0
  if(SCS == 1)    Select_SFI_1();                    // Select SPI1
  
  Memory_Linear_Mode();
  Select_SFI_DMA_Mode();                             // Select SPI DMA mode
  Select_SFI_32bit_Address();
  
  SPI_Clock_Period(Clk);                             // SPI speed
  SFI_DMA_Destination_Start_Address(memory_addr);    
  SFI_DMA_Transfer_Number(data_num);                
  SFI_DMA_Source_Start_Address(flash_addr);          
  Check_Busy_SFI_DMA(); 
  Start_SFI_DMA();
  Check_Busy_SFI_DMA();
  Memory_XY_Mode();
}

// TODO: SCS and Clk can be constants when only 1 SPI device connected to LT7683; 
// SCS = 0, Clk = 0
void LT7683_DMA_24bit_Block(unsigned char SCS, unsigned char Clk, unsigned short X1, unsigned short Y1,
                           unsigned short X_W, unsigned short Y_H, unsigned short P_W, unsigned long Addr) {
  Enable_SFlash_SPI();                            
  if(SCS == 0)  Select_SFI_0();                   // Select SPI0
  if(SCS == 1)  Select_SFI_1();                   // Select SPI1
	
  Memory_XY_Mode();                     
  Select_SFI_DMA_Mode();                          // Select SPI DMA mode
  SPI_Clock_Period(Clk);                          // Select SPI clock

  Goto_Pixel_XY(X1,Y1);                           // Setting the location of memory in the graphic mode
  SFI_DMA_Destination_Upper_Left_Corner(X1,Y1);   // DMA destination(SDRAM address)
  SFI_DMA_Transfer_Width_Height(X_W,Y_H);         // Setting Block data: width&height
  SFI_DMA_Source_Width(P_W);                      // Setting the width of the source data
  SFI_DMA_Source_Start_Address(Addr);             // Setting the FLASH address of the source data

  Start_SFI_DMA();                                
  Check_Busy_SFI_DMA();                        
}

// TODO: SCS and Clk can be constants when only 1 SPI device connected to LT7683; 
// SCS = 0, Clk = 0
void LT7683_DMA_32bit_Block(unsigned char SCS, unsigned char Clk, unsigned short X1, unsigned short Y1,
                           unsigned short X_W, unsigned short Y_H, unsigned short P_W, unsigned long Addr) {
  Enable_SFlash_SPI();                  
  if(SCS == 0)  Select_SFI_0();                          
  if(SCS == 1)  Select_SFI_1();                         
   
  Select_SFI_DMA_Mode();                  
  SPI_Clock_Period(Clk);

  Select_SFI_32bit_Address();               

  Goto_Pixel_XY(X1,Y1);                   
  SFI_DMA_Destination_Upper_Left_Corner(X1,Y1);     
  SFI_DMA_Transfer_Width_Height(X_W,Y_H);         
  SFI_DMA_Source_Width(P_W);                
  SFI_DMA_Source_Start_Address(Addr);              

  Start_SFI_DMA();                    
  Check_Busy_SFI_DMA();                
  Select_SFI_24bit_Address();
}

void LT7683_Select_Internal_Font_Init(unsigned char Size, unsigned char XxN, unsigned char YxN, 
                                     unsigned char ChromaKey, unsigned char Alignment) {
  if(Size==16)  Font_Select_8x16_16x16();
  if(Size==24)  Font_Select_12x24_24x24();
  if(Size==32)  Font_Select_16x32_32x32();

  //(*)
  if(XxN==1)  Font_Width_X1();
  if(XxN==2)  Font_Width_X2();
  if(XxN==3)  Font_Width_X3();
  if(XxN==4)  Font_Width_X4();

  //(*) 
  if(YxN==1)  Font_Height_X1();
  if(YxN==2)  Font_Height_X2();
  if(YxN==3)  Font_Height_X3();
  if(YxN==4)  Font_Height_X4();

  //(*)
  if(ChromaKey==0)  Font_Background_select_Color(); 
  if(ChromaKey==1)  Font_Background_select_Transparency();  

  //(*)
  if(Alignment==0)  Disable_Font_Alignment();
  if(Alignment==1)  Enable_Font_Alignment();
}

void LT7683_Print_Internal_Font_String(unsigned short x, unsigned short y, unsigned long FontColor,
                                       unsigned long BackGroundColor, char *c) {
  Text_Mode();
  CGROM_Select_Internal_CGROM();
  Foreground_color_65k(FontColor);
  Background_color_65k(BackGroundColor);

  // this is here so that the cursor continues to advance when writing text
  // left to right
  if (x < 65500) {
    Goto_Text_XY(x,y);
  }
  
  Show_String(c);
}

void Enable_PLL(void) {
/*  0: PLL disable; allow change PLL parameter.
    1: PLL enable; cannot change PLL parameter.*/
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  LT7683_DataWrite(temp);

  delay_us(1); //PLL 
}

void LT7683_Sleep(void) {
/*  0: Normal mode.
    1: Sleep mode.*/
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void LT7683_WakeUp(void) {
/*  0: Normal mode.
    1: Sleep mode.*/
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void Key_Scan_Enable(void) {
/*  0: Disable.
    1: Enable.*/
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  LT7683_DataWrite(temp);    
}

void Key_Scan_Disable(void) {
/*  0: Disable.
    1: Enable.*/
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  LT7683_DataWrite(temp);    
}

void TFT_24bit(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp &= cClrb4;
      temp &= cClrb3;
  LT7683_DataWrite(temp);    
}

void TFT_18bit(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp &= cClrb4;
  temp |= cSetb3;
  LT7683_DataWrite(temp);  
}

void TFT_16bit(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  temp &= cClrb3;
  LT7683_DataWrite(temp);  
}

void TFT_LVDS(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  temp |= cSetb3;
  LT7683_DataWrite(temp);  
}

void LT7683_I2CM_Enable(void) {
/*  I2C master Interface Enable/Disable
    0: Disable
    1: Enable*/
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp |= cSetb2;
  LT7683_DataWrite(temp);    
}

void LT7683_I2CM_Disable(void) {
/*  I2C master Interface Enable/Disable
    0: Disable
    1: Enable*/
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp &= cClrb2;
  LT7683_DataWrite(temp);     
}

void Enable_SFlash_SPI(void) {
/*  Serial Flash SPI Interface Enable/Disable
    0: Disable
    1: Enable*/
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  LT7683_DataWrite(temp);     
}

void Disable_SFlash_SPI(void) {
/*  Serial Flash SPI Interface Enable/Disable
    0: Disable
    1: Enable*/
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  LT7683_DataWrite(temp);     
}

void Host_Bus_8bit(void) {
/*  Parallel Host Data Bus Width Selection
    0: 8-bit Parallel Host Data Bus.
    1: 16-bit Parallel Host Data Bus.*/
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Host_Bus_16bit(void) {
/*  Parallel Host Data Bus Width Selection
    0: 8-bit Parallel Host Data Bus.
    1: 16-bit Parallel Host Data Bus.*/
  unsigned char temp;
  LT7683_CmdWrite(0x01);
  temp = LT7683_DataRead();
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void RGB_8b_8bpp(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void RGB_8b_16bpp(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void RGB_8b_24bpp(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void RGB_16b_8bpp(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void RGB_16b_16bpp(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void RGB_16b_24bpp_mode1(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void RGB_16b_24bpp_mode2(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void MemRead_Left_Right_Top_Down(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void MemRead_Right_Left_Top_Down(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void MemRead_Top_Down_Left_Right(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void MemRead_Down_Top_Left_Right(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void MemWrite_Left_Right_Top_Down(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp &= cClrb2;
  temp &= cClrb1;
  LT7683_DataWrite(temp);
}

void MemWrite_Right_Left_Top_Down(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp &= cClrb2;
  temp |= cSetb1;
  LT7683_DataWrite(temp);
}

void MemWrite_Top_Down_Left_Right(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp |= cSetb2;
  temp &= cClrb1;
  LT7683_DataWrite(temp);
}

void MemWrite_Down_Top_Left_Right(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x02);
  temp = LT7683_DataRead();
  temp |= cSetb2;
  temp |= cSetb1;
  LT7683_DataWrite(temp);
}

void Interrupt_Active_Low(void) {
/*  MPU Interrupt active level
    0 : active low.
    1 : active high.*/
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  LT7683_DataWrite(temp);
}

void Interrupt_Active_High(void) {
/*  MPU Interrupt active level
    0 : active low.
    1 : active high.*/
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  LT7683_DataWrite(temp);
}

void ExtInterrupt_Debounce(void) {
/*  External interrupt de-bounce
    0 : without de-bounce
    1 : enable de-bounce (1024 OSC clock)*/
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void ExtInterrupt_Nodebounce(void) {
/*  External interrupt de-bounce
    0 : without de-bounce
    1 : enable de-bounce (1024 OSC clock)*/
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void ExtInterrupt_Input_Low_Level_Trigger(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void ExtInterrupt_Input_High_Level_Trigger(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void ExtInterrupt_Input_Falling_Edge_Trigger(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void ExtInterrupt_Input_Rising_Edge_Trigger(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void LVDS_Format1(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  LT7683_DataWrite(temp);
}

void LVDS_Format2(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  LT7683_DataWrite(temp);
}

void Graphic_Mode(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void Text_Mode(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Memory_Select_SDRAM(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  temp &= cClrb0; // B
  LT7683_DataWrite(temp);
}

void Memory_Select_Graphic_Cursor_RAM(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Memory_Select_Color_Palette_RAM(void) {
  unsigned char temp;
  LT7683_CmdWrite(0x03);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Enable_Resume_Interrupt(void) {
/*
Resume Interrupt Enable
0: Disable.
1: Enable.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  LT7683_DataWrite(temp);
}

void Disable_Resume_Interrupt(void) {
/*
Resume Interrupt Enable
0: Disable.
1: Enable.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  LT7683_DataWrite(temp);
}

void Enable_ExtInterrupt_Input(void) {
/*
External Interrupt (PS[0] pin) Enable
0: Disable.
1: Enable.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void Disable_ExtInterrupt_Input(void) {
/*
External Interrupt (PS[0] pin) Enable
0: Disable.
1: Enable.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void Enable_I2CM_Interrupt(void) {
/*
I2C Master Interrupt Enable
0: Disable.
1: Enable.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  LT7683_DataWrite(temp);
}

void Disable_I2CM_Interrupt(void) {
/*
I2C Master Interrupt Enable
0: Disable.
1: Enable.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  LT7683_DataWrite(temp);
}

void Enable_Vsync_Interrupt(void) {
/*
Vsync time base interrupt Enable Bit
0: Disable Interrupt.
1: Enable Interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void Disable_Vsync_Interrupt(void) {
/*
Vsync time base interrupt Enable Bit
0: Disable Interrupt.
1: Enable Interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void Enable_KeyScan_Interrupt(void) {
/*
Key Scan Interrupt Enable Bit
0: Disable Key scan interrupt.
1: Enable Key scan interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  LT7683_DataWrite(temp);
}

void Disable_KeyScan_Interrupt(void) {
/*
Key Scan Interrupt Enable Bit
0: Disable Key scan interrupt.
1: Enable Key scan interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  LT7683_DataWrite(temp);
}

void Enable_DMA_Draw_BTE_Interrupt(void) {
/*
Serial flash DMA Complete | Draw task finished | BTE Process
Complete etc. Interrupt Enable
0: Disable Interrupt.
1: Enable Interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Disable_DMA_Draw_BTE_Interrupt(void) {
/*
Serial flash DMA Complete | Draw task finished | BTE Process
Complete etc. Interrupt Enable
0: Disable Interrupt.
1: Enable Interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void Enable_PWM1_Interrupt(void) {
/*
PWM timer 1 Interrupt Enable Bit
0: Disable Interrupt.
1: Enable Interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  LT7683_DataWrite(temp);
}

void Disable_PWM1_Interrupt(void) {
/*
PWM timer 1 Interrupt Enable Bit
0: Disable Interrupt.
1: Enable Interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  LT7683_DataWrite(temp);
}

void Enable_PWM0_Interrupt(void) {
/*
PWM timer 0 Interrupt Enable Bit
0: Disable Interrupt.
1: Enable Interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Disable_PWM0_Interrupt(void) {
/*
PWM timer 0 Interrupt Enable Bit
0: Disable Interrupt.
1: Enable Interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0B);
  temp = LT7683_DataRead();
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

unsigned char Read_Interrupt_status(void) {
/*
[Bit7]Read Function ..Resume Interrupt Status
0: No Resume interrupt happens.
1: Resume interrupt happens.
[Bit6]Read Function .. PS[0] pin Interrupt Status
0: No PS[0] pin interrupt happens.
1: PS[0] pin interrupt happens.
[Bit5]Read Function .. I2C master Interrupt Status
0: No I2C master interrupt happens.
1: I2C master interrupt happens.
[Bit4]Read Function .. Vsync Interrupt Status
0: No interrupt happens.
1: interrupt happens.
[Bit3]Read Function ..Key Scan Interrupt Status
0: No Key Scan interrupt happens.
1: Key Scan interrupt happens.
[Bit2]Read Function..Interrupt Status
0: No interrupt happens.
1: interrupt happens.
[Bit1]Read Function..Interrupt Status
0: No interrupt happens.
1: interrupt happens.
[Bit0]Read Function..Interrupt Status
0: No interrupt happens.
1: interrupt happens.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0C);
  temp = LT7683_DataRead();
  return temp;
}

void Clear_Resume_Interrupt_Flag(void) {
/*
Resume Interrupt flag
Write Function .. Resume Interrupt Clear Bit
0: No operation.
1: Clear Resume interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0C);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  LT7683_DataWrite(temp);
}

void Clear_ExtInterrupt_Input_Flag(void) {
/*
External Interrupt (PS[0] pin) flag
Write Function .. PS[0] pin edge Interrupt Clear Bit
0: No operation.
1: Clear the PS[0] pin edge interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0C);
  temp = LT7683_DataRead();
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void Clear_I2CM_Interrupt_Flag(void) {
/*
I2C master Interrupt flag
Write Function.. I2C master Interrupt Clear Bit
0: No operation.
1: Clear the I2C master interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0C);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  LT7683_DataWrite(temp);
}

void Clear_Vsync_Interrupt_Flag(void) {
/*
Vsync Time base interrupt flag
Write Function ..Vsync Interrupt Clear Bit
0: No operation.
1: Clear the interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0C);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void Clear_KeyScan_Interrupt_Flag(void) {
/*
Key Scan Interrupt flag
Write Function..Key Scan Interrupt Clear Bit
0: No operation.
1: Clear the Key Scan interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0C);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  LT7683_DataWrite(temp);
}

void Clear_DMA_Draw_BTE_Interrupt_Flag(void) {
/*
Serial flash DMA Complete | Draw task finished | BTE
Process Complete etc. Interrupt flag
Write Function.. Interrupt Clear Bit
0: No operation.
1: Clear interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0C);
  temp = LT7683_DataRead();
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Clear_PWM1_Interrupt_Flag(void) {
/*
PWM 1 timer Interrupt flag
Write Function..Interrupt Clear Bit
0: No operation.
1: Clear interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0C);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  LT7683_DataWrite(temp);
}

void Clear_PWM0_Interrupt_Flag(void) {
/*
PWM 0 timer Interrupt flag
Write Function..Interrupt Clear Bit
0: No operation.
1: Clear interrupt.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0C);
  temp = LT7683_DataRead();
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Mask_Resume_Interrupt_Flag(void) {
/*
Mask Resume Interrupt Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  LT7683_DataWrite(temp);
}

void Mask_ExtInterrupt_Input_Flag(void) {
/*
Mask External Interrupt (PS[0] pin) Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void Mask_I2CM_Interrupt_Flag(void) {
/*
Mask I2C Master Interrupt Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  LT7683_DataWrite(temp);
}

void Mask_Vsync_Interrupt_Flag(void) {
/*
Mask Vsync time base interrupt Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void Mask_KeyScan_Interrupt_Flag(void) {
/*
Mask Key Scan Interrupt Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  LT7683_DataWrite(temp);
}

void Mask_DMA_Draw_BTE_Interrupt_Flag(void) {
/*
Mask Serial flash DMA Complete | Draw task finished | BTE
Process Complete etc. Interrupt Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Mask_PWM1_Interrupt_Flag(void) {
/*
Mask PWM timer 1 Interrupt Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  LT7683_DataWrite(temp);
}

void Mask_PWM0_Interrupt_Flag(void) {
/*
Mask PWM timer 0 Interrupt Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Enable_Resume_Interrupt_Flag(void) {
/*
Mask Resume Interrupt Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  LT7683_DataWrite(temp);
}

void Enable_ExtInterrupt_Inpur_Flag(void) {
/*
Mask External Interrupt (PS[0] pin) Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void Enable_I2CM_Interrupt_Flag(void) {
/*
Mask I2C Master Interrupt Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  LT7683_DataWrite(temp);
}

void Enable_Vsync_Interrupt_Flag(void) {
/*
Mask Vsync time base interrupt Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void Enable_KeyScan_Interrupt_Flag(void) {
/*
Mask Key Scan Interrupt Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  LT7683_DataWrite(temp);
}

void Enable_DMA_Draw_BTE_Interrupt_Flag(void) {
/*
Mask Serial flash DMA Complete | Draw task finished | BTE
Process Complete etc. Interrupt Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void Enable_PWM1_Interrupt_Flag(void) {
/*
Mask PWM timer 1 Interrupt Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  LT7683_DataWrite(temp);
}

void Enable_PWM0_Interrupt_Flag(void) {
/*
Mask PWM timer 0 Interrupt Flag
0: Enable.
1: Mask.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0D);
  temp = LT7683_DataRead();
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Enable_GPIOF_PullUp(void) {
/*
GPIO_F[7:0] Pull-Up Enable (XPDAT[23:19, 15:13])
0: Pull-Up Disable
1: Pull-Up Enable
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0E);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  LT7683_DataWrite(temp);
}

void Enable_GPIOE_PullUp(void) {
/*
GPIO_E[7:0] Pull-Up Enable (XPDAT[12:10, 7:3])
0: Pull-Up Disable
1: Pull-Up Enable
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0E);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void Enable_GPIOD_PullUp(void) {
/*
GPIO_D[7:0] Pull-Up Enable (XPDAT[18, 2, 17, 16, 9, 8, 1,0])
0: Pull-Up Disable
1: Pull-Up Enable
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0E);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  LT7683_DataWrite(temp);
}

void Enable_GPIOC_PullUp(void) {
/*
GPIO_C[6:0] Pull-Up Enable (XSDA, XSCL, XnSFCS1,
XnSFCS0, XMISO, XMOSI , XSCK)
0: Pull-Up Disable
1: Pull-Up Enable
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0E);
  temp = LT7683_DataRead();
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Enable_XDB15_8_PullUp(void) {
/*
XDB[15:8] Pull-Up Enable
0: Pull-Up Disable
1: Pull-Up Enable
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0E);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  LT7683_DataWrite(temp);
}

void Enable_XDB7_0_PullUp(void) {
/*
XDB[7:0] Pull-Up Enable
0: Pull-Up Disable
1: Pull-Up Enable
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0E);
  temp = LT7683_DataRead();
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Disable_GPIOF_PullUp(void) {
/*
GPIO_F[7:0] Pull-Up Enable (XPDAT[23:19, 15:13])
0: Pull-Up Disable
1: Pull-Up Enable
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0E);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  LT7683_DataWrite(temp);
}

void Disable_GPIOE_PullUp(void) {
/*
GPIO_E[7:0] Pull-Up Enable (XPDAT[12:10, 7:3])
0: Pull-Up Disable
1: Pull-Up Enable
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0E);
  temp = LT7683_DataRead();
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void Disable_GPIOD_PullUp(void) {
/*
GPIO_D[7:0] Pull-Up Enable (XPDAT[18, 2, 17, 16, 9, 8, 1,0])
0: Pull-Up Disable
1: Pull-Up Enable
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0E);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  LT7683_DataWrite(temp);
}

void Disable_GPIOC_PullUp(void) {
/*
GPIO_C[6:0] Pull-Up Enable (XSDA, XSCL, XnSFCS1,
XnSFCS0, XMISO, XMOSI , XSCK)
0: Pull-Up Disable
1: Pull-Up Enable
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0E);
  temp = LT7683_DataRead();
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void Disable_XDB15_8_PullUp(void) {
/*
XDB[15:8] Pull-Up Enable
0: Pull-Up Disable
1: Pull-Up Enable
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0E);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  LT7683_DataWrite(temp);
}

void Disable_XDB7_0_PullUp(void) {
/*
XDB[7:0] Pull-Up Enable
0: Pull-Up Disable
1: Pull-Up Enable
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0E);
  temp = LT7683_DataRead();
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void XPDAT18_Set_GPIO_D7(void) {
/*
XPDAT[18] ¡V not scan function select
0: GPIO-D7
1: KOUT[4]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  LT7683_DataWrite(temp);
}

void XPDAT18_Set_KOUT4(void) {
/*
XPDAT[18] ¡V not scan function select
0: GPIO-D7
1: KOUT[4]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  LT7683_DataWrite(temp);
}

void XPDAT17_Set_GPIO_D5(void) {
/*
XPDAT[17] ¡V not scan function select
0: GPIO-D5
1: KOUT[2]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void XPDAT17_Set_KOUT2(void) {
/*
XPDAT[17] ¡V not scan function select
0: GPIO-D5
1: KOUT[2]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void XPDAT16_Set_GPIO_D4(void) {
/*
XPDAT[16] ¡V not scan function select
0: GPIO-D4
1: KOUT[1]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  LT7683_DataWrite(temp);
}

void XPDAT16_Set_KOUT1(void) {
/*
XPDAT[16] ¡V not scan function select
0: GPIO-D4
1: KOUT[1]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  LT7683_DataWrite(temp);
}

void XPDAT9_Set_GPIO_D3(void) {
/*
XPDAT[9] ¡V not scan function select
0: GPIO-D3
1: KOUT[3]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void XPDAT9_Set_KOUT3(void) {
/*
XPDAT[9] ¡V not scan function select
0: GPIO-D3
1: KOUT[3]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void XPDAT8_Set_GPIO_D2(void) {
/*
XPDAT[8] ¡V not scan function select
0: GPIO-D2
1: KIN[3]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  LT7683_DataWrite(temp);
}

void XPDAT8_Set_KIN3(void) {
/*
XPDAT[8] ¡V not scan function select
0: GPIO-D2
1: KIN[3]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  LT7683_DataWrite(temp);
}

void XPDAT2_Set_GPIO_D6(void) {
/*
XPDAT[2] ¡V not scan function select
0: GPIO-D6
1: KIN[4]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void XPDAT2_Set_KIN4(void) {
/*
XPDAT[2] ¡V not scan function select
0: GPIO-D6
1: KIN[4]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void XPDAT1_Set_GPIO_D1(void) {
/*
XPDAT[1] ¡V not scan function select
0: GPIO-D1
1: KIN[2]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  LT7683_DataWrite(temp);
}
 
void XPDAT1_Set_KIN2(void) {
/*
XPDAT[1] ¡V not scan function select
0: GPIO-D1
1: KIN[2]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  LT7683_DataWrite(temp);
}

void XPDAT0_Set_GPIO_D0(void) {
/*
XPDAT[0] ¡V not scan function select
0: GPIO-D0
1: KIN[1]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void XPDAT0_Set_KIN1(void) {
/*
XPDAT[0] ¡V not scan function select
0: GPIO-D0
1: KIN[1]
*/
  unsigned char temp;
  LT7683_CmdWrite(0x0F);
  temp = LT7683_DataRead();
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Enable_PIP1(void) {
/*
PIP 1 window Enable/Disable
0 : PIP 1 window disable.
1 : PIP 1 window enable
PIP 1 window always on top of PIP 2 window.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x10);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  LT7683_DataWrite(temp);
}

void Disable_PIP1(void) {
/*
PIP 1 window Enable/Disable
0 : PIP 1 window disable.
1 : PIP 1 window enable
PIP 1 window always on top of PIP 2 window.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x10);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  LT7683_DataWrite(temp);
}

void Enable_PIP2(void) {
/*
PIP 2 window Enable/Disable
0 : PIP 2 window disable.
1 : PIP 2 window enable
PIP 1 window always on top of PIP 2 window.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x10);
  temp = LT7683_DataRead();
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void Disable_PIP2(void) {
/*
PIP 2 window Enable/Disable
0 : PIP 2 window disable.
1 : PIP 2 window enable
PIP 1 window always on top of PIP 2 window.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x10);
  temp = LT7683_DataRead();
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void Select_PIP1_Parameter(void) {
/*
0: To configure PIP 1¡¦s parameters.
1: To configure PIP 2¡¦s parameters..
*/
  unsigned char temp;
  LT7683_CmdWrite(0x10);
  temp = LT7683_DataRead();
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void Select_PIP2_Parameter(void) {
/*
0: To configure PIP 1¡¦s parameters.
1: To configure PIP 2¡¦s parameters..
*/
  unsigned char temp;
  LT7683_CmdWrite(0x10);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void Select_Main_Window_8bpp(void) {
/*
Main Window Color Depth Setting
00b: 8-bpp generic TFT, i.e. 256 colors.
01b: 16-bpp generic TFT, i.e. 65K colors.
1xb: 24-bpp generic TFT, i.e. 1.67M colors.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x10);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void Select_Main_Window_16bpp(void) {
/*
Main Window Color Depth Setting
00b: 8-bpp generic TFT, i.e. 256 colors.
01b: 16-bpp generic TFT, i.e. 65K colors.
1xb: 24-bpp generic TFT, i.e. 1.67M colors.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x10);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Select_Main_Window_24bpp(void) {
/*
Main Window Color Depth Setting
00b: 8-bpp generic TFT, i.e. 256 colors.
01b: 16-bpp generic TFT, i.e. 65K colors.
1xb: 24-bpp generic TFT, i.e. 1.67M colors.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x10);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  //temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Select_PIP2_Window_8bpp(void) {
/*
PIP 1 Window Color Depth Setting
00b: 8-bpp generic TFT, i.e. 256 colors.
01b: 16-bpp generic TFT, i.e. 65K colors.
1xb: 24-bpp generic TFT, i.e. 1.67M colors.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x11);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Select_PIP2_Window_16bpp(void) {
/*
PIP 1 Window Color Depth Setting
00b: 8-bpp generic TFT, i.e. 256 colors.
01b: 16-bpp generic TFT, i.e. 65K colors.
1xb: 24-bpp generic TFT, i.e. 1.67M colors.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x11);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Select_PIP2_Window_24bpp(void) {
/*
PIP 1 Window Color Depth Setting
00b: 8-bpp generic TFT, i.e. 256 colors.
01b: 16-bpp generic TFT, i.e. 65K colors.
1xb: 24-bpp generic TFT, i.e. 1.67M colors.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x11);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Select_PIP1_Window_8bpp(void) {
/*
PIP 2 Window Color Depth Setting
00b: 8-bpp generic TFT, i.e. 256 colors.
01b: 16-bpp generic TFT, i.e. 65K colors.
1xb: 24-bpp generic TFT, i.e. 1.67M colors.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x11);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void Select_PIP1_Window_16bpp(void) {
/*
PIP 2 Window Color Depth Setting
00b: 8-bpp generic TFT, i.e. 256 colors.
01b: 16-bpp generic TFT, i.e. 65K colors.
1xb: 24-bpp generic TFT, i.e. 1.67M colors.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x11);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Select_PIP1_Window_24bpp(void) {
/*
PIP 2 Window Color Depth Setting
00b: 8-bpp generic TFT, i.e. 256 colors.
01b: 16-bpp generic TFT, i.e. 65K colors.
1xb: 24-bpp generic TFT, i.e. 1.67M colors.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x11);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void PCLK_Rising(void) {
/*
PCLK Inversion
0: PDAT, DE, HSYNC etc. Drive(/ change) at PCLK falling edge.
1: PDAT, DE, HSYNC etc. Drive(/ change) at PCLK rising edge.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  LT7683_DataWrite(temp);
}

void PCLK_Falling(void) {
/*
PCLK Inversion
0: PDAT, DE, HSYNC etc. Drive(/ change) at PCLK falling edge.
1: PDAT, DE, HSYNC etc. Drive(/ change) at PCLK rising edge.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  LT7683_DataWrite(temp);
}

void Display_ON(void) {
/*  
Display ON/OFF
0b: Display Off.
1b: Display On.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void Display_OFF(void)
{
/*  
Display ON/OFF
0b: Display Off.
1b: Display On.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void Color_Bar_ON(void) {
/*  
Display Test Color Bar
0b: Disable.
1b: Enable.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  LT7683_DataWrite(temp);
}

void Color_Bar_OFF(void) {
/*  
Display Test Color Bar
0b: Disable.
1b: Enable.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  LT7683_DataWrite(temp);
}

void HSCAN_L_to_R(void) {
/*  
Horizontal Scan Direction
0 : From Left to Right
1 : From Right to Left
PIP window will be disabled when HDIR set as 1.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void HSCAN_R_to_L(void) {
/*  
Horizontal Scan Direction
0 : From Left to Right
1 : From Right to Left
PIP window will be disabled when HDIR set as 1.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void VSCAN_T_to_B(void) {
/*  
Vertical Scan direction
0 : From Top to Bottom
1 : From bottom to Top
PIP window will be disabled when VDIR set as 1.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  LT7683_DataWrite(temp);
}
 
void VSCAN_B_to_T(void) {
/*  
Vertical Scan direction
0 : From Top to Bottom
1 : From bottom to Top
PIP window will be disabled when VDIR set as 1.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  LT7683_DataWrite(temp);
}

void PDATA_Set_RGB(void) {
/*  
parallel PDATA[23:0] Output Sequence
000b : RGB.
001b : RBG.
010b : GRB.
011b : GBR.
100b : BRG.
101b : BGR.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp &=0xf8;
  LT7683_DataWrite(temp);
}

void PDATA_Set_RBG(void) {
/*  
parallel PDATA[23:0] Output Sequence
000b : RGB.
001b : RBG.
010b : GRB.
011b : GBR.
100b : BRG.
101b : BGR.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp &=0xf8;
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void PDATA_Set_GRB(void) {
/*  
parallel PDATA[23:0] Output Sequence
000b : RGB.
001b : RBG.
010b : GRB.
011b : GBR.
100b : BRG.
101b : BGR.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp &=0xf8;
  temp |= cSetb1;
  LT7683_DataWrite(temp);
}

void PDATA_Set_GBR(void) {
/*  
parallel PDATA[23:0] Output Sequence
000b : RGB.
001b : RBG.
010b : GRB.
011b : GBR.
100b : BRG.
101b : BGR.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp &=0xf8;
  temp |= cSetb1;
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void PDATA_Set_BRG(void) {
/*  
parallel PDATA[23:0] Output Sequence
000b : RGB.
001b : RBG.
010b : GRB.
011b : GBR.
100b : BRG.
101b : BGR.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp &=0xf8;
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void PDATA_Set_BGR(void) {
/*  
parallel PDATA[23:0] Output Sequence
000b : RGB.
001b : RBG.
010b : GRB.
011b : GBR.
100b : BRG.
101b : BGR.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp &=0xf8;
  temp |= cSetb2;
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void PDATA_IDLE_STATE(void) {
  unsigned char temp;
  
  LT7683_CmdWrite(0x12);
  temp = LT7683_DataRead();
  temp |=0x07;
  LT7683_DataWrite(temp);
}

void HSYNC_Low_Active(void) {
/*  
HSYNC Polarity
0 : Low active.
1 : High active.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  LT7683_DataWrite(temp);
}

void HSYNC_High_Active(void) {
/*  
HSYNC Polarity
0 : Low active.
1 : High active.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();   
  temp |= cSetb7;
  LT7683_DataWrite(temp);
}

void VSYNC_Low_Active(void) {
/*  
VSYNC Polarity
0 : Low active.
1 : High active.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp &= cClrb6; 
  LT7683_DataWrite(temp);
}

void VSYNC_High_Active(void) {
/*  
VSYNC Polarity
0 : Low active.
1 : High active.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}
 
void DE_Low_Active(void) {
/*  
DE Polarity
0 : High active.
1 : Low active.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  LT7683_DataWrite(temp);
}

void DE_High_Active(void) {
/*  
DE Polarity
0 : High active.
1 : Low active.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  LT7683_DataWrite(temp);
}

void Idle_DE_Low(void) {
/*  
DE IDLE STATE(When STANDBY or DISPLAY OFF )
0 : Pin ¡§DE¡¨ output is low.
1 : Pin ¡§DE¡¨ output is high.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void Idle_DE_High(void) {
/*  
DE IDLE STATE(When STANDBY or DISPLAY OFF )
0 : Pin ¡§DE¡¨ output is low.
1 : Pin ¡§DE¡¨ output is high.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void Set_PCLK(unsigned char val) {
  if(val == 1) PCLK_Falling();
  else PCLK_Rising();
}

void Set_HSYNC_Active(unsigned char val) {
  if(val == 1) HSYNC_High_Active();
  else HSYNC_Low_Active();
}

void Set_VSYNC_Active(unsigned char val) {
  if(val == 1) VSYNC_High_Active();
  else VSYNC_Low_Active();
}

void Set_DE_Active(unsigned char val) {
  if(val == 1) DE_High_Active();
  else DE_Low_Active();
}

void Idle_PCLK_Low(void) {
/*  
PCLK IDLE STATE(When STANDBY or DISPLAY OFF )
0 : Pin ¡§PCLK¡¨ output is low.
1 : Pin ¡§PCLK¡¨ output is high.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  LT7683_DataWrite(temp);
}

void Idle_PCLK_High(void) {
/*  
PCLK IDLE STATE(When STANDBY or DISPLAY OFF )
0 : Pin ¡§PCLK¡¨ output is low.
1 : Pin ¡§PCLK¡¨ output is high.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  LT7683_DataWrite(temp);
}

void Idle_PDAT_Low(void) {
/*  
PDAT IDLE STATE(When STANDBY or DISPLAY OFF )
0 : Pins ¡§PDAT[23:0]¡¨ output is low.
1 : Pins ¡§PCLK[23:0]¡¨ output is high.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void Idle_PDAT_High(void) {
/*  
PDAT IDLE STATE(When STANDBY or DISPLAY OFF )
0 : Pins ¡§PDAT[23:0]¡¨ output is low.
1 : Pins ¡§PCLK[23:0]¡¨ output is high.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Idle_HSYNC_Low(void) {
/*  
HSYNC IDLE STATE(When STANDBY or DISPLAY OFF )
0 : Pin ¡§HSYNC¡¨ output is low.
1 : Pin ¡§HSYNC¡¨ output is high.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  LT7683_DataWrite(temp);
}

void Idle_HSYNC_High(void) {
/*  
HSYNC IDLE STATE(When STANDBY or DISPLAY OFF )
0 : Pin ¡§HSYNC¡¨ output is low.
1 : Pin ¡§HSYNC¡¨ output is high.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  LT7683_DataWrite(temp);
}

void Idle_VSYNC_Low(void) {
/*  
VSYNC IDLE STATE(When STANDBY or DISPLAY OFF )
0 : Pin ¡§VSYNC¡¨ output is low.
1 : Pin ¡§VSYNC¡¨ output is high.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Idle_VSYNC_High(void) {
/*  
VSYNC IDLE STATE(When STANDBY or DISPLAY OFF )
0 : Pin ¡§VSYNC¡¨ output is low.
1 : Pin ¡§VSYNC¡¨ output is high.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x13);
  temp = LT7683_DataRead();
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void LT7683_HorizontalWidth_VerticalHeight(unsigned short WX,unsigned short HY) {
/*
[14h] Horizontal Display Width Setting Bit[7:0]
[15h] Horizontal Display Width Fine Tuning (HDWFT) [3:0]
The register specifies the LCD panel horizontal display width in
the unit of 8 pixels resolution.
Horizontal display width(pixels) = (HDWR + 1) * 8 + HDWFTR

[1Ah] Vertical Display Height Bit[7:0]
Vertical Display Height(Line) = VDHR + 1
[1Bh] Vertical Display Height Bit[10:8]
Vertical Display Height(Line) = VDHR + 1
*/
  unsigned char temp;
  if(WX < 8) {
    LT7683_CmdWrite(0x14);
    LT7683_DataWrite(0x00);
        
    LT7683_CmdWrite(0x15);
    LT7683_DataWrite(WX);
        
    temp = HY - 1;
    LT7683_CmdWrite(0x1A);
    LT7683_DataWrite(temp);
        
    temp = (HY - 1) >> 8;
    LT7683_CmdWrite(0x1B);
    LT7683_DataWrite(temp);
  } else{
    temp=(WX / 8) - 1;
    LT7683_CmdWrite(0x14);
    LT7683_DataWrite(temp);
    
    temp = WX % 8;
    LT7683_CmdWrite(0x15);
    LT7683_DataWrite(temp);
    
    temp = HY - 1;
    LT7683_CmdWrite(0x1A);
    LT7683_DataWrite(temp);
      
    temp=(HY - 1) >> 8;
    LT7683_CmdWrite(0x1B);
    LT7683_DataWrite(temp);
  }
}

void LT7683_Horizontal_Non_Display(unsigned short WX) {
/*
[16h] Horizontal Non-Display Period(HNDR) Bit[4:0]
This register specifies the horizontal non-display period. Also
called back porch.
Horizontal non-display period(pixels) = (HNDR + 1) * 8 + HNDFTR

[17h] Horizontal Non-Display Period Fine Tuning(HNDFT) [3:0]
This register specifies the fine tuning for horizontal non-display
period; it is used to support the SYNC mode panel. Each level of
this modulation is 1-pixel.
Horizontal non-display period(pixels) = (HNDR + 1) * 8 + HNDFTR
*/
  unsigned char temp;
  if(WX < 8) {
    LT7683_CmdWrite(0x16);
    LT7683_DataWrite(0x00);
        
    LT7683_CmdWrite(0x17);
    LT7683_DataWrite(WX);
  } else {
    temp = (WX / 8) - 1;
    LT7683_CmdWrite(0x16);
    LT7683_DataWrite(temp);
    
    temp = WX % 8;
    LT7683_CmdWrite(0x17);
    LT7683_DataWrite(temp);
  } 
}

void LT7683_HSYNC_Start_Position(unsigned short WX) {
/*
[18h] HSYNC Start Position[4:0]
The starting position from the end of display area to the
beginning of HSYNC. Each level of this modulation is 8-pixel.
Also called front porch.
HSYNC Start Position(pixels) = (HSTR + 1)x8
*/
  unsigned char temp;
  if(WX < 8) {
    LT7683_CmdWrite(0x18);
    LT7683_DataWrite(0x00);
  } else {
    temp = (WX / 8) - 1;
    LT7683_CmdWrite(0x18);
    LT7683_DataWrite(temp);  
  }
}

void LT7683_HSYNC_Pulse_Width(unsigned short WX) {
/*
[19h] HSYNC Pulse Width(HPW) [4:0]
The period width of HSYNC.
HSYNC Pulse Width(pixels) = (HPW + 1)x8
*/
  unsigned char temp;
  if(WX < 8) {
    LT7683_CmdWrite(0x19);
    LT7683_DataWrite(0x00);
  } else {
    temp = (WX / 8) - 1;
    LT7683_CmdWrite(0x19);
    LT7683_DataWrite(temp);  
  }
}

void LT7683_Vertical_Non_Display(unsigned short HY) {
/*
[1Ch] Vertical Non-Display Period Bit[7:0]
Vertical Non-Display Period(Line) = (VNDR + 1)

[1Dh] Vertical Non-Display Period Bit[9:8]
Vertical Non-Display Period(Line) = (VNDR + 1)
*/
  unsigned char temp;
  temp=HY-1;
  LT7683_CmdWrite(0x1C);
  LT7683_DataWrite(temp);

  LT7683_CmdWrite(0x1D);
  LT7683_DataWrite(temp>>8);
}

void LT7683_VSYNC_Start_Position(unsigned short HY) {
/*
[1Eh] VSYNC Start Position[7:0]
The starting position from the end of display area to the beginning of VSYNC.
VSYNC Start Position(Line) = (VSTR + 1)
*/
  unsigned char temp;
  temp = HY - 1;
  LT7683_CmdWrite(0x1E);
  LT7683_DataWrite(temp);
}

void LT7683_VSYNC_Pulse_Width(unsigned short HY) {
/*
[1Fh] VSYNC Pulse Width[5:0]
The pulse width of VSYNC in lines.
VSYNC Pulse Width(Line) = (VPWR + 1)
*/
  unsigned char temp;
  temp = HY - 1;
  LT7683_CmdWrite(0x1F);
  LT7683_DataWrite(temp);
}

void Main_Image_Start_Address(unsigned long Addr) {
/*
[20h] Main Image Start Address[7:2]
[21h] Main Image Start Address[15:8]
[22h] Main Image Start Address [23:16]
[23h] Main Image Start Address [31:24]
*/
  LT7683_RegisterWrite(0x20,Addr);
  LT7683_RegisterWrite(0x21,Addr>>8);
  LT7683_RegisterWrite(0x22,Addr>>16);
  LT7683_RegisterWrite(0x23,Addr>>24);
}

void Main_Image_Width(unsigned short WX) {
/*
[24h] Main Image Width [7:0]
[25h] Main Image Width [12:8]
Unit: Pixel.
It must be divisible by 4. MIW Bit [1:0] tie to ¡§0¡¨ internally.
The value is physical pixel number. Maximum value is 8188 pixels
*/
  LT7683_RegisterWrite(0x24,WX);
  LT7683_RegisterWrite(0x25,WX>>8);
}

void Main_Window_Start_XY(unsigned short WX,unsigned short HY) {
/*
[26h] Main Window Upper-Left corner X-coordination [7:0]
[27h] Main Window Upper-Left corner X-coordination [12:8]
Reference Main Image coordination.
Unit: Pixel
It must be divisible by 4. MWULX Bit [1:0] tie to ¡§0¡¨ internally.
X-axis coordination plus Horizontal display width cannot large than 8188.

[28h] Main Window Upper-Left corner Y-coordination [7:0]
[29h] Main Window Upper-Left corner Y-coordination [12:8]
Reference Main Image coordination.
Unit: Pixel
Range is between 0 and 8191.
*/
  LT7683_RegisterWrite(0x26,WX);
  LT7683_RegisterWrite(0x27,WX>>8);

  LT7683_RegisterWrite(0x28,HY);
  LT7683_RegisterWrite(0x29,HY>>8);
}

void PIP_Display_Start_XY(unsigned short WX,unsigned short HY) {
/*
[2Ah] PIP Window Display Upper-Left corner X-coordination [7:0]
[2Bh] PIP Window Display Upper-Left corner X-coordination [12:8]
Reference Main Window coordination.
Unit: Pixel
It must be divisible by 4. PWDULX Bit [1:0] tie to ¡§0¡¨ internally.
X-axis coordination should less than horizontal display width.
According to bit of Select Configure PIP 1 or 2 Window¡¦s parameters. 
Function bit will be configured for relative PIP window.

[2Ch] PIP Window Display Upper-Left corner Y-coordination [7:0]
[2Dh] PIP Window Display Upper-Left corner Y-coordination [12:8]
Reference Main Window coordination.
Unit: Pixel
Y-axis coordination should less than vertical display height.
According to bit of Select Configure PIP 1 or 2 Window¡¦s parameters.
Function bit will be configured for relative PIP window.
*/
  LT7683_RegisterWrite(0x2A,WX);
  LT7683_RegisterWrite(0x2B,WX>>8);
  
  LT7683_RegisterWrite(0x2C,HY);
  LT7683_RegisterWrite(0x2D,HY>>8);
}

void PIP_Image_Start_Address(unsigned long Addr) {
/*
[2Eh] PIP Image Start Address[7:2]
[2Fh] PIP Image Start Address[15:8]
[30h] PIP Image Start Address [23:16]
[31h] PIP Image Start Address [31:24]
*/
  LT7683_RegisterWrite(0x2E,Addr);
  LT7683_RegisterWrite(0x2F,Addr>>8);
  LT7683_RegisterWrite(0x30,Addr>>16);
  LT7683_RegisterWrite(0x31,Addr>>24);
}

void PIP_Image_Width(unsigned short WX) {
/*
[32h] PIP Image Width [7:0]
[33h] PIP Image Width [12:8]
Unit: Pixel.
It must be divisible by 4. PIW Bit [1:0] tie to ¡§0¡¨ internally.
The value is physical pixel number.
This width should less than horizontal display width.
According to bit of Select Configure PIP 1 or 2 Window¡¦s parameters.
Function bit will be configured for relative PIP window.
*/
  LT7683_RegisterWrite(0x32,WX);
  LT7683_RegisterWrite(0x33,WX>>8);
}

void PIP_Window_Image_Start_XY(unsigned short WX,unsigned short HY) {
/*
[34h] PIP 1 or 2 Window Image Upper-Left corner X-coordination [7:0]
[35h] PIP Window Image Upper-Left corner X-coordination [12:8]
Reference PIP Image coordination.
Unit: Pixel
It must be divisible by 4. PWIULX Bit [1:0] tie to ¡§0¡¨ internally.
X-axis coordination plus PIP image width cannot large than 8188.
According to bit of Select Configure PIP 1 or 2 Window¡¦s parameters. 
Function bit will be configured for relative PIP window.

[36h] PIP Windows Display Upper-Left corner Y-coordination [7:0]
[37h] PIP Windows Image Upper-Left corner Y-coordination [12:8]
Reference PIP Image coordination.
Unit: Pixel
Y-axis coordination plus PIP window height should less than 8191.
According to bit of Select Configure PIP 1 or 2 Window¡¦s parameters. 
Function bit will be configured for relative PIP window.
*/
  LT7683_RegisterWrite(0x34,WX);
  LT7683_RegisterWrite(0x35,WX>>8);
  
  LT7683_RegisterWrite(0x36,HY);
  LT7683_RegisterWrite(0x37,HY>>8);
}

void PIP_Window_Width_Height(unsigned short WX,unsigned short HY) {
/*
[38h] PIP Window Width [7:0]
[39h] PIP Window Width [10:8]
Unit: Pixel.
It must be divisible by 4. The value is physical pixel number.
Maximum value is 8188 pixels.
According to bit of Select Configure PIP 1 or 2 Window¡¦s parameters. 
Function bit will be configured for relative PIP window.

[3Ah] PIP Window Height [7:0]
[3Bh] PIP Window Height [10:8]
Unit: Pixel
The value is physical pixel number. Maximum value is 8191 pixels.
According to bit of Select Configure PIP 1 or 2 Window¡¦s parameters. 
Function bit will be configured for relative PIP window.
*/
  LT7683_RegisterWrite(0x38,WX);
  LT7683_RegisterWrite(0x39,WX>>8);
  
  LT7683_RegisterWrite(0x3A,HY);
  LT7683_RegisterWrite(0x3B,HY>>8);
}

void Enable_Graphic_Cursor(void) {
/*
Graphic Cursor Enable
0 : Graphic Cursor disable.
1 : Graphic Cursor enable.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x3C);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void Disable_Graphic_Cursor(void) {
/*
Graphic Cursor Enable
0 : Graphic Cursor disable.
1 : Graphic Cursor enable.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x3C);
  temp = LT7683_DataRead();
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void Select_Graphic_Cursor_1(void) {
/*
Graphic Cursor Selection Bit
Select one from four graphic cursor types. (00b to 11b)
00b : Graphic Cursor Set 1.
01b : Graphic Cursor Set 2.
10b : Graphic Cursor Set 3.
11b : Graphic Cursor Set 4.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x3C);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void Select_Graphic_Cursor_2(void) {
/*
Graphic Cursor Selection Bit
Select one from four graphic cursor types. (00b to 11b)
00b : Graphic Cursor Set 1.
01b : Graphic Cursor Set 2.
10b : Graphic Cursor Set 3.
11b : Graphic Cursor Set 4.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x3C);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Select_Graphic_Cursor_3(void) {
/*
Graphic Cursor Selection Bit
Select one from four graphic cursor types. (00b to 11b)
00b : Graphic Cursor Set 1.
01b : Graphic Cursor Set 2.
10b : Graphic Cursor Set 3.
11b : Graphic Cursor Set 4.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x3C);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void Select_Graphic_Cursor_4(void) {
/*
Graphic Cursor Selection Bit
Select one from four graphic cursor types. (00b to 11b)
00b : Graphic Cursor Set 1.
01b : Graphic Cursor Set 2.
10b : Graphic Cursor Set 3.
11b : Graphic Cursor Set 4.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x3C);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Enable_Text_Cursor(void) {
/*
Text Cursor Enable
0 : Disable.
1 : Enable.
Text cursor & Graphic cursor cannot enable simultaneously.
Graphic cursor has higher priority then Text cursor if enabled simultaneously.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x3C);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  LT7683_DataWrite(temp);
}

void Disable_Text_Cursor(void) {
/*
Text Cursor Enable
0 : Disable.
1 : Enable.
Text cursor & Graphic cursor cannot enable simultaneously.
Graphic cursor has higher priority then Text cursor if enabled simultaneously.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x3C);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  LT7683_DataWrite(temp);
}

void Enable_Text_Cursor_Blinking(void) {
/*
Text Cursor Blinking Enable
0 : Disable.
1 : Enable.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x3C);
  temp = LT7683_DataRead();
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Disable_Text_Cursor_Blinking(void) {
/*
Text Cursor Blinking Enable
0 : Disable.
1 : Enable.
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x3C);
  temp = LT7683_DataRead();
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Blinking_Time_Frames(unsigned char temp) {
/*
Text Cursor Blink Time Setting (Unit: Frame)
00h : 1 frame time.
01h : 2 frames time.
02h : 3 frames time.
:
FFh : 256 frames time.
*/
  LT7683_CmdWrite(0x3D);
  LT7683_DataWrite(temp);
}

void Text_Cursor_H_V(unsigned short WX,unsigned short HY) {
/*
[3Eh]
Text Cursor Horizontal Size Setting[4:0]
Unit : Pixel
Zero-based number. Value ¡§0¡¨ means 1 pixel.
Note : When font is enlarged, the cursor setting will multiply the
same times as the font enlargement.
[3Fh]
Text Cursor Vertical Size Setting[4:0]
Unit : Pixel
Zero-based number. Value ¡§0¡¨ means 1 pixel.
Note : When font is enlarged, the cursor setting will multiply the
same times as the font enlargement.
*/
  LT7683_CmdWrite(0x3E);
  LT7683_DataWrite(WX);
  LT7683_CmdWrite(0x3F);
  LT7683_DataWrite(HY);
}

void Graphic_Cursor_XY(unsigned short WX,unsigned short HY) {
/*
[40h] Graphic Cursor Horizontal Location[7:0]
[41h] Graphic Cursor Horizontal Location[12:8]
[42h] Graphic Cursor Vertical Location[7:0]
[43h] Graphic Cursor Vertical Location[12:8]
Reference main Window coordination.
*/  
  LT7683_RegisterWrite(0x40,WX);
  LT7683_RegisterWrite(0x41,WX>>8);

  LT7683_RegisterWrite(0x42,HY);
  LT7683_RegisterWrite(0x43,HY>>8);
}

void Set_Graphic_Cursor_Color_1(unsigned char temp) {
/*
[44h] Graphic Cursor Color 0 with 256 Colors
RGB Format [7:0] = RRRGGGBB.
*/  
  LT7683_RegisterWrite(0x44,temp);
}

void Set_Graphic_Cursor_Color_2(unsigned char temp) {
/*
[45h] Graphic Cursor Color 1 with 256 Colors
RGB Format [7:0] = RRRGGGBB.
*/  
  LT7683_RegisterWrite(0x45,temp);
}

void Canvas_Image_Start_address(unsigned long Addr) {
/*
[50h] Start address of Canvas [7:0]
[51h] Start address of Canvas [15:8]
[52h] Start address of Canvas [23:16]
[53h] Start address of Canvas [31:24]
*/
  LT7683_RegisterWrite(0x50,Addr);
  LT7683_RegisterWrite(0x51,Addr>>8);
  LT7683_RegisterWrite(0x52,Addr>>16);
  LT7683_RegisterWrite(0x53,Addr>>24);
}

void Canvas_image_width(unsigned short WX) {
/*
[54h] Canvas image width [7:2]
[55h] Canvas image width [12:8]
*/
  LT7683_RegisterWrite(0x54,WX);
  LT7683_RegisterWrite(0x55,WX>>8);
}

void Active_Window_XY(unsigned short WX,unsigned short HY) {
/*
[56h] Active Window Upper-Left corner X-coordination [7:0]
[57h] Active Window Upper-Left corner X-coordination [12:8]
[58h] Active Window Upper-Left corner Y-coordination [7:0]
[59h] Active Window Upper-Left corner Y-coordination [12:8]
*/
  LT7683_RegisterWrite(0x56,WX);
  LT7683_RegisterWrite(0x57,WX>>8);
  
  LT7683_RegisterWrite(0x58,HY);
  LT7683_RegisterWrite(0x59,HY>>8);
}

void Active_Window_WH(unsigned short WX,unsigned short HY) {
/*
[5Ah] Width of Active Window [7:0]
[5Bh] Width of Active Window [12:8]
[5Ch] Height of Active Window [7:0]
[5Dh] Height of Active Window [12:8]
*/
  LT7683_RegisterWrite(0x5A,WX);
  LT7683_RegisterWrite(0x5B,WX>>8);
 
  LT7683_RegisterWrite(0x5C,HY);
  LT7683_RegisterWrite(0x5D,HY>>8);
}

//[5Eh]=========================================================================
void Select_Write_Data_Position(void) {
/*
Select to read back Graphic Read/Write position
0: read back Graphic Write position
1: read back Graphic Read position
*/
  unsigned char temp;

  LT7683_CmdWrite(0x5E);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  LT7683_DataWrite(temp);
}

void Select_Read_Data_Position(void) {
/*
Select to read back Graphic Read/Write position
0: read back Graphic Write position
1: read back Graphic Read position
*/
  unsigned char temp;

  LT7683_CmdWrite(0x5E);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  LT7683_DataWrite(temp);
}

void Memory_XY_Mode(void) {
/*
Canvas addressing mode
0: Block mode (X-Y coordination addressing)
1: linear mode
*/
  unsigned char temp;

  LT7683_CmdWrite(0x5E);
  temp = LT7683_DataRead();
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void Memory_Linear_Mode(void) {
/*
Canvas addressing mode
0: Block mode (X-Y coordination addressing)
1: linear mode
*/
  unsigned char temp;

  LT7683_CmdWrite(0x5E);
  temp = LT7683_DataRead();
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Memory_8bpp_Mode(void) {
/*
Canvas image¡¦s color depth & memory R/W data width
In Block Mode:
00: 8bpp
01: 16bpp
1x: 24bpp
In Linear Mode:
X0: 8-bits memory data read/write.
X1: 16-bits memory data read/write
*/
  unsigned char temp;

  LT7683_CmdWrite(0x5E);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Memory_16bpp_Mode(void) {
/*
Canvas image¡¦s color depth & memory R/W data width
In Block Mode:
00: 8bpp
01: 16bpp
1x: 24bpp
In Linear Mode:
X0: 8-bits memory data read/write.
X1: 16-bits memory data read/write
*/
  unsigned char temp;

  LT7683_CmdWrite(0x5E);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Memory_24bpp_Mode(void) {
/*
Canvas image¡¦s color depth & memory R/W data width
In Block Mode:
00: 8bpp
01: 16bpp
1x: 24bpp
In Linear Mode:
X0: 8-bits memory data read/write.
X1: 16-bits memory data read/write
*/
  unsigned char temp;

  LT7683_CmdWrite(0x5E);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Goto_Pixel_XY(unsigned short WX,unsigned short HY) {
/*
[Write]: Set Graphic Read/Write position
[Read]: Current Graphic Read/Write position
Read back is Read position or Write position depends on
REG[5Eh] bit3, Select to read back Graphic Read/Write position.
When DPRAM Linear mode:Graphic Read/Write Position [31:24][23:16][15:8][7:0]
When DPRAM Active window mode:Graphic Read/Write 
Horizontal Position [12:8][7:0], 
Vertical Position [12:8][7:0].
Reference Canvas image coordination. Unit: Pixel
*/
  LT7683_RegisterWrite(0x5F,WX);
  LT7683_RegisterWrite(0x60,WX>>8);
  
  LT7683_RegisterWrite(0x61,HY);
  LT7683_RegisterWrite(0x62,HY>>8);
}

void Goto_Linear_Addr(unsigned long Addr) {
 LT7683_RegisterWrite(0x5F,Addr);
 LT7683_RegisterWrite(0x60,Addr>>8); 
 LT7683_RegisterWrite(0x61,Addr>>16);
 LT7683_RegisterWrite(0x62,Addr>>24);
}

void Goto_Text_XY(unsigned short WX,unsigned short HY) {
/*
Write: Set Text Write position
Read: Current Text Write position
Text Write X-coordination [12:8][7:0]
Text Write Y-coordination [12:8][7:0]
Reference Canvas image coordination.
Unit: Pixel
*/
  LT7683_RegisterWrite(0x63,WX);
  LT7683_RegisterWrite(0x64,WX>>8);
  
  LT7683_RegisterWrite(0x65,HY);
  LT7683_RegisterWrite(0x66,HY>>8);
}

/*
[bit7]Draw Line / Triangle Start Signal
Write Function
0 : Stop the drawing function.
1 : Start the drawing function.
Read Function
0 : Drawing function complete.
1 : Drawing function is processing.
[bit5]Fill function for Triangle Signal
0 : Non fill.
1 : Fill.
[bit1]Draw Triangle or Line Select Signal
0 : Draw Line
1 : Draw Triangle
*/
void Start_Line(void) {
  LT7683_CmdWrite(0x67);
  LT7683_DataWrite(0x80);
  Check_Busy_Draw();
}

void Start_Triangle(void) {
  LT7683_CmdWrite(0x67);
  LT7683_DataWrite(0x82);//B1000_0010
  Check_Busy_Draw();
}

void Start_Triangle_Fill(void) {

  LT7683_CmdWrite(0x67);
  LT7683_DataWrite(0xA2);//B1010_0010
  Check_Busy_Draw();
}

void Line_Start_XY(unsigned short WX,unsigned short HY) {
/*
[68h] Draw Line/Square/Triangle Start X-coordination [7:0]
[69h] Draw Line/Square/Triangle Start X-coordination [12:8]
[6Ah] Draw Line/Square/Triangle Start Y-coordination [7:0]
[6Bh] Draw Line/Square/Triangle Start Y-coordination [12:8]
*/
  LT7683_CmdWrite(0x68);
  LT7683_DataWrite(WX);

  LT7683_CmdWrite(0x69);
  LT7683_DataWrite(WX>>8);

  LT7683_CmdWrite(0x6A);
  LT7683_DataWrite(HY);

  LT7683_CmdWrite(0x6B);
  LT7683_DataWrite(HY>>8);
}

void Line_End_XY(unsigned short WX,unsigned short HY) {
/*
[6Ch] Draw Line/Square/Triangle End X-coordination [7:0]
[6Dh] Draw Line/Square/Triangle End X-coordination [12:8]
[6Eh] Draw Line/Square/Triangle End Y-coordination [7:0]
[6Fh] Draw Line/Square/Triangle End Y-coordination [12:8]
*/
  LT7683_CmdWrite(0x6C);
  LT7683_DataWrite(WX);

  LT7683_CmdWrite(0x6D);
  LT7683_DataWrite(WX>>8);

  LT7683_CmdWrite(0x6E);
  LT7683_DataWrite(HY);

  LT7683_CmdWrite(0x6F);
  LT7683_DataWrite(HY>>8);
}

void Triangle_Point1_XY(unsigned short WX,unsigned short HY) {
/*
[68h] Draw Line/Square/Triangle Start X-coordination [7:0]
[69h] Draw Line/Square/Triangle Start X-coordination [12:8]
[6Ah] Draw Line/Square/Triangle Start Y-coordination [7:0]
[6Bh] Draw Line/Square/Triangle Start Y-coordination [12:8]
*/
  LT7683_CmdWrite(0x68);
  LT7683_DataWrite(WX);

  LT7683_CmdWrite(0x69);
  LT7683_DataWrite(WX>>8);

  LT7683_CmdWrite(0x6A);
  LT7683_DataWrite(HY);

  LT7683_CmdWrite(0x6B);
  LT7683_DataWrite(HY>>8);
}

void Triangle_Point2_XY(unsigned short WX,unsigned short HY) {
/*
[6Ch] Draw Line/Square/Triangle End X-coordination [7:0]
[6Dh] Draw Line/Square/Triangle End X-coordination [12:8]
[6Eh] Draw Line/Square/Triangle End Y-coordination [7:0]
[6Fh] Draw Line/Square/Triangle End Y-coordination [12:8]
*/
  LT7683_CmdWrite(0x6C);
  LT7683_DataWrite(WX);

  LT7683_CmdWrite(0x6D);
  LT7683_DataWrite(WX>>8);

  LT7683_CmdWrite(0x6E);
  LT7683_DataWrite(HY);

  LT7683_CmdWrite(0x6F);
  LT7683_DataWrite(HY>>8);
}

void Triangle_Point3_XY (unsigned short WX,unsigned short HY) {
/*
[70h] Draw Triangle Point 3 X-coordination [7:0]
[71h] Draw Triangle Point 3 X-coordination [12:8]
[72h] Draw Triangle Point 3 Y-coordination [7:0]
[73h] Draw Triangle Point 3 Y-coordination [12:8]
*/
  LT7683_CmdWrite(0x70);
  LT7683_DataWrite(WX);

  LT7683_CmdWrite(0x71);
  LT7683_DataWrite(WX>>8);

  LT7683_CmdWrite(0x72);
  LT7683_DataWrite(HY);

  LT7683_CmdWrite(0x73);
  LT7683_DataWrite(HY>>8);
}

void Square_Start_XY(unsigned short WX,unsigned short HY) {
/*
[68h] Draw Line/Square/Triangle Start X-coordination [7:0]
[69h] Draw Line/Square/Triangle Start X-coordination [12:8]
[6Ah] Draw Line/Square/Triangle Start Y-coordination [7:0]
[6Bh] Draw Line/Square/Triangle Start Y-coordination [12:8]
*/
  LT7683_CmdWrite(0x68);
  LT7683_DataWrite(WX);

  LT7683_CmdWrite(0x69);
  LT7683_DataWrite(WX>>8);

  LT7683_CmdWrite(0x6A);
  LT7683_DataWrite(HY);

  LT7683_CmdWrite(0x6B);
  LT7683_DataWrite(HY>>8);
}

void Square_End_XY(unsigned short WX,unsigned short HY) {
/*
[6Ch] Draw Line/Square/Triangle End X-coordination [7:0]
[6Dh] Draw Line/Square/Triangle End X-coordination [12:8]
[6Eh] Draw Line/Square/Triangle End Y-coordination [7:0]
[6Fh] Draw Line/Square/Triangle End Y-coordination [12:8]
*/
  LT7683_CmdWrite(0x6C);
  LT7683_DataWrite(WX);

  LT7683_CmdWrite(0x6D);
  LT7683_DataWrite(WX>>8);

  LT7683_CmdWrite(0x6E);
  LT7683_DataWrite(HY);

  LT7683_CmdWrite(0x6F);
  LT7683_DataWrite(HY>>8);
}

/*
[bit7]
Draw Circle / Ellipse / Square /Circle Square Start Signal 
Write Function
0 : Stop the drawing function.
1 : Start the drawing function.
Read Function
0 : Drawing function complete.
1 : Drawing function is processing.
[bit6]
Fill the Circle / Ellipse / Square / Circle Square Signal
0 : Non fill.
1 : fill.
[bit5 bit4]
Draw Circle / Ellipse / Square / Ellipse Curve / Circle Square Select
00 : Draw Circle / Ellipse
01 : Draw Circle / Ellipse Curve
10 : Draw Square.
11 : Draw Circle Square.
[bit1 bit0]
Draw Circle / Ellipse Curve Part Select
00 : 
01 : 
10 : 
11 : 
*/
void Start_Circle_or_Ellipse(void) {
  LT7683_CmdWrite(0x76);
  LT7683_DataWrite(0x80);//B1000_XXXX
  Check_Busy_Draw();  
}

void Start_Circle_or_Ellipse_Fill(void) {
  LT7683_CmdWrite(0x76);
  LT7683_DataWrite(0xC0);//B1100_XXXX
  Check_Busy_Draw();  
}

void Start_Left_Down_Curve(void) {
  LT7683_CmdWrite(0x76);
  LT7683_DataWrite(0x90);//B1001_XX00
  Check_Busy_Draw();  
}

void Start_Left_Up_Curve(void) {
  LT7683_CmdWrite(0x76);
  LT7683_DataWrite(0x91);//B1001_XX01
  Check_Busy_Draw();  
}
 
void Start_Right_Up_Curve(void) {
  LT7683_CmdWrite(0x76);
  LT7683_DataWrite(0x92);//B1001_XX10
  Check_Busy_Draw();  
}

void Start_Right_Down_Curve(void) {
  LT7683_CmdWrite(0x76);
  LT7683_DataWrite(0x93);//B1001_XX11
  Check_Busy_Draw();  
}

void Start_Left_Down_Curve_Fill(void) {
  LT7683_CmdWrite(0x76);
  LT7683_DataWrite(0xD0);//B1101_XX00
  Check_Busy_Draw();
}

void Start_Left_Up_Curve_Fill(void) {
  LT7683_CmdWrite(0x76);
  LT7683_DataWrite(0xD1);//B1101_XX01
  Check_Busy_Draw();
}

void Start_Right_Up_Curve_Fill(void) {
  LT7683_CmdWrite(0x76);
  LT7683_DataWrite(0xD2);//B1101_XX10
  Check_Busy_Draw();
}

void Start_Right_Down_Curve_Fill(void) {
  LT7683_CmdWrite(0x76);
  LT7683_DataWrite(0xD3);//B1101_XX11
  Check_Busy_Draw();
}

void Start_Square(void) {
  LT7683_CmdWrite(0x76);
  LT7683_DataWrite(0xA0);//B1010_XXXX
  Check_Busy_Draw();
}

void Start_Square_Fill(void) {
  LT7683_CmdWrite(0x76);
  LT7683_DataWrite(0xE0);//B1110_XXXX
  Check_Busy_Draw();
}

void Start_Circle_Square(void) {
  LT7683_CmdWrite(0x76);
  LT7683_DataWrite(0xB0);//B1011_XXXX
  Check_Busy_Draw();  
}

void Start_Circle_Square_Fill(void) {
  LT7683_CmdWrite(0x76);
  LT7683_DataWrite(0xF0);//B1111_XXXX
  Check_Busy_Draw();  
}

void Circle_Center_XY(unsigned short WX,unsigned short HY) {
/*
[7Bh] Draw Circle/Ellipse/Circle Square Center X-coordination [7:0]
[7Ch] Draw Circle/Ellipse/Circle Square Center X-coordination [12:8]
[7Dh] Draw Circle/Ellipse/Circle Square Center Y-coordination [7:0]
[7Eh] Draw Circle/Ellipse/Circle Square Center Y-coordination [12:8]
*/
  LT7683_CmdWrite(0x7B);
  LT7683_DataWrite(WX);

  LT7683_CmdWrite(0x7C);
  LT7683_DataWrite(WX>>8);

  LT7683_CmdWrite(0x7D);
  LT7683_DataWrite(HY);

  LT7683_CmdWrite(0x7E);
  LT7683_DataWrite(HY>>8);
}

void Ellipse_Center_XY(unsigned short WX,unsigned short HY) {
/*
[7Bh] Draw Circle/Ellipse/Circle Square Center X-coordination [7:0]
[7Ch] Draw Circle/Ellipse/Circle Square Center X-coordination [12:8]
[7Dh] Draw Circle/Ellipse/Circle Square Center Y-coordination [7:0]
[7Eh] Draw Circle/Ellipse/Circle Square Center Y-coordination [12:8]
*/
  LT7683_CmdWrite(0x7B);
  LT7683_DataWrite(WX);

  LT7683_CmdWrite(0x7C);
  LT7683_DataWrite(WX>>8);

  LT7683_CmdWrite(0x7D);
  LT7683_DataWrite(HY);

  LT7683_CmdWrite(0x7E);
  LT7683_DataWrite(HY>>8);
}

void Circle_Radius_R(unsigned short WX) {
/*
[77h] Draw Circle/Ellipse/Circle Square Major radius [7:0]
[78h] Draw Circle/Ellipse/Circle Square Major radius [12:8]
[79h] Draw Circle/Ellipse/Circle Square Minor radius [7:0]
[7Ah] Draw Circle/Ellipse/Circle Square Minor radius [12:8]
*/
  LT7683_CmdWrite(0x77);
  LT7683_DataWrite(WX);

  LT7683_CmdWrite(0x78);
  LT7683_DataWrite(WX>>8);

  LT7683_CmdWrite(0x79);
  LT7683_DataWrite(WX);

  LT7683_CmdWrite(0x7A);
  LT7683_DataWrite(WX>>8);
}

void Ellipse_Radius_RxRy(unsigned short WX,unsigned short HY) {
/*
[77h] Draw Circle/Ellipse/Circle Square Major radius [7:0]
[78h] Draw Circle/Ellipse/Circle Square Major radius [12:8]
[79h] Draw Circle/Ellipse/Circle Square Minor radius [7:0]
[7Ah] Draw Circle/Ellipse/Circle Square Minor radius [12:8]
*/
  LT7683_CmdWrite(0x77);
  LT7683_DataWrite(WX);

  LT7683_CmdWrite(0x78);
  LT7683_DataWrite(WX>>8);

  LT7683_CmdWrite(0x79);
  LT7683_DataWrite(HY);

  LT7683_CmdWrite(0x7A);
  LT7683_DataWrite(HY>>8);
}

void Circle_Square_Radius_RxRy(unsigned short WX,unsigned short HY) {
/*
[77h] Draw Circle/Ellipse/Circle Square Major radius [7:0]
[78h] Draw Circle/Ellipse/Circle Square Major radius [12:8]
[79h] Draw Circle/Ellipse/Circle Square Minor radius [7:0]
[7Ah] Draw Circle/Ellipse/Circle Square Minor radius [12:8]
*/
  LT7683_CmdWrite(0x77);
  LT7683_DataWrite(WX);

  LT7683_CmdWrite(0x78);
  LT7683_DataWrite(WX>>8);

  LT7683_CmdWrite(0x79);
  LT7683_DataWrite(HY);

  LT7683_CmdWrite(0x7A);
  LT7683_DataWrite(HY>>8);
}

void Set_PWM_Prescaler_1_to_256(unsigned short WX) {
/*
PWM Prescaler Register
These 8 bits determine prescaler value for Timer 0 and 1.
Time base is ¡§Core_Freq / (Prescaler + 1)¡¨
*/
  WX=WX-1;
  LT7683_CmdWrite(0x84);
  LT7683_DataWrite(WX);
}

void Select_PWM1_Clock_Divided_By_1(void) {
/*
Select MUX input for PWM Timer 1.
00 = 1; 01 = 1/2; 10 = 1/4 ; 11 = 1/8;
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x85);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void Select_PWM1_Clock_Divided_By_2(void) {
/*
Select MUX input for PWM Timer 1.
00 = 1; 01 = 1/2; 10 = 1/4 ; 11 = 1/8;
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x85);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void Select_PWM1_Clock_Divided_By_4(void) {
/*
Select MUX input for PWM Timer 1.
00 = 1; 01 = 1/2; 10 = 1/4 ; 11 = 1/8;
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x85);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void Select_PWM1_Clock_Divided_By_8(void) {
/*
Select MUX input for PWM Timer 1.
00 = 1; 01 = 1/2; 10 = 1/4 ; 11 = 1/8;
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x85);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void Select_PWM0_Clock_Divided_By_1(void) {
/*
Select MUX input for PWM Timer 0.
00 = 1; 01 = 1/2; 10 = 1/4 ; 11 = 1/8;
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x85);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void Select_PWM0_Clock_Divided_By_2(void) {
/*
Select MUX input for PWM Timer 0.
00 = 1; 01 = 1/2; 10 = 1/4 ; 11 = 1/8;
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x85);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void Select_PWM0_Clock_Divided_By_4(void) {
/*
Select MUX input for PWM Timer 0.
00 = 1; 01 = 1/2; 10 = 1/4 ; 11 = 1/8;
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x85);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void Select_PWM0_Clock_Divided_By_8(void) {
/*
Select MUX input for PWM Timer 0.
00 = 1; 01 = 1/2; 10 = 1/4 ; 11 = 1/8;
*/
  unsigned char temp;
  
  LT7683_CmdWrite(0x85);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

/*
XPWM[1] pin function control
0X: XPWM[1] output system error flag (REG[00h] bit[1:0], Scan bandwidth insufficient + Memory access out of range)
10: XPWM[1] enabled and controlled by PWM timer 1
11: XPWM[1] output oscillator clock
//If XTEST[0] set high, then XPWM[1] will become panel scan clock input.
*/
void Select_PWM1_is_ErrorFlag(void) {
  unsigned char temp;
  
  LT7683_CmdWrite(0x85);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  LT7683_DataWrite(temp);
}

void Select_PWM1(void) {
  unsigned char temp;
  
  LT7683_CmdWrite(0x85);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}
 
void Select_PWM1_is_Osc_Clock(void) {
  unsigned char temp;
  
  LT7683_CmdWrite(0x85);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

/*
XPWM[0] pin function control
0X: XPWM[0] becomes GPIO-C[7]
10: XPWM[0] enabled and controlled by PWM timer 0
11: XPWM[0] output core clock
*/
void Select_PWM0_is_GPIO_C7(void) {
  unsigned char temp;
  
  LT7683_CmdWrite(0x85);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  LT7683_DataWrite(temp);
}

void Select_PWM0(void) {
  unsigned char temp;
  
  LT7683_CmdWrite(0x85);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Select_PWM0_is_Core_Clock(void) {
  unsigned char temp;
  
  LT7683_CmdWrite(0x85);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Enable_PWM1_Inverter(void) {
/*
PWM Timer 1 output inverter on/off.
Determine the output inverter on/off for Timer 1. 
0 = Inverter off 
1 = Inverter on for PWM1
*/
  unsigned char temp;
  LT7683_CmdWrite(0x86);
  temp = LT7683_DataRead();
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void Disable_PWM1_Inverter(void) {
/*
PWM Timer 1 output inverter on/off.
Determine the output inverter on/off for Timer 1. 
0 = Inverter off 
1 = Inverter on for PWM1
*/
  unsigned char temp;
  LT7683_CmdWrite(0x86);
  temp = LT7683_DataRead();
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void Auto_Reload_PWM1(void) {
/*
PWM Timer 1 auto reload on/off
Determine auto reload on/off for Timer 1. 
0 = One-shot 
1 = Interval mode(auto reload)
*/
  unsigned char temp;
  LT7683_CmdWrite(0x86);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  LT7683_DataWrite(temp);
}

void One_Shot_PWM1(void) {
/*
PWM Timer 1 auto reload on/off
Determine auto reload on/off for Timer 1. 
0 = One-shot 
1 = Interval mode(auto reload)
*/
  unsigned char temp;
  LT7683_CmdWrite(0x86);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  LT7683_DataWrite(temp);
}

void Start_PWM1(void) {
/*
PWM Timer 1 start/stop
Determine start/stop for Timer 1. 
0 = Stop 
1 = Start for Timer 1
*/
  unsigned char temp;
  LT7683_CmdWrite(0x86);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void Stop_PWM1(void) {
/*
PWM Timer 1 start/stop
Determine start/stop for Timer 1. 
0 = Stop 
1 = Start for Timer 1
*/
  unsigned char temp;
  LT7683_CmdWrite(0x86);
  temp = LT7683_DataRead();
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void Enable_PWM0_Dead_Zone(void) {
/*
PWM Timer 0 Dead zone enable
Determine the dead zone operation. 0 = Disable. 1 = Enable.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x86);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  LT7683_DataWrite(temp);
}

void Disable_PWM0_Dead_Zone(void) {
/*
PWM Timer 0 Dead zone enable
Determine the dead zone operation. 0 = Disable. 1 = Enable.
*/
  unsigned char temp;
  LT7683_CmdWrite(0x86);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  LT7683_DataWrite(temp);
}

void Enable_PWM0_Inverter(void) {
/*
PWM Timer 0 output inverter on/off
Determine the output inverter on/off for Timer 0. 
0 = Inverter off 
1 = Inverter on for PWM0
*/
  unsigned char temp;
  LT7683_CmdWrite(0x86);
  temp = LT7683_DataRead();
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Disable_PWM0_Inverter(void) {
/*
PWM Timer 0 output inverter on/off
Determine the output inverter on/off for Timer 0. 
0 = Inverter off 
1 = Inverter on for PWM0
*/
  unsigned char temp;
  LT7683_CmdWrite(0x86);
  temp = LT7683_DataRead();
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void Auto_Reload_PWM0(void) {
/*
PWM Timer 0 auto reload on/off
Determine auto reload on/off for Timer 0. 
0 = One-shot 
1 = Interval mode(auto reload)
*/
  unsigned char temp;
  LT7683_CmdWrite(0x86);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  LT7683_DataWrite(temp);
}

void One_Shot_PWM0(void) {
/*
PWM Timer 1 auto reload on/off
Determine auto reload on/off for Timer 1. 
0 = One-shot 
1 = Interval mode(auto reload)
*/
  unsigned char temp;
  LT7683_CmdWrite(0x86);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  LT7683_DataWrite(temp);
}

void Start_PWM0(void) {
/*
PWM Timer 0 start/stop
Determine start/stop for Timer 0. 
0 = Stop 
1 = Start for Timer 0
*/
  unsigned char temp;
  LT7683_CmdWrite(0x86);
  temp = LT7683_DataRead();
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Stop_PWM0(void) {
/*
PWM Timer 0 start/stop
Determine start/stop for Timer 0. 
0 = Stop 
1 = Start for Timer 0
*/
  unsigned char temp;
  LT7683_CmdWrite(0x86);
  temp = LT7683_DataRead();
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Set_Timer0_Dead_Zone_Length(unsigned char temp) {
/*
Timer 0 Dead zone length register
These 8 bits determine the dead zone length. The 1 unit time of
the dead zone length is equal to that of timer 0.
*/
  LT7683_CmdWrite(0x87);
  LT7683_DataWrite(temp);
}

void Set_Timer0_Compare_Buffer(unsigned short WX) {
/*
Timer 0 compare buffer register
Compare buffer register total has 16 bits.
When timer counter equal or less than compare buffer register will cause PWM out
high level if inv_on bit is off.
*/
  LT7683_CmdWrite(0x88);
  LT7683_DataWrite(WX);
  LT7683_CmdWrite(0x89);
  LT7683_DataWrite(WX>>8);
}

void Set_Timer0_Count_Buffer(unsigned short WX) {
/*
Timer 0 count buffer register
Count buffer register total has 16 bits.
When timer counter equal to 0 will cause PWM timer reload Count buffer register if reload_en bit set as enable.
It may read back timer counter¡¦s real time value when PWM timer start.
*/
  LT7683_CmdWrite(0x8A);
  LT7683_DataWrite(WX);
  LT7683_CmdWrite(0x8B);
  LT7683_DataWrite(WX>>8);
}

void Set_Timer1_Compare_Buffer(unsigned short WX) {
/*
Timer 0 compare buffer register
Compare buffer register total has 16 bits.
When timer counter equal or less than compare buffer register will cause PWM out
high level if inv_on bit is off.
*/
  LT7683_CmdWrite(0x8C);
  LT7683_DataWrite(WX);
  LT7683_CmdWrite(0x8D);
  LT7683_DataWrite(WX>>8);
}

void Set_Timer1_Count_Buffer(unsigned short WX) {
/*
Timer 0 count buffer register
Count buffer register total has 16 bits.
When timer counter equal to 0 will cause PWM timer reload Count buffer register if reload_en bit set as enable.
It may read back timer counter¡¦s real time value when PWM timer start.
*/
  LT7683_CmdWrite(0x8E);
  LT7683_DataWrite(WX);
  LT7683_CmdWrite(0x8F);
  LT7683_DataWrite(WX>>8);
}

void BTE_Enable(void) { 
/*
BTE Function Enable
0 : BTE Function disable.
1 : BTE Function enable.
*/
    unsigned char temp;
    LT7683_CmdWrite(0x90);
    temp = LT7683_DataRead();
    temp |= cSetb4 ;
  LT7683_DataWrite(temp);  
}

void BTE_Disable(void) { 
/*
BTE Function Enable
0 : BTE Function disable.
1 : BTE Function enable.
*/
    unsigned char temp;
    LT7683_CmdWrite(0x90);
    temp = LT7683_DataRead();
    temp &= cClrb4 ;
  LT7683_DataWrite(temp);  
}

void Check_BTE_Busy(void) { 
/*
BTE Function Status
0 : BTE Function is idle.
1 : BTE Function is busy.
*/
  unsigned char temp;   
  do
  {
    temp=LT7683_StatusRead();
  }while(temp&0x08);

}

void Pattern_Format_8X8(void) { 
/*
Pattern Format
0 : 8X8
1 : 16X16
*/
    unsigned char temp;
    LT7683_CmdWrite(0x90);
    temp = LT7683_DataRead();
    temp &= cClrb0 ;
  LT7683_DataWrite(temp);
} 

void Pattern_Format_16X16(void) { 
/*
Pattern Format
0 : 8X8
1 : 16X16
*/
    unsigned char temp;
    LT7683_CmdWrite(0x90);
    temp = LT7683_DataRead();
    temp |= cSetb0 ;
    LT7683_DataWrite(temp);
} 

void BTE_ROP_Code(unsigned char setx) { 
/*
BTE ROP Code[Bit7:4]
  
0000 : 0(Blackness)
0001 : ~S0.~S1 or ~ ( S0+S1 )
0010 : ~S0.S1
0011 : ~S0
0100 : S0.~S1
0101 : ~S1
0110 : S0^S1
0111 : ~S0+~S1 or ~ ( S0.S1 )
1000 : S0.S1
1001 : ~ ( S0^S1 )
1010 : S1
1011 : ~S0+S1
1100 : S0
1101 : S0+~S1
1110 : S0+S1
1111 : 1 ( Whiteness )
*/
    unsigned char temp;
    LT7683_CmdWrite(0x91);
    temp = LT7683_DataRead();
    temp &= 0x0f ;
    temp |= (setx<<4);
    LT7683_DataWrite(temp);
}
  
void BTE_Operation_Code(unsigned char setx) { 
/*
BTE Operation Code[Bit3:0]
  
0000 : MPU Write BTE with ROP.
0001 : MPU Read BTE w/o ROP.
0010 : Memory copy (move) BTE in positive direction with ROP.
0011 : Memory copy (move) BTE in negative direction with ROP.
0100 : MPU Transparent Write BTE. (w/o ROP.)
0101 : Transparent Memory copy (move) BTE in positive direction (w/o ROP.)
0110 : Pattern Fill with ROP.
0111 : Pattern Fill with key-chroma
1000 : Color Expansion
1001 : Color Expansion with transparency
1010 : Move BTE in positive direction with Alpha blending
1011 : MPU Write BTE with Alpha blending
1100 : Solid Fill
1101 : Reserved
1110 : Reserved
1111 : Reserved
*/
    unsigned char temp;
    LT7683_CmdWrite(0x91);
    temp = LT7683_DataRead();
    temp &= 0xf0 ;
    temp |= setx ;
    LT7683_DataWrite(temp);

}

void BTE_S0_Color_8bpp(void) { 
/*
S0 Color Depth
00 : 256 Color
01 : 64k Color
1x : 16M Color
*/  
    unsigned char temp;
    LT7683_CmdWrite(0x92);
    temp = LT7683_DataRead();
    temp &= cClrb6 ;
    temp &= cClrb5 ;
    LT7683_DataWrite(temp);
} 

void BTE_S0_Color_16bpp(void) { 
/*
S0 Color Depth
00 : 256 Color
01 : 64k Color
1x : 16M Color
*/  
    unsigned char temp;
    LT7683_CmdWrite(0x92);
    temp = LT7683_DataRead();
    temp &= cClrb6 ;
    temp |= cSetb5 ;
    LT7683_DataWrite(temp);

} 

void BTE_S0_Color_24bpp(void) { 
/*
S0 Color Depth
00 : 256 Color
01 : 64k Color
1x : 16M Color
*/  
    unsigned char temp;
    LT7683_CmdWrite(0x92);
    temp = LT7683_DataRead();
    temp |= cSetb6 ;
    //temp |= cSetb5 ;
    LT7683_DataWrite(temp);
}

void BTE_S1_Color_8bpp(void) { 
/*
S1 Color Depth
000 : 256 Color
001 : 64k Color
010 : 16M Color
011 : Constant Color
100 : 8 bit pixel alpha blending
101 : 16 bit pixel alpha blending
*/  
    unsigned char temp;
    LT7683_CmdWrite(0x92);
    temp = LT7683_DataRead();
    temp &= cClrb4 ;
    temp &= cClrb3 ;
    temp &= cClrb2 ;
    LT7683_DataWrite(temp);
} 

void BTE_S1_Color_16bpp(void) { 
/*
S1 Color Depth
000 : 256 Color
001 : 64k Color
010 : 16M Color
011 : Constant Color
100 : 8 bit pixel alpha blending
101 : 16 bit pixel alpha blending
*/  
    unsigned char temp;
    LT7683_CmdWrite(0x92);
    temp = LT7683_DataRead();
    temp &= cClrb4 ;
    temp &= cClrb3 ;
    temp |= cSetb2 ;
    LT7683_DataWrite(temp);

}

void BTE_S1_Color_24bpp(void) { 
/*
S1 Color Depth
000 : 256 Color
001 : 64k Color
010 : 16M Color
011 : Constant Color
100 : 8 bit pixel alpha blending
101 : 16 bit pixel alpha blending
*/  
    unsigned char temp;
    LT7683_CmdWrite(0x92);
    temp = LT7683_DataRead();
    temp &= cClrb4 ;
    temp |= cSetb3 ;
    temp &= cClrb2 ;
    LT7683_DataWrite(temp);
}

void BTE_S1_Color_Constant(void) { 
/*
S1 Color Depth
000 : 256 Color
001 : 64k Color
010 : 16M Color
011 : Constant Color
100 : 8 bit pixel alpha blending
101 : 16 bit pixel alpha blending
*/  
    unsigned char temp;
    LT7683_CmdWrite(0x92);
    temp = LT7683_DataRead();
    temp &= cClrb4 ;
    temp |= cSetb3 ;
    temp |= cSetb2 ;
    LT7683_DataWrite(temp);
}

void BTE_S1_Color_8bit_Alpha(void) { 
/*
S1 Color Depth
000 : 256 Color
001 : 64k Color
010 : 16M Color
011 : Constant Color
100 : 8 bit pixel alpha blending
101 : 16 bit pixel alpha blending
*/  
    unsigned char temp;
    LT7683_CmdWrite(0x92);
    temp = LT7683_DataRead();
    temp |= cSetb4 ;
    temp &= cClrb3 ;
    temp &= cClrb2 ;
    LT7683_DataWrite(temp);
}

void BTE_S1_Color_16bit_Alpha(void) { 
/*
S1 Color Depth
000 : 256 Color
001 : 64k Color
010 : 16M Color
011 : Constant Color
100 : 8 bit pixel alpha blending
101 : 16 bit pixel alpha blending
*/  
    unsigned char temp;
    LT7683_CmdWrite(0x92);
    temp = LT7683_DataRead();
    temp |= cSetb4 ;
    temp &= cClrb3 ;
    temp |= cSetb2 ;
    LT7683_DataWrite(temp);
}

void BTE_Destination_Color_8bpp(void) { 
/*
Destination Color Depth
00 : 256 Color
01 : 64k Color
1x : 16M Color
*/  
    unsigned char temp;
    LT7683_CmdWrite(0x92);
    temp = LT7683_DataRead();
    temp &= cClrb1 ;
    temp &= cClrb0 ;
    LT7683_DataWrite(temp);
} 

void BTE_Destination_Color_16bpp(void) { 
/*
Destination Color Depth
00 : 256 Color
01 : 64k Color
1x : 16M Color
*/  
    unsigned char temp;
    LT7683_CmdWrite(0x92);
    temp = LT7683_DataRead();
    temp &= cClrb1 ;
    temp |= cSetb0 ;
    LT7683_DataWrite(temp);

} 

void BTE_Destination_Color_24bpp(void) { 
/*
Destination Color Depth
00 : 256 Color
10 : 64k Color
1x : 16M Color
*/  
    unsigned char temp;
    LT7683_CmdWrite(0x92);
    temp = LT7683_DataRead();
    temp |= cSetb1 ;
    //temp |= cSetb0 ;
    LT7683_DataWrite(temp);
}

void BTE_S0_Memory_Start_Address(unsigned long Addr) {
/*
[93h] BTE S0 Memory Start Address [7:0]
[94h] BTE S0 Memory Start Address [15:8]
[95h] BTE S0 Memory Start Address [23:16]
[96h] BTE S0 Memory Start Address [31:24]
Bit [1:0] tie to ¡§0¡¨ internally.
*/
  LT7683_RegisterWrite(0x93,Addr);
  LT7683_RegisterWrite(0x94,Addr>>8);
  LT7683_RegisterWrite(0x95,Addr>>16);
  LT7683_RegisterWrite(0x96,Addr>>24);
}

void BTE_S0_Image_Width(unsigned short WX) {
/*
[97h] BTE S0 Image Width [7:0]
[98h] BTE S0 Image Width [12:8]
Unit: Pixel.
Bit [1:0] tie to ¡§0¡¨ internally.
*/
  LT7683_RegisterWrite(0x97,WX);
  LT7683_RegisterWrite(0x98,WX>>8);
}

void BTE_S0_Window_Start_XY(unsigned short WX,unsigned short HY) {
/*
[99h] BTE S0 Window Upper-Left corner X-coordination [7:0]
[9Ah] BTE S0 Window Upper-Left corner X-coordination [12:8]
[9Bh] BTE S0 Window Upper-Left corner Y-coordination [7:0]
[9Ch] BTE S0 Window Upper-Left corner Y-coordination [12:8]
*/
  LT7683_RegisterWrite(0x99,WX);
  LT7683_RegisterWrite(0x9A,WX>>8);

  LT7683_RegisterWrite(0x9B,HY);
  LT7683_RegisterWrite(0x9C,HY>>8);
}

void BTE_S1_Memory_Start_Address(unsigned long Addr) {
/*
[9Dh] BTE S1 Memory Start Address [7:0]
[9Eh] BTE S1 Memory Start Address [15:8]
[9Fh] BTE S1 Memory Start Address [23:16]
[A0h] BTE S1 Memory Start Address [31:24]
Bit [1:0] tie to ¡§0¡¨ internally.
*/
  LT7683_RegisterWrite(0x9D,Addr);
  LT7683_RegisterWrite(0x9E,Addr>>8);
  LT7683_RegisterWrite(0x9F,Addr>>16);
  LT7683_RegisterWrite(0xA0,Addr>>24);
}

//Input data format:R3G3B2
void S1_Constant_color_256(unsigned char temp) {
    LT7683_CmdWrite(0x9D);
    LT7683_DataWrite(temp);

    LT7683_CmdWrite(0x9E);
    LT7683_DataWrite(temp<<3);

    LT7683_CmdWrite(0x9F);
    LT7683_DataWrite(temp<<6);
}

//Input data format:R5G6B6
void S1_Constant_color_65k(unsigned short temp) {
    LT7683_CmdWrite(0x9D);
    LT7683_DataWrite(temp>>8);

    LT7683_CmdWrite(0x9E);
    LT7683_DataWrite(temp>>3);

    LT7683_CmdWrite(0x9F);
    LT7683_DataWrite(temp<<3);
}

//Input data format:R8G8B8
void S1_Constant_color_16M(unsigned long temp) {
    LT7683_CmdWrite(0x9D);
    LT7683_DataWrite(temp>>16);

    LT7683_CmdWrite(0x9E);
    LT7683_DataWrite(temp>>8);

    LT7683_CmdWrite(0x9F);
    LT7683_DataWrite(temp);
}

void BTE_S1_Image_Width(unsigned short WX) {
/*
[A1h] BTE S1 Image Width [7:0]
[A2h] BTE S1 Image Width [12:8]
Unit: Pixel.
Bit [1:0] tie to ¡§0¡¨ internally.
*/
  LT7683_RegisterWrite(0xA1,WX);
  LT7683_RegisterWrite(0xA2,WX>>8);
}

void BTE_S1_Window_Start_XY(unsigned short WX,unsigned short HY) {
/*
[A3h] BTE S1 Window Upper-Left corner X-coordination [7:0]
[A4h] BTE S1 Window Upper-Left corner X-coordination [12:8]
[A5h] BTE S1 Window Upper-Left corner Y-coordination [7:0]
[A6h] BTE S1 Window Upper-Left corner Y-coordination [12:8]
*/
  LT7683_RegisterWrite(0xA3,WX);
  LT7683_RegisterWrite(0xA4,WX>>8);

  LT7683_RegisterWrite(0xA5,HY);
  LT7683_RegisterWrite(0xA6,HY>>8);
}

void BTE_Destination_Memory_Start_Address(unsigned long Addr) {
/*
[A7h] BTE Destination Memory Start Address [7:0]
[A8h] BTE Destination Memory Start Address [15:8]
[A9h] BTE Destination Memory Start Address [23:16]
[AAh] BTE Destination Memory Start Address [31:24]
Bit [1:0] tie to ¡§0¡¨ internally.
*/
  LT7683_RegisterWrite(0xA7,Addr);
  LT7683_RegisterWrite(0xA8,Addr>>8);
  LT7683_RegisterWrite(0xA9,Addr>>16);
  LT7683_RegisterWrite(0xAA,Addr>>24);
}

void BTE_Destination_Image_Width(unsigned short WX) {
/*
[ABh] BTE Destination Image Width [7:0]
[ACh] BTE Destination Image Width [12:8]
Unit: Pixel.
Bit [1:0] tie to ¡§0¡¨ internally.
*/
  LT7683_RegisterWrite(0xAB,WX);
  LT7683_RegisterWrite(0xAC,WX>>8);
}

void BTE_Destination_Window_Start_XY(unsigned short WX,unsigned short HY) {
/*
[ADh] BTE Destination Window Upper-Left corner X-coordination [7:0]
[AEh] BTE Destination Window Upper-Left corner X-coordination [12:8]
[AFh] BTE Destination Window Upper-Left corner Y-coordination [7:0]
[B0h] BTE Destination Window Upper-Left corner Y-coordination [12:8]
*/
  LT7683_RegisterWrite(0xAD,WX);
  LT7683_RegisterWrite(0xAE,WX>>8);

  LT7683_RegisterWrite(0xAF,HY);
  LT7683_RegisterWrite(0xB0,HY>>8);
}

void BTE_Window_Size(unsigned short WX, unsigned short WY) {
/*
[B1h] BTE Window Width [7:0]
[B2h] BTE Window Width [12:8]

[B3h] BTE Window Height [7:0]
[B4h] BTE Window Height [12:8]
*/
    LT7683_RegisterWrite(0xB1,WX);
    LT7683_RegisterWrite(0xB2,WX>>8);

    LT7683_RegisterWrite(0xB3,WY);
    LT7683_RegisterWrite(0xB4,WY>>8);
}

void BTE_Alpha_Blending_Effect(unsigned char temp) { 
/*
Window Alpha Blending effect for S0 & S1
The value of alpha in the color code ranges from 0.0 to 1.0,
where 0.0 represents a fully transparent color, and 1.0
represents a fully opaque color.
00h: 0
01h: 1/32
02h: 2/32
:
1Eh: 30/32
1Fh: 31/32
2Xh: 1
Output Effect = (S0 image x (1 - alpha setting value)) + (S1 image x alpha setting value)
*/
  LT7683_CmdWrite(0xB5);
  LT7683_DataWrite(temp);  
}

void Start_SFI_DMA(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB6);
  temp = LT7683_DataRead();
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Check_Busy_SFI_DMA(void) {
  LT7683_CmdWrite(0xB6);
  do {   
  }while((LT7683_DataRead()&0x01)==0x01);
}

void Select_SFI_0(void) {
/*[bit7]
Serial Flash/ROM I/F # Select
0: Serial Flash/ROM 0 I/F is selected.
1: Serial Flash/ROM 1 I/F is selected.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  LT7683_DataWrite(temp);
}

void Select_SFI_1(void) {
/*[bit7]
Serial Flash/ROM I/F # Select
0: Serial Flash/ROM 0 I/F is selected.
1: Serial Flash/ROM 1 I/F is selected.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  LT7683_DataWrite(temp);
}

void Select_SFI_Font_Mode(void) {
/*[bit6]
Serial Flash /ROM Access Mode
0: Font mode ¡V for external cgrom
1: DMA mode ¡V for cgram , pattern , bootstart image or osd
*/
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void Select_SFI_DMA_Mode(void) {
/*[bit6]
Serial Flash /ROM Access Mode
0: Font mode ¡V for external cgrom
1: DMA mode ¡V for cgram , pattern , bootstart image or osd
*/
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void Select_SFI_24bit_Address(void) {
/*[bit5]
Serial Flash/ROM Address Mode
0: 24 bits address mode
1: 32 bits address mode
*/
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  LT7683_DataWrite(temp);
}

void Select_SFI_32bit_Address(void) {
/*[bit5]
Serial Flash/ROM Address Mode
0: 24 bits address mode
1: 32 bits address mode
*/
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  LT7683_DataWrite(temp);
}

void Select_SFI_Waveform_Mode_0(void) {
/*[bit4]
Serial Flash/ROM Waveform Mode
Mode 0.
Mode 3.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void Select_SFI_Waveform_Mode_3(void) {
/*[bit4]
Serial Flash/ROM Waveform Mode
Mode 0.
Mode 3.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void Select_SFI_0_DummyRead(void) {
/*[bit3][bit2]
Serial Flash /ROM Read Cycle 0 RW
00b: no dummy cycle mode
01b: 1 dummy cycle mode
10b: 2 dummy cycle mode
11b: 4 dummy cycle mode
*/
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp &= 0xF3;
  LT7683_DataWrite(temp);
}

void Select_SFI_8_DummyRead(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp &= 0xF3;
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Select_SFI_16_DummyRead(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp &= 0xF3;
  temp |= cSetb3;
  LT7683_DataWrite(temp);
}

void Select_SFI_24_DummyRead(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp |= 0x0c;
  LT7683_DataWrite(temp);
}

void Select_SFI_Single_Mode(void) {
/*[bit1][bit0]
Serial Flash /ROM I/F Data Latch Mode Select
0Xb: Single Mode
10b: Dual Mode 0.
11b: Dual Mode 1.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp &= 0xFC;
  LT7683_DataWrite(temp);
}

void Select_SFI_Dual_Mode0(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp &= 0xFC;
  temp |= cSetb1;
  LT7683_DataWrite(temp);
}

void Select_SFI_Dual_Mode1(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB7);
  temp = LT7683_DataRead();
  temp |= 0x03;
  LT7683_DataWrite(temp);
}

//REG[B8h] SPI master Tx /Rx FIFO Data Register (SPIDR) 
unsigned char SPI_Master_FIFO_Data_Put(unsigned char Data) {
  unsigned char temp;
  LT7683_CmdWrite(0xB8);
  LT7683_DataWrite(Data);
  while(Tx_FIFO_Empty_Flag()==0); 
  temp = SPI_Master_FIFO_Data_Get();
  return temp;
}

unsigned char SPI_Master_FIFO_Data_Get(void) {
  unsigned char temp;

  while(Rx_FIFO_Empty_Flag()==1);
  LT7683_CmdWrite(0xB8);
  temp=LT7683_DataRead();
  //while(Rx_FIFO_full_flag());
   return temp;
}

//REG[B9h] SPI master Control Register (SPIMCR2) 
void Mask_SPI_Master_Interrupt_Flag(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB9);
  temp = LT7683_DataRead();
  temp |= cSetb6;
  LT7683_DataWrite(temp);
} 

void Select_nSS_drive_on_xnsfcs0(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB9);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  LT7683_DataWrite(temp); 
}

void Select_nSS_drive_on_xnsfcs1(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB9);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  LT7683_DataWrite(temp);
}

//0: inactive (nSS port will goes high) 
void nSS_Inactive(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB9);
  temp = LT7683_DataRead();
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

//1: active (nSS port will goes low) 
void nSS_Active(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB9);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

//Interrupt enable for FIFO overflow error [OVFIRQEN] 
void OVFIRQEN_Enable(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB9);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  LT7683_DataWrite(temp);
}

//Interrupt enable for while Tx FIFO empty & SPI engine/FSM idle
void EMTIRQEN_Enable(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB9);
  temp = LT7683_DataRead();
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

//At CPOL=0 the base value of the clock is zero   
//o  For CPHA=0, data are read on the clock's rising edge (low->high transition) and 
//data are changed on a falling edge (high->low clock transition). 
//o  For CPHA=1, data are read on the clock's falling edge and data are changed on a 
//rising edge. 

//At CPOL=1 the base value of the clock is one (inversion of CPOL=0)   
//o  For CPHA=0, data are read on clock's falling edge and data are changed on a 
//rising edge. 
//o  For CPHA=1, data are read on clock's rising edge and data are changed on a 
//falling edge.

void Reset_CPOL(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB9);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  LT7683_DataWrite(temp);
}

void Set_CPOL(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB9);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  LT7683_DataWrite(temp);
}

void Reset_CPHA(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB9);
  temp = LT7683_DataRead();
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Set_CPHA(void) {
  unsigned char temp;
  LT7683_CmdWrite(0xB9);
  temp = LT7683_DataRead();
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

//REG[BAh] SPI master Status Register (SPIMSR) 
unsigned char Tx_FIFO_Empty_Flag(void) {
  LT7683_CmdWrite(0xBA);
  if((LT7683_DataRead()&0x80)==0x80)
  return 1;
  else
  return 0;
}

unsigned char Tx_FIFO_Full_Flag(void) {
  LT7683_CmdWrite(0xBA);
  if((LT7683_DataRead()&0x40)==0x40)
  return 1;
  else
  return 0;
} 

unsigned char Rx_FIFO_Empty_Flag(void) {
  LT7683_CmdWrite(0xBA);
  if((LT7683_DataRead()&0x20)==0x20)
  return 1;
  else
  return 0;
} 

unsigned char Rx_FIFO_full_flag(void) {
   LT7683_CmdWrite(0xBA);
   if((LT7683_DataRead()&0x10)==0x10)
   return 1;
   else
   return 0;
} 

unsigned char OVFI_Flag(void) {
   LT7683_CmdWrite(0xBA);
   if((LT7683_DataRead()&0x08)==0x08)
   return 1;
   else
   return 0;
}

void Clear_OVFI_Flag(void) {
   unsigned char temp;
   LT7683_CmdWrite(0xBA);
   temp = LT7683_DataRead();
   temp |= cSetb3;
   LT7683_DataWrite(temp);
}

unsigned char EMTI_Flag(void) {
   LT7683_CmdWrite(0xBA);
   if((LT7683_DataRead()&0x04)==0x04)
   return 1;
   else
   return 0;
}

void Clear_EMTI_Flag(void) {
   unsigned char temp;
   LT7683_CmdWrite(0xBA);
   temp = LT7683_DataRead();
   temp |= cSetb2;
   LT7683_DataWrite(temp);
}

//REG[BB] SPI Clock period (SPIDIV) 
void SPI_Clock_Period(unsigned char temp) {
   LT7683_CmdWrite(0xBB);
   LT7683_DataWrite(temp);
} 

void SFI_DMA_Source_Start_Address(unsigned long Addr) {
/*
DMA Source START ADDRESS
This bits index serial flash address [7:0][15:8][23:16][31:24]
*/
  LT7683_CmdWrite(0xBC);
  LT7683_DataWrite(Addr);
  LT7683_CmdWrite(0xBD);
  LT7683_DataWrite(Addr>>8);
  LT7683_CmdWrite(0xBE);
  LT7683_DataWrite(Addr>>16);
  LT7683_CmdWrite(0xBF);
  LT7683_DataWrite(Addr>>24);
}

void SFI_DMA_Destination_Start_Address(unsigned long Addr) {
/*
DMA Destination START ADDRESS 
[1:0]Fix at 0
This bits index SDRAM address [7:0][15:8][23:16][31:24]
*/
  LT7683_CmdWrite(0xC0);
  LT7683_DataWrite(Addr);
  LT7683_CmdWrite(0xC1);
  LT7683_DataWrite(Addr>>8);
  LT7683_CmdWrite(0xC2);
  LT7683_DataWrite(Addr>>16);
  LT7683_CmdWrite(0xC3);
  LT7683_DataWrite(Addr>>24);
}

void SFI_DMA_Destination_Upper_Left_Corner(unsigned short WX,unsigned short HY) {
/*
C0h
This register defines DMA Destination Window Upper-Left corner 
X-coordination [7:0] on Canvas area. 
When REG DMACR bit 1 = 1 (Block Mode) 
This register defines Destination address [7:2] in SDRAM. 
C1h
When REG DMACR bit 1 = 0 (Linear Mode) 
This register defines DMA Destination Window Upper-Left corner 
X-coordination [12:8] on Canvas area. 
When REG DMACR bit 1 = 1 (Block Mode) 
This register defines Destination address [15:8] in SDRAM.
C2h
When REG DMACR bit 1 = 0 (Linear Mode) 
This register defines DMA Destination Window Upper-Left corner
Y-coordination [7:0] on Canvas area. 
When REG DMACR bit 1 = 1 (Block Mode) 
This register defines Destination address [23:16] in SDRAM. 
C3h
When REG DMACR bit 1 = 0 (Linear Mode) 
This register defines DMA Destination Window Upper-Left corner 
Y-coordination [12:8] on Canvas area. 
When REG DMACR bit 1 = 1 (Block Mode) 
This register defines Destination address [31:24] in SDRAM. 
*/
 
  LT7683_CmdWrite(0xC0);
  LT7683_DataWrite(WX);
  LT7683_CmdWrite(0xC1);
  LT7683_DataWrite(WX>>8);
 
  LT7683_CmdWrite(0xC2);
  LT7683_DataWrite(HY);
  LT7683_CmdWrite(0xC3);
  LT7683_DataWrite(HY>>8);
}

void SFI_DMA_Transfer_Number(unsigned long Addr) {
/*
Unit : Pixel
When REG DMACR bit 1 = 0 (Linear Mode)
DMA Transfer Number [7:0][15:8][23:16][31:24]

When REG DMACR bit 1 = 1 (Block Mode)
DMA Block Width [7:0][15:8]
DMA Block HIGH[7:0][15:8]
*/
  LT7683_CmdWrite(0xC6);
  LT7683_DataWrite(Addr);
  LT7683_CmdWrite(0xC7);
  LT7683_DataWrite(Addr>>8);
  LT7683_CmdWrite(0xC8);
  LT7683_DataWrite(Addr>>16);
  LT7683_CmdWrite(0xC9);
  LT7683_DataWrite(Addr>>24);
}

void SFI_DMA_Transfer_Width_Height(unsigned short WX,unsigned short HY) {
/*
When REG DMACR bit 1 = 0 (Linear Mode)
DMA Transfer Number [7:0][15:8][23:16][31:24]

When REG DMACR bit 1 = 1 (Block Mode)
DMA Block Width [7:0][15:8]
DMA Block HIGH[7:0][15:8]
*/
  LT7683_CmdWrite(0xC6);
  LT7683_DataWrite(WX);
  LT7683_CmdWrite(0xC7);
  LT7683_DataWrite(WX>>8);

  LT7683_CmdWrite(0xC8);
  LT7683_DataWrite(HY);
  LT7683_CmdWrite(0xC9);
  LT7683_DataWrite(HY>>8);
}

void SFI_DMA_Source_Width(unsigned short WX) {
/*
DMA Source Picture Width [7:0][12:8]
Unit: pixel
*/
  LT7683_CmdWrite(0xCA);
  LT7683_DataWrite(WX);
  LT7683_CmdWrite(0xCB);
  LT7683_DataWrite(WX>>8);
}

void Font_Select_UserDefine_Mode(void) {
/*[bit7-6]
User-defined Font /CGROM Font Selection Bit in Text Mode
00 : Internal CGROM
01 : Genitop serial flash
10 : User-defined Font
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCC);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void CGROM_Select_Internal_CGROM(void) {
/*[bit7-6]
User-defined Font /CGROM Font Selection Bit in Text Mode
00 : Internal CGROM
01 : Genitop serial flash
10 : User-defined Font
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCC);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void CGROM_Select_Genitop_FontROM(void) {
/*[bit7-6]
User-defined Font /CGROM Font Selection Bit in Text Mode
00 : Internal CGROM
01 : Genitop serial flash
10 : User-defined Font
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCC);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void Font_Select_8x16_16x16(void) {
/*[bit5-4]
Font Height Setting
00b : 8x16 / 16x16.
01b : 12x24 / 24x24.
10b : 16x32 / 32x32.
*** User-defined Font width is decided by font code. Genitop
serial flash¡¦s font width is decided by font code or GT Font ROM
control register.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCC);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void Font_Select_12x24_24x24(void) {
/*[bit5-4]
Font Height Setting
00b : 8x16 / 16x16.
01b : 12x24 / 24x24.
10b : 16x32 / 32x32.
*** User-defined Font width is decided by font code. Genitop
serial flash¡¦s font width is decided by font code or GT Font ROM
control register.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCC);
  temp = LT7683_DataRead();
  temp &= cClrb5;
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void Font_Select_16x32_32x32(void) {
/*[bit5-4]
Font Height Setting
00b : 8x16 / 16x16.
01b : 12x24 / 24x24.
10b : 16x32 / 32x32.
*** User-defined Font width is decided by font code. Genitop
serial flash¡¦s font width is decided by font code or GT Font ROM
control register.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCC);
  temp = LT7683_DataRead();
  temp |= cSetb5;
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void Internal_CGROM_Select_ISOIEC8859_1(void) {
/*
Font Selection for internal CGROM
When FNCR0 B7 = 0 and B5 = 0, Internal CGROM supports the
8x16 character sets with the standard coding of ISO/IEC 8859-1~4, 
which supports English and most of European country languages.
00b : ISO/IEC 8859-1.
01b : ISO/IEC 8859-2.
10b : ISO/IEC 8859-3.
11b : ISO/IEC 8859-4.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCC);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Internal_CGROM_Select_ISOIEC8859_2(void) {
/*
Font Selection for internal CGROM
When FNCR0 B7 = 0 and B5 = 0, Internal CGROM supports the
8x16 character sets with the standard coding of ISO/IEC 8859-1~4, 
which supports English and most of European country languages.
00b : ISO/IEC 8859-1.
01b : ISO/IEC 8859-2.
10b : ISO/IEC 8859-3.
11b : ISO/IEC 8859-4.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCC);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Internal_CGROM_Select_ISOIEC8859_3(void) {
/*
Font Selection for internal CGROM
When FNCR0 B7 = 0 and B5 = 0, Internal CGROM supports the
8x16 character sets with the standard coding of ISO/IEC 8859-1~4, 
which supports English and most of European country languages.
00b : ISO/IEC 8859-1.
01b : ISO/IEC 8859-2.
10b : ISO/IEC 8859-3.
11b : ISO/IEC 8859-4.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCC);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Internal_CGROM_Select_ISOIEC8859_4(void) {
/*
Font Selection for internal CGROM
When FNCR0 B7 = 0 and B5 = 0, Internal CGROM supports the
8x16 character sets with the standard coding of ISO/IEC 8859-1~4, 
which supports English and most of European country languages.
00b : ISO/IEC 8859-1.
01b : ISO/IEC 8859-2.
10b : ISO/IEC 8859-3.
11b : ISO/IEC 8859-4.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCC);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Enable_Font_Alignment(void) {
/*
Full Alignment Selection Bit
0 : Full alignment disable.
1 : Full alignment enable.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCD);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  LT7683_DataWrite(temp);
}

void Disable_Font_Alignment(void) {
/*
Full Alignment Selection Bit
0 : Full alignment disable.
1 : Full alignment enable.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCD);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  LT7683_DataWrite(temp);
}

void Font_Background_select_Transparency(void) {
/*
Font Transparency
0 : Font with background color.
1 : Font with background transparency.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCD);
  temp = LT7683_DataRead();
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void Font_Background_select_Color(void) {
/*
Font Transparency
0 : Font with background color.
1 : Font with background transparency.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCD);
  temp = LT7683_DataRead();
  temp &= cClrb6;
  LT7683_DataWrite(temp);
}

void Font_0_degree(void) {
/*
Font Rotation
0 : Normal
Text direction from left to right then from top to bottom
1 : Counterclockwise 90 degree & horizontal flip
Text direction from top to bottom then from left to right
(it should accommodate with set VDIR as 1)
This attribute can be changed only when previous font write
finished (core_busy = 0)
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCD);
  temp = LT7683_DataRead();
  temp &= cClrb4;
  LT7683_DataWrite(temp);
}

void Font_90_degree(void) {
/*
Font Rotation
0 : Normal
Text direction from left to right then from top to bottom
1 : Counterclockwise 90 degree & horizontal flip
Text direction from top to bottom then from left to right
(it should accommodate with set VDIR as 1)
This attribute can be changed only when previous font write
finished (core_busy = 0)
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCD);
  temp = LT7683_DataRead();
  temp |= cSetb4;
  LT7683_DataWrite(temp);
}

void Font_Width_X1(void) {
/*
Horizontal Font Enlargement
00b : X1.
01b : X2.
10b : X3.
11b : X4.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCD);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void Font_Width_X2(void) {
/*
Horizontal Font Enlargement
00b : X1.
01b : X2.
10b : X3.
11b : X4.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCD);
  temp = LT7683_DataRead();
  temp &= cClrb3;
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Font_Width_X3(void) {
/*
Horizontal Font Enlargement
00b : X1.
01b : X2.
10b : X3.
11b : X4.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCD);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  temp &= cClrb2;
  LT7683_DataWrite(temp);
}

void Font_Width_X4(void) {
/*
Horizontal Font Enlargement
00b : X1.
01b : X2.
10b : X3.
11b : X4.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCD);
  temp = LT7683_DataRead();
  temp |= cSetb3;
  temp |= cSetb2;
  LT7683_DataWrite(temp);
}

void Font_Height_X1(void) {
/*
Vertical Font Enlargement
00b : X1.
01b : X2.
10b : X3.
11b : X4.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCD);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Font_Height_X2(void) {
/*
Vertical Font Enlargement
00b : X1.
01b : X2.
10b : X3.
11b : X4.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCD);
  temp = LT7683_DataRead();
  temp &= cClrb1;
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void Font_Height_X3(void) {
/*
Vertical Font Enlargement
00b : X1.
01b : X2.
10b : X3.
11b : X4.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCD);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  temp &= cClrb0;
  LT7683_DataWrite(temp);
}

void Font_Height_X4(void) {
/*
Vertical Font Enlargement
00b : X1.
01b : X2.
10b : X3.
11b : X4.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCD);
  temp = LT7683_DataRead();
  temp |= cSetb1;
  temp |= cSetb0;
  LT7683_DataWrite(temp);
}

void GTFont_Select_GT21L16TW_GT21H16T1W(void) {
/*
GT Serial Font ROM Select
000b: GT21L16TW / GT21H16T1W
001b: GT23L16U2W
010b: GT23L24T3Y / GT23H24T3Y
011b: GT23L24M1Z
100b: GT23L32S4W / GT23H32S4W
101b: GT20L24F6Y
110b: GT21L24S1W
111b: GT22L16A1Y
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCE);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  temp &= cClrb6;
  temp &= cClrb5;
  LT7683_DataWrite(temp);
}

void GTFont_Select_GT23L16U2W(void) {
/*
GT Serial Font ROM Select
000b: GT21L16TW / GT21H16T1W
001b: GT23L16U2W
010b: GT23L24T3Y / GT23H24T3Y
011b: GT23L24M1Z
100b: GT23L32S4W / GT23H32S4W
101b: GT20L24F6Y
110b: GT21L24S1W
111b: GT22L16A1Y
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCE);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  temp &= cClrb6;
  temp |= cSetb5;
  LT7683_DataWrite(temp);
}

void GTFont_Select_GT23L24T3Y_GT23H24T3Y(void) {
/*
GT Serial Font ROM Select
000b: GT21L16TW / GT21H16T1W
001b: GT23L16U2W
010b: GT23L24T3Y / GT23H24T3Y
011b: GT23L24M1Z
100b: GT23L32S4W / GT23H32S4W
101b: GT20L24F6Y
110b: GT21L24S1W
111b: GT22L16A1Y
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCE);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  temp |= cSetb6;
  temp &= cClrb5;
  LT7683_DataWrite(temp);
}

void GTFont_Select_GT23L24M1Z(void) {
/*
GT Serial Font ROM Select
000b: GT21L16TW / GT21H16T1W
001b: GT23L16U2W
010b: GT23L24T3Y / GT23H24T3Y
011b: GT23L24M1Z
100b: GT23L32S4W / GT23H32S4W
101b: GT20L24F6Y
110b: GT21L24S1W
111b: GT22L16A1Y
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCE);
  temp = LT7683_DataRead();
  temp &= cClrb7;
  temp |= cSetb6;
  temp |= cSetb5;
  LT7683_DataWrite(temp);
}

void GTFont_Select_GT23L32S4W_GT23H32S4W(void) {
/*
GT Serial Font ROM Select
000b: GT21L16TW / GT21H16T1W
001b: GT23L16U2W
010b: GT23L24T3Y / GT23H24T3Y
011b: GT23L24M1Z
100b: GT23L32S4W / GT23H32S4W
101b: GT20L24F6Y
110b: GT21L24S1W
111b: GT22L16A1Y
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCE);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  temp &= cClrb6;
  temp &= cClrb5;
  LT7683_DataWrite(temp);
}

void GTFont_Select_GT20L24F6Y(void) {
/*
GT Serial Font ROM Select
000b: GT21L16TW / GT21H16T1W
001b: GT23L16U2W
010b: GT23L24T3Y / GT23H24T3Y
011b: GT23L24M1Z
100b: GT23L32S4W / GT23H32S4W
101b: GT20L24F6Y
110b: GT21L24S1W
111b: GT22L16A1Y
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCE);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  temp &= cClrb6;
  temp |= cSetb5;
  LT7683_DataWrite(temp);
}

void GTFont_Select_GT21L24S1W(void) {
/*
GT Serial Font ROM Select
000b: GT21L16TW / GT21H16T1W
001b: GT23L16U2W
010b: GT23L24T3Y / GT23H24T3Y
011b: GT23L24M1Z
100b: GT23L32S4W / GT23H32S4W
101b: GT20L24F6Y
110b: GT21L24S1W
111b: GT22L16A1Y
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCE);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  temp |= cSetb6;
  temp &= cClrb5;
  LT7683_DataWrite(temp);
}

void GTFont_Select_GT22L16A1Y(void) {
/*
GT Serial Font ROM Select
000b: GT21L16TW / GT21H16T1W
001b: GT23L16U2W
010b: GT23L24T3Y / GT23H24T3Y
011b: GT23L24M1Z
100b: GT23L32S4W / GT23H32S4W
101b: GT20L24F6Y
110b: GT21L24S1W
111b: GT22L16A1Y
*/
  unsigned char temp;
  LT7683_CmdWrite(0xCE);
  temp = LT7683_DataRead();
  temp |= cSetb7;
  temp |= cSetb6;
  temp |= cSetb5;
  LT7683_DataWrite(temp);
}

void Set_GTFont_Decoder(unsigned char temp) {
/*
[bit7-3]
FONT ROM Coding Setting
For specific GT serial Font ROM, the coding method must be set for decoding.
00000b: GB2312
00001b: GB12345/GB18030
00010b: BIG5
00011b: UNICODE
00100b: ASCII
00101b: UNI-Japanese
00110b: JIS0208
00111b: Latin/Greek/ Cyrillic / Arabic/Thai/Hebrew
01000b: Korea
10001b: ISO-8859-1
10010b: ISO-8859-2
10011b: ISO-8859-3
10100b: ISO-8859-4
10101b: ISO-8859-5
10110b: ISO-8859-6
10111b: ISO-8859-7
11000b: ISO-8859-8
11001b: ISO-8859-9
11010b: ISO-8859-10
11011b: ISO-8859-11
11100b: ISO-8859-12
11101b: ISO-8859-13
11110b: ISO-8859-14
11111b: ISO-8859-15

[bit1-0]
ASCII / Latin/Greek/ Cyrillic / Arabic
    (ASCII)   (Latin/Greek/Cyrillic)  (Arabic)
00b   Normal      Normal          NA
01b   Arial     Variable Width      Presentation Forms-A
10b   Roman       NA          Presentation Forms-B
11b   Bold      NA            NA
*/
  LT7683_CmdWrite(0xCF);
  LT7683_DataWrite(temp);
}

void Font_Line_Distance(unsigned char temp) {
/*[bit4-0]
Font Line Distance Setting
Setting the font character line distance when setting memory font
write cursor auto move. (Unit: pixel)
*/
  LT7683_CmdWrite(0xD0);
  LT7683_DataWrite(temp);
}

void Set_Font_to_Font_Width(unsigned char temp) {
/*[bit5-0]
Font to Font Width Setting (Unit: pixel)
*/
  LT7683_CmdWrite(0xD1);
  LT7683_DataWrite(temp);
}

void Foreground_RGB(unsigned char red, unsigned char green, unsigned char blue) {
/*
[D2h] Foreground Color - Red, for draw, text or color expansion
[D3h] Foreground Color - Green, for draw, text or color expansion
[D4h] Foreground Color - Blue, for draw, text or color expansion
*/  
    LT7683_CmdWrite(0xD2);
    LT7683_DataWrite(red);

    LT7683_CmdWrite(0xD3);
    LT7683_DataWrite(green);

    LT7683_CmdWrite(0xD4);
    LT7683_DataWrite(blue);
}

//Input data format:R3G3B2  
void Foreground_color_256(unsigned char temp) {
    LT7683_CmdWrite(0xD2);
    LT7683_DataWrite(temp);

    LT7683_CmdWrite(0xD3);
    LT7683_DataWrite(temp<<3);

    LT7683_CmdWrite(0xD4);
    LT7683_DataWrite(temp<<6);
}
 
//Input data format:R5G6B5 
void Foreground_color_65k(unsigned short temp) {
    LT7683_CmdWrite(0xD2);
    LT7683_DataWrite(temp>>8);

    LT7683_CmdWrite(0xD3);
    LT7683_DataWrite(temp>>3);

    LT7683_CmdWrite(0xD4);
    LT7683_DataWrite(temp<<3);
}
 
//Input data format:R8G8B8 
void Foreground_color_16M(unsigned long temp) {
    LT7683_CmdWrite(0xD2);
    LT7683_DataWrite(temp>>16);

    LT7683_CmdWrite(0xD3);
    LT7683_DataWrite(temp>>8);

    LT7683_CmdWrite(0xD4);
    LT7683_DataWrite(temp);
}
 
/*
[D5h] Background Color - Red, for Text or color expansion
[D6h] Background Color - Green, for Text or color expansion
[D7h] Background Color - Blue, for Text or color expansion
*/ 
void Background_RGB(unsigned char red, unsigned char green,unsigned char blue) {
   
  LT7683_CmdWrite(0xD5);
  LT7683_DataWrite(red);
  
  LT7683_CmdWrite(0xD6);
  LT7683_DataWrite(green);
   
  LT7683_CmdWrite(0xD7);
  LT7683_DataWrite(blue);
}
 
//Input data format:R3G3B2
void Background_color_256(unsigned char temp) {
    LT7683_CmdWrite(0xD5);
    LT7683_DataWrite(temp);

    LT7683_CmdWrite(0xD6);
    LT7683_DataWrite(temp<<3);

    LT7683_CmdWrite(0xD7);
    LT7683_DataWrite(temp<<6);
}
 
//Input data format:R5G6B6
void Background_color_65k(unsigned short temp) {
    LT7683_CmdWrite(0xD5);
    LT7683_DataWrite(temp>>8);

    LT7683_CmdWrite(0xD6);
    LT7683_DataWrite(temp>>3);

    LT7683_CmdWrite(0xD7);
    LT7683_DataWrite(temp<<3);
}
 
//Input data format:R8G8B8
void Background_color_16M(unsigned long temp) {
    LT7683_CmdWrite(0xD5);
    LT7683_DataWrite(temp>>16);

    LT7683_CmdWrite(0xD6);
    LT7683_DataWrite(temp>>8);

    LT7683_CmdWrite(0xD7);
    LT7683_DataWrite(temp);
}
 
void CGRAM_Start_address(unsigned long Addr) {
/*
CGRAM START ADDRESS [31:0]
*/   
    LT7683_CmdWrite(0xDB);
    LT7683_DataWrite(Addr);
    LT7683_CmdWrite(0xDC);
    LT7683_DataWrite(Addr>>8);
    LT7683_CmdWrite(0xDD);
    LT7683_DataWrite(Addr>>16);
    LT7683_CmdWrite(0xDE);
    LT7683_DataWrite(Addr>>24);
}

/*
[bit7] Enter Power saving state
0: Normal state.
1: Enter power saving state.
[bit1][bit0] Power saving Mode definition
00: NA
01: Standby Mode
10: Suspend Mode
11: Sleep Mode
*/
void Power_Normal_Mode(void) {
  LT7683_CmdWrite(0xDF);
  LT7683_DataWrite(0x00);
}

void Power_Saving_Standby_Mode(void) {
  LT7683_CmdWrite(0xDF);
  LT7683_DataWrite(0x01);
  LT7683_CmdWrite(0xDF);
  LT7683_DataWrite(0x81);
}

void Power_Saving_Suspend_Mode(void) {
  LT7683_CmdWrite(0xDF);
  //LT7683_DataWrite(0x02);
  //LT7683_CmdWrite(0xDF);
  LT7683_DataWrite(0x82);
}

void Power_Saving_Sleep_Mode(void) {
  LT7683_CmdWrite(0xDF);
  //LT7683_DataWrite(0x03);
  //LT7683_CmdWrite(0xDF);
  LT7683_DataWrite(0x83);
}

void LT7683_I2CM_Clock_Prescale(unsigned short WX) {
/*
I2C Master Clock Pre-scale [7:0]
I2C Master Clock Pre-scale [15:8]
*/  
    LT7683_CmdWrite(0xE5);
    LT7683_DataWrite(WX);
    LT7683_CmdWrite(0xE6);
    LT7683_DataWrite(WX>>8);
}
 
void LT7683_I2CM_Transmit_Data(unsigned char temp) {
/*
I2C Master Transmit[7:0]
*/  
    LT7683_CmdWrite(0xE7);
    LT7683_DataWrite(temp);
}

unsigned char LT7683_I2CM_Receiver_Data(void) {
/*
I2C Master Receiver [7:0]
*/  
    unsigned char temp;
    LT7683_CmdWrite(0xE8);
    temp=LT7683_DataRead();
    return temp;
}

/*
[bit7] START
Generate (repeated) start condition and be cleared by hardware automatically
Note : This bit is always read as 0.
[bit6] STOP
Generate stop condition and be cleared by hardware automatically
Note : This bit is always read as 0.
[bit5]
READ(READ and WRITE can¡¦t be used simultaneously)
Read form slave and be cleared by hardware automatically
Note : This bit is always read as 0.
[bit4]
WRITE(READ and WRITE can¡¦t be used simultaneously)
Write to slave and be cleared by hardware automatically
Note : This bit is always read as 0.
[bit3] ACKNOWLEDGE
When as a I2C master receiver
0 : Sent ACK.
1 : Sent NACK.
Note : This bit is always read as 0.
[bit0] Noise Filter
0 : Enable.
1 : Disable.
*/

#define Disable_I2CM_Noise_Filter
 
void LT7683_I2CM_Stop(void) {
  LT7683_CmdWrite(0xE9);
  #ifdef Disable_I2CM_Noise_Filter 
  LT7683_DataWrite(0x40);
  #endif
 
  #ifdef Enable_I2CM_Noise_Filter 
  LT7683_DataWrite(0x41);
  #endif
}

void LT7683_I2CM_Read_With_Ack(void) {
 
  LT7683_CmdWrite(0xE9);
  #ifdef Disable_I2CM_Noise_Filter 
  LT7683_DataWrite(0x20);
  #endif
  
  #ifdef Enable_I2CM_Noise_Filter 
  LT7683_DataWrite(0x21);
  #endif
}
 
void LT7683_I2CM_Read_With_Nack(void) {
  LT7683_CmdWrite(0xE9);
  #ifdef Disable_I2CM_Noise_Filter 
  LT7683_DataWrite(0x68);
  #endif
  
  #ifdef Enable_I2CM_Noise_Filter 
  LT7683_DataWrite(0x69);
  #endif
}
 
void LT7683_I2CM_Write_With_Start(void) {
  LT7683_CmdWrite(0xE9);
  #ifdef Disable_I2CM_Noise_Filter 
  LT7683_DataWrite(0x90);
  #endif
 
  #ifdef Enable_I2CM_Noise_Filter 
  LT7683_DataWrite(0x91);
  #endif
}
 
void LT7683_I2CM_Write(void) {
  LT7683_CmdWrite(0xE9);
  #ifdef Disable_I2CM_Noise_Filter 
  LT7683_DataWrite(0x10);
  #endif
  
  #ifdef Enable_I2CM_Noise_Filter 
  LT7683_DataWrite(0x11);
  #endif
}
 
/*
 0=Ack
 1=Nack
*/
unsigned char LT7683_I2CM_Check_Slave_ACK(void) { 
  unsigned char temp;
/*[bit7]
Received acknowledge from slave
0 : Acknowledge received.
1 : No Acknowledge received. 
*/ 
  LT7683_CmdWrite(0xEA);
  temp=LT7683_DataRead();
  if((temp&0x80)==0x80)
  return 1;
  else
  return 0; 
}
 
/*
 0=Idle
 1=Busy
*/
unsigned char LT7683_I2CM_Bus_Busy(void) {
  unsigned char temp; 
 
/*[bit6]
I2C Bus is Busy
0 : Idle.
1 : Busy.  
*/ 
  LT7683_CmdWrite(0xEA);
  temp=LT7683_DataRead();
  if((temp&0x40)==0x40)
    return 1;
  else
    return 0;    
}
 
/*
 0=Complete
 1=Transferring
*/
unsigned char LT7683_I2CM_transmit_Progress(void) {
  unsigned char temp; 
/*[bit6]
 0=Complete
 1=Transferring
*/ 
  LT7683_CmdWrite(0xEA);
  temp=LT7683_DataRead();
  if((temp&0x02)==0x02)
  return 1;
  else
  return 0;
}
 
 /*
 0= Arbitration win
 1= Arbitration lost
*/
unsigned char LT7683_I2CM_Arbitration(void) {
  unsigned char temp; 
/*[bit6]
I2C Bus is Busy
0 : Idle.
1 : Busy.  
*/ 
  LT7683_CmdWrite(0xEA);
  temp=LT7683_DataRead();
  temp&=0x01;
  return temp;
}

void Set_GPIO_A_In_Out(unsigned char temp) {
/*
GPO-A_dir[7:0] : General Purpose I/O direction control.
0: Output
1: Input
*/
  LT7683_CmdWrite(0xF0);
  LT7683_DataWrite(temp);
}

void Write_GPIO_A_7_0(unsigned char temp) {
/*
GPI-A[7:0] : General Purpose Input, share with DB[15:8]
GPO-A[7:0] : General Purpose Output, share with DB[15:8]
*/
  LT7683_CmdWrite(0xF1);
  LT7683_DataWrite(temp);
}

unsigned char Read_GPIO_A_7_0(void) {
/*
GPI-A[7:0] : General Purpose Input, share with DB[15:8]
GPO-A[7:0] : General Purpose Output, share with DB[15:8]
*/
  unsigned char temp;
  LT7683_CmdWrite(0xF1);
  temp=LT7683_DataRead();
  return temp;
}

void Write_GPIO_B_7_4(unsigned char temp) {
/*
GPI-B[7:0] : General Purpose Input ; share with {XKIN[3:0], XA0, XnWR, XnRD, XnCS}
GPO-B[7:4] : General Purpose Output ; share with XKOUT[3:0] ;
*/
  LT7683_CmdWrite(0xF2);
  LT7683_DataWrite(temp);
}

unsigned char Read_GPIO_B_7_0(void) {
/*
GPI-B[7:0] : General Purpose Input ; share with {XKIN[3:0], XA0, XnWR, XnRD, XnCS}
GPO-B[7:4] : General Purpose Output ; share with XKOUT[3:0] ;
*/
  unsigned char temp;
  LT7683_CmdWrite(0xF2);
  temp=LT7683_DataRead();
  return temp;
}

void Set_GPIO_C_In_Out(unsigned char temp) {
/*
GPIO-C_dir[7:0] : General Purpose I/O direction control.
0: Output
1: Input
*/
  LT7683_CmdWrite(0xF3);
  LT7683_DataWrite(temp);
}

void Write_GPIO_C_7_0(unsigned char temp) {
/*
GPIO-C[7:0] : General Purpose Input / Output
share with {XPWM0, XI2CSDA, XI2CSCL, XnSFCS1, XnSFCS0,XMISO, XMOSI, XSCLK}
*/
  LT7683_CmdWrite(0xF4);
  LT7683_DataWrite(temp);
}

unsigned char Read_GPIO_C_7_0(void) {
/*
GPIO-C[7:0] : General Purpose Input / Output
share with {XPWM0, XI2CSDA, XI2CSCL, XnSFCS1, XnSFCS0,XMISO, XMOSI, XSCLK}
*/
  unsigned char temp;
  LT7683_CmdWrite(0xF4);
  temp=LT7683_DataRead();
  return temp;
}

void Set_GPIO_D_In_Out(unsigned char temp) {
/*
GPIO-D_dir[7:0] : General Purpose I/O direction control.
0: Output
1: Input
*/
  LT7683_CmdWrite(0xF5);
  LT7683_DataWrite(temp);
}

void Write_GPIO_D_7_0(unsigned char temp) {
/*
GPIO-D[7:0] : General Purpose Input/Output
*/
  LT7683_CmdWrite(0xF6);
  LT7683_DataWrite(temp);
}

unsigned char Read_GPIO_D_7_0(void) {
/*
GPIO-D[7:0] : General Purpose Input/Output
*/
  unsigned char temp;
  LT7683_CmdWrite(0xF6);
  temp=LT7683_DataRead();
  return temp;
}

void Set_GPIO_E_In_Out(unsigned char temp) {
/*
GPIO-E_dir[7:0] : General Purpose I/O direction control.
0: Output
1: Input
*/
  LT7683_CmdWrite(0xF7);
  LT7683_DataWrite(temp);
}

void Write_GPIO_E_7_0(unsigned char temp) {
/*
GPIO-E[7:0] : General Purpose Input/Output.
share with {PDAT[23:19], PDAT[15:13]}
*/
  LT7683_CmdWrite(0xF8);
  LT7683_DataWrite(temp);
}

unsigned char Read_GPIO_E_7_0(void) {
/*
GPIO-E[7:0] : General Purpose Input/Output.
share with {PDAT[23:19], PDAT[15:13]}
*/
  unsigned char temp;
  LT7683_CmdWrite(0xF8);
  temp=LT7683_DataRead();
  return temp;
}

void Set_GPIO_F_In_Out(unsigned char temp) {
/*
GPIO-F_dir[7:0] : General Purpose I/O direction control.
0: Output
1: Input
*/
  LT7683_CmdWrite(0xF9);
  LT7683_DataWrite(temp);
}

void Write_GPIO_F_7_0(unsigned char temp) {
/*
GPIO-F[7:0] : General Purpose Input/Output.
share with {XPDAT[12:10], XPDAT[7:3]}
*/
  LT7683_CmdWrite(0xFA);
  LT7683_DataWrite(temp);
}

unsigned char Read_GPIO_F_7_0(void) {
/*
GPIO-F[7:0] : General Purpose Input/Output.
share with {XPDAT[12:10], XPDAT[7:3]}
*/
  unsigned char temp;
  LT7683_CmdWrite(0xFA);
  temp=LT7683_DataRead();
  return temp;
}

void Long_Key_enable(void) {
 /*
Key-Scan Control Register 1
[bit6]    LongKey Enable Bit
1 : Enable. Long key period is set by KSCR2 bit4-2.
0 : Disable.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xFB);
  temp=LT7683_DataRead();
  temp |= cSetb6;
  LT7683_DataWrite(temp);
}

void Key_Scan_Freg(unsigned char setx) {
/*KF2-0: Key-Scan Frequency */ 
  unsigned char temp;
  LT7683_CmdWrite(0xFB);
  temp = LT7683_DataRead();
  temp &= 0xf0;
  temp|= (setx&0x07);
  LT7683_DataWrite(temp);  
}

void Key_Scan_Wakeup_Function_Enable(void) {
/*
Key-Scan Controller Register 2
[bit7]  
Key-Scan Wakeup Function Enable Bit
0: Key-Scan Wakeup function is disabled.
1: Key-Scan Wakeup function is enabled.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xFC);
  temp=LT7683_DataRead();
  temp |= cSetb7;
  LT7683_DataWrite(temp);
}

void Long_Key_Timing_Adjustment(unsigned char setx) {
 /*Long Key Timing Adjustment*/ 
  unsigned char temp,temp1;
  temp = setx & 0x1c;
  LT7683_CmdWrite(0xFC);
  temp1 = LT7683_DataRead();
  temp1|=temp;
  LT7683_DataWrite(temp1);   
}

unsigned char Numbers_of_Key_Hit(void) {  
   unsigned char temp;
   LT7683_CmdWrite(0xFC);
   temp = LT7683_DataRead();   //read key touch number
   temp = temp & 0x03;   //¤U
   return temp;
}

unsigned char Read_Key_Strobe_Data_0(void) {
/*
Key Strobe Data 0
The corresponding key code 0 that is pressed.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xFD);
  temp=LT7683_DataRead();
  return temp;
}

unsigned char Read_Key_Strobe_Data_1(void) {
/*
Key Strobe Data 1
The corresponding key code 1 that is pressed.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xFE);
  temp=LT7683_DataRead();
  return temp;
}

unsigned char Read_Key_Strobe_Data_2(void) {
/*
Key Strobe Data 2
The corresponding key code 2 that is pressed.
*/
  unsigned char temp;
  LT7683_CmdWrite(0xFF);
  temp=LT7683_DataRead();
  return temp;
}

void Show_String(char *str) {   
    Text_Mode();     
    LT7683_CmdWrite(0x04);
    while(*str != '\0')
    {
      LT7683_DataWrite(*str);
      Check_Mem_WR_FIFO_not_Full();
      ++str;   
    }
    Check_2D_Busy();

    Graphic_Mode(); //back to graphic mode;
}

void Show_picture(unsigned long numbers,const unsigned short *datap) {   
  unsigned long i;

  LT7683_CmdWrite(0x04);  
  for(i=0;i<numbers;i++) {
    LT7683_DataWrite(datap[i]);
    Check_Mem_WR_FIFO_not_Full();
  }
}

/*0Bh bit1*/
void PWM0_ON(void) {
    //Graphic_cursor_initial(); //initaial and enable graphic cursor 

    Set_PWM_Prescaler_1_to_256(200); //Time base is "Core_Freq / (Prescaler + 1)";

    //Select_PWM0_Clock_Divided_By_1();
    //Select_PWM0_Clock_Divided_By_2();
    //Select_PWM0_Clock_Divided_By_4();
    Select_PWM0_Clock_Divided_By_8();   

    //Select_PWM0_is_GPIO_C7();
    //Select_PWM0_is_Core_Clock();
    Select_PWM0();
    //Select_PWM1();  //Enable dead zone function needs set 

    //Set_Timer0_Dead_Zone_Length(50);
    //Enable_PWM0_Dead_Zone();
    //Disable_PWM0_Dead_Zone();

    //Enable_PWM0_Inverter();
    //Disable_PWM0_Inverter();

    //Auto_Reload_PWM0(); //Default
    //One_Shot_PWM0();  //Buzzer

    Set_Timer0_Count_Buffer(0x100);
    Set_Timer0_Compare_Buffer(0xE6); //DUTY
  
    Start_PWM0();
}

void Enable_SPI_Flash_DMA(unsigned char val) {
  if(val == 0)  Select_SFI_0(); 
  else      Select_SFI_1();
  
  Select_SFI_DMA_Mode();      
  Select_SFI_24bit_Address(); 

  //Select_SFI_Waveform_Mode_0();//Choose communication methods based on FLASH
  Select_SFI_Waveform_Mode_3();

  //Select_SFI_0_DummyRead(); //normal read mode
  Select_SFI_8_DummyRead(); //1byte dummy cycle
  //Select_SFI_16_DummyRead();
  //Select_SFI_24_DummyRead();

  Select_SFI_Single_Mode();
  //Select_SFI_Dual_Mode0();
  //Select_SFI_Dual_Mode1();

  Enable_SFlash_SPI(); 
}