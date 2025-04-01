/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

//==============================================================================
//<<<<< LCD API Variables   >>>>>
uint8_t bf; // A buffer for the I2C send function. It is global so that it canbe monitored with the debugger.

//<<<<<   LCD Defined Values   >>>>>
#define LCD_CLR 0x01
#define LCD_CLR 0x01
#define LCD_HOME 0x02

#define LCD_ENTRY_MODE 0x04
#define LCD_ENTRY_INC 0x02
#define LCD_ENTRY_SHIFT 0x01

#define LCD_ON_CTRL 0x08
#define LCD_ON_DISPLAY 0x04
#define LCD_ON_CURSOR 0x02
#define LCD_ON_BLINK 0x01

#define LCD_MOVE 0x10
#define LCD_MOVE_DISP 0x08
#define LCD_MOVE_RIGHT 0x04

#define LCD_FUNCTION 0x20
#define LCD_FUNCTION_8BIT 0x10
#define LCD_FUNCTION_2LINES 0x08
#define LCD_FUNCTION_10DOTS 0x04
#define LCD_FUNCTION_RESET 0x30

#define LCD_CGRAM 0x40
#define LCD_DDRAM 0x80

#define LCD_RS_CMD 0
#define LCD_RS_DATA 1

#define LCD_RW_WRITE 0
#define LCD_RW_READ 1

#define _XTAL_FREQ 32000000 // Defined clock frequency for the __delay_ms() function

//<<<<<   LCD structure   >>>>>
struct self{
	int i2c_addr;
	int i2c;
	int num_lines;
	int num_columns;
	int _current_state;
	int _backlight;
};
struct self LCD;


//<<<<<   Functions Begin   >>>>>

void I2C_start(){
    // start bit
    SSP2CON2bits.SEN = 1;
    
    while(SSP2CON2bits.SEN == 1){
        // waiting on start sequence completion
    }
}

void I2C_write(uint8_t data){
    SSP2BUF = data;
    bf = SSP2STATbits.BF;
    while(SSP2STATbits.BF == 1){
        NOP();
    }
    bf = SSP2STATbits.BF;

}

uint8_t I2C_ack_status(){
    if(SSP2CON2bits.ACKSTAT == 1){
        return 1;
    }
    return 0;
}

void I2C_stop(){
    SSP2CON2bits.PEN = 1;
    while(SSP2CON2bits.PEN == 1){
        // wait on stop condition to complete
    }
}

void configI2C(){
    // disabling module
    SSP2CON1bits.SSPEN = 0;
    
    // configuring I2C clock
    SSP2ADDbits.SSPADD = 19; // set to 7 to configure for 1 MHz
    
    // configuring host mode to I2C host
    SSP2CON1bits.SSPM = 0b1000;
    
    // enabling the module
    SSP2CON1bits.SSPEN = 1;
}

void HAL_I2C_Master_Transmit(uint8_t slaveAddress, uint8_t* dataByte, uint8_t numberOfDataBytes){
    I2C_start();
    I2C_write(slaveAddress);
    I2C_write(*dataByte);
    I2C_stop();
}

void HAL_Pulse_Enable(uint8_t byte){
	byte = (byte | 0x04 | LCD._backlight);
	HAL_I2C_Master_Transmit(LCD.i2c_addr, &byte, 1);
	byte = ((byte & ~0x04) | LCD._backlight);
	HAL_I2C_Master_Transmit(LCD.i2c_addr, &byte, 1);
}

void _write_byte(uint8_t byte){
	byte = (byte | LCD._backlight);
    HAL_I2C_Master_Transmit(LCD.i2c_addr, &byte,1);
    HAL_Pulse_Enable(byte);
}

void HAL_Write_Init_Nibble(uint8_t nibble){
	uint8_t byte = ((nibble >> 4) & 0x0F) << 4;
	byte = (byte | LCD_RS_CMD | LCD._backlight);
	HAL_I2C_Master_Transmit(LCD.i2c_addr, &byte, 1);
	HAL_Pulse_Enable(byte);
}

void HAL_Write_Command(uint8_t cmd){
	_write_byte((cmd & 0xF0) | LCD_RS_CMD);
	_write_byte(((cmd<<4) & 0xF0) | LCD_RS_CMD);
	if (cmd <= 3){
		__delay_ms(5);
	}
}

void HAL_Backlight_On(){
    LCD._backlight = 0x08;
    uint8_t byte = (LCD._current_state | LCD._backlight);
	HAL_I2C_Master_Transmit(LCD.i2c_addr, &byte,1);
}

void HAL_Backlight_Off(){
    LCD._backlight = 0x00;
    uint8_t byte = (LCD._current_state | LCD._backlight);
    HAL_I2C_Master_Transmit(LCD.i2c_addr, &byte,1);
}



void HAL_Write_Data(uint8_t data){
	_write_byte((data & 0xF0)| LCD_RS_DATA);
	_write_byte(((data << 4) & 0xF0) | LCD_RS_DATA);
}


void HAL_Clear(){
	HAL_Write_Command(LCD_CLR);
	__delay_ms(3);
}

void HAL_Show_Cursor(){
	HAL_Write_Command(LCD_ON_CTRL | LCD_ON_DISPLAY | LCD_ON_CURSOR);
}

void HAL_Hide_Cursor(){
	HAL_Write_Command(LCD_ON_CTRL | LCD_ON_DISPLAY);
}

void HAL_Blink_Cursor_On(){
	HAL_Write_Command(LCD_ON_CTRL | LCD_ON_DISPLAY | LCD_ON_CURSOR | LCD_ON_BLINK);
}

void HAL_Blink_Cursor_Off(){
	HAL_Write_Command(LCD_ON_CTRL | LCD_ON_DISPLAY | LCD_ON_CURSOR);
}

void HAL_Display_On(){
	HAL_Write_Command(LCD_ON_CTRL | LCD_ON_DISPLAY);
}

void HAL_Display_Off(){
	HAL_Write_Command(LCD_ON_CTRL);
}

void HAL_Set_Entry_Mode(int inc, int shift){
	int entry_mode = LCD_ENTRY_MODE;
	if (inc){
		entry_mode |= LCD_ENTRY_INC;
	}
	if (shift){
		entry_mode |= LCD_ENTRY_SHIFT;
	}
	HAL_Write_Command(entry_mode);
}

void HAL_Home(){
	HAL_Write_Command(LCD_HOME);
	__delay_ms(3);
}

void HAL_Set_Cursor(int col, int line){
	int addr = col & 0x3F;
	if (line & 1){
		addr += 0x40;
	}
	if(line & 2){
		addr += 0x14;
	}
	HAL_Write_Command(LCD_DDRAM | addr);
}

void HAL_LCD_Print(char* string, int size){
	for(int i=0;i<size;i++){
		HAL_Write_Data(*string);
		string = string + 1;
		__delay_ms(5);
	}
}

// init_LCD simply clears the display, turns the display on, and sets the cursor. It is for ease of use. 
void init_LCD(){
	HAL_Display_On();
	HAL_Clear();
	HAL_Set_Entry_Mode(1,0);
}


// initinit is a initialization function which makes the LCD useable. It must be run before any other function requiring the LCD in your code. 
void initinit(){
	__delay_ms(20);
	HAL_Write_Init_Nibble(LCD_FUNCTION_RESET);
	__delay_ms(5);
	HAL_Write_Init_Nibble(LCD_FUNCTION_RESET);
	__delay_ms(1);
	HAL_Write_Init_Nibble(LCD_FUNCTION_RESET);
	__delay_ms(1);
	HAL_Write_Init_Nibble(LCD_FUNCTION);
	__delay_ms(1);
	init_LCD();
}










// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

