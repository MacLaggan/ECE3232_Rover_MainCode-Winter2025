/*
 * File:   Rover_Main_Ver01.c
 * Author:  Matthew D. MacLaggan
 *          Student, Electrical and Computer Engineering
 *          University of New Brunswick
 *          ECE3232, Winter 2025, The Harvest Competition
 *
 * Created on March 13, 2025, 4:19 PM
 * 
 * This code is the main code for which all systems on the harvest rover must interact through. 
 * A function will be provided for each members task. When that function is called, the task must be executed. 
 */
//==============================================================================
//<<<<<   Configuration Bits   >>>>>
// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)
// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)
// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)
// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = available// Scanner Enable bit (Scanner module is available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)
// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

//==============================================================================
//<<<<<   Included Libraries   >>>>>
#include <xc.h>

//==============================================================================
// Global Variables

//<<<<<   Character Arrays For LCD   >>>>>
char Rover_Team[] = {'<','<','R', 'O', 'V', 'E', 'R', ' ', 'T', 'E', 'A', 'M', ' ', '5', '>','>'};
char FFT_Home1[] = {'F','A','S','T',' ','F','O','U','R','I','E','R', ' ',' ',' ',' '};
char FFT_Home2[] = {'T','R','A','N','S','F','O','R','M',' ',' ',' ',' ',' ',' ',' '};
char Loading[] = {'|', '/','-','\\'};
char FFT_Result1[] = {'F', 'U', 'N', 'D', 'A', 'M','E','N','T','A','L'};
char FFT_Result2[] = {' ', 'H', 'z'};

//<<<<<   Variables   >>>>>
uint8_t UART_Not_Recieved = 0;  // This variable serves as a flag to count the number of UART transmissions recieved.
uint8_t bf; // A buffer for the I2C send function. It is global so that it canbe monitored with the debugger.

//==============================================================================
/* Structures Begin Here
 */

//<<<<<   LCD Structure   >>>>>
struct self{
	int i2c_addr;
	int i2c;
	int num_lines;
	int num_columns;
	int _current_state;
	int _backlight;
};
struct self LCD;

//==============================================================================
// Defined Values Begin Here

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

//<<<<<   System Defined Values   >>>>>
#define _XTAL_FREQ 32000000 // Defined clock frequency for the __delay_ms() function

//==============================================================================
// Functions Begin Here

//<<<<< Initialization Functions Begin Here   >>>>>
void configUART1(){
    //UART SETUP
    BAUD1CONbits.RCIDL = 1;
    BAUD1CONbits.SCKP = 0;
    BAUD1CONbits.WUE = 0;
    BAUD1CONbits.ABDEN = 0;
    BRG16 = 1;
    BRGH = 1;
    SP1BRGH = 0x3;
    SP1BRGL = 0x40;
    RC1STAbits.RX9 = 0;
    TX1STAbits.SYNC = 0;
    TX1STAbits.TXEN = 1;
    RC1STAbits.SPEN = 1;
    RC1STAbits.CREN = 1;
    RC1STAbits.FERR = 0;
    RC1STAbits.OERR = 0;
}

void configISR(){
    //Interrupt setup
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    
    //Enabling Interrupts
    PIE3bits.RCIE = 1;
}


//<<<<<   I2C Functions Begin Here   >>>>>
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

void HAL_I2C_Master_Transmit(uint8_t slaveAddress, uint8_t* dataByte, uint8_t numberOfDataBytes){
    I2C_start();
    I2C_write(slaveAddress);
    I2C_write(*dataByte);
    I2C_stop();
}



//<<<<<   UART1 Functions Begin Here   >>>>>
void UART1_sendData(uint8_t byte){
    TX1REG = byte;
    while(TX1STAbits.TRMT == 0){
    }  // Wait here until the byte has been sent
}

void UART1_recieveData(){
}
uint16_t message = 0; // Message recieved from other microcontroller

//<<<<<   TI ASYNCHRONOUS SERIAL   >>>>>
uint16_t TI_Recieve(){
    //<<<<<   Local Variable Declaration   >>>>>
    uint8_t stopBit = 1; // Flag which flips if a stop bit is detected (both lines low)
    uint16_t message = 0; // Message recieved from other microcontroller
    uint8_t framingError = 0; // Flag to detect framing error or overrun
    //<<<<<   Timeout Timer Beginning   >>>>>
    
    //<<<<<   Protocol Begins   >>>>>
    while((PORTBbits.RB0) == 1){    // While both lines are high, wait for communication (waiting till RB0 is low)
    }
    // RB0 is low
    LATBbits.LATB1 = 0;                             // Responding to start with 'ready'  
    while((PORTBbits.RB0) == 0){                    // Waiting till RB0 is brought high
    }
    LATBbits.LATB1 = 1;                             // Releasing ready bit
    
    //<<<<<   Data Transfer Beginning   >>>>>
    while((stopBit == 1)){               // Exit loop on stop bit or error
        while((PORTBbits.RB0 & PORTBbits.RB1) == 1){    // Waitin till a bit is sent
        }
        __delay_us(50);
        if((PORTBbits.RB0 == 0) & (PORTBbits.RB1 == 0)){       // A stop bit was sent, end transmission
            stopBit = 0;
            while((PORTBbits.RB0 & PORTBbits.RB1) == 0){// Waiting till both lines are high
            }
            LATBbits.LATB0 = 0;                         // Both are pulled low for "stop bit recieved"
            LATBbits.LATB1 = 0;
            __delay_us(20);
            LATBbits.LATB0 = 1;                         // Both are pulled low for "stop bit recieved"
            LATBbits.LATB1 = 1;
        }
        else if(PORTBbits.RB0 == 0){                    // A '0' was sent, a '0' is inserted to the right of lsb (msb first communication)
            message = (message << 1);

            LATBbits.LATB1 = 0;                         // Pulling RB1 down to signify 'recieved'
            while(PORTBbits.RB0 == 0){                  // Wait till RB0 is released
            }
            LATBbits.LATB1 = 1;                         // Releasing RB1
            // Bit Recieve Complete!
        }
        else{                                           // Otherwise a '1' was sent, message is left shifted and appended a '1'.
            message = (message << 1) + 1;
            LATBbits.LATB0 = 0;                         // Pulling RB0 down to signify 'recieved'
            while(PORTBbits.RB1 == 0){                  // Wait till RB1 is released
            }
            LATBbits.LATB0 = 1;                         // Releasing RB0
            // Bit Recieve Complete!
        }
        //framingError ++;
        
    } 
    return(message);
}

void TI_Send(uint16_t message, uint8_t length){
    //<<<<<   Local Variable Declaration   >>>>>
    uint8_t bitCounter = 0;                     // Counter which keeps track of bits sent
    uint8_t bit = 0;                            // individual bit sent
    uint16_t mask = (1 << (length - 1));
    //<<<<<   Protocol Begins   >>>>>
    while((PORTBbits.RB0 & PORTBbits.RB1) != 1){ // Ensure both lines are high before continuing
    }
    LATBbits.LATB0 = 0;                         // RB0 is pulled low to signify start bit
    while(PORTBbits.RB1 == 1){                  // Wait until ready is sent on RB1
    }                                           
    LATBbits.LATB0 = 1;                         // Releasing start bit
    while(PORTBbits.RB1 == 0){                  // Waiting till ready bit is released
    }
    
    //<<<<<   Data Transfer Begin   >>>>>
    for(bitCounter; bitCounter <= length; bitCounter++){
        if((message & mask) == 1){
            LATBbits.LATB1 = 0;                 // Sending a '0'
            while(PORTBbits.RB0 == 1){          // Waiting for response
            }
            LATBbits.LATB1 = 1;                 // Releasing RB1
            while(PORTBbits.RB0 == 0){          // Waiting until other line is released
            }
        }
        else{
            LATBbits.LATB0 = 0;                 // Sending a '1'
            while(PORTBbits.RB1 == 1){          // Waiting for response
            }
            LATBbits.LATB0 = 1;                 // Releasing RB0
            while(PORTBbits.RB1 == 0){          // Waiting till other line is released
            }
        }
    }
    LATBbits.LATB0 = 0;
    LATBbits.LATB1 = 0;
    __delay_us(50);
    LATBbits.LATB0 = 1;
    LATBbits.LATB1 = 1;
    while((PORTBbits.RB0 & PORTBbits.RB1) == 1){// Waiting till both lines are pulled low 
    }
    while((PORTBbits.RB0 & PORTBbits.RB1) == 0){// Waiting till both lines are pulled high
    }
    // Transmission over
}




//<<<<<   LCD Functions Begin Here   >>>>>
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

void init_LCD(){
	HAL_Display_On();
	HAL_Clear();
	HAL_Set_Entry_Mode(1,0);
}

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

//<<<<<   LED ON/OFF Functions   >>>>>
// HAL_LED_On when called, will turn on D2, D3, D4, or D5, by entering 1 through 4 respectively.
// Note that the function can only turn on one LED per call, and does not affect any other LEDs
void HAL_LED_ON(int LED){
	switch(LED){
		case 1:                     // Turning on D2
			LATAbits.LATA0 = 1;
			break;
		case 2:                     // Turning on D3
			LATAbits.LATA1 = 1;
			break;
		case 3:                     // Turning on D4
			LATAbits.LATA2 = 1;
			break;
        case 4:                     // Turning on D5
            LATAbits.LATA3 = 1;
            break;
	}
}

void HAL_LED_OFF(int LED){
	switch(LED){
		case 1:
            LATAbits.LATA0 = 0;
			break;
		case 2:
            LATAbits.LATA1 = 0;
			break;
		case 3:
            LATAbits.LATA2 = 0;
			break;
        case 4:
            LATAbits.LATA3 = 0;
            break; 
	}
}


//<<<<<   Pin Config   >>>>>
void configPins(){
    //<<<<<   LED Config   >>>>>
    TRISAbits.TRISA0 = 0;
    ANSELAbits.ANSA0 = 0;
    
    TRISAbits.TRISA1 = 0;
    ANSELAbits.ANSA1 = 0;
    
    TRISAbits.TRISA2 = 0;
    ANSELAbits.ANSA2 = 0;
    
    TRISAbits.TRISA3 = 0;
    ANSELAbits.ANSA4 = 0;
    
    //<<<<<   Configuring Communication Pins   >>>>>
    // configuring UART 1 pin Rx
    TRISCbits.TRISC7 = 1;
    ANSELCbits.ANSC7 = 0;
    
    // configuring I2C SDA
    TRISCbits.TRISC3 = 1;
    ANSELCbits.ANSC3 = 0;
    
    // configuring I2C SCL
    TRISCbits.TRISC4 = 1;
    ANSELCbits.ANSC4 = 0;
    
    // configuring FFT enable pin
    TRISBbits.TRISB4 = 0;
    ANSELBbits.ANSB4 = 0;
    
    // Texas Instrument Serial Handshake Pin Configuration
    // TX0 pin config, represents a 0 when pulled low
    TRISBbits.TRISB0 = 0;
    ANSELBbits.ANSB0 = 0;
    ODCONBbits.ODCB0 = 1;  // Enable open-drain on RB0
    WPUBbits.WPUB0 = 1;   // Enable weak pull-up on RB0
    // TX1 pin config, represents a 1 when pulled low
    TRISBbits.TRISB1 = 0;
    ANSELBbits.ANSB1 = 0;
    ODCONBbits.ODCB1 = 1;  // Enable open-drain on RB1
    WPUBbits.WPUB1 = 1;   // Enable weak pull-up on RB1
    
    //<<<<<   Configuring User Input Pins   >>>>>
    // Switch 2 pin config
    TRISAbits.TRISA5 = 1;
    ANSELAbits.ANSA5 = 0;
    
    // <<<<<   Configuring PPS   >>>>>
    PPSLOCKbits.PPSLOCKED = 0;
    
    RC4PPS = 0x16; // I2C SCL (RB3)
    SSP2CLKPPSbits.SSP2CLKPPS = 0x14;
    
    SSP2DATPPSbits.SSP2DATPPS = 0x13;
    RC3PPS = 0x17; // I2C SDA (RC3)
    
    RC5PPS = 0x10; // UART1 TX
    
    PPSLOCKbits.PPSLOCKED = 1;
    
}//

char intToChar(uint16_t integer){
    switch(integer){
        case 0:
            return('0');
        case 1:
            return('1');
        case 2:
            return('2');
        case 3:
            return('3');
        case 4:
            return('4');
        case 5:
            return('5');
        case 6:
            return('6');
        case 7:
            return('7');
        case 8:
            return('8');
        case 9:
            return('9');    
    }
    return('_');
}

//==============================================================================
/* Member Function Begin Here
 */

char freq_char[4];
uint16_t freq_digit[4];

void laserTurretDefence(){
}//

uint16_t frequency = 0xFFFF;
void alienFrequencyTask(){
   
    // Running LCD code
    HAL_Clear();
    HAL_LCD_Print(&FFT_Home1[0], 16);
    HAL_Set_Cursor(0, 1);
    HAL_LCD_Print(&FFT_Home2[0], 16);
    __delay_ms(1000);
    HAL_Clear();
    
    // writing 1 to RB4 to enable FFT function
    LATBbits.LATB4 = 0;
    __delay_ms(1);
    LATBbits.LATB4 = 1;
    
    
    // Timeout loop
    UART_Not_Recieved = 0;
    while(UART_Not_Recieved != 0){
    }
    HAL_Clear();

    frequency = TI_Recieve();
    
    uint16_t counter = 0;
    HAL_Clear();
    uint16_t i = 0;
    
    
    uint16_t temp = frequency;  // temporary storage of frequency
    counter = 0;
    while(temp/=10){
        counter ++;  // counting digits
    }
    uint8_t digit = counter;
    while(frequency > 0){
        freq_digit[counter]= frequency % 10;
        frequency/=10;
        counter--;
    }
    
    freq_char[0] = intToChar(freq_digit[0]);
    freq_char[1] = intToChar(freq_digit[1]);
    freq_char[2] = intToChar(freq_digit[2]);
    freq_char[3] = intToChar(freq_digit[3]);
    
    HAL_LCD_Print(&FFT_Result1[0], 11);
    
    HAL_Set_Cursor(0, 1);
    switch(digit +    1){
        case 1:
            HAL_LCD_Print(&freq_char[0], 1);
            break;
        case 2:
            HAL_LCD_Print(&freq_char[0], 2);
            break;
        case 3:
            HAL_LCD_Print(&freq_char[0], 3);
            break;
        case 4:
            HAL_LCD_Print(&freq_char[0], 4);
            break;
    
    }
    HAL_Set_Cursor(4,1);
    HAL_LCD_Print(&FFT_Result2[0], 3);

    
    __delay_ms(100);
    HAL_Clear();
    // 
}

void magneticMaterialDetection(){
}

void IRdetection(){
}

//==============================================================================
//<<<<<   Main Initialization Code   >>>>>

void initialize_all(){
    LCD._backlight = 0x8;
    LCD._current_state = 0x0;
    LCD.i2c = 1;
    LCD.i2c_addr = 39<<1;
    LCD.num_columns = 16;
    LCD.num_lines = 2;
    configPins();
    configI2C();
    configUART1();
    initinit();
    init_LCD();
    configISR();
    //<<<<<   Initializing LEDs   >>>>>
    HAL_LED_OFF(1);
    HAL_LED_OFF(2);
    HAL_LED_OFF(3);
    HAL_LED_OFF(4);
    //<<<<<   Init TI Com   >>>>>
    LATBbits.LATB0 = 1;
    LATBbits.LATB1 = 1;
    
}
//==============================================================================
//<<<<<   Main Code   >>>>>

void main(void) {
    initialize_all();
    LATBbits.LATB4 = 1;
    __delay_ms(100);
    while(1){
        HAL_LCD_Print(&Rover_Team[0], 16);
        while(PORTAbits.RA5 == 1){
        }
        alienFrequencyTask();
    }
    return;
}

void __interrupt() ISR ()
{
    if(PIR3bits.RCIF == 1){
        if(UART_Not_Recieved == 2){
            frequency = RC1REG << 8;
        }
        else{
            frequency = frequency + RC1REG;
        }

        UART_Not_Recieved--;
        PIR3bits.RCIF = 0;
    }
}
