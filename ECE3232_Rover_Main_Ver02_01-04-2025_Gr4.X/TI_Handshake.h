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
#ifndef XC_HEADER
#define	XC_HEADER

#include <xc.h> // include processor files - each processor file is guarded.  

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

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER */

