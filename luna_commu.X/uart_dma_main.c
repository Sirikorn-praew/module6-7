

#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "configuration.h"

#define FCY      40000000
#define BAUDRATE 115200             
#define BRGVAL   ((FCY/BAUDRATE)/16)-1 

volatile uint64_t millis = 0;
volatile unsigned char state = 0; 
volatile unsigned char old_state = 0; 
volatile unsigned char state_ack_nano = 0;
unsigned char Buffer_in[50] __attribute__((space(dma)));
unsigned char Buffer_out[50] __attribute__((space(dma)));
unsigned char Buffer_nano_in[50] __attribute__((space(dma)));
unsigned char Buffer_nano_out[50] __attribute__((space(dma)));
double setpoint_x = 0;
double setpoint_y = 0;
int posx=0,posy=0;

// UART Configuration
void cfgUart1(void)
{
	U1MODEbits.STSEL = 0;			// 1-stop bit
	U1MODEbits.PDSEL = 0;			// No Parity, 8-data bits
	U1MODEbits.ABAUD = 0;			// Autobaud Disabled

	U1BRG = BRGVAL;					// BAUD Rate Setting for 115200


	//********************************************************************************
	//  STEP 1:
	//  Configure UART for DMA transfers
	//********************************************************************************/
	U1STAbits.UTXISEL0 = 0;			// Interrupt after one Tx character is transmitted
	U1STAbits.UTXISEL1 = 0;			                            
	U1STAbits.URXISEL  = 0;			// Interrupt after one RX character is received

	
	//********************************************************************************
	//  STEP 2:
	//  Enable UART Rx and Tx
	//********************************************************************************/
	U1MODEbits.UARTEN   = 1;		// Enable UART
	U1STAbits.UTXEN     = 1;		// Enable UART Tx


	IEC4bits.U1EIE = 0;
}  
void cfgUart2(void)
{
	U2MODEbits.STSEL = 0;			// 1-stop bit
	U2MODEbits.PDSEL = 0;			// No Parity, 8-data bits
	U2MODEbits.ABAUD = 0;			// Autobaud Disabled

	U2BRG = BRGVAL;					// BAUD Rate Setting for 115200


	//********************************************************************************
	//  STEP 1:
	//  Configure UART for DMA transfers
	//********************************************************************************/
	U2STAbits.UTXISEL0 = 0;			// Interrupt after one Tx character is transmitted
	U2STAbits.UTXISEL1 = 0;			                            
	U2STAbits.URXISEL  = 0;			// Interrupt after one RX character is received

	
	//********************************************************************************
	//  STEP 2:
	//  Enable UART Rx and Tx
	//********************************************************************************/
	U2MODEbits.UARTEN   = 1;		// Enable UART
	U2STAbits.UTXEN     = 1;		// Enable UART Tx
}  
// DMA0 configuration
void cfgDma0UartTx(void)
{
	//********************************************************************************
	//  STEP 3:
	//  Associate DMA Channel 0 with UART Tx
	//********************************************************************************/
	DMA0REQ = 0x000C;					// Select UART1 Transmitter
	DMA0PAD = (volatile unsigned int) &U1TXREG;
	
	//********************************************************************************
	//  STEP 5:
	//  Configure DMA Channel 0 to:
	//  Transfer data from RAM to UART
	//  One-Shot mode
	//  Register Indirect with Post-Increment
	//  Using single buffer
	//  8 transfers per buffer
	//  Transfer words
	//********************************************************************************/
	// One-Shot, Post-Increment, RAM-to-Peripheral
	DMA0CONbits.AMODE = 0;
	DMA0CONbits.MODE  = 1;
	DMA0CONbits.DIR   = 1;
	DMA0CONbits.SIZE  = 1;
	DMA0CNT = 1;						// 1 DMA requests

	//********************************************************************************
	//  STEP 6:
	// Associate one buffer with Channel 0 for one-shot operation
	//********************************************************************************/
	DMA0STA = __builtin_dmaoffset(Buffer_out);

	//********************************************************************************
	//  STEP 8:
	//	Enable DMA Interrupts
	//********************************************************************************/
	IFS0bits.DMA0IF  = 0;			// Clear DMA Interrupt Flag
	IEC0bits.DMA0IE  = 1;			// Enable DMA interrupt

}
// DMA1 configuration
void cfgDma1UartRx(void)
{
	//********************************************************************************
	//  STEP 3:
	//  Associate DMA Channel 1 with UART Rx
	//********************************************************************************/
	DMA1REQ = 0x000B;					// Select UART1 Receiver
	DMA1PAD = (volatile unsigned int) &U1RXREG;

	//********************************************************************************
	//  STEP 4:
	//  Configure DMA Channel 1 to:
	//  Transfer data from UART to RAM Continuously
	//  Register Indirect with Post-Increment
	//  Using two ‘ping-pong’ buffers
	//  8 transfers per buffer
	//  Transfer words
	//********************************************************************************/
	//DMA1CON = 0x0002;					// Continuous,  Post-Inc, Periph-RAM
	DMA1CONbits.AMODE = 0;
	DMA1CONbits.MODE  = 0;
	DMA1CONbits.DIR   = 0;
	DMA1CONbits.SIZE  = 1;
	DMA1CNT = 10;						// 6 DMA requests

	//********************************************************************************
	//  STEP 6:
	//  Associate two buffers with Channel 1 for ‘Ping-Pong’ operation
	//********************************************************************************/
	DMA1STA = __builtin_dmaoffset(Buffer_in);

	//********************************************************************************
	//  STEP 8:
	//	Enable DMA Interrupts
	//********************************************************************************/
	IFS0bits.DMA1IF  = 0;			// Clear DMA interrupt
	IEC0bits.DMA1IE  = 1;			// Enable DMA interrupt

	//********************************************************************************
	//  STEP 9:
	//  Enable DMA Channel 1 to receive UART data
	//********************************************************************************/
	DMA1CONbits.CHEN = 1;			// Enable DMA Channel
}
void cfgDma2UartTx2(void)
{
	//********************************************************************************
	//  STEP 3:
	//  Associate DMA Channel 2 with UART2 Tx
	//********************************************************************************/
	DMA2REQ = 0x001F;					// Select UART2 Transmitter
	DMA2PAD = (volatile unsigned int) &U2TXREG;
	
	//********************************************************************************
	//  STEP 5:
	//  Configure DMA Channel 0 to:
	//  Transfer data from RAM to UART
	//  One-Shot mode
	//  Register Indirect with Post-Increment
	//  Using single buffer
	//  8 transfers per buffer
	//  Transfer words
	//********************************************************************************/
	// One-Shot, Post-Increment, RAM-to-Peripheral
	DMA2CONbits.AMODE = 0;
	DMA2CONbits.MODE  = 1;
	DMA2CONbits.DIR   = 1;
	DMA2CONbits.SIZE  = 1;
	DMA2CNT = 5;						// 1 DMA requests

	//********************************************************************************
	//  STEP 6:
	// Associate one buffer with Channel 0 for one-shot operation
	//********************************************************************************/
	DMA2STA = __builtin_dmaoffset(Buffer_nano_out);

	//********************************************************************************
	//  STEP 8:
	//	Enable DMA Interrupts
	//********************************************************************************/
	IFS1bits.DMA2IF  = 0;			// Clear DMA Interrupt Flag
	IEC1bits.DMA2IE  = 1;			// Enable DMA interrupt

}
void cfgDma3UartRx2(void)
{
	//********************************************************************************
	//  STEP 3:
	//  Associate DMA Channel 1 with UART Rx
	//********************************************************************************/
	DMA3REQ = 0x1E;					// Select UART1 Receiver
	DMA3PAD = (volatile unsigned int) &U2RXREG;

	//********************************************************************************
	//  STEP 4:
	//  Configure DMA Channel 1 to:
	//  Transfer data from UART to RAM Continuously
	//  Register Indirect with Post-Increment
	//  Using two ‘ping-pong’ buffers
	//  8 transfers per buffer
	//  Transfer words
	//********************************************************************************/
	//DMA1CON = 0x0002;					// Continuous,  Post-Inc, Periph-RAM
	DMA3CONbits.AMODE = 0;
	DMA3CONbits.MODE  = 0;
	DMA3CONbits.DIR   = 0;
	DMA3CONbits.SIZE  = 1;
	DMA3CNT = 10;						// 1 DMA requests

	//********************************************************************************
	//  STEP 6:
	//  Associate two buffers with Channel 1 for ‘Ping-Pong’ operation
	//********************* ***********************************************************/
	DMA3STA = __builtin_dmaoffset(Buffer_nano_in);

	//********************************************************************************
	//  STEP 8:
	//	Enable DMA Interrupts
	//********************************************************************************/
	IFS2bits.DMA3IF  = 0;			// Clear DMA interrupt
	IEC2bits.DMA3IE  = 1;			// Enable DMA interrupt

	//********************************************************************************
	//  STEP 9:
	//  Enable DMA Channel 1 to receive UART data
	//********************************************************************************/
	DMA3CONbits.CHEN = 1;			// Enable DMA Channel
}
void initGPIO()
{
    AD1PCFGL = 0xFFFF;          //set analog input to digital pin
    TRISBbits.TRISB2 = 0; //set RB2 to output
    TRISBbits.TRISB3 = 0; //set RB3 to output
    TRISAbits.TRISA0 =0;
    
}

//interrupt function
void __attribute__((interrupt,no_auto_psv)) _T1Interrupt(void)
{
    millis++;   //add millis by 1 every 1 ms
    _T1IF =0 ;  //clear interrupt flag
}
void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
	IFS0bits.DMA0IF = 0;			// Clear the DMA0 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)
{   
    DMA0STA = __builtin_dmaoffset(Buffer_out); // Point DMA 0 to data to be transmitted
    memcpy ( &Buffer_nano_out, &Buffer_in, sizeof(Buffer_in) );
    DMA2CONbits.CHEN  = 1;			// Re-enable DMA2 Channel
    DMA2REQbits.FORCE = 1;	
    if(Buffer_in[0] == 0xFF){
        if(Buffer_in[1] == 0xF0){
            state = 0;
        }
        else if(Buffer_in[1] == 0xF1){
            //home
            state = 1;
        }
        else if(Buffer_in[1] == 0xF2){
            //capture
            state = 2;         
        }
        else if(Buffer_in[1] == 0xF3){
            //catch 
            state = 3;
        }
        else if(Buffer_in[1] == 0xF4){
          //move to
            state = 4;
        }
        else if(Buffer_in[1] == 0xF5){
            //move one
            state = 5;
        }
        else if(Buffer_in[1] == 0xFF){
            //move one
            state = 255;
        }
        else if(Buffer_in[1] == 0xBB){
            sprintf(Buffer_out,"reset done");
            print_uart1();
            state = 0;
        }
     }		
	IFS0bits.DMA1IF = 0;			// Clear the DMA1 Interrupt Flag
}

void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void)
{
	IFS1bits.DMA2IF = 0;			// Clear the DMA0 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA3Interrupt(void)
{
    if (Buffer_nano_in[0] == 0xFF)
    {
        if (Buffer_nano_in[1] == 0xAA)
        {
            state_ack_nano = Buffer_nano_in[2];
        }
        else if (Buffer_nano_in[1] == 0x99)
        {
            sprintf(Buffer_in, "communication error dspic & arduino !");
            print_uart1();
        }
    }
    IFS2bits.DMA3IF = 0; // Clear the DMA0 Interrupt Flag;
}

void initPLL()
{
    PLLFBD = 63;           // M  = 152
    CLKDIVbits.PLLPRE = 1;  // N1 = 7
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    OSCTUN = 0;             // Tune FRC oscillator, if FRC is used
    
    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x01);    // Initiate Clock Switch to FRCPLL
    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01);    // Start clock switching

    while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur
    while (OSCCONbits.LOCK!=1) {};    // Wait for PLL to lock
}

void print_uart1()
{
    DMA0STA = __builtin_dmaoffset(Buffer_out);
    DMA0CNT = strlen(Buffer_out) - 1;
    DMA0CONbits.CHEN = 1;  // Re-enable DMA0 Channel
    DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer
}

void print_uart2()
{
    DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
    DMA2CNT = strlen(Buffer_nano_out) - 1;
    DMA2CONbits.CHEN = 1;  // Re-enable DMA2 Channel
    DMA2REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer
}
void delay(int time)
{
    int i = 0, j = 0;
    for (i = 0; i <= time; i++)
    {
        for (j = 0; j <= 4000; j++)
        {
        }
    }
}
int main(int argc, char** argv) {
    __builtin_disable_interrupts();
    
    initPLL();
    initGPIO();
    
    __builtin_write_OSCCONL(OSCCON & 0xBF); // to clear IOLOCK
        // Assign U1TX to RP6, Pin 15
        RPOR2bits.RP5R = 3;
        // Assign U1RX to RP5, Pin 14
        _U1RXR = 6;
        RPOR2bits.RP4R = 0b00101; 
        _U2RXR = 11;
        
    __builtin_write_OSCCONL(OSCCON | 0x40); // to set IOLOCK
    
    
    T1CONbits.TCKPS = 0b01; //set timer prescaler to 1:8
    PR1 = 5000;             //set period to 1 ms
    _T1IE = 1;              //enable interrupt for timer1
    _T1IP = 3;              //set interrupt priority to 3 
  
     /*enable global interrupt*/
    __builtin_enable_interrupts();
    
    //This routine Configures DMAchannel 0 for transmission.	
	cfgDma0UartTx();
    cfgDma2UartTx2();
	//This routine Configures DMAchannel 1 for reception.
	cfgDma1UartRx();
    cfgDma3UartRx2();
	// UART Configurations
    cfgUart1();
    cfgUart2();
    T1CONbits.TON =1;            //enable timer1
    while(1){
        if(state != old_state) {
            if (state == 0) {
                sprintf(Buffer_out,"state = 0 ");
                print_uart1();
                //nop();
            } else if (state == 1) {
                sprintf(Buffer_out,"state = 1 ");
                print_uart1();
                state = 0;
                //comebackhome();
            } else if (state == 2) {
                sprintf(Buffer_out,"state = 2 ");
                print_uart1();
                //capture();
                state = 0;
            } else if (state == 3) {
                sprintf(Buffer_out,"state = 3 ");
                print_uart1();
                //catch();
                state = 0;
            } else if (state == 4) {
                sprintf(Buffer_out,"state = 4 ");
                print_uart1();
                state = 0;
            } 
            else if (state == 255) {
                sprintf(Buffer_out,"state extra ");
                print_uart1();
                delay(500);
                while(state_ack_nano != 0xFA){
                    //sprintf(Buffer_out,"wtf");
                    //print_uart1();
                    U2TXREG = 0xAA;
                    delay(300);
                }
                sprintf(Buffer_out,"complete");
                print_uart1();
                state_ack_nano = 0;
                state = 0;
            }
            old_state = state;
        }
    }
    
    return (EXIT_SUCCESS);
}

