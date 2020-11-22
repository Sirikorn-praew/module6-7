

#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "configuration.h"

//pin out
#define lim_sw1 2
#define lim_sw2 3
#define en1_a 7
#define en1_b 8
#define en2_a 9
#define en2_b 10
#define inA1 0
#define inB1 1
#define inA2 13
#define inB2 15

#define FCY      40000000
#define BAUDRATE 115200             
#define BRGVAL   ((FCY/BAUDRATE)/16)-1 

//Position
#define kp_x 2
#define ki_x 0
#define kd_x 0
#define kp_y 2
#define ki_y 0
#define kd_y 0
#define k 150
double setpoint_x = (long int) 0;
double setpoint_y = (long int) 0;
uint16_t posx = 0, posy = 0;

//traj.
volatile double T, t;
volatile double theta;
volatile double c0, c1, c, c2, c3;
volatile double rt = 0.0;
volatile double rf = 0.0;
volatile long int x0, y0;

volatile char home_y_state = 0;
volatile char home_x_state = 0;

volatile unsigned char state = 0;
volatile unsigned char old_state = 0;
volatile unsigned char state_ack_nano = 0;
volatile unsigned char state_ack_com = 0;
volatile unsigned char state_ack_end = 0;
unsigned char Buffer_in[50] __attribute__((space(dma)));
unsigned char Buffer_out[50] __attribute__((space(dma)));
unsigned char Buffer_nano_in[50] __attribute__((space(dma)));
unsigned char Buffer_nano_out[50] __attribute__((space(dma)));


void initPLL() {
    PLLFBD = 63; // M  = 152
    CLKDIVbits.PLLPRE = 1; // N1 = 7
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    OSCTUN = 0; // Tune FRC oscillator, if FRC is used

    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to FRCPLL
    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01); // Start clock switching

    while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur
    while (OSCCONbits.LOCK != 1) {
    }; // Wait for PLL to lock
}

void init_GPIO() {
    AD1PCFGL = 0xFFFF;
    TRISBbits.TRISB2 = 1; //set RB2 to output
    TRISBbits.TRISB3 = 1; //set RB3 to output
    _TRISB7 = 1;
    _TRISB8 = 1;
    _TRISB9 = 1;
    _TRISB10 = 1;
    _TRISB12 = 0;
    _TRISB13 = 0;
    _TRISB14 = 0;
    _TRISB15 = 0;
    _TRISA0 = 0;
    _TRISA1 = 0;

}

void setpwm(uint16_t prescale, uint16_t freq) {
    if (prescale == 0) {
        T2CONbits.TCKPS = 0b00;
    } else if (prescale == 8) {
        T2CONbits.TCKPS = 0b01;
    } else if (prescale == 64) {
        T2CONbits.TCKPS = 0b10;
    } else if (prescale == 256) {
        T2CONbits.TCKPS = 0b11;
    }
    PR2 = (FCY / prescale) / freq;
    OC1RS = 0;
    OC2RS = 0;
    OC1CONbits.OCM = 0b000; //Disable Output Compare Module
    OC1CONbits.OCTSEL = 0; //OC1 use timer2 as counter source
    OC1CONbits.OCM = 0b110; //set to pwm without fault pin mode
    OC2CONbits.OCM = 0b000; //Disable Output Compare Module
    OC2CONbits.OCTSEL = 0; //OC2 use timer2 as counter source
    OC2CONbits.OCM = 0b110; //set to pwm without fault pin mode
    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK
    _RP12R = 0b10010; //remap RP11 connect to OC1
    _RP14R = 0b10011; //remap RP14 connect to OC2
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK
}

void setperiod_position() {
    T1CONbits.TCKPS = 0b10; //pre-scale 1:8
    PR1 = 25000; //period 25000 tick per cycle
    _T1IE = 1; //enable interrupt for timer1
    _T1IP = 4; //set interrupt priority to  4
} //control loop 200 hz

void init_ex_int0(uint8_t edge, uint8_t priority) {
    IPC0bits.INT0IP = priority; //set interrupt 0
    INTCON2bits.INT0EP = edge; //positive edge
    IEC0bits.INT0IE = 1; //enable interrupt 0
#ifdef debug
    printf("INT0 enable detect at edge = %d and set priority = %d \n", edge, priority);
#endif
}

void init_ex_int1(uint8_t pin, uint8_t edge, uint8_t priority) {
    __builtin_write_OSCCONL(OSCCON & 0xBF);
    RPINR0bits.INT1R = pin;
    __builtin_write_OSCCONL(OSCCON | 0x40);
    IPC5bits.INT1IP = priority;
    INTCON2bits.INT1EP = edge;
    IEC1bits.INT1IE = 1;
#ifdef debug
    printf("INT1 enable at pin %d detect at edge = %d and set priority = %d \n", pin, edge, priority);
#endif
}

void init_ex_int2(uint8_t pin, uint8_t edge, uint8_t priority) {
    __builtin_write_OSCCONL(OSCCON & 0xBF);
    RPINR1bits.INT2R = pin;
    __builtin_write_OSCCONL(OSCCON | 0x40);
    IPC7bits.INT2IP = priority;
    INTCON2bits.INT2EP = edge;
    IEC1bits.INT2IE = 1;
#ifdef debug
    printf("INT2 enable at pin %d detect at edge = %d and set priority = %d \n", pin, edge, priority);
#endif
}

void init_qei() {
    __builtin_write_OSCCONL(OSCCON & 0xBF);
    _QEA1R = 7; //remap RP7 connect to QEI1_A
    _QEB1R = 8; //remap RP8 connect to QEI1_B
    _QEA2R = 10; //remap RP9 connect to QEI2_A
    _QEB2R = 9; //remap RP10 connect to QEI2_B
    __builtin_write_OSCCONL(OSCCON | 0x40);
    QEI1CONbits.QEIM = 0b000; // QEI Mode disable
    QEI1CONbits.PCDOUT = 0; // no direction pin out
    QEI1CONbits.QEIM = 0b101; // 2x ,no index
    QEI2CONbits.QEIM = 0b000; // QEI Mode disable
    QEI2CONbits.PCDOUT = 0; // no direction pin out
    QEI2CONbits.QEIM = 0b101; // 2x ,no index
}

// UART Configuration

void init_uart1(void) {
    U1MODEbits.STSEL = 0; // 1-stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0; // Autobaud Disabled

    U1BRG = BRGVAL; // BAUD Rate Setting for 115200


    //********************************************************************************
    //  STEP 1:
    //  Configure UART for DMA transfers
    //********************************************************************************/
    U1STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.URXISEL = 0; // Interrupt after one RX character is received


    //********************************************************************************
    //  STEP 2:
    //  Enable UART Rx and Tx
    //********************************************************************************/
    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART Tx


    IEC4bits.U1EIE = 0;
}

void init_uart2(void) {
    U2MODEbits.STSEL = 0; // 1-stop bit
    U2MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U2MODEbits.ABAUD = 0; // Autobaud Disabled

    U2BRG = BRGVAL; // BAUD Rate Setting for 115200


    //********************************************************************************
    //  STEP 1:
    //  Configure UART for DMA transfers
    //********************************************************************************/
    U2STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U2STAbits.UTXISEL1 = 0;
    U2STAbits.URXISEL = 0; // Interrupt after one RX character is received


    //********************************************************************************
    //  STEP 2:
    //  Enable UART Rx and Tx
    //********************************************************************************/
    U2MODEbits.UARTEN = 1; // Enable UART
    U2STAbits.UTXEN = 1; // Enable UART Tx
}
// DMA0 configuration

void cfgDma0UartTx(void) {
    //********************************************************************************
    //  STEP 3:
    //  Associate DMA Channel 0 with UART Tx
    //********************************************************************************/
    DMA0REQ = 0x000C; // Select UART1 Transmitter
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
    DMA0CONbits.MODE = 1;
    DMA0CONbits.DIR = 1;
    DMA0CONbits.SIZE = 1;
    DMA0CNT = 1; // 1 DMA requests

    //********************************************************************************
    //  STEP 6:
    // Associate one buffer with Channel 0 for one-shot operation
    //********************************************************************************/
    DMA0STA = __builtin_dmaoffset(Buffer_out);

    //********************************************************************************
    //  STEP 8:
    //	Enable DMA Interrupts
    //********************************************************************************/
    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt

}
// DMA1 configuration

void cfgDma1UartRx(void) {
    //********************************************************************************
    //  STEP 3:
    //  Associate DMA Channel 1 with UART Rx
    //********************************************************************************/
    DMA1REQ = 0x000B; // Select UART1 Receiver
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
    DMA1CONbits.MODE = 0;
    DMA1CONbits.DIR = 0;
    DMA1CONbits.SIZE = 1;
    DMA1CNT = 10; // 6 DMA requests

    //********************************************************************************
    //  STEP 6:
    //  Associate two buffers with Channel 1 for ‘Ping-Pong’ operation
    //********************************************************************************/
    DMA1STA = __builtin_dmaoffset(Buffer_in);

    //********************************************************************************
    //  STEP 8:
    //	Enable DMA Interrupts
    //********************************************************************************/
    IFS0bits.DMA1IF = 0; // Clear DMA interrupt
    IEC0bits.DMA1IE = 1; // Enable DMA interrupt

    //********************************************************************************
    //  STEP 9:
    //  Enable DMA Channel 1 to receive UART data
    //********************************************************************************/
    DMA1CONbits.CHEN = 1; // Enable DMA Channel
}

void cfgDma2UartTx2(void) {
    DMA2REQ = 0x001F; // Select UART2 Transmitter
    DMA2PAD = (volatile unsigned int) &U2TXREG;
    DMA2CONbits.AMODE = 0;
    DMA2CONbits.MODE = 1;
    DMA2CONbits.DIR = 1;
    DMA2CONbits.SIZE = 1;
    DMA2CNT = 5; // 1 DMA requests
    DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
    IFS1bits.DMA2IF = 0; // Clear DMA Interrupt Flag
    IEC1bits.DMA2IE = 1; // Enable DMA interrupt

}

void cfgDma3UartRx2(void) {
    DMA3REQ = 0x1E; // Select UART1 Receiver
    DMA3PAD = (volatile unsigned int) &U2RXREG;
    //DMA1CON = 0x0002;					// Continuous,  Post-Inc, Periph-RAM
    DMA3CONbits.AMODE = 0;
    DMA3CONbits.MODE = 0;
    DMA3CONbits.DIR = 0;
    DMA3CONbits.SIZE = 1;
    DMA3CNT = 10; // 1 DMA requests
    DMA3STA = __builtin_dmaoffset(Buffer_nano_in);
    IFS2bits.DMA3IF = 0; // Clear DMA interrupt
    IEC2bits.DMA3IE = 1; // Enable DMA interrupt
    DMA3CONbits.CHEN = 1; // Enable DMA Channel
}

void init_all() {
    initPLL();
    setpwm(8, 500);
    setperiod_position();
    init_ex_int1(lim_sw1, 1, 7);
    init_ex_int2(lim_sw2, 1, 7);
    init_qei();
    init_GPIO();
    T1CONbits.TON = 0;
}
//interrupt function

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void) {
    IFS0bits.INT0IF = 0;
} //interrupt 0 function

void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void) {
    home_y_state ^= 1;
    IFS1bits.INT1IF = 0;
} //interrupt 1 function

void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void) {
    home_x_state ^= 1;
    IFS1bits.INT2IF = 0;
} //interrupt 2 function

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {

    //traj.
    if (state == 5) {
        rt = c0 + c1 * t + c2 * t * t + c3 * t * t*t;
        setpoint_x = x0 + rt * cos(theta);
        setpoint_y = y0 + rt * sin(theta);
        setpoint_x *= k;
        setpoint_y *= k;
        t += 0.005;
    }
    //position control 
    static long int prev_error_x = 0, i_term_x = 0, prev_error_y = 0, i_term_y = 0;
    long int pwm_x = 0, pwm_y = 0;
    long int d_term_x, d_term_y;
    long int now_error_x = setpoint_x - POS1CNT;
    long int now_error_y = setpoint_y - POS2CNT;
    //pid x
    i_term_x += now_error_x;
    d_term_x = now_error_x - prev_error_x;
    prev_error_x = now_error_x;
    pwm_x = kp_x * now_error_x + ki_x * i_term_x + kd_x * d_term_x;
    //pid y
    i_term_y += now_error_y;
    d_term_y = now_error_y - prev_error_y;
    prev_error_x = now_error_x;
    pwm_y = kp_y * now_error_y + ki_y * i_term_y + kd_y * d_term_y;
    //printf("  %u %ld  \n",(unsigned int)setpoint_x ,pwm_x);
    if (pwm_x >= PR2) {
        if (now_error_x > 0) {
            _LATA0 = 1; //A=0
            _LATA1 = 0; //B=1
            OC1RS = PR2;
        } else if (now_error_x >= -500 && now_error_x <= 500) {
            _LATA0 = 1; //A=0
            _LATA1 = 1; //B=1
            OC1RS = 0;
            } else {
                _LATA0 = 0; //A=0
                _LATA1 = 1; //B=1
                OC1RS = PR2;
            }
        } else {
            if (now_error_x > 0) {
                _LATA0 = 1; //A=0
                _LATA1 = 0; //B=1
                OC1RS = pwm_x;
            } else if (now_error_x >= -500 && now_error_x <= 500) {
                _LATA0 = 1; //A=0
                _LATA1 = 1; //B=1
                OC1RS = 0;
            } else {
                _LATA0 = 0; //A=0
                _LATA1 = 1; //B=1
                OC1RS = pwm_x;
            }
        }
        if (pwm_y >= PR2) {
            if (now_error_y > 0) {
                _LATB13 = 1; //A=0
                _LATB15 = 0; //B=1
                OC2RS = PR2;
            } else if (now_error_y >= -500 && now_error_y <= 500) {
                _LATB13 = 1; //A=0
                _LATB15 = 1; //B=1
                OC2RS = 0;
            } else {
                _LATB13 = 0; //A=0
                _LATB15 = 1; //B=1
                OC2RS = PR2;
            }
        } else {
            if (now_error_y > 0) {
                _LATB13 = 1; //A=0
                _LATB15 = 0; //B=1
                OC2RS = pwm_y;
            } else if (now_error_y >= -500 && now_error_y <= 500) {
                _LATB13 = 1; //A=0
                _LATB15 = 1; //B=1
                OC2RS = 0;
            } else {
                _LATB13 = 0; //A=0
                _LATB15 = 1; //B=1
                OC2RS = pwm_y;
            }
        }
        //    sprintf(Buffer_out, "%f ", t);
        //    print_uart1();
        _T1IF = 0;
    } //timer 1 :position control PID

    void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void) {
        IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag;
    }

    void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void) {

        //sent data to arduino
        memcpy(&Buffer_nano_out, &Buffer_in, sizeof (Buffer_in));
        DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
        DMA2CONbits.CHEN = 1; // Re-enable DMA2 Channel
        DMA2REQbits.FORCE = 1;
        delay(100);
        memcpy(&Buffer_nano_out, &Buffer_in, sizeof (Buffer_in));
        DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
        DMA2CONbits.CHEN = 1; // Re-enable DMA2 Channel
        DMA2REQbits.FORCE = 1;

        static long int xf, yf, delta_x, delta_y;

        //state
        if (Buffer_in[0] == 0xFF) {
            if (Buffer_in[1] == 0xF0) {
                state = 0;
            } else if (Buffer_in[1] == 0xF1) {
                //home
                state = 1;
            } else if (Buffer_in[1] == 0xF2) {
                //capture
                state = 2;
            } else if (Buffer_in[1] == 0xF3) {
                //catch 
                state = 3;
            } else if (Buffer_in[1] == 0xF4) {
                //move to
                posx = (Buffer_in[2] << 8) + Buffer_in[3];
                posy = (Buffer_in[4] << 8) + Buffer_in[5];
                state = 4;
            } else if (Buffer_in[1] == 0xF5) {
                T1CONbits.TON = 0;
                xf = (Buffer_in[2] << 8) + Buffer_in[3];
                yf = (Buffer_in[4] << 8) + Buffer_in[5];
                delta_x = xf - x0;
                delta_y = yf - y0;
                rf = sqrt((delta_x * delta_x) + (delta_y * delta_y));
                theta = atan2(delta_y, delta_x);
                T = rf / 620.0;
                c0 = 0;
                c1 = 0;
                c2 = (3.0 / (T * T)) * rf;
                c3 = (-2.0 / (T * T * T)) * rf;
                t = 0;
                sprintf(Buffer_out, "%f %f  %ld  %ld", rf, T, x0, y0);
                print_uart1();
                delay(1500);
                T1CONbits.TON = 1;
                state = 5;
            }  else if (Buffer_in[1] == 0xAA) {
                state_ack_com = Buffer_in[2];
                state_ack_end = Buffer_in[3];
            } else if (Buffer_in[1] == 0xFF) {
                state = 255;
            }
        }
        IFS0bits.DMA1IF = 0; // Clear the DMA1 Interrupt Flag
    }

    void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void) {
        IFS1bits.DMA2IF = 0; // Clear the DMA0 Interrupt Flag;
    }

    void __attribute__((interrupt, no_auto_psv)) _DMA3Interrupt(void) {
        if (Buffer_nano_in[0] == 0xFF) {
            if (Buffer_nano_in[1] == 0xAA) {
                state_ack_nano = Buffer_nano_in[2];
            } else if (Buffer_nano_in[1] == 0x99) {
                //coming soon !
                //nop();
            }
        }
        IFS2bits.DMA3IF = 0; // Clear the DMA0 Interrupt Flag;
    }


    //function
    //////// movement function /////////

    void motorX_drive(char motor_state, char duty_cycle) {
        if (motor_state == -1) {
            _LATA0 = 0; //A=0
            _LATA1 = 1; //B=1
            OC1RS = (PR2 / 100) * duty_cycle;
        } else if (motor_state == 0) { //A=0 B=0
            _LATA0 = 1; //A=0
            _LATA1 = 1; //B=0
            OC1RS = 0;
        } else if (motor_state == 1) { //A=1 B=0
            _LATA0 = 1; //A=1
            _LATA1 = 0; //B=0
            OC1RS = (PR2 / 100) * duty_cycle;
        }
    }

    void motorY_drive(char motor_state, char duty_cycle) {
        if (motor_state == -1) { //A=0 B=1
            _LATB13 = 0; //A=0
            _LATB15 = 1; //B=1
            OC2RS = (PR2 / 100) * duty_cycle;
        } else if (motor_state == 0) { //A=0 B=0
            _LATB13 = 1; //A=0
            _LATB15 = 1; //B=0
            OC2RS = 0;
        } else if (motor_state == 1) { //A=1 B=0
            _LATB13 = 1; //A=1
            _LATB15 = 0; //B=0
            OC2RS = (PR2 / 100) * duty_cycle;
        }
    }

    //////// communication function /////////

    void print_uart1() {
        DMA0STA = __builtin_dmaoffset(Buffer_out);
        DMA0CNT = strlen(Buffer_out) - 1;
        DMA0CONbits.CHEN = 1; // Re-enable DMA0 Channel
        DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer
    }

    void print_uart2() {
        DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
        DMA2CNT = strlen(Buffer_nano_out) - 1;
        DMA2CONbits.CHEN = 1; // Re-enable DMA2 Channel
        DMA2REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer
    }

    void sent_ack_nano(unsigned char ack) {
        U2TXREG = ack;
    }

    void sent_ack_com(unsigned char ack) {
        U1TXREG = ack;
    }

    void wait_ack_nano(unsigned char ack) {
        while (state_ack_nano != ack) {
        }
    }

    void wait_ack_com(unsigned char ack) {
        while (state_ack_com != ack) {
        }
    }


    /////// command function /////////

    void comebackhome() {
        char set_x = 0, set_y = 0;
        //disable timer
        T1CONbits.TON = 0;
        //if x is already set
        if (home_x_state == 1 && set_x == 0) {
            motorX_drive(1, 15);
            delay(200);
            motorX_drive(0, 0);
            //printf("encoder x pos : %u\n",POS1CNT);
            set_x = 1;
        }
        //if y is already set
        if (home_y_state == 1 && set_y == 0) {
            motorY_drive(1, 20);
            delay(200);
            motorY_drive(0, 0);
            //printf("encoder y pos : %u\n",POS2CNT);
            set_y = 1;
        }
        //when x is not set and we want to set home X
        while (home_x_state == 0 || set_x == 0) {
            motorX_drive(-1, 10);
            while (!(home_x_state == 0) && set_x == 0) {
                motorX_drive(0, 0);
                //U1TXREG = POS1CNT;
                //printf("encoder 1 pos : %u\n", POS1CNT);
                set_x = 1;
            }
        }
        //when y is not set and we want to set home Y
        while (home_y_state == 0 || set_y == 0) {
            motorY_drive(-1, 15);
            if (home_y_state == 1 && set_y == 0) {
                motorY_drive(0, 0);
                //U1TXREG = POS2CNT;
                //printf("encoder 2 pos : %u\n", POS2CNT);
                set_y = 1;
            }
        }
        if (set_x == 1 && set_y == 1) {
            motorX_drive(1, 10);
            motorY_drive(1, 15);
            delay(100);
            motorX_drive(0, 0);
            motorY_drive(0, 0);
            delay(1000);
            POS1CNT = 0;
            POS2CNT = 0;
            delay(100);
        }
        //on timer
        T1CONbits.TON = 1;
    }

    //trajectory function///////
    ////// Other function ////////

    void delay(int time) {
        int i = 0, j = 0;
        for (i = 0; i <= time; i++) {
            for (j = 0; j <= 4000; j++) {
            }
        }
    }

    int main(void) {
        __builtin_disable_interrupts();
        init_all();
        __builtin_write_OSCCONL(OSCCON & 0xBF); // to clear IOLOCK
        RPOR2bits.RP5R = 3;
        _U1RXR = 6;
        RPOR2bits.RP4R = 5;
        _U2RXR = 11;
        __builtin_write_OSCCONL(OSCCON | 0x40); // to set IOLOCK

        __builtin_enable_interrupts();

        cfgDma0UartTx();
        cfgDma2UartTx2();
        cfgDma1UartRx();
        cfgDma3UartRx2();
        init_uart1();
        init_uart2();

        //main program
        while (1) {
            if (state == 0) {
            } else if (state == 1) {
                sprintf(Buffer_out, "state = 1 ");
                print_uart1();
                wait_ack_nano(0xB1);
                comebackhome();
                state_ack_nano = 0;
                state = 0;
            } else if (state == 2) {
                sprintf(Buffer_out, "state = 2 ");
                print_uart1();
                //capture();
                state = 0;
            } else if (state == 3) {
                sprintf(Buffer_out, "state = 3 ");
                print_uart1();
                //catch();
                state = 0;
            } else if (state == 4) {
                sprintf(Buffer_out, "state = 4 ");
                print_uart1();
                setpoint_x = (long int) (posx * k);
                setpoint_y = (long int) (posy * k);
                state = 0;
            } else if (state == 5) {
                if ( t >=  T) {
                    x0 = (POS1CNT) / k;
                    y0 = (POS2CNT) / k;
                    sent_ack_com(0xA6);
                    state = 0;
                }
            } else if (state == 255) {
                sprintf(Buffer_out, "state extra ");
                print_uart1();
                delay(1000);
                sent_ack_nano(0xCC);
                sprintf(Buffer_out, "complete");
                print_uart1();
                state_ack_nano = 0;
                state = 0;
            }
            old_state = state;
        }
        return 0;
    }
