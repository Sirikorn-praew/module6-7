

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
#define kd_x 2
#define kp_y 2
#define ki_y 0
#define kd_y 2
#define k 151
double setpoint_x = (long int) 0;
double setpoint_y = (long int) 0;
long int pwm_x = 0, pwm_y = 0;
unsigned char state_setpoint = 0;
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

void setpwm() {
    T2CONbits.TCKPS = 0b01;
    PR2 = 10000;
    OC1RS = 0;
    OC2RS = 0;
    OC1CONbits.OCM = 0b000; //Disable Output Compare Module
    OC1CONbits.OCTSEL = 0; //OC1 use timer2 as counter source
    OC1CONbits.OCM = 0b110; //set to pwm without fault pin mode
    OC2CONbits.OCM = 0b000; //Disable Output Compare Module
    OC2CONbits.OCTSEL = 0; //OC2 use timer2 as counter source
    OC2CONbits.OCM = 0b110; //set to pwm without fault pin mode
    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK
    _RP12R = 0b10010; //remap RP12 connect to OC1
    _RP14R = 0b10011; //remap RP14 connect to OC2
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK
}

void print_timer() {
    T4CONbits.TCKPS = 0b11; //pre-scale 1:8
    PR4 = 3125; //period 25000 tick per cycle
    _T4IE = 1; //enable interrupt for timer1
    _T4IP = 1; //set interrupt priority to  1
} //control loop 50 hz

void setperiod_position() {
    T1CONbits.TCKPS = 0b01; //pre-scale 1:8
    PR1 = 10000; //period 10000 tick per cycle
    _T1IE = 1; //enable interrupt for timer1
    _T1IP = 5; //set interrupt priority to  4
} //control loop 500 hz

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
    U1STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.URXISEL = 0; // Interrupt after one RX character is received
    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART Tx
    IEC4bits.U1EIE = 0;
}

void init_uart2(void) {
    U2MODEbits.STSEL = 0; // 1-stop bit
    U2MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U2MODEbits.ABAUD = 0; // Autobaud Disabled
    U2BRG = BRGVAL; // BAUD Rate Setting for 115200
    U2STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U2STAbits.UTXISEL1 = 0;
    U2STAbits.URXISEL = 0; // Interrupt after one RX character is received
    U2MODEbits.UARTEN = 1; // Enable UART
    U2STAbits.UTXEN = 1; // Enable UART Tx
}
// DMA0 configuration

void cfgDma0UartTx(void) {
    DMA0REQ = 0x000C; // Select UART1 Transmitter
    DMA0PAD = (volatile unsigned int) &U1TXREG;
    DMA0CONbits.AMODE = 0;
    DMA0CONbits.MODE = 1;
    DMA0CONbits.DIR = 1;
    DMA0CONbits.SIZE = 1;
    DMA0CNT = 10; // 1 DMA requests
    DMA0STA = __builtin_dmaoffset(Buffer_out);
    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt

}
// DMA1 configuration

void cfgDma1UartRx(void) {
    DMA1REQ = 0x000B; // Select UART1 Receiver
    DMA1PAD = (volatile unsigned int) &U1RXREG;
    //DMA1CON = 0x0002;					// Continuous,  Post-Inc, Periph-RAM
    DMA1CONbits.AMODE = 0;
    DMA1CONbits.MODE = 0;
    DMA1CONbits.DIR = 0;
    DMA1CONbits.SIZE = 1;
    DMA1CNT = 10;
    DMA1STA = __builtin_dmaoffset(Buffer_in);
    IFS0bits.DMA1IF = 0; // Clear DMA interrupt
    IEC0bits.DMA1IE = 1; // Enable DMA interrupt
    DMA1CONbits.CHEN = 1; // Enable DMA Channel
}

void cfgDma2UartTx2(void) {
    DMA2REQ = 0x001F; // Select UART2 Transmitter
    DMA2PAD = (volatile unsigned int) &U2TXREG;
    DMA2CONbits.AMODE = 0;
    DMA2CONbits.MODE = 1;
    DMA2CONbits.DIR = 1;
    DMA2CONbits.SIZE = 1;
    DMA2CNT = 10; // 1 DMA requests
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
    setpwm();
    setperiod_position();
    print_timer();
    init_ex_int1(lim_sw1, 1, 7);
    init_ex_int2(lim_sw2, 1, 7);
    init_qei();
    init_GPIO();
    //T1CONbits.TON = 0;

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

void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void) {
    //sprintf(Buffer_out, "  %u %u %u \n", (unsigned int) setpoint_x, POS1CNT, POS2CNT);
    sprintf(Buffer_out, "%f\n", t);
    print_uart1();
    _T4IF = 0;
} //print timer

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {

    //traj.
    if (state == 5) {
        rt = c0 + (c1 * t)+ (c2 * t * t) + (c3 * t * t * t);
        setpoint_x = x0 + (rt * cos(theta));
        setpoint_y = y0 + (rt * sin(theta));
        setpoint_x *= k;
        setpoint_y *= k;
        t += 0.002;
    }
    //position control 
    static long int prev_error_x = 0, i_term_x = 0, prev_error_y = 0, i_term_y = 0;

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
    if (pwm_x <= 2000) {
        pwm_x = 2500;
    }
    if (pwm_y <= 2000) {
        pwm_y = 2500;
    }
    if (pwm_x >= PR2) {
        if (now_error_x > 0) {
            _LATA0 = 1; //A=0
            _LATA1 = 0; //B=1
            OC1RS = 8000;
        } else if (now_error_x >= -450 && now_error_x <= 450) {
            _LATA0 = 1; //A=0
            _LATA1 = 1; //B=1
            OC1RS = 0;
            state_setpoint = 1;
        } else {
            _LATA0 = 0; //A=0
            _LATA1 = 1; //B=1
            OC1RS = 8000;
        }
    }
    else {

        if (now_error_x > 0) {
            _LATA0 = 1; //A=0
            _LATA1 = 0; //B=1
            OC1RS = pwm_x;
        } else if (now_error_x >= -450 && now_error_x <= 450) {
            _LATA0 = 1; //A=0
            _LATA1 = 1; //B=1
            OC1RS = 0;
            state_setpoint = 1;
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
            OC2RS = 8000;
        } else if (now_error_y >= -450 && now_error_y <= 450) {
            _LATB13 = 1; //A=0
            _LATB15 = 1; //B=1
            OC2RS = 0;
            state_setpoint = 1;
        } else {
            _LATB13 = 0; //A=0
            _LATB15 = 1; //B=1
            OC2RS = 8000;
        }
    } else {
        if (now_error_y > 0) {
            _LATB13 = 1; //A=0
            _LATB15 = 0; //B=1
            OC2RS = pwm_y;
        } else if (now_error_y >= -450 && now_error_y <= 450) {
            _LATB13 = 1; //A=0
            _LATB15 = 1; //B=1
            OC2RS = 0;
            state_setpoint = 1;
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

    static long int xf, yf, delta_x, delta_y;
    //state
    if (Buffer_in[0] == 0xFF) {
        if (Buffer_in[1] == 0xF0) {
            memcpy(Buffer_nano_out, Buffer_in, sizeof (Buffer_in));
            DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
            DMA2CNT = 10;
            DMA2CONbits.CHEN = 1; // Re-enable DMA2 Channel
            DMA2REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer
            delay(100);
            DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
            DMA2CNT = 10;
            DMA2CONbits.CHEN = 1; // Re-enable DMA2 Channel
            DMA2REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer
            state = 0;
        } else if (Buffer_in[1] == 0xF1) {
            memcpy(Buffer_nano_out, Buffer_in, sizeof (Buffer_in));
            DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
            DMA2CNT = 10;
            DMA2CONbits.CHEN = 1; // Re-enable DMA2 Channel
            DMA2REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer
            delay(100);
            DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
            DMA2CNT = 10;
            DMA2CONbits.CHEN = 1; // Re-enable DMA2 Channel
            DMA2REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer

            state = 1;
        } else if (Buffer_in[1] == 0xF2) {
            memcpy(Buffer_nano_out, Buffer_in, sizeof (Buffer_in));
            DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
            DMA2CNT = 10;
            DMA2CONbits.CHEN = 1; // Re-enable DMA2 Channel
            DMA2REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer
            delay(100);
            DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
            DMA2CNT = 10;
            DMA2CONbits.CHEN = 1; // Re-enable DMA2 Channel
            DMA2REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer
            state = 2;
        } else if (Buffer_in[1] == 0xF3) {
            memcpy(Buffer_nano_out, Buffer_in, sizeof (Buffer_in));
            DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
            DMA2CNT = 10;
            DMA2CONbits.CHEN = 1; // Re-enable DMA2 Channel
            DMA2REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer
            delay(100);
            DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
            DMA2CNT = 10;
            posx = (Buffer_in[2] << 8) + Buffer_in[3];
            posy = (Buffer_in[4] << 8) + Buffer_in[5];
            state = 3;
        } else if (Buffer_in[1] == 0xF4) {
            memcpy(Buffer_nano_out, Buffer_in, sizeof (Buffer_in));
            DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
            DMA2CNT = 10;
            DMA2CONbits.CHEN = 1; // Re-enable DMA2 Channel
            DMA2REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer
            posx = (Buffer_in[2] << 8) + Buffer_in[3];
            posy = (Buffer_in[4] << 8) + Buffer_in[5];
            state = 4;
        } else if (Buffer_in[1] == 0xF5) {
            T1CONbits.TON = 0;
            //static unsigned char theta_l, theta_h;
            xf = (Buffer_in[2] << 8) + Buffer_in[3];
            yf = (Buffer_in[4] << 8) + Buffer_in[5];
            delta_x = xf - x0;
            delta_y = yf - y0;
            rf = sqrt((delta_x * delta_x) + (delta_y * delta_y));
            theta = atan2(delta_y, delta_x);
            T = 7;
            c0 = 0;
            c1 = 0;
            c2 = (3.0 / (T * T)) * rf;
            c3 = (-2.0 / (T * T * T)) * rf;
            t = 0;
            memcpy(Buffer_nano_out, Buffer_in, sizeof (Buffer_in));
            //theta_l = theta % 256;
            //theta_h = theta >> 8;
            //Buffer_nano_out[8] = theta_h;
            //Buffer_nano_out[9] = theta_l;
            DMA2STA = __builtin_dmaoffset(Buffer_nano_out);
            DMA2CNT = 11;
            DMA2CONbits.CHEN = 1; // Re-enable DMA2 Channel
            DMA2REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer

            state = 5;
        } else if (Buffer_in[1] == 0xAA) {
            state_ack_com = Buffer_in[2];
        } else if (Buffer_in[1] == 0xFF) {
            state = 255;
        } else if (Buffer_in[1] == 0xBB) {
            state = 99;
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
//// communication function /////////

void print_uart1() {
    DMA0STA = __builtin_dmaoffset(Buffer_out);
    //DMA0CNT = strlen(Buffer_out) - 1;
    DMA0CNT = 12;
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
    //T2CONbits.TON = 0;
    //if x is already set
    if (home_x_state == 1 && set_x == 0) {
        _LATA0 = 1; //A=1
        _LATA1 = 0; //B=0
        OC1RS = 2000;
        delay(100);
        _LATA0 = 1; //A=1
        _LATA1 = 1; //B=0
        OC1RS = 0;
        set_x = 1;
    }
    //if y is already set
    if (home_y_state == 1 && set_y == 0) {
        _LATB13 = 1; //A=1
        _LATB15 = 0; //B=0
        OC2RS = 2000;
        delay(100);
        _LATB13 = 1; //A=1
        _LATB15 = 1; //B=0
        OC2RS = 0;
        set_y = 1;
    }
    //when x is not set and we want to set home X
    while (home_x_state == 0 || set_x == 0) {
        _LATA0 = 0; //A=0
        _LATA1 = 1; //B=1
        OC1RS = 2000;
        while (home_x_state == 1 && set_x == 0) {
            _LATA0 = 1; //A=0
            _LATA1 = 1; //B=1
            OC1RS = 0;
            set_x = 1;
        }
    }
    //when y is not set and we want to set home Y
    while (home_y_state == 0 || set_y == 0) {
        _LATB13 = 0; //A=0
        _LATB15 = 1; //B=1
        OC2RS = 2000;
        if (home_y_state == 1 && set_y == 0) {
            _LATB13 = 1; //A=0
            _LATB15 = 1; //B=1
            OC2RS = 0;
            set_y = 1;
        }
    }
    if (set_x == 1 && set_y == 1) {
        _LATA0 = 1; //A=1
        _LATA1 = 0; //B=0
        OC1RS = 2000;
        delay(150);
        _LATA0 = 1; //A=1
        _LATA1 = 1; //B=0
        OC1RS = 0;
        delay(150);
        _LATB13 = 1; //A=1
        _LATB15 = 0; //B=0
        OC2RS = 2000;
        delay(150);
        _LATB13 = 1; //A=1
        _LATB15 = 1; //B=0
        OC2RS = 0;
        delay(1000);
        POS1CNT = 0;
        POS2CNT = 0;
        delay(100);
    }
    //on timer
    T1CONbits.TON = 1;
}

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
    T2CONbits.TON = 1;
    //main program
    while (1) {
        if (state == 0) {
        } else if (state == 1) {
            wait_ack_nano(0xB1);
            comebackhome();
            sent_ack_com(0xF1);
            state_ack_nano = 0;
            state = 0;
        } else if (state == 2) {
            //capture();
            unsigned char cap_state = 0;
            long int goy;
            wait_ack_nano(0xB2);
            delay(100);
            int y;
            for (y = 1; y <= 5; y++) {
                goy = y * 50;
                setpoint_x = 0;
                setpoint_y = (long int) (goy * k);
                while (state_setpoint == 0) {
                }
                state_setpoint = 0;
                cap_state++;
                //delay(300);
                sent_ack_com(cap_state);
                wait_ack_com(cap_state);
            }
            /*for (y = 1; y < 5; y++) {
                for (x = 1; x < 5; x++) {
                    gox = x * 50;
                    goy = y * 50;
                    setpoint_x = (long int) (gox * k);
                    setpoint_y = (long int) (goy * k);
                    while (state_setpoint == 0) {
                    }
                    state_setpoint = 0;
                    cap_state++;
                    //delay(300);
                    sent_ack_com(cap_state);
                    //sprintf(Buffer_out, "%.1f  %.1f %d %d %d", setpoint_x / k, setpoint_y / k, x, y, k);
                    //print_uart1();
                    wait_ack_com(cap_state);
                }
            }*/
            home_x_state = 0;
            home_y_state = 0;
            comebackhome();
            setpoint_x = 0;
            setpoint_y = 0;
            delay(100);
            sent_ack_com(0xF2);
            state = 0;
        } else if (state == 3) {
            wait_ack_nano(0xB3);
            unsigned int x = 1 * k;
            unsigned int y = 360 * k;
            setpoint_x = (long int) (x);
            setpoint_y = (long int) (y); // go target
            while (state_setpoint == 0) {
            }
            state_setpoint = 0;
            sent_ack_nano(0xB3);
            wait_ack_nano(0xB9); // gripper down open close and up to 300
            setpoint_x = (long int) (posx * k);
            setpoint_y = (long int) (posy * k); // go start point
            while (state_setpoint == 0) {
            }
            state_setpoint = 0;
            sent_ack_com(0xF3);
            state_ack_nano = 0;
            state = 0;
        } else if (state == 4) {
            state_setpoint = 0;
            setpoint_x = (long int) (posx * k);
            setpoint_y = (long int) (posy * k);
            if (state_setpoint == 1) {
                sent_ack_com(0xF4);
                state = 0;
            }
        } else if (state == 5) {
            //wait_ack_nano(0xAA);
            T1CONbits.TON = 1;
            //T4CONbits.TON = 1;
            if (t >= T) {
                x0 = (POS1CNT) / k;
                y0 = (POS2CNT) / k;
                sent_ack_com(0xF5);
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
        } else if (state == 99) {
            sent_ack_com(0xBB);
            state = 0;
        }
        old_state = state;
    }
    return 0;
}