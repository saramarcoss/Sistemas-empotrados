// DSPIC33FJ32MC204 Configuration Bit Settings

// 'C' source line config statements

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Mode (Internal Fast RC (FRC) with divide by N)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = ON              // Watchdog Timer Enable (Watchdog timer always enabled)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.



#include <xc.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>          // Defines NULL
#include <stdbool.h>         // Defines true
#include <math.h>


#define baud_4800    1041


const char cmd1[] = {"lon"}; 
const char cmd2[] = {"loff"}; 
const char cmd3[] = {"play"};   
const char cmd4[] = {"pause"};   

char dataCMD[50]; 
char txbuffer[200];
unsigned int contador = 0;
unsigned char var_recv = 0;

unsigned int nextchar = 0;
unsigned char BufferLoadDone = 1;

unsigned int data_count = 0;
unsigned char comando_detectado = 0;
      
void delay_ms(unsigned long time_ms)
{
    unsigned long u;
    for(u = 0; u < time_ms*90; u++) 
    {
        asm("NOP");
    }
}


void uart_config(unsigned int baud)
{
    //Interface uart (tx/rx)
    TRISCbits.TRISC1  = 1;
    TRISCbits.TRISC0  = 0;
    RPOR1bits.RP2R    = 3;   //U1TX conetada al pin RP2 (RB2)
    RPINR18bits.U1RXR = 3;   //U1RX conectada al pin RP3 (RB3)
    
    //mode
    U1MODEbits.UARTEN = 0;
    U1MODEbits.USIDL  = 0;
    U1MODEbits.IREN   = 0;
    U1MODEbits.RTSMD  = 1;
    U1MODEbits.UEN    = 0;
    U1MODEbits.WAKE   = 0;
    U1MODEbits.LPBACK = 0;
    U1MODEbits.ABAUD  = 0;
    U1MODEbits.URXINV = 0;
    U1MODEbits.BRGH   = 1;
    U1MODEbits.PDSEL  = 0;
    U1MODEbits.STSEL  = 0;
    //status
    U1STAbits.UTXISEL0 = 0;  // AUN NO TOCA. VER TEMA INTERRUPCIONES
    U1STAbits.UTXISEL1 = 0;  // AUN NO TOCA. VER TEMA INTERRUPCIONES
    U1STAbits.UTXINV   = 0;
    U1STAbits.UTXBRK   = 0;
    U1STAbits.UTXEN    = 1;
    U1STAbits.URXISEL  = 0;  // AUN NO TOCA. VER TEMA INTERRUPCIONES
    U1STAbits.ADDEN    = 0;
    U1STAbits.OERR     = 0;
    //baudios
    U1BRG = baud;
    
    U1MODEbits.UARTEN = 1;  // activa mi uart despues de ser configurada
}

void uart_send_byte (char c)
{
    while(U1STAbits.UTXBF); 
    U1TXREG = c; 
}


void uart_send_text(char *s)
{
    while(*s != '\0') uart_send_byte (*s++);   
}


bool compareChar(char data[], char cmd[]) 
{
    int i = 0;
    while (data[i] == cmd[i] && data[i] != '\0')
        i++;
    if (data[i] == cmd[i]) return true;
    return false;
}


int main (void)
{
    // Fcpu = Fosc/2 = 20mhz
    // Fosc = 40mhz
    // Fin  = 4mhz
    // Fosc = Fin *(M/N1*N2)
    PLLFBD = 38;             //M  = PLLFBD + 2
    CLKDIVbits.PLLPOST = 0;  //N1 = PLLPOST + 2
    CLKDIVbits.PLLPRE  = 0;  //N1 = PLLPOST + 2
    while(OSCCONbits.LOCK != 1);
    
    AD1PCFGL = 0xFFFF;   // Todos los pines configurados como pines digitales
    
    uart_config(baud_4800);
    
    while(1)
     {
        if (U1STAbits.URXDA){
            if (compareChar(((const char*) dataCMD), cmd1)) sprintf(txbuffer, "5001AA\r");
            if (!strcmp(((const char*) dataCMD), cmd2)) sprintf(txbuffer, "5000AA\r");
            if (!strcmp(((const char*) dataCMD), cmd3)) sprintf(txbuffer, "5101AA\r");
            if (!strcmp(((const char*) dataCMD), cmd4)) sprintf(txbuffer, "5100AA\r");

            nextchar = 0;
            BufferLoadDone = 0;


            memset(dataCMD, '\0', sizeof (dataCMD)); 
            data_count = 0;
            comando_detectado = 0;
        }
        delay_ms(200);
    }

    return 0;
}




