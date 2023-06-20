// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Mode (Primary Oscillator (XT, HS, EC) w/ PLL)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Source (HS Oscillator Mode)
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
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

#include <xc.h>
#include "xc.h"


void delay_ms(unsigned long time_ms)
{
    unsigned long u = 0;
    for(u=0; u<time_ms*450;u++)     
    {
        asm("NOP");
    }
}

void main (void){
    //Fosc = 8 mhz
    //Fcpu = Fosc/2 = 4 mhz

    //Fosc = Fin*M/(N1*N2)      
    PLLFBD = 38;
    CLKDIVbits.PLLPOST = 0;
    CLKDIVbits.PLLPRE = 0;
    while(OSCCONbits.LOCK != 1);
    
    
    //int cont = 0;
    //Declarar pines como pines digitales
    AD1PCFGL = 0xFFFF;  //(Todos los puertos son pines digitales)
    
    //Configuración de direccionalidad
    TRISBbits.TRISB3 = 0;   //pin como salida digital
    TRISBbits.TRISB4 = 1;   //pin como entrada digital
    
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    
    TRISBbits.TRISB7 = 1;
         
    while(1)
    {   
        //Ejercicio 1
        //if(cont==2)
        //{
            LATBbits.LATB3 = !PORTBbits.RB3;
            delay_ms(500);
            delay_ms(500);           
        //}
        
        //Ejercicio 2
        if(PORTBbits.RB4 = 0)
        {
            LATAbits.LATA0 = 0;
            LATAbits.LATA1 = 0;
        }
        else 
        {
            LATAbits.LATA0 = 1;
            LATAbits.LATA1 = 1;
        }
        
        //Ejercicio 3
        if(PORTBbits.RB7 = 0){}
        else
        {
                LATAbits.LATA1 = 1;
                delay_ms(500);
                LATAbits.LATA1 = 0;
                delay_ms(500);
        }      
       //cont++;
    }
}