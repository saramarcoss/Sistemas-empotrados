#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Mode (Internal Fast RC (FRC) with divide by N)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = EC              // Primary Oscillator Source (Primary Oscillator Disabled)
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


void delay_ms(unsigned long time_ms);
void EnviarCaracter(char c);
void EnviarString(char *s);
void uart_config (unsigned int baud);
void timer1_config (void);
unsigned int contador = 0;

#define baud_9600    51 

char txbuffer[200];
volatile char byte_recibido = ' ';
volatile char toggle_led_timer1 = 0;   // Off state by default
bool BufferLoadDone =false;

void delay_ms(unsigned long time_ms)
{
    unsigned long u;
    for(u = 0; u < time_ms*90; u++) 
    {
        asm("NOP");
    }
}

void EnviarCaracter(char c)
{    
    while(U1STAbits.UTXBF);   // Mientras el buffer del puerto U1 este lleno, esperar en bucle while
    U1TXREG = c;              // Si no esta lleno, proceder a enviar el byte
}

void EnviarString(char *s)
{    
    while((*s) != '\0') EnviarCaracter(*(s++));  // Mientras no se haya llegado al caracter nulo (final de trama), continuar imprimiendo datos.
}                                                // *s es un puntero que apunta hacia la dirección del string de datos que le indiquemos. (*(s++) toma el contenido actual, y posteriormente aumenta el valor de la dirección (siguiente caracter))

void timer1_config (void)
{
    T1CONbits.TON   = 0; 
    T1CONbits.TSIDL = 0;
    T1CONbits.TCS   = 0;
    T1CONbits.TSYNC = 0;
    T1CONbits.TCKPS = 1; //prescaler 8    
    //interrupciones
    IFS0bits.T1IF = 0; //limpio la bandera de interrupcion
    IEC0bits.T1IE = 1; //habilito la interrupcion
    IPC0bits.T1IP = 7; //prioridad 7

    PR1 = 24999; // 50ms
    TMR1 = 0;
    T1CONbits.TON = 1;
}

void uart_config (unsigned int baud)
{    
    //Interface uart (tx/rx)
    TRISBbits.TRISB3  = 1;
    TRISBbits.TRISB2  = 0;
    RPOR1bits.RP2R    = 3;   //U1TX conetada al pin RP2 (RB2)
    RPINR18bits.U1RXR = 3;   //U1RX conectada al pin RP3 (RB3)
    
    // Configuración de registro de U1MODE
    U1MODEbits.UARTEN = 0;     // Deshabilitar Uart.
    U1MODEbits.USIDL  = 0;     // Continuar operación en modo IDLE
    U1MODEbits.IREN   = 0;     // IR no usado
    U1MODEbits.RTSMD  = 1;     // Control de flujo desactivado.
    U1MODEbits.UEN    = 0;     // Solo usamos pin de Tx y pin de Rx
    U1MODEbits.WAKE   = 0;     // No quiero que la UART despierte del modo sleep
    U1MODEbits.LPBACK = 0;     // Loopback deshabilitado.
    U1MODEbits.ABAUD  = 0;     // Automedición de baudios (bps) deshabilidada
    U1MODEbits.URXINV = 0;     // En estado de reposo, el receptor mantiene un estado alto, high
    U1MODEbits.BRGH   = 1;     // Modo High-Speed
    U1MODEbits.PDSEL  = 0;     // 8 Bits de datos y paridad Nula (8N)
    U1MODEbits.STSEL  = 0;     // 1-bit de stop al final de la trama de datos.   (8N1)

    // Configuración de registro de U1STA
    U1STAbits.UTXISEL0 = 0;    // Tema interrupciones (no mirar aun)
    U1STAbits.UTXISEL1 = 0;    // Tema interrupciones (no mirar aun)
    U1STAbits.UTXINV   = 0;    // El estado en reposo del pin de transmisión es High
    U1STAbits.UTXBRK   = 0;    // No usamos trama de sincronización
    U1STAbits.UTXEN    = 1;    // El transmisor a pleno funcionamiento.
    U1STAbits.URXISEL  = 0;    // Tema interrupciones. La interrupcion salta cada vez que haya al menos un caracter en el buffer de recepcion
    U1STAbits.ADDEN    = 0;    // No usamos direccionamiento.
    U1STAbits.OERR     = 0;    // Reseteamos buffer de recepción
    
    // Configuramos la velocidad de transmisión/recepcción de los datos
    U1BRG = baud;

    IPC2bits.U1RXIP = 6;
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;
      
    U1MODEbits.UARTEN = 1;     // Uart habilitada por completo
}

int main(void) 
{    
    //Configurar el oscilador para hacer funcionar la CPU a 4 MHz a partir de un reloj de entrada de 8MHz
    //Fosc = Fin * M/(N1 * N2), Fcy = Fosc/2
    //Fosc = 8M * 2/(2 * 2) = 8 MHz para un reloj de 8MHz de entrada
    //Fcy = Fosc/2 = 8/2 = 4MHz (Frecuencia CPU)
    PLLFBD = 0;                     // M  = 4
    CLKDIVbits.PLLPOST = 0;         // N1 = 2
    CLKDIVbits.PLLPRE  = 0;         // N2 = 2
    while(OSCCONbits.LOCK != 1);    // Esperar a un PLL estable   
    
    
    AD1PCFGL         = 0xFFFF;      // Primer paso. Todos los pines configurados como pines digitales
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    LATAbits.LATA0   = 0;
    LATAbits.LATA1   = 0;

 
    uart_config(baud_9600); 
    timer1_config();
    
    bool BufferLoadDone = false;

//apartado 1
void __attribute((interrupt_, no_auto_psv)) _U1TXInterrupt(void)
{
    IEC0bits.U1TXIE = 0;
   // IFS0bits.U1TXIF = 0; //limpio la bandera de interrupcion
    if(!U1STAbits.UTXBF){
       U1TXREG = txbuffer[contador++];
       asm("NOP");
       if(U1STAbits.UTXBF){
           IFS0bits.U1TXIF=0;
       }
    }else  IFS0bits.U1TXIF=0;
    
        if(contador == 65535){
         BufferLoadDone = true;   
        }else{
           IEC0bits.U1TXIE = 1; 
     }
}
    while(1)
    {   
        sprintf(txbuffer,"Valor contador: %d \r\n", contador);
        //IEC0bits.U1TXIE = 1;
        EnviarString(txbuffer);
        contador++;
        if(contador > 65535) contador = 0;
        delay_ms(100);     
    }
}

//apartado 2
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{ 
  if(toggle_led_timer1) LATAbits.LATA0 = !PORTAbits.RA0;
  else LATAbits.LATA0 = 0;
  
  IFS0bits.T1IF = 0;  
}

//apartado 3
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{ 
  LATAbits.LATA1 = !PORTAbits.RA1;
  if(U1RXREG == 'P') toggle_led_timer1 = !toggle_led_timer1;
    IFS0bits.U1RXIF = 0;  
  if(U1RXREG == ' '){
    IEC0bits.U1TXIE = 1; //habilito la interrupcion de transmision
    IFS0bits.U1TXIF = 0; //limpio la bandera de interrupcion
    sprintf(txbuffer,"contador: %d\r ",contador++);
    U1STAbits.TRMT = 1;
    //se desactiva la interrupcion de recepcion solo si se ha completado la transferencia:
    if(BufferLoadDone){
        IEC0bits.U1RXIE = 0; //deshabilito la interrupcion de recepcion
    }
}
}
