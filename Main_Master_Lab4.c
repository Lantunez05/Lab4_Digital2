/* 
 * File:   Main_Master_Lab4.c
 * Author: Luis Antunez
 *
 * Created on 2 de agosto de 2023, 08:54 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdlib.h>
#include <stdio.h>
#include "I2C.h"

#define _XTAL_FREQ 8000000

void setup(void);

uint8_t bcd_to_decimal(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

void main(void)
{
    setup();
    int RTC;
    while(1)
    {
        uint8_t sec;
        I2C_Master_Start();
        I2C_Master_Write(0x51);
        PORTB = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(200);
        
        I2C_Master_Start();          // Iniciar el I2C
        I2C_Master_Write(0xD0);      // Escribir la direccion del RTC para escritura
        I2C_Master_Write(0x00);      // Seleccionar los segundos
        I2C_Master_RepeatedStart();  // Reiniciar el I2C
        I2C_Master_Write(0xD1);      // Escribir la direccion del RTC para lectura
        sec = I2C_Master_Read(0);
        I2C_Master_Stop();
        // Convertir el valor BCD a decimal
        uint8_t seconds_decimal = bcd_to_decimal(sec);
        PORTD = seconds_decimal;
        __delay_ms(200);
        
    }
    return;
}
void setup(void)
{
    ANSEL = 0;
    ANSELH = 0;
    TRISB = 0;
    TRISD = 0;
    PORTB = 0;
    PORTD = 0;
    I2C_Master_Init(100000);        // Inicializar Comuncación I2C
}

void RTC_ReadTime(unsigned char *hour, unsigned char *minute, unsigned char *second) {
    
    SSPCON2bits.SEN = 1;    //inicia la comunicación i2c
    
}