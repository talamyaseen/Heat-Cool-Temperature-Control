/*
 * File:   pwm_asl.c
 * Author: raed
 * PWM + ADC + SERIAL + LCD
 * Created on March 30, 2019, 1:05 PM
 * LCD is set to work on the simulator, must be fixed to work with real
 */


#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdio.h>
#include "my_ser.h"
#include "my_adc.h"
#include "my_pwm.h"
#include "lcd_x8.h"
//function prototypes
#define STARTVALUE  3036
typedef enum {
    OFF = 0,
    COOL =1,
    HEAT =2,
    AUTO_COOL=3
} mode_e;

typedef enum {
    HS_0 = 0,
    HS_1 =1,
    HS_2 =2,
    HS_3 =3,
    HS_4=4
} HC_e;
HC_e HS = 0;
float HC = 0.0;
int raw_val;
float AN[3];     // To store the voltages of AN0, AN1, AN2
mode_e mode =OFF;
float temp, sp ;

//float tempf;
//float spf;
char Buffer[32];
unsigned int RPS_count=0;
unsigned int RPS=0;
int ft=0;

void setupPorts(void) {
    INTCON2 =0 ;//enable pull up ports
    ADCON0 = 0;
    ADCON1 = 0b00001100; //3 analog channels, change this according to your application
    TRISB = 0xFF; // all pushbuttons are inputs
    TRISC = 0x80; // RX input , others output
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
}

void initInterrupts(void) {
       INTCON = 0;
    RCONbits.IPEN = 0;

    // Clear all interrupt flags
    INTCONbits.INT0IF = 0;
    INTCON3bits.INT2IF = 0;
    PIR2bits.TMR3IF = 0;

    // Enable interrupts
    INTCONbits.INT0IE = 1;  // Enable INT0 external interrupt
    INTCON3bits.INT2IE = 1;  // Enable INT2 external interrupt
    T3CON = 0;
    T3CONbits.TMR3ON = 1;  // turn the timer on , prescaler = 1
    INTCON2 = 0;
    INTCON2bits.INTEDG2 = 1;
    INTCON2bits.INTEDG0= 1;
    PIE1 = 0;
    PIR1 = 0;
    IPR1 = 0;
    PIE2 = 0;
    PIE2 = 0;
    PIR2 = 0;
    IPR2 = 0;
    PIE2bits.CCP2IE = 1; 
    INTCONbits.GIE = 1;  // Enable global interrupts

}
void __interrupt(high_priority) highIsr(void)//new syntax
{
      if (INTCONbits.INT0IF) {
             __delay_ms(250);
      if (INTCONbits.INT0IF) {  
          mode = (mode + 1) % 4;
        INTCONbits.INT0IF = 0;  // Clear the interrupt flag
      }
     
    }
    if (INTCON3bits.INT2IF) {
        __delay_ms(250);
           if (INTCON3bits.INT2IF) {
        HS = (HS + 1) % (HS_4 + 1);
        INTCON3bits.INT2IF = 0;  // Clear the interrupt flag
           }
    }
    if (PIR2bits.TMR3IF) {
       // handleHeating();
        PIR2bits.TMR3IF = 0;  // Clear the interrupt flag
    }

}

void operation (void){
    switch(mode) {
        case OFF:
            if(PORTBbits.RB3==0){
            delay_ms(250);
            if(PORTBbits.RB3==0)
           {
             set_pwm1_raw(0);  
            PORTCbits.RC2=0;
            HC=0;
            PIE2bits.CCP2IE = 0; 
            }
            }
            break;    
        case COOL:
         
            HC =(AN[1]/5.0)*100.0; 
            raw_val = read_adc_raw_no_lib(1); // read raw value for POT1 
            set_pwm1_raw(raw_val);  // set the Pwm to that value 0--1023
            break;
        
        case HEAT:
            HC =(AN[1]/5.0)*100.0; // HC value
            // Set PWM or compare value for heater based on H value
            break;
        
        case AUTO_COOL:
            // Calculate CoolError
            /*int CoolError = temp - sp;
            if (CoolError > 0) {
                C = (CoolError * 100) / 10; // Calculate PWM percentage for cooling
                // Ensure minimum PWM level
                if (C < 25) C = 25;
                // Set PWM for cooling
            } else if (temp < (sp - HS)) {
                C = 0; // Turn cooling off
                // Activate heater with 50% duty cycle or appropriate control as specified
            }*/
            break;
    }
    
}
void display(void) {
    char LCD[64];
    char LCDP[64];
    unsigned char C, H;
    switch(mode) {
        case OFF:
            sprintf(LCDP, "OFF                 ");
            H = 'N';
            C = 'N'; 
            break;
            
        case COOL:
            sprintf(LCDP, "Cool                ");
              H = 'N';
              C = 'Y'; 
            break;
        
        case HEAT:
            sprintf(LCDP, "Heat                ");
              H = 'Y';
             C = 'N'; 
            break;
        
        case AUTO_COOL:
            sprintf(LCDP, "Auto Cool         ");
            H = 'N';
            C = 'N'; 
            break;
    }
    
    // Display temperature and other relevant information
    lcd_gotoxy(1, 1);
    sprintf(LCD, "RT: %4.1fC", temp);
    lcd_puts(LCD);
    
    lcd_gotoxy(14, 1);
    lcd_puts("H C"); 
    
    lcd_gotoxy(1, 2);
    sprintf(LCD, "SP: %4.1fC ", sp);
    lcd_puts(LCD);
    
    lcd_gotoxy(14, 2);
    lcd_putc(H);
    lcd_gotoxy(16, 2);
    lcd_putc(C);
    
    lcd_gotoxy(1, 3);
    sprintf(LCD, "HS: %d", HS);
    lcd_puts(LCD);
    
    lcd_gotoxy(8, 3);
    sprintf(LCD, "HC: %4.1f%%", HC);
    lcd_puts(LCD);
    
    
    lcd_gotoxy(1, 4);
    sprintf(LCD, "MD:");
    lcd_puts(LCD);
    
    lcd_gotoxy(5, 4);
    lcd_puts(LCDP);
}


void main(void) {
    unsigned char channel;
    float voltage;
    setupPorts();
    initInterrupts();
    lcd_init();
    init_adc_no_lib();
    init_pwm1();
    lcd_putc('\f'); 
    mode = OFF;
    HS = HS_0;
    while (1) {
        CLRWDT(); // no need for this inside the delay below
        delay_ms(200); //read ADC AN0,AN1, AN2 every 2 seconds
        for (channel = 0; channel < 3; channel++) {
            voltage = read_adc_voltage((unsigned char) channel);
            AN[channel] = voltage; // store in array AN0--AN2
        }
        sp=AN[0]*20; // for scaling
        temp=AN[2]*100.0;
        operation();
        display();
    }
}
