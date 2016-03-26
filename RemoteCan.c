#include <xc.h>
#include <pic_config.h>
#define LCD_DEFAULT
#define _XTAL_FREQ 16000000
#include "idCan.h"
#include "CANlib.h"
#include "delay.h"
#include "delay.c"
#include "LCD_44780.h" 
#include "LCD_44780.c"
#include <stdio.h>
#include <math.h>
#define HIGH 1
#define LOW 0
#define X_AXIS 0 ///Steering
#define Y_AXIS 1//Speed
#define HIGH_POS 0
#define MID_POS 1
#define LOW_POS 2
#define FWD 2
#define BKWD 1

//prototipi delle funzioni
void board_initialization(void);
void board_initialization(void);
void PWR_Button_Polling(void);
void Joystick_Polling(void);
void USART_Send(void);
void LCD_Handler(void);
void USART_RX(void);

//variabili per delay non blocking
unsigned long timeCounter = 0;
unsigned long prev

//Variabili per can bus
BYTE data[] = 0;
bit newMessageCan = 0;
bit RTR_Flag = 0;
long id = 0;
__interrupt(high_priority) void ISR_alta(void) {
    if ((PIR3bits.RXB1IF == 1) || (PIR3bits.RXB0IF == 1)) { //RICEZIONE CAN
        if (CANisRxReady()) {
            CANreceiveMessage(&msg); //leggilo e salvalo
            RTR_Flag = msg.RTR;
            id = msg.identifier;
            newMessageCan = 1;
            for (unsigned char i = 0; i < 8; i++) {
                data[i] = msg.data[i];
            }
        }
        PIR3bits.RXB1IF = 0;
        PIR3bits.RXB0IF = 0;
    }
}

__interrupt(low_priority) void ISR_bassa(void) {
    if (PIR2bits.TMR3IF == 1) { //interrupt timer, ogni 10mS
        timeCounter++; //incrementa di 1 la variabile timer
        TMR3H = 0x63; //reimposta il timer
        TMR3L = 0xC0; //reimposta il timer
        PIR2bits.TMR3IF = 0; //azzera flag interrupt timer
    }
}


void main(void) {
    board_initialization();
         while (1) {
             //[CHECK ECU]

        PWR_Button_Polling();

        if (power_switch == LOW) {
            dir = FWD;
            set_speed = 0;
            set_steering = 90;
            analogic_brake = 0;
            while (BusyUSART() == HIGH) {
            };
            USART_Send();
            while (power_switch == LOW) {
                LCD_clear();
                LCD_goto_line(1);
                LCD_write_message("====================");
                LCD_goto_line(2);
                LCD_write_message("==> VEHICLE  OFF <==");
                LCD_goto_line(3);
                LCD_write_message("Turn the switch ON! ");
                LCD_goto_line(4);
                LCD_write_message("====================");
                if ((time_counter - pr_time_1) >= 50) {
                    pr_time_1 = time_counter;
                    PORTDbits.RD7 = ~PORTDbits.RD7;
                }
                PWR_Button_Polling();
                delay_ms(300); //[!!]Verificare
            }
            PORTDbits.RD7 = LOW; //Turn off ON/OFF switch backlight
        }

        Joystick_Polling();

        //Gestione switch tre posizioni
        if (PORTAbits.RA2 == HIGH) {
            switch_position = HIGH_POS;
        } else {
            if (PORTAbits.RA3 == LOW) {
                switch_position = MID_POS;
                dir = FWD;
            } else {
                switch_position = LOW_POS;
                dir = BKWD;
            }
        }

        set_steering = (JoystickValues[X_AXIS])*(JoystickConstants[X_AXIS]);
        if (switch_position != HIGH_POS) {
            if (JoystickValues[Y_AXIS] > 132) {
                set_speed = (JoystickValues[Y_AXIS] - 130)*(JoystickConstants[Y_AXIS]); //guardare
                analogic_brake = 0;
            } else {
                set_speed = 0;
                analogic_brake = ((130 - JoystickValues[Y_AXIS]))*(1.9);
            }
        }

        if ((time_counter - pr_time_2) >= 2) {
            pr_time_2 = time_counter;
            USART_Send();
        }

        if ((time_counter - pr_time_3) >= 50) {
            pr_time_3 = time_counter;
            LCD_Handler();
        }
    }
}
    }
    }

void board_initialization(void) {
    //Inputs and Outputs Configuration
    LATA = 0x00;
    TRISA = 0b00001111; // X-Axis / Y-Axis / 3P Switch
    LATB = 0x00;
    TRISB = 0xFF; // ON/OFF Switch
    LATC = 0x00;
    TRISC = 0b10110000; //USART Tx and Rx / LCD
    LATD = 0x00;
    TRISD = 0x00; //LCD / Backlight ON/OFF
    LATE = 0x00;
    TRISE = 0x00;

    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE & CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON); //Canbus 125kHz (da cambiare)


    //Interrupt Flags
    PIR2bits.TMR3IF = LOW;
    PIR3bits.RXB1IF = 0; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = 0; //azzera flag interrupt can bus buffer0

    //Interrupts Priority
    RCONbits.IPEN = HIGH; //abilita priorità interrupt
    IPR3bits.RXB1IP = HIGH; //interrupt alta priorità per can
    IPR3bits.RXB0IP = HIGH; //interrupt alta priorità per can
    IPR2bits.TMR3IP = LOW; // interrupt bassa priorità TMR3

    //Configurazione ADC======================================================
    ADCON1 = 0b00001101; //RA0 and RA1 analogic, AVdd and AVss voltage reference (?)
    ADCON0bits.CHS2 = 0;
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS0 = 0;
    ADCON2bits.ACQT2 = 1;
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT0 = 0;
    ADCON2bits.ADCS2 = 1;
    ADCON2bits.ADCS1 = 0;
    ADCON2bits.ADCS0 = 1;
    ADCON2bits.ADFM = 0; //Left Justified
    ADCON0bits.ADON = HIGH;
    //========================================================================
    //LCD Initialize
    LCD_initialize(16);
    LCD_backlight(0);
    LCD_clear();
    LCD_goto_line(1);

    LCD_write_message("Wait...");
    delay_ms(300);

    PORTDbits.RD2 = 0;
    PORTDbits.RD3 = 0;

    //Configurations
    TMR3H = 0x63;
    TMR3L = 0xC0;

    //Interrupts Enables
    PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
    PIE2bits.TMR3IE = HIGH;


    RCSTAbits.SPEN = LOW; //USART disable
    T3CON = 0x01; //Timer Enable
    LCD_clear();
    INTCONbits.GIEH = HIGH;
    INTCONbits.GIEL = HIGH;
}