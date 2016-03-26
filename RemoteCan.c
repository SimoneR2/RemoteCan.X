#include <xc.h>
#include "pic_config.h"
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
#define FWD 0
#define BKWD 1

//prototipi delle funzioni
void board_initialization(void);
void board_initialization(void);
void PWR_Button_Polling(void);
void Joystick_Polling(void);
void CAN_Send(void);
void CAN_interpreter(void);
void LCD_Handler(void);

//variabili per delay non blocking
volatile unsigned long time_counter = 0;
volatile unsigned long pr_time_1 = 0;
volatile unsigned long pr_time_2 = 0;
volatile unsigned long pr_time_3 = 0;
volatile unsigned long pr_time_4 = 0;

//Variabili per can bus invio
BYTE data_steering[] = 0;
BYTE data_speed [] = 0;
BYTE data_brake [] = 0;
volatile bit power_switch = LOW;
CANmessage msg;

//variabili can bus ricezione
unsigned int left_speed = 0;
unsigned int right_speed = 0;
BYTE data[] = 0;
volatile bit newMessageCan = 0;
volatile bit RTR_Flag = 0;
volatile long id = 0;
volatile bit MotoreFlag = 0;
volatile bit AbsFlag = 0;
volatile bit SterzoFlag = 0;
volatile unsigned char battery = 0;

//variabili LCD
unsigned char str [12] = 0;
signed float actual_speed_kmh = 0;
signed float actual_speed = 0;

//variabili per il programma
volatile char dir = 0;
volatile unsigned char switch_position = 0;
volatile unsigned char set_steering = 0;
volatile unsigned int set_speed = 0;
volatile unsigned char JoystickValues[2] = 0; //steering - speed
volatile signed float JoystickConstants[2] = 0;
volatile bit wait_low = LOW;

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
        time_counter++; //incrementa di 1 la variabile timer
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
            data_steering [0] = 90;
            data_brake [0] = 0;
            data_brake [1] = 1;
            CAN_Send();
            while (power_switch == LOW) {
                LCD_clear();
                delay_ms(10);
                LCD_goto_line(1);
                LCD_write_message("====================");
                LCD_goto_line(2);
                LCD_write_message("==> VEHICLE  OFF <==");
                LCD_goto_line(3);
                LCD_write_message("Turn the switch ON! ");
                LCD_goto_line(4);
                LCD_write_message("====================");
                if ((time_counter - pr_time_1) >= 30) {
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

        data_steering [0] = (JoystickValues[X_AXIS])*(JoystickConstants[X_AXIS]);
        if (switch_position != HIGH_POS) {
            if (JoystickValues[Y_AXIS] > 132) {
                set_speed = (JoystickValues[Y_AXIS] - 130)*(JoystickConstants[Y_AXIS]); //guardare
                data_brake [0] = 0;
            } else {
                set_speed = 0;
                data_brake [0] = ((130 - JoystickValues[Y_AXIS]))*(1.9);
            }
        }

        if (newMessageCan == 1) {
            CAN_interpreter();
            newMessageCan = 0;
        }

        if ((time_counter - pr_time_2) >= 2) {
            pr_time_2 = time_counter;
            CAN_Send();
        }

        if ((time_counter - pr_time_3) >= 50) {
            pr_time_3 = time_counter;
            LCD_Handler();
        }
    }
}

void CAN_Send(void) {
    //while (CANisTxReady()!=1);
    CANsendMessage(STEERING_CHANGE, data_steering, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
    data_speed[0] = set_speed;
    data_speed[1] = (set_speed >> 8);
    data_speed[2] = dir;
//    while (CANisTxReady()!=1);
    CANsendMessage(SPEED_CHANGE, data_speed, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
//    while (CANisTxReady()!=1);
    CANsendMessage(BRAKE_SIGNAL, data_brake, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
LATDbits.LATD2 = 1;
    LATDbits.LATD3 = 1;
}

void LCD_Handler(void) {
    actual_speed_kmh = actual_speed / 278;

    LCD_clear();
    LCD_goto_line(1);
    LCD_write_message("=== VEHICLE DATA ===");

    LCD_goto_line(2);
    LCD_write_message("Direction: ");
    if (switch_position != HIGH_POS) {
        if (dir == FWD) {
            LCD_write_message("FWD");
        } else {
            LCD_write_message("BKWD");
        }
    } else {

        LCD_write_message("P");
    }

    LCD_goto_line(3);
    sprintf(str, "Speed: %.3f", actual_speed_kmh);
    str[11] = '\0';
    LCD_write_string(str);
    LCD_write_message("Km/h");
    LCD_goto_line(4);
    LCD_write_message("====================");
}

void CAN_interpreter(void) {

    if (id == ECU_STATE) {
        if (RTR_Flag == 1) { //Se è arrivata la richiesta presenza centraline
            pr_time_4 = time_counter;
            data[0] = 0x03;
            while (CANisTxReady() != 1);
            CANsendMessage(ECU_STATE, data, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
            MotoreFlag = 1;
            AbsFlag = 0; //resetta flag
            SterzoFlag = 0; //resetta flag
        } else {
            if (data[0] == 0x01) {
                AbsFlag = 1;
                
            }
            if (data[0] == 0x02) {
                SterzoFlag = 1;
                
            }
        }
        if (pr_time_4 - time_counter > 450) {
            //DO SOMETHING HERE MATE!
        }
    }

    if ((id == ACTUAL_SPEED)&&(RTR_Flag == 0)) {
        left_speed = data[1];
        left_speed = ((left_speed << 8) | (data[0]));
        right_speed = data[3];
        right_speed = ((right_speed << 8) | (data[2]));
        actual_speed = (right_speed + left_speed) / 2;
    }

    if (id == LOW_BATTERY) {
        battery = data[0];
    }
}

void PWR_Button_Polling(void) {
    if ((PORTBbits.RB5 == LOW) || (wait_low == LOW)) {
        wait_low = LOW;
        if (PORTBbits.RB5 == HIGH) {
            power_switch = ~power_switch;
            wait_low = HIGH;
        }
    }
}

void Joystick_Polling(void) {
    for (unsigned char i = 0; i < 2; i++) {
        ADCON0bits.GO = HIGH;
        while (ADCON0bits.GO);
        JoystickValues[i] = ADRESH;
        ADCON0bits.CHS0 = ~ADCON0bits.CHS0;
    }
}

void board_initialization(void) {
    //Inputs and Outputs Configuration
    LATA = 0x00;
    TRISA = 0b00001111; // X-Axis / Y-Axis / 3P Switch
    LATB = 0x00;
    TRISB = 0b11111011; //CAN BUS / ON-OFF SWITCH
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
   // LCD Initialize
        LCD_initialize(16);
        LCD_backlight(0);
        LCD_clear();
        LCD_goto_line(1);

        LCD_write_message("Wait...");
        LCD_goto_line(2);
        LCD_write_message("hola muchacho");
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


    T3CON = 0x01; //Timer Enable
    LCD_clear();
    INTCONbits.GIEH = HIGH;
    INTCONbits.GIEL = HIGH;
}