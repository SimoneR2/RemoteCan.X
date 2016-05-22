// User defined parameters /////////////////////////////////////////////////////
#define SPD_CNST_STD 15 //max value: 35
#define SPD_CNST_PKG 1  //max value: 35
#define LCD_DLY 100   //[ms] multiple of 10 only
#define LCD_PKG_DLY 2000   //[ms] multiple of 10 only
#define LCD_4TH_ROW_MODE 1  /* 0 to visualize the parking messages and 1 to
                            visualize the data sent on CANBUS with the ID 0xAA */
////////////////////////////////////////////////////////////////////////////////

#define LCD_DEFAULT
#define _XTAL_FREQ 16000000

#include <xc.h>
#include "pic_config.h"
#include "idCan.h"
#include "CANlib.h"
#include "delay.h"
#include "delay.c"
#include "LCD_44780.h" 
#include "LCD_44780.c"
#include <stdio.h>
//#include <math.h>

#define HIGH 1
#define LOW 0

#define X_AXIS 0 //Steering
#define Y_AXIS 1 //Speed

#define HIGH_POS 0
#define MID_POS 1
#define LOW_POS 2

#define FWD 1
#define BKWD 0

#define OFF 0
#define SEARCH 1
#define PARKING 2

//prototipi delle funzioni
void board_initialization(void);
void PWR_Button_Polling(void);
void F1_Button_Polling(void);
void F2_Button_Polling(void);
void Joystick_Polling(void);
void CAN_Tx(void);
void CAN_Rx(void);
void LCD_Handler(void);
void LCD_Gap(void); //Space found, continue forward till distance
void LCD_Park(void); //Distance REACHED, Parking authorization required!
void LCD_Parking(void); //Parking procedures wait...
void LCD_End(void); //Parking procedures COMPLETED SUCCESSFULLY
void LCD_Error(void); //Error occured

//variabili per delay non blocking
volatile unsigned long time_counter = 0;
volatile unsigned long pr_time_1 = 0;
volatile unsigned long pr_time_2 = 0;
volatile unsigned long pr_time_3 = 0;
volatile unsigned long pr_time_4 = 0;
volatile unsigned long pr_time_5 = 0;
volatile unsigned long pr_time_6 = 0;

//Variabili per can bus invio
CANmessage msg;
volatile bit Can_Tx_Force = LOW;
BYTE data_steering[] = 0;
BYTE data_speed [] = 0;
BYTE data_brake [] = 0;
BYTE data[] = 0; //random
BYTE park_assist_state[8] = 0;

//variabili can bus ricezione
volatile bit RTR_Flag = LOW;
volatile bit newMessageCan = LOW;
volatile bit MotoreFlag = LOW;
volatile bit AbsFlag = LOW;
volatile bit SterzoFlag = LOW;
volatile unsigned int left_speed = 0;
volatile unsigned int right_speed = 0;
volatile unsigned char battery = 0;
volatile unsigned long id = 0;
BYTE data_speed_rx[7] = 0;

//variabili LCD
volatile bit display_refresh = LOW;
volatile bit parking_message_E = LOW;
volatile unsigned char parking_message_ID = 0; //0=>GAP | 2=>PARK | 4=>PARKING | 6=>END
volatile unsigned char str [12] = 0;
volatile float actual_speed_kmh = 0;
volatile unsigned int user_data = 0;
volatile unsigned int actual_speed = 0;

//Variabili parcheggio
volatile bit x = LOW;
volatile bit y = LOW;
volatile bit z = LOW;
volatile bit space_available = LOW;
volatile bit space_gap_reached = LOW;
volatile bit steering_correction_dir = LOW;
volatile bit parking_error = LOW;
volatile unsigned char steering_correction = 0;
volatile unsigned char parking_state = OFF;
volatile signed long check = 0;

//variabili per il programma
volatile bit wait_low_1 = LOW;
volatile bit wait_low_2 = LOW;
volatile bit wait_low_3 = LOW;
volatile bit power_switch = LOW;
volatile bit F1_switch = LOW;
volatile bit F2_switch = LOW;
volatile bit FWD_sensor = LOW;
volatile bit BKWD_sensor = LOW;
volatile unsigned char dir = 0;
volatile unsigned char switch_position = 0;
volatile unsigned char set_steering = 0;
volatile unsigned int set_speed = 0;
volatile unsigned char JoystickValues[2] = 0; //steering - speed
volatile signed float JoystickConstants[2] = 0;

__interrupt(high_priority) void ISR_alta(void) {
    if ((PIR3bits.RXB1IF == HIGH) || (PIR3bits.RXB0IF == HIGH)) { //RICEZIONE CAN
        if (CANisRxReady()) {
            CANreceiveMessage(&msg);
            RTR_Flag = msg.RTR;
            id = msg.identifier;
            newMessageCan = HIGH;
            if (id == ACTUAL_SPEED) {
                for (unsigned char i = 0; i < 8; i++) {
                    data_speed_rx[i] = msg.data[i];
                }
            }

            if (id == PARK_ASSIST_STATE) {// 1=> SPAZIO 2=> GAP 3=> END
                if (msg.data[0] == 1) {
                    LATDbits.LATD2 = HIGH; //DEBUG
                    space_available = HIGH;
                    space_gap_reached = LOW;
                    F2_switch = LOW;
                    parking_message_E = HIGH;
                    parking_message_ID = 0;
                }

                if (msg.data[0] == 2) {
                    LATDbits.LATD3 = HIGH; //DEBUG
                    space_available = HIGH; //DEBUG
                    space_gap_reached = HIGH;
                    Can_Tx_Force = HIGH;
                    parking_message_ID = 2;
                }

                if (msg.data[0] == 3) {
                    parking_message_ID = 6;
                    F1_switch = LOW; //Reset parking
                    pr_time_6 = time_counter + (LCD_PKG_DLY / 10);
                }
                if (msg.data[0] == 4) {
                    parking_message_ID = 6;
                    parking_error = HIGH; //Error flag
                    F1_switch = LOW; //Reset parking
                    pr_time_6 = time_counter + (LCD_PKG_DLY / 10);
                }
            }

            if (id == STEERING_CORRECTION) {
                steering_correction_dir = msg.data[1];
                steering_correction = msg.data[0];
            }

            if (id == SENSOR_DISTANCE) {
                FWD_sensor = (msg.data[0] && 0b01000000) >> 6;
                BKWD_sensor = (msg.data[0] && 0b00001000) >> 3;
            }
            if (id == 0xAA) {
                user_data = msg.data[1];
                user_data = ((user_data << 8) | msg.data[0]);
            }

            if (id == ECU_STATE) {
                if (RTR_Flag == 1) { //Se è arrivata la richiesta presenza centraline
                    pr_time_4 = time_counter;
                    data[0] = 0x03;
                    __delay_us(10);
                    while (CANisTxReady() != 1);
                    CANsendMessage(ECU_STATE, data, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);

                    //[!]CONTROLLARE QUESTA PARTE!
                    //MotoreFlag = HIGH;
                    //AbsFlag = LOW; //resetta flag
                    //SterzoFlag = LOW; //resetta flag
                }
            }
        }
        PIR3bits.RXB1IF = LOW;
        PIR3bits.RXB0IF = LOW;
    }
}

__interrupt(low_priority) void ISR_bassa(void) {
    if (PIR2bits.TMR3IF == HIGH) { //interrupt timer, ogni 10mS
        time_counter++;
        TMR3H = 0x63;
        TMR3L = 0xC0;
        PIR2bits.TMR3IF = LOW;
    }
}

void main(void) {
    board_initialization();

    //    LATDbits.LD6 = HIGH;
    //    LATDbits.LD5 = HIGH;
    //    delay_ms(500);
    //    LATDbits.LD6 = LOW;
    //    LATDbits.LD5 = LOW;

    JoystickConstants[X_AXIS] = 0.703;
    JoystickConstants[Y_AXIS] = SPD_CNST_STD;

    data_brake [1] = 0b00000000;

    while (1) {

        //Buttons Polling
        PWR_Button_Polling();
        F1_Button_Polling();
        F2_Button_Polling();

        // ON/OFF routine
        if (power_switch == LOW) {
            dir = FWD;
            set_speed = 0;
            data_steering [0] = 90;
            data_brake [0] = 0b00000000;
            CAN_Tx();
            PORTDbits.RD6 = LOW;
            PORTDbits.RD5 = LOW;
            LCD_initialize(16);
            LCD_goto_line(1);
            LCD_write_message("====================");
            LCD_goto_line(2);
            LCD_write_message("==> VEHICLE  OFF <==");
            LCD_goto_line(3);
            LCD_write_message("  Press the button  ");
            LCD_goto_line(4);
            LCD_write_message("====================");
            while (power_switch == LOW) {
                if ((time_counter - pr_time_1) >= 30) {
                    pr_time_1 = time_counter;
                    PORTDbits.RD7 = ~PORTDbits.RD7;
                }
                PWR_Button_Polling();
                delay_ms(10);
            }
            PORTDbits.RD7 = LOW; //Turn off ON/OFF switch backlight
            display_refresh = HIGH;
        }

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

        //Parking
        if (F1_switch == HIGH) {
            y = LOW;
            if ((x == LOW)&&(F2_switch == LOW)) {
                space_available = LOW;
                parking_state = SEARCH;
                JoystickConstants[Y_AXIS] = SPD_CNST_PKG;
                while (!CANisTxReady());
                park_assist_state[0] = 0b00000001;
                CANsendMessage(PARK_ASSIST_ENABLE, park_assist_state, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
                x = HIGH;
            }

            if ((F2_switch == HIGH)&&(space_available == HIGH)&&(space_gap_reached == HIGH)) {
                if (z == LOW) {
                    parking_message_ID = 4;
                    parking_state = PARKING;
                    while (!CANisTxReady());
                    CANsendMessage(PARK_ASSIST_BEGIN, data, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
                    x = LOW;
                    z = HIGH;
                }
            } else {
                z = LOW;
            }

            if ((time_counter - pr_time_5) >= 30) {
                pr_time_5 = time_counter;
                if (space_available == LOW) {
                    PORTDbits.RD6 = ~PORTDbits.RD6;
                    PORTDbits.RD5 = LOW;
                } else {
                    PORTDbits.RD6 = HIGH;
                    if (F2_switch == LOW) {
                        PORTDbits.RD5 = ~PORTDbits.RD5;
                    } else {
                        PORTDbits.RD5 = HIGH;

                    }
                }
            }
        } else { //RD6/RB4-- RD5/RB1
            x = LOW;
            z = LOW;
            parking_message_E = LOW;
            F2_switch = LOW;
            space_available = LOW;
            space_gap_reached = LOW;
            PORTDbits.RD5 = LOW;
            PORTDbits.RD6 = LOW;
            if (y == LOW) {
                parking_state = OFF;
                JoystickConstants[Y_AXIS] = SPD_CNST_STD;
                while (!CANisTxReady());
                park_assist_state[0] = 0b00000000;
                CANsendMessage(PARK_ASSIST_ENABLE, park_assist_state, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
                y = HIGH;
            }
            PORTDbits.RD6 = LOW;
            PORTDbits.RD5 = LOW;
        }

        Joystick_Polling();

        //Steering
        data_steering [0] = 180 - (JoystickValues[X_AXIS])*(JoystickConstants[X_AXIS]);
        if (parking_state == SEARCH) {
            if (steering_correction_dir == HIGH) {
                check = data_steering[0] + steering_correction;
            } else {
                check = data_steering[0] - steering_correction;
            }
            if ((check >= 0)&&(check <= 180)) {
                data_steering[0] = check;
            } else {
                if (check > 180) {
                    data_steering[0] = 180;
                } else {
                    data_steering[0] = 0;
                }
            }
        } else {
            steering_correction_dir = LOW;
            steering_correction = 0;
        }

        //Speed
        if ((switch_position != HIGH_POS)&&(((switch_position == MID_POS)&&(FWD_sensor == LOW)) || (switch_position == LOW_POS)&&(BKWD_sensor == LOW))) {
            if (JoystickValues[Y_AXIS] > 132) {
                set_speed = (JoystickValues[Y_AXIS] - 130)*(JoystickConstants[Y_AXIS]); //guardare
                data_brake [0] = 0b00000011;
            } else {
                set_speed = 0;
                if (JoystickValues[Y_AXIS] <= 65) {
                    data_brake [0] = 0b00000010;
                }
                if (JoystickValues[Y_AXIS] <= 20) {
                    data_brake [0] = 0b00000001;
                }
                if (JoystickValues[Y_AXIS] <= 5) {
                    data_brake [0] = 0b00000000;
                }
            }
        } else {
            set_speed = 0;
            data_brake [0] = 0b00000000;
        }

        if (newMessageCan == HIGH) {
            CAN_Rx();
            newMessageCan = LOW;
        }

        if ((((time_counter - pr_time_2) >= 2) && (parking_message_ID < 2)) || (Can_Tx_Force == HIGH)) {
            if (Can_Tx_Force == HIGH) {
                dir = FWD;
                set_speed = 0;
                data_steering [0] = 90;
                data_brake [0] = 0b00000000; //<== VALORE FRENATA CAMBIARE?
                Can_Tx_Force = LOW;
            }
            pr_time_2 = time_counter;
            CAN_Tx();
        }

        //LCD
        if (time_counter >= pr_time_6) {
            if ((time_counter - pr_time_3) >= (LCD_DLY / 10)) {
                pr_time_3 = time_counter;
                if (parking_message_E == HIGH) {
                    display_refresh = HIGH;
                    if (parking_message_ID == 0) {
                        LCD_Gap();
                        parking_message_ID++;
                    }
                    if (parking_message_ID == 2) {
                        LCD_Park();
                        parking_message_ID++;
                    }
                    if (parking_message_ID == 4) {
                        LCD_Parking();
                        parking_message_ID++;
                    }
                } else {
                    LCD_Handler();
                }
            }
        } else {
            if (parking_message_ID == 6) {
                if (parking_error == HIGH) {
                    parking_error = LOW;
                    LCD_Error();
                    while (CANisTxReady() != HIGH);
                    data_brake [0] = 0b00000000;
                    CANsendMessage(BRAKE_SIGNAL, data_brake, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
                } else {
                    LCD_End();
                }
                parking_message_ID = 0;
                parking_message_E = LOW;
            }
        }
    }
}

void LCD_Handler(void) {
    //ROUTINE PER RILEVARE LA VELOCITA' VIA CAN
    while (CANisTxReady() != HIGH);
    CANsendMessage(ACTUAL_SPEED, data_speed, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);

    actual_speed_kmh = ((actual_speed) / 278.0);
    sprintf(str, "%.2f", actual_speed_kmh);
    str[11] = '\0';

    if (display_refresh == HIGH) {
        LCD_initialize(16);
        LCD_clear();
        LCD_goto_line(1);
        LCD_write_message("=== VEHICLE DATA ===");
        LCD_goto_line(2);
        LCD_write_message("Direction: ");
        LCD_goto_line(3);
        LCD_write_message("Speed: 0.00 Km/h");
        LCD_goto_line(4);
        if (LCD_4TH_ROW_MODE == LOW) {
            LCD_write_message("Park assist: ");
        } else {
            LCD_write_message("Data: ");
        }
        display_refresh = LOW;
    }

    //Print direction data
    LCD_goto_xy(12, 2);
    if (switch_position != HIGH_POS) {
        if (dir == FWD) {
            LCD_write_message("FWD ");
        } else {
            LCD_write_message("BKWD");
        }
    } else {
        LCD_write_message("P   ");
    }

    //Print speed data 
    LCD_goto_xy(8, 3);
    LCD_write_string(str);

    if (LCD_4TH_ROW_MODE == LOW) {
        //Print parking data
        LCD_goto_xy(14, 4);
        if (parking_state == OFF) {
            LCD_write_message("OFF    ");
        } else {
            LCD_write_message("SEARCH ");
        }
    } else {
        //Print user data sent with id 0xAA
        LCD_goto_xy(7, 4);
        LCD_write_integer(user_data, 0, LCD_ZERO_CLEANING_ON);
    }
}

void LCD_Gap(void) {
    LCD_initialize(16);
    LCD_clear();
    LCD_goto_line(1);
    LCD_write_message("= PARKING MESSAGES =");
    LCD_goto_line(2);
    LCD_write_message(" Parking space FOUND");
    LCD_goto_line(3);
    LCD_write_message("->  KEEP MOVING   <-");
    LCD_goto_line(4);
    LCD_write_message(" until next message ");
}

void LCD_Park(void) {
    LCD_initialize(16);
    LCD_clear();
    LCD_goto_line(1);
    LCD_write_message("= PARKING MESSAGES =");
    LCD_goto_line(2);
    LCD_write_message("- Distance REACHED -");
    LCD_goto_line(3);
    LCD_write_message("Press the BUTTON to ");
    LCD_goto_line(4);
    LCD_write_message("   start parking    ");
}

void LCD_Parking(void) {
    LCD_initialize(16);
    LCD_clear();
    LCD_goto_line(1);
    LCD_write_message("= PARKING MESSAGES =");
    LCD_goto_line(2);
    LCD_write_message("=    Parking in    =");
    LCD_goto_line(3);
    LCD_write_message("=    progress...   =");
    LCD_goto_line(4);
    LCD_write_message("====================");
}

void LCD_End(void) {
    LCD_initialize(16);
    LCD_clear();
    LCD_goto_line(1);
    LCD_write_message("= PARKING MESSAGES =");
    LCD_goto_line(2);
    LCD_write_message(" Parking procedures ");
    LCD_goto_line(3);
    LCD_write_message("    SUCCESSFULLY    ");
    LCD_goto_line(4);
    LCD_write_message("     COMPLETED!     ");
}

void LCD_Error(void) {
    LCD_initialize(16);
    LCD_clear();
    LCD_goto_line(1);
    LCD_write_message("= PARKING MESSAGES =");
    LCD_goto_line(2);
    LCD_write_message(" Parking procedures ");
    LCD_goto_line(3);
    LCD_write_message("       FAILED!      ");
    LCD_goto_line(4);
    LCD_write_message("   due to an error  ");
}

void PWR_Button_Polling(void) {
    if ((PORTBbits.RB5 == LOW) || (wait_low_1 == LOW)) {
        wait_low_1 = LOW;
        if (PORTBbits.RB5 == HIGH) {
            power_switch = ~power_switch;
            wait_low_1 = HIGH;
        }
    }
}

void F1_Button_Polling(void) { //Backlight RD6
    if ((PORTBbits.RB4 == LOW) || (wait_low_2 == LOW)) {
        wait_low_2 = LOW;
        if (PORTBbits.RB4 == HIGH) {
            F1_switch = ~F1_switch;
            wait_low_2 = HIGH;
        }
    }
}

void F2_Button_Polling(void) { //Backlight RD5
    if ((PORTBbits.RB1 == LOW) || (wait_low_3 == LOW)) {
        wait_low_3 = LOW;
        if (PORTBbits.RB1 == HIGH) {
            F2_switch = ~F2_switch;
            wait_low_3 = HIGH;
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

void CAN_Tx(void) {
    //Send steering message
    while (CANisTxReady() != HIGH);
    CANsendMessage(STEERING_CHANGE, data_steering, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);

    //Send speed message
    data_speed[0] = set_speed;
    data_speed[1] = (set_speed >> 8);
    data_speed[2] = dir;
    while (CANisTxReady() != HIGH);
    CANsendMessage(SPEED_CHANGE, data_speed, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);

    //Send brake message
    while (CANisTxReady() != HIGH);
    CANsendMessage(BRAKE_SIGNAL, data_brake, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);

}

void CAN_Rx(void) {
    if (id == ECU_STATE) {
        if (RTR_Flag == HIGH) { //Se è arrivata la richiesta presenza centraline
            pr_time_4 = time_counter;
            MotoreFlag = HIGH;
            AbsFlag = LOW; //resetta flag
            SterzoFlag = LOW; //resetta flag
        } else {
            if (data[0] == 0x01) {
                AbsFlag = HIGH;
            }
            if (data[0] == 0x02) {
                SterzoFlag = HIGH;
            }
        }
    }

    if (pr_time_4 - time_counter > 450) {
        //CONTROLLO DEI FLAG DI RISPOSTA ECU
    }

    if ((id == ACTUAL_SPEED)&&(RTR_Flag == 0)) {
        left_speed = data_speed_rx[1];
        left_speed = ((left_speed << 8) | (data_speed_rx[0]));
        right_speed = data_speed_rx[3];
        right_speed = ((right_speed << 8) | (data_speed_rx[2]));
        actual_speed = (right_speed + left_speed) / 2;
    }

    if (id == LOW_BATTERY) { //?
        battery = data[0];
    }
}

void board_initialization(void) {
    //Inputs and Outputs Configuration
    LATA = 0x00;
    TRISA = 0b00011111; // X-Axis / Y-Axis / 3P Switch
    LATB = 0x00;
    TRISB = 0b11111011; //CAN BUS / ON-OFF SWITCH
    LATC = 0x00;
    TRISC = 0b11110000; //USART Tx and Rx / LCD
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
    //LCD_backlight(0);
    LCD_clear();
    LCD_goto_line(1);
    //LCD_write_message("Wait...");
    //delay_ms(300);

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