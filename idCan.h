/*
 * file contenente tutti gli ID dei messaggi can bus
 */

// Versione 31/05/16

#define EMERGENCY                0b00000000001
#define ECU_STATE_ABS            0b00000000010
#define ECU_STATE_EPS            0b00000000011
#define ECU_STATE_REMOTECAN      0b00000000100
#define SPEED_CHANGE             0b00000000101
#define BRAKE_SIGNAL             0b00000000110
#define STEERING_CHANGE          0b00000000111
#define ACTUAL_SPEED             0b00000001000
#define COUNT_STOP               0b00000001001
#define COUNT_START              0b00000001010
#define LOW_BATTERY              0b00000001011
#define PARK_ASSIST_ENABLE       0b00000001100
#define PARK_ASSIST_STATE        0b00000001101
#define PARK_ASSIST_BEGIN        0b00000001110
#define DISTANCE_SET             0b00000001111
#define STEERING_CORRECTION      0b00000010000
#define SENSOR_DISTANCE          0b00000010001
#define BATTERY_CHARGING         0b00000010010