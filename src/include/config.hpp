// enum Boards
// {
//     ESP32,
//     ADUINO_NANO,
//     ARDUINO_UNO
// }

// #define BOARD ESP32

#define NUMBER_OF_MOTORS 4

// the pulse width of the ESC in microseconds
#define LOW_PWM 1000
#define HIGH_PWM 2000

// R1 = 1000 and R2 = 220 gives a reduction ratio od 5.45. With a max voltage of 3.3v on the
// analog pin, that gives a max voltage of 18.3V which allows up to 4S battery (14.7V)
#define VOLTAGE_DIVIDER_R1 = 1000
#define VOLTAGE_DIVIDER_R2 = 220
