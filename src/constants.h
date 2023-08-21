#define         MOCK_DATA                       true

// Device pins
#define         SPI_HDR_CS                      10
#define         EXTERNAL_DIGITAL_PIN            6

// LED
#define         LED1                            21
#define         LED2                            22
#define         LED3                            23

// Digital I/O
// inputs
#define         DIGITAL_IO1                     24
#define         DIGITAL_IO2                     25
#define         DIGITAL_IO3                     26
#define         DIGITAL_IO4                     27
// outputs
#define         DIGITAL_IO5                     28
#define         DIGITAL_IO6                     23 // Shared with LED3
#define         DIGITAL_IO7                     39

// PWM
#define         EXTERNAL_PWM1                   5
#define         EXTERNAL_PWM2                   6
#define         EXTERNAL_PWM3                   29
#define         EXTERNAL_PWM4                   30

// Analog
#define         EXTERNAL_ADC1                   PIN_A14
#define         EXTERNAL_ADC2                   PIN_A15
#define         EXTERNAL_ADC3                   PIN_A16
#define         EXTERNAL_ADC4                   PIN_A17
#define         EXTERNAL_NTC1                   PIN_A18
#define         EXTERNAL_NTC2                   PIN_A19

// DAC
#define         EXTERNAL_DAC1                   PIN_A21
#define         MIN_DAC_VAL                     0
#define         MAX_DAC_VAL                     990

// Thermocouple
#define         SPI_CS_TC1                      14
#define         SPI_CS_TC2                      15


// CANBUS
#define         CAN_TX_PIN                      3
#define         CAN_RX_PIN                      4
#define         CANID_THERMOCOUPLE              2025
#define         CANID_ANALOG                    2026
#define         CANID_NTC                       2027
#define         CANID_PWM                       2028
#define         CANID_DIO                       2029
#define         CANID_COOLER_SYSTEM             2030

// UART
#define         UART1_TX_PIN                    8
#define         UART1_RX_PIN                    7
#define         UART2_TX_PIN                    32
#define         UART2_RX_PIN                    31
