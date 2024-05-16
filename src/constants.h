#define         MOCK_DATA                       false

#define         ADC_MAX                         65535

// Device pins
#define         SPI_HDR_CS                      31
#define         SPI_DISPLAY_CS                  2
#define         UI_BUTTON                       11
#define         FLOW_SENSOR                     24
#define         COOLANT_SWITCH                  25
#define         PTT_SWITCH                      26

// PWM
#define         PWM1                            3
#define         PWM2                            4
#define         PWM3                            5
#define         PWM4                            6

// Analog
#define         ADC_SPARE_12V                   PIN_A14
#define         ADC_SPARE_5V                    PIN_A15
#define         ADC_SYSTEM_12V                  PIN_A16
#define         ADC_SYSTEM_5V                   PIN_A22
#define         ADC_SYSTEM_3V3                  PIN_A18
#define         ADC_MAIN_SWITCH                 PIN_A17
#define         ADC_COMPRESSOR_LDR              PIN_A20
#define         ADC_COMPRESSOR_CURRENT          PIN_A2
#define         ADC_PRESSURE_SENSOR             PIN_A3
#define         NTC_AMBIENT                     PIN_A5
#define         NTC_CONDENSER_1                 PIN_A9
#define         NTC_CONDENSER_2                 PIN_A6
#define         NTC_EVAPORATOR_1                PIN_A8
#define         NTC_EVAPORATOR_2                PIN_A7
#define         NTC_EVAPORATOR_DIFF_1           PIN_A10
#define         NTC_EVAPORATOR_DIFF_2           PIN_A11

#define         NTC_ADC_NUM                     0
#define         ADC_SPARE_12V_ADC_NUM           0
#define         ADC_SPARE_5V_ADC_NUM            0
#define         ADC_SYSTEM_12V_ADC_NUM          1
#define         ADC_SYSTEM_5V_ADC_NUM           1
#define         ADC_SYSTEM_3V3_ADC_NUM          1
#define         ADC_MAIN_SWITCH_ADC_NUM         1
#define         ADC_LDR_ADC_NUM                 1
#define         ADC_COMPRESSOR_CURRENT_ADC_NUM  1
#define         ADC_PRESSURE_SENSOR_ADC_NUM     1

#define         DAC_COMPRESSOR_SPEED            PIN_A21

// CANBUS
#define         CAN_TX_PIN                      3
#define         CAN_RX_PIN                      4
#define         CANID_COOLER_SYSTEM             2030

// UART
#define         UART1_RX_PIN                    7
#define         UART1_TX_PIN                    8
#define         UART2_RX_PIN                    9
#define         UART2_TX_PIN                    10

// NTCs
// TE thermistor = GA10K3A1IA (condenser)
#define         TE_THERMISTOR_STEINHART_A       1.128653750e-3
#define         TE_THERMISTOR_STEINHART_B       2.342041378e-4
#define         TE_THERMISTOR_STEINHART_C       8.737724626e-8
// TDK thermistor = B57541G1103F005 (evaporator), 7003 curve
// Evaporator inlet (NTC1)
#define         TDK_THERMISTOR_1_STEINHART_A    9.195598396e-4
#define         TDK_THERMISTOR_1_STEINHART_B    2.552863593e-4
#define         TDK_THERMISTOR_1_STEINHART_C    1.064756872e-7
// Evaporator outlet (NTC2) from calibration relative to NTC1
#define         TDK_THERMISTOR_2_STEINHART_A    9.205205975e-4
#define         TDK_THERMISTOR_2_STEINHART_B    2.552981188e-4
#define         TDK_THERMISTOR_2_STEINHART_C    1.065197564e-7

// Flow sensor calibration, measured at 10C with 10% IPA / 90% water mix
#define FLOW_SENSOR_HERTZ_PER_LPM 7.3242
