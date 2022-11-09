/*
Credit to Jacob Walser (jaxxzer) for the pinout!!!
https://github.com/jaxxzer
*/
#if !defined(TARGET_R9SLIM_RX)
    #define TARGET_USE_EEPROM               1
    #define TARGET_EEPROM_ADDR              0x50
#endif

#define GPIO_PIN_NSS            PB12 //confirmed on SLIMPLUS, R900MINI
#define GPIO_PIN_DIO0           PA15 //confirmed on SLIMPLUS, R900MINI
#define GPIO_PIN_DIO1           PA1  // NOT CORRECT!!! PIN STILL NEEDS TO BE FOUND BUT IS CURRENTLY UNUSED
/////////////////////////////////////// NOT FOUND ON SLIMPLUS EITHER.
#define GPIO_PIN_MOSI           PB15 //confirmed on SLIMPLUS, R900MINI
#define GPIO_PIN_MISO           PB14 //confirmed on SLIMPLUS, R900MINI
#define GPIO_PIN_SCK            PB13 //confirmed on SLIMPLUS, R900MINI
#define GPIO_PIN_RST            PC14 //confirmed on SLIMPLUS, R900MINI
#define GPIO_PIN_SDA            PB7
#define GPIO_PIN_SCL            PB6

#if defined(USE_R9MM_R9MINI_SBUS)
    #define GPIO_PIN_RCSIGNAL_RX    PA3
    #define GPIO_PIN_RCSIGNAL_TX    PA2
    #define DEVICE_NAME "FrSky R9MM SBUS"
#elif defined(TARGET_R9SLIM_RX)
    #define GPIO_PIN_RCSIGNAL_RX    PA3  // RX1 PIN OF CONNECTOR 1 ON SLIM
    #define GPIO_PIN_RCSIGNAL_TX    PA2  // TX1 PIN OF CONNECTOR 1 ON SLIM
    #define DEVICE_NAME "FrSky R9SLIM RX"
#elif defined(TARGET_R9SLIMPLUS_RX)      // R9SLIMPLUS USES DUAL UART CONFIGURATION FOR TX1/RX1
    #define GPIO_PIN_RCSIGNAL_RX    PB11 // RX1 PIN OF CONNECTOR 1 ON SLIMPLUS
    #define GPIO_PIN_RCSIGNAL_TX    PA9  // TX1 PIN OF CONNECTOR 1 ON SLIMPLUS
    #define DEVICE_NAME "FrSky R9SLIM+"
#elif defined(TARGET_R900MINI_RX)
    #define GPIO_PIN_RCSIGNAL_RX    PA3 // convinient pin for direct chip solder
    #define GPIO_PIN_RCSIGNAL_TX    PA2 // convinient pin for direct chip solder
    #define DEVICE_NAME "Jumper R900 MINI"
#else
    #define GPIO_PIN_RCSIGNAL_RX    PA10
    #define GPIO_PIN_RCSIGNAL_TX    PA9
    #ifndef DEVICE_NAME
        #define DEVICE_NAME "FrSky R9MM"
    #endif
#endif

#if defined(TARGET_R9MX_RX)
    #define GPIO_PIN_LED_RED        PB2 // Red
    #define GPIO_PIN_LED_GREEN      PB3 // Green
    #define GPIO_PIN_BUTTON         PB0  // pullup e.g. LOW when pressed
#elif defined(TARGET_R9SLIM_RX)
    #define GPIO_PIN_LED_RED        PA11 // Red
    #define GPIO_PIN_LED_GREEN      PA12 // Green
    #define GPIO_PIN_BUTTON         PC13  // pullup e.g. LOW when pressed
    /* PB3: RX = HIGH, TX = LOW */
    #define GPIO_PIN_RX_ENABLE      PB3
#elif defined(TARGET_R9SLIMPLUS_RX)
    #define GPIO_PIN_LED_RED        PA11 // Red
    #define GPIO_PIN_LED_GREEN      PA12 // Green
    #define GPIO_PIN_BUTTON         PC13  // pullup e.g. LOW when pressed
    /* PB3: RX = HIGH, TX = LOW */
    #define GPIO_PIN_RX_ENABLE      PB3
    /* PB9: antenna 1 (left) = HIGH, antenna 2 (right) = LOW
     * Note: Right Antenna is selected by default, LOW */
    #define GPIO_PIN_ANTENNA_SELECT PB9
#elif defined(TARGET_R900MINI_RX)
    #define GPIO_PIN_LED_RED        PA11 // Red
    #define GPIO_PIN_LED_GREEN      PA12 // Green
    #define GPIO_PIN_BUTTON         PC13 // pullup e.g. LOW when pressed
    // RF Switch: HIGH = RX, LOW = TX
    #define GPIO_PIN_RX_ENABLE      PB3
#else //R9MM_R9MINI
    #define GPIO_PIN_LED_RED        PC1  // Red
    #define GPIO_PIN_LED_GREEN      PB3  // Green
    #define GPIO_PIN_BUTTON         PC13 // pullup e.g. LOW when pressed
#endif

#if defined(TARGET_R9SLIMPLUS_RX)
    // PET, slim+ OTA CON1
    #define r9slimplusOTA_GPIO_PIN_SPORT        SN74LVC2G240
    #define r9slimplusOTA_GPIO_PIN_SBUS_IN      SN74LVC2G240
    #define r9slimplusOTA_GPIO_PIN_SBUS_OUT     SN74LVC2G240
    #define r9slimplusOTA_GPIO_PIN_RSSI_OUT     PB8     // rssi out analog (analogWrite())
    #define r9slimplusOTA_GPIO_PIN_TX1          PA9     // USART1_TX, RX is on PA10, INAV - comm. with FC
    #define r9slimplusOTA_GPIO_PIN_RX1          PB11    // USART3_RX, TX us on PB10, INAV - comm. with FC

    // PET, slim+ OTA CON2  // t=timer
    #define r9slimplusOTA_GPIO_PIN_CH1          PB1     // t1, t3, t8, runcam2 4k remote (thru 2n7000 mosfet pulls it's remote pin to GND  )
    #define r9slimplusOTA_GPIO_PIN_CH2          PB0     // t1, t3, t8
    #define r9slimplusOTA_GPIO_PIN_CH3          PA7     // t1, t3, t8, t17
    #define r9slimplusOTA_GPIO_PIN_CH4          PA6     // t1, t3, t8, t16
    #define r9slimplusOTA_GPIO_PIN_CH5_RX       PA3     // t2, t15, USART2_RX, PET - GPS in
    #define r9slimplusOTA_GPIO_PIN_CH6_TX       PA2     // t2, t15, USART2_TX
    
    #define pin_ch1_triggering_RCchannel        6       // ch1..ch11; ch6 = failsafe/beeper at 2000, 1000 = idle
    #define pin_ch1_singleclick_us              1275U   // uS on RCchannel to activate
    #define pin_ch1_doubleclick_us              1425UL  // uS on RCchannel to activate

    #define tClickDuration                       85      // mS RC24k remote pulse high duration
    #define tClickPause                          85      // mS RC24k remote pause between 2 pulses in doubleclick
    #define tClickRepeat                        1800UL   // ms minimum time to repat a click   // 1500mS also works but irregullary
    // AUX2(ch6) - AUX7(ch11): 1000, 1275, 1425, 1500, 1575, 1725, 2000                                                 // 3bit, 7 pos; no signal idle = 1500 except for CH5(ARM) that is 1875
    // AUX8(ch12): 1000, 1066, 1133, 1200, 1266, 1333, 1400, 1467, 1533, 1600, 1666, 1733, 1800, 1866, 1933, 2000       // 4bit, 16pos; no signal idle = 1500
#endif

// External pads
// #define R9m_Ch1    PA8
// #define R9m_Ch2    PA11
// #define R9m_Ch3    PA9
// #define R9m_Ch4    PA10
// #define R9m_sbus   PA2
// #define R9m_sport  PA5
// #define R9m_isport PB11

//method to set HSE and clock speed correctly//
// #if defined(HSE_VALUE)
// /* Redefine the HSE value; it's equal to 8 MHz on the STM32F4-DISCOVERY Kit */
//#undef HSE_VALUE
//#define HSE_VALUE ((uint32_t)16000000).
//#define HSE_VALUE    25000000U
// #endif /* HSE_VALUE */
//#define SYSCLK_FREQ_72MHz


// Output Power - Default to SX1272 max output
