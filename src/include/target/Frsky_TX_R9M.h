
#if !defined(DEVICE_NAME)
    #define DEVICE_NAME "FrSky R9M"
#endif

#define TARGET_USE_EEPROM               1
#define TARGET_EEPROM_ADDR              0x51    // M24C02W 2kBit, 400kHz, eeprom from STmicro
#define TARGET_EEPROM_400K

// GPIO pin definitions
#define GPIO_PIN_RFamp_APC1             PA6  //APC2 is connected through a I2C dac and is handled elsewhere
#define GPIO_PIN_RFswitch_CONTROL       PB3  //HIGH = RX, LOW = TX

#define GPIO_PIN_NSS                    PB12
#define GPIO_PIN_DIO0                   PA15
#define GPIO_PIN_MOSI                   PB15
#define GPIO_PIN_MISO                   PB14
#define GPIO_PIN_SCK                    PB13
#define GPIO_PIN_RST                    PC14
#define GPIO_PIN_RX_ENABLE              GPIO_PIN_RFswitch_CONTROL
#define GPIO_PIN_TX_ENABLE              GPIO_PIN_RFamp_APC1

#define GPIO_PIN_SDA                    PB7
#define GPIO_PIN_SCL                    PB6

#define GPIO_PIN_DAC_ADR0               PB5     // ADR0 on the DAC, added by PET, not used

#define GPIO_PIN_RCSIGNAL_RX            PB11    // S.Port rx, USART3_RX, S.PORT pin (backside and jr-bay are interconnected), SN74LVC2G240, switched by PA5
#define GPIO_PIN_RCSIGNAL_TX            PB10    // S.Port tx, USART3_TX, S.PORT pin (backside and jr-bay are interconnected), SN74LVC2G240, switched by PA5
#define GPIO_PIN_BAY_RX                 PA3     // jr-bay rx pin, inverted ttl serial, USART2_RX
#define GPIO_PIN_BAY_TX                 PA2     // jr-bay tx pin, inverted ttl serial, USART2_TX

#define GPIO_PIN_LED_RED                PA11    // Red LED
#define GPIO_PIN_LED_GREEN              PA12    // Green LED
#define GPIO_PIN_BUTTON                 PA8     // pullup e.g. LOW when pressed
#define GPIO_PIN_BUZZER                 PB1
#define GPIO_PIN_DIP1                   PA12    // dip switch 1
#define GPIO_PIN_DIP2                   PA11    // dip switch 2
#define GPIO_PIN_FAN_EN                 PB9     // Fan mod https://github.com/AlessandroAU/ExpressLRS/wiki/R9M-Fan-Mod-Cover, also 28E11 eeprom with SHA!

#define GPIO_PIN_DEBUG_RX               PA10    // confirmed, USART1_RX, backside of the module where LED's are
#define GPIO_PIN_DEBUG_TX               PA9     // confirmed, USART1_TX, backside of the module where LED's are


#define GPIO_PIN_BUFFER_OE              PA5     //CONFIRMED
#define GPIO_PIN_BUFFER_OE_INVERTED     0
#define GPIO_PIN_DIO1                   PA1     //Not Needed, HEARTBEAT pin

// Power Output
#define POWER_OUTPUT_DAC                0b0001100       // address 0x0C,  DAC081C081 (TI)
#define MinPower                        PWR_10mW
#define HighPower                       PWR_250mW
#define MaxPower                        PWR_1000mW
#if !defined(POWER_OUTPUT_VALUES)
    #if defined(Regulatory_Domain_EU_868)
        //                          10mW, 25mW, 50mW, 100mW, 250mW, 500mW, 1000mW, 2000mW;
        #define POWER_OUTPUT_VALUES {650,  860, 1000,  1160,  1420,  1730,   2100,   2600 }         // voltages for APC2 of SKY65111-348LF 2W PA; 
    #else
        #define POWER_OUTPUT_VALUES {720,875,1000,1140,1390,1730,2100,2600}
    #endif
#endif
