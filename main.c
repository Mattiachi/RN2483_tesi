/*  
 * Author   :      Chiappalone Mattia
 * MPLAB    :      MPLABX IDE v3.40
 * Compiler :      XC8 v1.38 PRO
 * 
 * main.c
 * 
 * This program enstablish an ABP LoRaWAN communication once every hour sending 
 * out temperature and light telemetry to the ChirpStack server on port 2.
 * When the MCU is not sending it's sleeping.
 * Every 16 seconds an interrupt wakes the board up and checks if a hour is passed.
 *  
 */

#include "mcc_generated_files/mcc.h"

#define LED_GREEN PORTDbits.RD5
#define LED_ORANGE PORTCbits.RC5
#define ONE_HOUR_TIMEOUT_COUNTS         85// set to 225 for 1 hour sleep
#define TEMP 27    //is the ADC channel of the pin 
#define LIGHT 26


void handle16sInterrupt();
void IO_pins_init(void);
void ADC_SelChannel(uint8_t c);
uint16_t ADC_Read(uint8_t channel);
void ADC_Init(void);
void SysConfigSleep(void);
void readAndSend(void);

// LoRaWAN stuff
uint8_t nwkSKey[16] = {0x75, 0xE0, 0x66, 0x1A, 0xA0, 0xBC, 0x21, 0xE4, 0x4B, 0x57, 0x21, 0xB0, 0xD6, 0x30, 0xF7, 0xB2};
uint8_t appSKey[16] = {0xDD, 0x32, 0x54, 0xF0, 0x8D, 0x55, 0xD6, 0x52, 0xE7, 0x73, 0x0B, 0xBC, 0x26, 0x3E, 0x0A, 0xDA};
uint32_t devAddr = 0x12345678;
uint16_t payload[2];
void RxDataDone(uint8_t* pData, uint8_t dataLength, OpStatus_t status);
void RxJoinResponse(bool status);
void LoRaSleep(void);
void LoRaWakeUp(void);

uint8_t TimeToSend;
uint8_t portNumber = 1;

void main(void)
{

    SYSTEM_Initialize();
    // T3CONbits.T3CKPS = 0;     // if uncommented, the timer3 interrupt is requested after every 2 seconds
    // Enable the Global Interrupts
    LED_ORANGE = 1;
    
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    TMR3_SetInterruptHandler(handle16sInterrupt);
    SysConfigSleep();
    ADC_Init();
    IO_pins_init();
    
    LORAWAN_Init(RxDataDone, RxJoinResponse);
    LORAWAN_SetNetworkSessionKey(nwkSKey);
    LORAWAN_SetApplicationSessionKey(appSKey);
    LORAWAN_SetDeviceAddress(devAddr);
    LORAWAN_Join(ABP);
    // Application main loop
       
    TimeToSend = 1;
    
    while (1)
    {   
        LORAWAN_Mainloop();      
        
        if(TimeToSend){
            LoRaWakeUp();
            readAndSend();  // While transmitting orange led is lit
            TimeToSend = 0;
            
        }
        
        if(LORAWAN_GetState() == IDLE){   // it's not transmitting nor receiving
            LoRaSleep();
            SLEEP();
        }
    }
}

void handle16sInterrupt() {
    
    static volatile uint8_t counterSleepTimeout = 0;
    if( ++counterSleepTimeout == ONE_HOUR_TIMEOUT_COUNTS )
    {
        // a hour passed
        TimeToSend = 1;
        counterSleepTimeout = 0;
    } 
    else 
    {
        SLEEP();
    } 
}

void RxDataDone(uint8_t* pData, uint8_t dataLength, OpStatus_t status) 
{
    //This is a prototype for downlink. Any received data is stored in a buffer pointed by *pData with a lenght of dataLength bites
    portNumber = pData[0];
    LED_ORANGE = pData[1];
}

void RxJoinResponse(bool status)
{
    LED_ORANGE = 0; //When network is joined orange led is turned off
}

void readAndSend(void){
    LED_GREEN = 1;
    for(int j = 1; j<100; j++) __delay_ms(1);
    payload[0] = ADC_Read(TEMP);
    payload[1] = ADC_Read(LIGHT);
    LORAWAN_Send(UNCNF, portNumber, &payload, sizeof(payload)); //4 is the number of bytes sent
    LED_GREEN = 0;
    
}

void IO_pins_init(void){
    //TRISx = 1 input ; TRISx = 0 output
    TRISCbits.TRISC5 = 0;       // Orange led
    TRISDbits.TRISD5 = 0;       // Green led
    TRISDbits.TRISD6 = 1;       // Light sensor
    TRISDbits.TRISD7 = 1;       // Temperature sensor
    //ANSELx = 0 digital, ANSELx = 1 analog
    ANSELCbits.ANSC5 = 0;       // Orange led
    ANSELDbits.ANSD5 = 0;       // Green led
    ANSELDbits.ANSD6 = 1;       // Light sensor
    ANSELDbits.ANSD7 = 1;       // Temperature sensor
}   
void ADC_SelChannel(uint8_t c){
   ADCON0bits.CHS = c;    //Set the channel (analog pin))
   return;
}

uint16_t ADC_Read(uint8_t channel){
    uint16_t result = 0;
    ADCON0bits.ADON = 1;    //Turns on ADC
    ADC_SelChannel(channel);
    ADCON0bits.GO = 1;      //Starts conversion
    while(ADCON0bits.GO);
    result = (ADRESH<<8) | ADRESL;
    ADCON0bits.ADON = 0;    //turns off ADC
    return result;
}

void ADC_Init(void){
    ADCON0bits.GO = 0;   // starts conversion if set
    ADCON2bits.ADFM = 1; // right justified (MSB conversione sono il bit 1 di ADRESH e il LSB il bit 0 di ADRESL )
    ADCON2bits.ADCS = 3; // clock is derived from internal oscillator 
    return;
}

void SysConfigSleep(void)
{
    // Turn off everything the project won't ever use
    UART2MD = 1;
    UART1MD = 1;
    TMR6MD = 1;
    TMR5MD = 1;    
    TMR4MD = 1;
    TMR2MD = 1;
    MSSP1MD = 1;
    CCP5MD = 1;
    CCP4MD = 1;
    CCP3MD = 1;
    CCP2MD = 1;
    CCP1MD = 1;
    CTMUMD = 1;
    CMP2MD = 1;
    CMP1MD = 1;
    
    //Device enters Sleep mode on SLEEP instruction
    IDLEN = 0;
    
    // Configure as output all unused port pins and set them to low
    //PORT A
    TRISA = 0x00;
    LATA = 0x00;
    
    //PORT B
    TRISBbits.RB4 = OUTPUT;
    TRISBbits.RB5 = OUTPUT;
    TRISBbits.RB6 = OUTPUT;
    TRISBbits.RB7 = OUTPUT;
    LATBbits.LATB4 = LOW;
    LATBbits.LATB5 = LOW;
    LATBbits.LATB6 = LOW;
    LATBbits.LATB7 = LOW;
    
    //PORT C
    TRISCbits.RC0 = OUTPUT;
    TRISCbits.RC1 = OUTPUT;
    TRISCbits.RC3 = OUTPUT;
    TRISCbits.RC4 = OUTPUT;
    //TRISCbits.RC5 = OUTPUT;
    TRISCbits.RC6 = OUTPUT;
    TRISCbits.RC7 = OUTPUT;
    LATCbits.LATC0 = LOW;
    LATCbits.LATC1 = LOW;
    LATCbits.LATC3 = LOW;
    LATCbits.LATC4 = LOW;
    //LATCbits.LATC5 = LOW;
    LATCbits.LATC6 = LOW;
    LATCbits.LATC7 = LOW;
    
    //PORT D
    TRISDbits.RD0 = OUTPUT;
    TRISDbits.RD1 = OUTPUT;
    TRISDbits.RD2 = OUTPUT;
    TRISDbits.RD4 = OUTPUT;
    TRISDbits.RD5 = OUTPUT;
    TRISDbits.RD7 = OUTPUT;
    LATDbits.LATD0 = LOW;
    LATDbits.LATD1 = LOW;
    LATDbits.LATD2 = LOW;
    LATDbits.LATD4 = LOW;
    //LATDbits.LATD5 = LOW;
    //LATDbits.LATD7 = LOW;
    
    //PORT E
    TRISEbits.RE0 = OUTPUT;
    TRISEbits.RE1 = OUTPUT;
    TRISEbits.RE2 = OUTPUT;
    LATEbits.LATE0 = LOW;
    LATEbits.LATE1 = LOW;
    LATEbits.LATE2 = LOW;

}

void LoRaSleep (void) 
{
    // SSPEN disabled; WCOL no_collision; SSPOV no_overflow; CKP Idle:Low, Active:High; SSPM FOSC/4;
    SSP2CON1 = 0x00;
    
    //Disable MSSP2 module
    MSSP2MD = 1;
    
    //Make sure SPI2 pins are not left in floating state during sleep
    //NCS
    RADIO_nCS_ANS = 0;
    RADIO_nCS_TRIS = 0;
    RADIO_nCS_LAT = 1;
    
    //MISO
    ANSD1 = 0;
    TRISD1 = 0;
    LATD1 = 0;
    
    //MOSI
    ANSD4 = 0;
    TRISD4 = 0;
    LATD4 = 0;
    
    //CLK
    ANSD0 = 0;
    TRISD0 = 0;
    LATD0 = 0;
}

void LoRaWakeUp(void) 
{   
    MSSP2MD = 0;    
    SPI2_Initialize();
}

/**
 End of File
*/