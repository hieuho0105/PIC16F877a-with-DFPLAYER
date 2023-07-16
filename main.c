/*
 * File:   main.c
 * Author: HO CONG HIEU
 *
 * Created on June 3, 2023, 7:11 PM
 */
// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 20000000

#define Start_Byte 0x7E
#define Version_Byte 0xFF
#define Command_Length 0x06
#define End_Byte 0xEF
#define Acknowledge 0x00

#define isPush  0
#define buttonPrevious  PORTDbits.RD0
#define buttonPause     PORTDbits.RD1
#define buttonNext      PORTDbits.RD2
#define buttonVolUp     PORTDbits.RD3
#define buttonVolDown   PORTDbits.RD4

#define t1 25
#define t2 10
// Khai báo các chân k?t n?i v?i 74LS595
#define DS  RB0
#define SH  RB2 
#define ST  RB1
#define DSD RB3
#define SHD RB5 
#define STD RB4
#define DB3 RB6

uint8_t isPlaying = 0;
uint8_t volCurrent = 5;

void UART_Init(uint32_t BAUDRATE){     
    TRISCbits.TRISC6 = 0;  //TX
    TRISCbits.TRISC7 = 1;  //RX
    SPBRG = (uint8_t)((_XTAL_FREQ/(64.0*BAUDRATE))-1);
    TXSTA = 0x20;
    RCSTA = 0x90;  
    RCIE  = 1;
}
void UART_Send(char cmd){
    while(TXSTAbits.TRMT==0);
    TXREG = cmd;
}
void execute_CMD(uint8_t CMD, uint8_t Par1, uint8_t Par2){
    uint16_t checksum = -(Version_Byte + Command_Length + CMD + Acknowledge + Par1 + Par2);
    uint8_t highByteOfChecksum = checksum >> 8;
    uint8_t lowByteOfChecksum = checksum ;
    uint8_t Command_line[10] = { Start_Byte, Version_Byte, Command_Length, CMD,
    Acknowledge, Par1, Par2, highByteOfChecksum, lowByteOfChecksum, End_Byte};
    for (uint8_t k = 0; k < 10; k++){
        UART_Send( Command_line[k]);
    }
    __delay_ms(11); 
}
void playNext(){
    execute_CMD(0x01,0,0);
    __delay_ms(500);
}
void playPrevious(){
    execute_CMD(0x02,0,0);
    __delay_ms(500);
}
void play(){
    execute_CMD(0x0D,0,0); 
    __delay_ms(500);
}
void pause(){
    execute_CMD(0x0E,0x0,0x0);
    __delay_ms(500);
}
void setVolume(uint8_t volume){
    execute_CMD(0x06, 0, volume); // Set the volume (0x00~0x30)
    __delay_ms(500);
}
void playFirst(){
    setVolume(volCurrent);
    __delay_ms(200);
    execute_CMD(0x03,0x0,0x05); // Phat bai thu 5
}


float NhietDo(){
    ADCON0bits.ADON = 1; // B?t module ADC
    ADCON0bits.CHS = 0b00001; // Ch?n kênh analog RA0
    ADCON0bits.GO_DONE = 1; // B?t ??u quá trình chuy?n ??i ADC
    while(ADCON0bits.GO_DONE); // Ch? quá trình chuy?n ??i hoàn thành
    unsigned int adc_value = ADRESH << 8 | ADRESL; // ??c giá tr? ADC
    float voltage = (adc_value / 1024.0) * 5.0; // Chuy?n ??i giá tr? ADC sang ?i?n áp
    float temperature = voltage * 1000.0; // Chuy?n ??i thành giá tr? nhi?t ??
    return temperature;
}
void shiftOutD(unsigned char data) {
    for (int i = 0; i < 8; i++) {
        DSD = (data >> (7 - i)) & 0x01;
        SHD = 1;
        __delay_us(1);
        SHD = 0;
    }
}

void displayLEDsD(unsigned char ledsD) {
    STD = 0;
    shiftOutD(ledsD);  // G?i d? li?u c?a IC th? nh?t
    STD = 1;
}

void shiftOut(unsigned char data) {
    for (int i = 0; i < 8; i++) {
        DS  = (data >> (7 - i)) & 0x01;
        SH  = 1;
        __delay_us(1);
        SH  = 0;
    }
}


void displayLEDs(unsigned char leds1, unsigned char leds2) {
    ST  = 0;
    shiftOut(leds2);  // G?i d? li?u c?a IC th? hai
    shiftOut(leds1);  // G?i d? li?u c?a IC th? nh?t
    ST  = 1;
}

void main(void) {
    TRISC = 0;
    TRISD = 0xff;   
    UART_Init(9600);
    ADCON1 = 0x80; // C?u hình chân RA1 là ??u vào analog
    TRISAbits.TRISA1 = 1; // C?u hình chân RA1 là ??u vào
    TRISB = 0; 
    
  unsigned char led1[][7] = {
    {1, 25, 153, 153, 153, 153, 0},
    {3, 19, 19, 19, 19, 0},
    {2, 34, 98, 98, 0},
    {4, 44, 172, 0},
    {5, 53, 0},
    {6, 0}
    };
  unsigned char led2[][7] = {
    {0, 0, 0, 8, 88, 472, 0},
    {0, 0, 1, 11, 107, 0},
    {0, 0, 1, 13, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 0},
    {0, 0}
    };
    unsigned char DL[] = {1, 3, 2, 4, 5, 6};
    int b, c, i;
    __delay_ms(1000);
    playFirst();
    isPlaying = 1;
    while(1){    
        PORTC = 0xff;
          float T = NhietDo();
          if (T < 30) {i = 5;}
          else if (T > 30 && T < 60) {i = 4;}
          else if (T > 60 && T < 90) { i = 3;}
          else if (T> 90 && T < 120) { i = 2;}
          else if (T > 120 && T < 150) {i = 1;}
          else { i = 0;}
          i = 0;
                  c = 0;
                  b = DL[i];
                  DB3 = 0;
                  for (int d = 0; d < 3; d++){
                      int multipliers[] = {1, 8, 8};
                      b = b * multipliers[d];
                      if (b == 256 || b == 320 || b == 384) { DB3 = 1;}
                      displayLEDsD(b);
                      __delay_ms(t1);
                  }
                  int numIterations = 7 - i;
                  while (c < numIterations) {
                      displayLEDs(led1[i][c], led2[i][c]);
                      __delay_ms(t1);
                      c = c + 1;
                    }
        
        if (buttonPause == isPush){
             while(buttonPause == isPush);
            if(isPlaying){
                pause();
                isPlaying = 0;
            }else{
                isPlaying = 1;
                play();
            }
        }
        if (buttonNext == isPush){
            while(buttonNext == isPush);
            if(isPlaying){
                 playNext();
            }
        }
        if (buttonPrevious == isPush){
           while(buttonPrevious == isPush);
           if(isPlaying){
                playPrevious();
            }
        }
        if (buttonVolUp == isPush){
            while(buttonVolUp == isPush);
            if(isPlaying){
                volCurrent += 5;
                if(volCurrent > 30)
                    volCurrent = 30;
                setVolume(volCurrent);
            }
        }
        if (buttonVolDown == isPush){
            while(buttonVolDown == isPush);
            if(isPlaying){
                volCurrent -= 5;
                if(volCurrent < 0)
                    volCurrent = 0;
                setVolume(volCurrent);
            }
        }
    } 
}
