/* 
 * File:   solar_PIC18F2550.c
 * Author: Sajuuk
 *
 * Created on October 9, 2013, 3:48 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <math.h>
#include <LCD.h>
/*
 *
 */

// PIC18F2550 Configuration Bit Settings

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = INTOSCIO_EC// Oscillator Selection bits (Internal oscillator, port function on RA6, EC used by USB (INTIO))
#pragma config FCMEN = ON      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = ON       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting)
#pragma config VREGEN = ON      // USB Voltage Regulator Enable bit (USB voltage regulator enabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = OFF     // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF       // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode enabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)
#define _XTAL_FREQ 8000000

float fZenithAngle;
float fAzimuth;
float fElevation;
char a = 0;
char temp;
signed char h = 3;
signed char m = 0;
signed char s = 0;
char month = 11;
char day = 3;
int year = 2013;

void delay20() {
    __delay_ms(20);
}

int day_of_week(int cyear, char cmonth, char cday) {
    int h;
    int q = cday;
    int m;
    int K = cyear % 100;
    int J = cyear / 100;
    switch (cmonth) {
        case 1:
            m = 13;
            break;
        case 2:
            m = 14;
            break;
        case 3:
            m = 3;
            break;
        case 4:
            m = 4;
            break;
        case 5:
            m = 5;
            break;
        case 6:
            m = 6;
            break;
        case 7:
            m = 7;
            break;
        case 8:
            m = 8;
            break;
        case 9:
            m = 9;
            break;
        case 10:
            m = 10;
            break;
        case 11:
            m = 11;
            break;
        case 12:
            m = 12;
            break;
    }
    h = (q + (13 * (m + 1) / 5) + K + K / 4 + J / 4 + 5 * J) % 7;
    return h;
}

float time_to_decimal(unsigned char hour, unsigned char minute, unsigned char second) {
    float time;
    time = hour + (minute / 60.0) + (second / 3600.0);
    time = time / 24.0;
    return time;
}

float julian_date(unsigned char days, unsigned char months, unsigned int years) {
    long lpart1 = 367L * years;
    long lpart2 = 7 * (years + (months + 9) / 12) / 4;
    long lpart3 = (275L * months / 9) + days;
    /*float fsign = (100*years+months)-190002.5;
    if(fsign>=0){
        fsign=1;
    }
    else{
        fsign=-1;
    }*/
    float fJulianDate = lpart1 - lpart2 + lpart3 + 1721013.5; //- 0.5*fsign + 0.5;
    return fJulianDate;
}

void sun_position(float longtitude, float latitude, int years, char months, char days, signed char hour, char minute, char second, signed char time_zone) {
    char cDayofWeek;
    char cSecondSunDayofMarch = 1;
    char cFirstSundayofNovember = 1;
    char cflag = 0;
    //float fUTC = time_to_decimal(2,0,0)-(time_zone/24);
    float fJDN = julian_date(days, months, years) + time_to_decimal(hour, minute, second)-(time_zone / 24);
    while (cflag < 2) { //Finding second Sunday of March
        cDayofWeek = day_of_week(years, 3, cSecondSunDayofMarch);
        if (cDayofWeek == 1) {
            cflag++;
        }
        if (cflag != 2) {
            cSecondSunDayofMarch++;
        }
    }
    cflag = 0;
    while (cflag < 1) { //Finding first Sunday of November
        cDayofWeek = day_of_week(years, 11, cFirstSundayofNovember);
        if (cDayofWeek == 1) {
            cflag++;
        }
        if (cflag != 1) {
            cFirstSundayofNovember++;
        }
    }
    float fLowerlimit = julian_date(cSecondSunDayofMarch, 3, years) + time_to_decimal(2, 0, 0)-(-8 / 24);
    float fUpperlimit = julian_date(cFirstSundayofNovember, 11, years) + time_to_decimal(2, 0, 0)-(-8 / 24);
    if ((fJDN >= fLowerlimit) && (fJDN <= fUpperlimit) && (time_zone == -8)) {
        hour = hour - 1;
        if (hour == -1) {
            hour = 24;
        }
    }
    /*else{
        hour=hour+1;
        if(hour==24){
            hour=0;
        }
    }*/
    float pi = 3.14159265359;
    float rad = pi / 180.0;
    //float fUTC = time_to_decimal(hour,minute,second)-(time_zone/24);
    float fJDN = julian_date(days, months, years) + time_to_decimal(hour, minute, second)-(time_zone / 24);
    float fJC = (fJDN - 2451545.0) / 36525.0;
    float fMeanLongitude = fmod(280.46646 + fJC * (36000.76983 + fJC * 0.0003032), 360);
    float fMeanAnomaly = 357.52911 + fJC * (35999.05029 - 0.0001537 * fJC);
    float fEccentricEarthOrbit = 0.016708634 - fJC * (0.000042037 + 0.0000001267 * fJC);
    float fSunEquofCtr = sin(fMeanAnomaly * rad)*(1.914602 - fJC * (0.004817 + 0.000014 * fJC)) + sin((2 * fMeanAnomaly) * rad)*(0.019993 - 0.000101 * fJC) + sin((3 * fMeanAnomaly) * rad)*0.000289;
    float fSunTrueLongtitude = fSunEquofCtr + fMeanLongitude;
    float fMeanObliqueEcliptic = 23 + (26 + ((21.448 - fJC * (46.815 + fJC * (0.00059 - fJC * 0.001813)))) / 60) / 60;
    float fObliqueCorrect = fMeanObliqueEcliptic + 0.00256 * cos((125.04 - 1934.136 * fJC) * rad);
    float fY = tan((fObliqueCorrect / 2) * rad) * tan((fObliqueCorrect / 2) * rad);
    float fEquationofTime = fY * sin(2 * fMeanLongitude * rad) - 2 * fEccentricEarthOrbit * sin(fMeanAnomaly * rad) + 4 * fEccentricEarthOrbit * fY * sin(fMeanAnomaly * rad) * cos(2 * fMeanLongitude * rad) - 0.5 * fY * fY * sin(4 * fMeanLongitude * rad);
    fEquationofTime = fEquationofTime - 1.25 * fEccentricEarthOrbit * fEccentricEarthOrbit * sin(2 * fMeanAnomaly * rad);
    fEquationofTime = 4 * (fEquationofTime / rad);
    float fTrueSolarTime = time_to_decimal(hour, minute, second)*1440.0 + fEquationofTime + 4.0 * longtitude - 60.0 * time_zone;
    if (fTrueSolarTime < 0.0) {
        fTrueSolarTime = 1440 + fTrueSolarTime;
    }

    float fHourAngle;
    float fSunAppLongtitude = fSunTrueLongtitude - 0.00569 - 0.00478 * sin((125.04 - 1934.136 * fJC) * rad);
    float fSunDeclination = asin(sin(fObliqueCorrect * rad) * sin(fSunAppLongtitude * rad));
    if (fTrueSolarTime < 0.0) {
        fHourAngle = fTrueSolarTime / 4 + 180;
    } else {
        fHourAngle = fTrueSolarTime / 4 - 180;
    }
    fZenithAngle = acos(sin(latitude * rad) * sin(fSunDeclination) + cos(latitude * rad) * cos(fSunDeclination) * cos(fHourAngle * rad)) / rad;
    fElevation = 90 - fZenithAngle;
    if (fHourAngle > 0.0) {
        fAzimuth = fmod(acos((sin(latitude * rad) * cos(fZenithAngle * rad) - sin(fSunDeclination)) / (cos(latitude * rad) * sin(fZenithAngle * rad))) / rad + 180, 360);
    } else {
        fAzimuth = fmod(540 - (acos(((sin(latitude * rad) * cos(fZenithAngle * rad)) - sin(fSunDeclination)) / (cos(latitude * rad) * sin(fZenithAngle * rad)))) / rad, 360);
    }
}
unsigned int ADCresult = 0;
unsigned int ctemp;

void ADCstart() {
    ADCON0 = 0b00001001; // Enable ADC module
    ADCON1 = 0b00001100; // Use Vdd and Vss as reference
    // AN0,AN1,AN2 enable
    ADCON2 = 0b10001100; // Fosc/4
    //__delay_ms(1);
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1) {
    }



    ADCON1 = 0b00001111; // disable AN pins
    ADCresult = 0;
    ADCresult = ADRESH;
    ADCresult = ADCresult << 8;
    ctemp = ADRESL;
    ADCresult = (ADCresult | ctemp);
    LCD_code(0b10001001);
    printf("AN:%d   ", ADCresult);
    if ((ADCresult >= 500) && (ADCresult <= 524)) {
        PORTCbits.RC2 = 1;
    } else {
        PORTCbits.RC2 = 0;
    }
}

void ADC(char channel) {
    switch (channel) {
        case 0:
            ADCON0 = 0b00000001; // Channel 0
            break;
        case 1:
            ADCON0 = 0b00000101; // Channel 1
            break;
        case 2:
            ADCON0 = 0b00001001; // Channel 2
            break;
        case 3:
            ADCON0 = 0b00001101; // Channel 3
            break;
    }
    ADCON1 = 0b00001011; // Use Vdd and Vss as reference
    // AN0,AN1,AN2,AN3 enable
    ADCON2 = 0b10001100; // Fosc/4
    __delay_ms(20);
    __delay_ms(20);
    __delay_ms(20);
    __delay_ms(20);
    __delay_ms(20);
    __delay_ms(20);
    __delay_ms(20);
    __delay_ms(20);
    ADCON0bits.GO = 1; // Start connversion
    while (ADCON0bits.GO == 1) {
    }

    ADCON1 = 0b00001111; // disable AN pins
    ADCresult = 0;
    ADCresult = ADRESH;
    ADCresult = ADCresult << 8;
    //ctemp = ADRESL;
    ADCresult = (ADCresult | ADRESL);
}

void delay200() {
    delay20();
    delay20();
    delay20();
    delay20();
    delay20();
    delay20();
    delay20();
    delay20();
    delay20();
    delay20();
}

INIT() {
    ADON = 0; // Disable ADC module
    ADCON1 = 0b00001111; // Dissable all Analog pins
    TRISA = 0b11001101; // PORTA is input
    TRISB = 0b00001111; // PORTB is output
    TRISC = 0b11111011; // RC2 is output

    OSCCON = 0b01110010; // 8MHz Internal OSC, Device enter sleep mode when SLEEP is issued
    CCP1CON = 0b00000000; // CCP1 disable
    CCP2CON = 0b00000000; // CCP2 disable
    CMCON = 0x07; // Comparator disable
    E = 0;
    RW = 0;
    INTCON = 0b11000000; // Enable Interrupts
    TMR0IE = 1; // Timer0 overflow interrupt enable
    T0CON = 0b00000101; // Prescaler 1:64
    PORTCbits.RC2 = 0;
}
unsigned int Azimuth;
signed int Elevation;
//char b = 1;

void button1() {
    TMR0ON = 0;
    while (PORTEbits.RE3 == 0) {
        h++;
        if (h == 25) {
            h = 1;
        }
        LCD_code(0b10000000);
        printf("%02d:%02d:%02d ", h, m, s);
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
    }
    TMR0ON = 1;
}

void button2() {
    TMR0ON = 0;
    while (PORTAbits.RA0 == 0) {
        m++;
        if (m == 60) {
            m = 0;
        }
        LCD_code(0b10000000);
        printf("%02d:%02d:%02d ", h, m, s);
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
    }
    TMR0ON = 1;
}
char I2C_data;

void interrupt timing() {
    a = 1;
    //TMR0IF = 0;
    if (SSPIF == 1) {
        I2C_data = SSPBUF;
        LCD_code(0b11000000); // Line 2
        printf("Received:%d    ", I2C_data);
        SSPIF = 0;
    }
    if (TMR0IF == 1) {
        TMR0H = 0x85; //Insert 34286 into TMR0 to get 1s interrupt
        TMR0L = 0xEE;
        s++;
        if (s == 60) {
            s = 0;
            m++;
            if (m == 60) {
                m = 0;
                h++;
                if (h == 25) {
                    h = 1;
                }
            }
        }
        TMR0IF = 0; // Clear the flag
    }
    //LCD_code(0b10000000);       // Line 1
    //printf("%02d:%02d:%02d ",h,m,s);
}
char program_branch = 0;
char c, d, e, f = 0;

void I2C_write(unsigned char data) {
    SEN = 1;
    while (SEN == 1) {
    }

    SSPBUF = 0b10110000; // Address, and Write
    while (BF == 1) {
    }
    while (ACKSTAT == 1) {
    }
    __delay_us(20);
    //delay20();
    SSPBUF = data;
    while (BF == 1) {
    }
    while (ACKSTAT == 1) {
    }
    __delay_us(20);
    //delay20();
    PEN = 1;
    while (PEN == 1) {
    }
}

void I2C_read() {
    SEN = 1;
    while (SEN == 1) {
    }

    SSPBUF = 0b10110001; // Address, and Write
    while (BF == 1) {
    }
    while (ACKSTAT == 1) {
    }
    __delay_us(20);
    //delay20();
    RCEN = 1;
    while (RCEN == 1) {
    }
    __delay_us(20);
    //ACKDT=0;
    //__delay_us(200);
    ACKDT = 1;
    __delay_us(20);
    ACKEN = 1;
    while (ACKEN == 1) {
    }
    __delay_us(200);
    //delay20();
    PEN = 1;
    while (PEN == 1) {
    }
}

void main() {
    //sun_position(-121.98,37.39,year,month,day,h,m,s,-8);

    INIT();
    LCD_init();
    LCD_code(0b10000000); // Line 1
    printf("PIC18F2550 8MHz");
    LCD_code(0b11000000); // Line 2
    printf("Solar Tracker");

    TMR0H = 0x0B; // to achieve approximately 2s interrupt
    TMR0L = 0xDC; // Add 3036 into Timer0 16 bits registers
    a = 0;
    TMR0ON = 1;
    while (a == 0) {
    }
    LCD_code(0b00000001); // Clear display
    TMR0ON = 0;

    while (f == 0) {
        LCD_code(0b10000000); // Line 1
        printf("Select Mode:");
        LCD_code(0b11000000); // Line 2
        switch (program_branch) {
            case 0:
                printf("Solar Tracker      ");
                break;
            case 1:
                printf("Power Management   ");
                break;
            case 2:
                printf("I2C Test           ");
                break;
        }
        if (PORTEbits.RE3 == 0) {
            program_branch++;
            if (program_branch == 3) {
                program_branch = 0;
            }
        }
        if (PORTAbits.RA0 == 0) {
            f = 1;
        }
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();
        delay20();

    }

    switch (program_branch) {
        case 0:
            LCD_code(0b00000001); // Clear display
            while (c == 0) {
                LCD_code(0b10000000); // Line 1
                printf("Enter month: %d    ", month);
                if (PORTEbits.RE3 == 0) {
                    month++;
                    if (month == 13) {
                        month = 1;
                    }
                }
                if (PORTAbits.RA0 == 0) {
                    c = 1;
                }
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
            }
            LCD_code(0b00000001); // Clear display
            while (d == 0) {
                LCD_code(0b10000000); // Line 1
                printf("Enter day: %d    ", day);
                if (PORTEbits.RE3 == 0) {
                    day++;
                    if (day == 32) {
                        day = 1;
                    }
                }
                if (PORTAbits.RA0 == 0) {
                    d = 1;
                }
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
            }
            LCD_code(0b00000001); // Clear display
            while (e == 0) {
                LCD_code(0b10000000); // Line 1
                printf("Enter year: %d", year);
                if (PORTEbits.RE3 == 0) {
                    year++;
                    if (year == 2020) {
                        year = 2013;
                    }
                }
                if (PORTAbits.RA0 == 0) {
                    e = 1;
                }
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
                delay20();
            }
            TMR0ON = 1;
            LCD_code(0b00000001); // Clear display
            while (1) {
                LCD_code(0b10000000); // Line 1
                printf("%02d:%02d:%02d", h, m, s);
                sun_position(-121.98, 37.39, year, month, day, h, m, s, -8);
                LCD_code(0b11000000); // Line 2
                Azimuth = fAzimuth;
                printf("Az:%d.", Azimuth);
                Azimuth = (fAzimuth - Azimuth)*100;
                printf("%02d", Azimuth);
                putch(0b11011111);
                printf(" ");
                Elevation = fElevation;
                LCD_code(0b11001010);
                printf("E:%d", Elevation);
                //Elevation = (abs(fElevation)-abs(Elevation))*100;
                //printf("%02d",Elevation);
                putch(0b11011111);
                printf("   ");


                ADCstart();
                if (PORTEbits.RE3 == 0) {
                    button1();
                }
                if (PORTAbits.RA0 == 0) {
                    button2();
                }
            }
            break;
        case 1:
            LCD_code(0b00000001); // Clear display
            while (1) {
                ADC(3);
                float R1 = 46800;
                float R2 = 17920;
                float Rconstant = R2 / (R1 + R2);
                float Vout;
                float Vin;
                float Voltage_division = 0.0048828;
                float Current;
                Vout = ADCresult*Voltage_division;
                Vin = Vout / Rconstant;
                Current = (Vin / (R1 + R2))*1000;
                unsigned int integer;
                unsigned int decimal;
                integer = Vin;
                Vin = Vin - integer;
                decimal = Vin * 100;
                LCD_code(0b10000000); // Line 1
                printf("Voltage:%d.%02dV     ", integer, decimal);
                LCD_code(0b11000000); // Line 2
                integer = Current;
                Current = Current - integer;
                decimal = Current * 100;
                printf("Current:%d.%02dmA     ", integer, decimal);

                if (PORTEbits.RE3 == 0) {
                    asm("RESET");
                }
            }
            break;
        case 2:
            LCD_code(0b00000001); // Clear display
            //PEIE = 1; // Peripheral Interrupt Enable
            //SSPIE = 1; // Master Synchronous Serial Port Interrupt Enable
            TRISBbits.TRISB1 = 1; // SCL is output
            TRISBbits.TRISB0 = 1; // SDA is input
            /*SSPSTAT = 0b10000000;
            SSPCON1 = 0b00110110; // Slave mode, 7 bits address, no interrupt(for now)
            SSPCON2 = 0x00; // No masking period
            SSPADD = 0xAE; // Address is AE*/

            SSPSTAT = 0b10000000;
            SSPCON1 = 0b00101000; // I2C Master Mode
            SSPADD = 19; // 100KHz
            RCEN = 1;
            LCD_code(0b10000000); // Line 1
            printf("I2C Test");

            while (1) {

                unsigned char i;
                for (i = 0; i < 16; i++) {
                    I2C_write(i);
                    LCD_code(0b11000000); // Line 2
                    printf("TX: %d ", i);
                    I2C_read();
                    LCD_code(0b11000111);
                    printf(" RX: %d  ", SSPBUF);
                    __delay_ms(20);
                }

            }
            break;
    }

}
