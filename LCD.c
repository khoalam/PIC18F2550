#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <LCD.h>
#define _XTAL_FREQ 8000000
void send(){
    E = 1;
    //__delay_us(20);
    E = 0;
}
void LCD_code(char command){
    RS = 0;                    // Select Instruction register
    RW = 0;                    // Select write
    if(command == 0b00000001 || command == 0b00000010){
        DataPort = command;
        send();
        DataPort = command<<4;     //4bits mode
        send();
        __delay_ms(1.57);
    }
    else if(command == 0b00111100){
        DataPort = command;
        send();
        DataPort = command<<4;     //4bits mode
        send();
        __delay_ms(4.1);
    }
    else{
        DataPort = command;
        send();
        DataPort = command<<4;     //4bits mode
        send();
        __delay_ms(1);
    }
}
void putch(char data){
    RS = 1;
    DataPort = data;
    send();
    DataPort = data<<4;     //4bits mode
    send();
    __delay_us(600);
}
void LCD_init(){
    LCD_code(0b00100000);       // Function set
    LCD_code(0b00101000);       // 4 Bits input mode
                                // 2 lines
                                // 5x8 font
    LCD_code(0b00000001);       // Clear Display
    LCD_code(0b00000010);       // Return home
    LCD_code(0b00000110);       // Entry mode set
                                // Increment, no display shift
    LCD_code(0b00001100);       // Display ON
                                // Cursor OFF
                                // Cursor blinking OFF
    //LCD_code(0b00111000);       // Function set
                                // 8 Bits input mode
                                // 2 lines
                                // 5x8 font
    LCD_code(0b00101000);       // Function set
                                // 4 Bits input mode
                                // 2 lines
                                // 5x8 font

}
