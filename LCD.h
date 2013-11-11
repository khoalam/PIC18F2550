/* 
 * File:   Suntracker.h
 * Author: Khoa Lam
 *
 * Created on October 23, 2013, 3:23 PM
 */

#ifndef SUNTRACKER_H
#define	SUNTRACKER_H
#define RS PORTAbits.RA1        // Register select
#define RW PORTAbits.RA4        // Read Write select
#define E PORTAbits.RA5         // Triger Write or Read
#define DataPort PORTB          // 4 upper bits are used
void send();
void LCD_code(char command);
void putch(char data);
void LCD_init();                // Initilizing in 4 bits mode
#ifdef	__cplusplus
extern "C" {
#endif



#ifdef	__cplusplus
}
#endif

#endif	/* SUNTRACKER_H */
