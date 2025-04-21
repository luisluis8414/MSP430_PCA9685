#include "intrinsics.h"
#include <msp430.h>
#include <stdint.h>

#define PCA9685_ADDR      0x40

#define LED0_ON_L         0x06
#define LED0_ON_H         0x07
#define LED0_OFF_L        0x08
#define LED0_OFF_H        0x09
// #define LED_ON_L(n)       (LED0_ON_L  + 4*(n))
// #define LED_ON_H(n)       (LED0_ON_H  + 4*(n))
// #define LED_OFF_L(n)      (LED0_OFF_L + 4*(n))
// #define LED_OFF_H(n)      (LED0_OFF_H + 4*(n))

#define PCA9685_MODE1     0x00
#define PCA9685_MODE2     0x01
#define PCA9685_PRESCALE  0xFE

#define MODE1_RESTART     0x80
#define MODE1_SLEEP       0x10
#define MODE1_AI          0x20
#define MODE2_OUTDRV      0x04

#define SERVO_MIN         150   // ≈1 ms (0°)
#define SERVO_MAX         600   // ≈2 ms (180°)

// Tx ist send mode, Rx ist write mode

unsigned int TXBUF;	

#define MAX_I2C_BYTES 8

volatile uint8_t tx_data[MAX_I2C_BYTES];
volatile uint8_t tx_index = 0;
volatile uint8_t tx_length = 0;



void I2C_Init(int addr){
    UCB0CTLW0 |= UCSWRST;       // Put eUSCI_B0 into software reset

    //-- Configure uUSCI_B0-------------------------------------------
    UCB0CTLW0 |= UCSSEL_3;      // Choose BRCLK = SMCLK = 1MHz
    UCB0BRW = 10;               // Divide BRCLK by 10 for SCL = 100kHz

    UCB0CTLW0 |= UCMODE_3;      // Put into I2C Mode
    UCB0CTLW0 |= UCMST;         // Put into Master Mode
    UCB0CTLW0 |= UCTR;          // Put into Tx Mode
    UCB0I2CSA = addr;           // Slave Address = 0x40

    UCB0CTLW1 |= UCASTP_2;      // Auto stop when UCB0TBCNT reached
    UCB0TBCNT =0x01;            // Send 1 byte of data

    //-- Configure Ports----------------------------------------------
    P1SEL1 &= ~BIT3;            // P1.3 = SCL
    P1SEL0 |=  BIT3;

    P1SEL1 &= ~BIT2;            // P1.2 = SDA
    P1SEL0 |=  BIT2;

    PM5CTL0 &= ~LOCKLPM5;       // Turn on GPIO
    //-- Take eUSCI_B0 out of SW reset---------------------------------
    UCB0CTLW0 &= ~UCSWRST;      // Put eUSCI_B0 out of software reset

    UCB0IE |= UCTXIE0;          // Enable I2C_B0 Tx IRQ
    __enable_interrupt();
}

void i2c_write_register_interrupt(uint8_t reg, uint8_t value) {
    while (UCB0CTLW0 & UCTXSTP);       // Warten falls vorheriger STOP läuft

    tx_data[0] = reg;
    tx_data[1] = value;
    tx_index = 0;
    tx_length = 2;

    UCB0TBCNT = tx_length;             // Zwei Bytes senden
    UCB0CTLW0 |= UCTR | UCTXSTT;       // TX mode, Start senden
}

void pca9685_set_pwm_interrupt(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t base = LED0_ON_L + 4 * channel;

    while (UCB0CTLW0 & UCTXSTP);  // auf vorherigen STOP warten

    tx_index = 0;
    tx_data[0] = base;         // Start-Register
    tx_data[1] = on & 0xFF;
    tx_data[2] = on >> 8;
    tx_data[3] = off & 0xFF;
    tx_data[4] = off >> 8;
    tx_length = 5;

    UCB0TBCNT = tx_length;
    UCB0CTLW0 |= UCTR | UCTXSTT;
}



int main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    I2C_Init(PCA9685_ADDR);

    __delay_cycles(10000);

    // PCA9685 Setup
    i2c_write_register_interrupt(PCA9685_MODE1, MODE1_AI | MODE1_SLEEP);
    __delay_cycles(100000);  // Warte, damit Chip bereit ist

    i2c_write_register_interrupt(PCA9685_PRESCALE, 121); // 50 Hz
    __delay_cycles(100000);

    i2c_write_register_interrupt(PCA9685_MODE1, MODE1_AI); // Wake up
    __delay_cycles(100000);

    i2c_write_register_interrupt(PCA9685_MODE2, MODE2_OUTDRV);
    __delay_cycles(100000);

    // Servo bei LED0 auf Mittelstellung
    pca9685_set_pwm_interrupt(0, 0, 375);  // 1.5ms Puls → 90°

    while (1) {
        __delay_cycles(1000000);
    }
}


#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void) {
    switch (__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG)) {
        case USCI_NONE: break;
        case USCI_I2C_UCALIFG: break;       // Arbitration lost
        case USCI_I2C_UCNACKIFG:            // NACK received
            P1OUT |= BIT0;
            break;    
        case USCI_I2C_UCSTTIFG: break;      // START condition received
        case USCI_I2C_UCSTPIFG: break;      // STOP condition received
        case USCI_I2C_UCRXIFG3: break;
        case USCI_I2C_UCTXIFG3: break;
        case USCI_I2C_UCRXIFG2: break;
        case USCI_I2C_UCTXIFG2: break;
        case USCI_I2C_UCRXIFG1: break;
        case USCI_I2C_UCTXIFG1: break;
        case USCI_I2C_UCRXIFG0: break;
        case USCI_I2C_UCTXIFG0:
            if (tx_index < tx_length) {
                UCB0TXBUF = tx_data[tx_index++];
            }
            break;
        case USCI_I2C_UCBCNTIFG: break;
        case USCI_I2C_UCCLTOIFG: break;
        case USCI_I2C_UCBIT9IFG: break;
        default: break;
    }
}
