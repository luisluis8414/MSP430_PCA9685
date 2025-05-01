#include "intrinsics.h"
#include "msp430fr2355.h"
#include <stdint.h>

#define PCA9685_ADDR      0x40

#define LED0_ON_L         0x06
#define LED0_ON_H         0x07
#define LED0_OFF_L        0x08
#define LED0_OFF_H        0x09

#define LED_ON_L(channel)  (LED0_ON_L + 4 * (channel))
#define LED_ON_H(channel)  (LED0_ON_H + 4 * (channel))
#define LED_OFF_L(channel) (LED0_OFF_L + 4 * (channel))
#define LED_OFF_H(channel) (LED0_OFF_H + 4 * (channel))

char data_in; 

// for writing
int data_cnt;

char* packet;
int packet_length;

// simple I2C write, first entry in data is addr, rest is data, so {startaddr, data for startaddr, data for startaddr + 1,....}
void write_I2C(char data[], int length) {
    packet = data;
    packet_length = length;

    UCB0CTLW0 |= UCTR; 
    UCB0TBCNT = length;

    data_cnt = 0;   

    UCB0CTLW0 |= UCTXSTT; // generate start bit
}

void read_I2C(int addr) {
    char MODE1_ADDR[] = {addr};
    write_I2C(MODE1_ADDR, 1);

    LPM3;

    UCB0CTLW0 &= ~UCTR; 

    UCB0TBCNT = 1;

    UCB0CTLW0 |= UCTXSTT; // generate start bit
}


// init, start in write mode
void init_I2C(uint8_t addr) {
    WDTCTL = WDTPW | WDTHOLD;

    //-- setup B0 I2C

    UCB0CTLW0 |= UCSWRST;    // put into SW reset

    UCB0CTLW0 |= UCSSEL_3;  // choose SMCLK

    UCB0BRW = 20;            // sets prescaler to 50khz -- 1Mhz / 20 

    UCB0CTLW0 |= UCMODE_3;  // put into i2c mode
    UCB0CTLW0 |= UCMST;     // put into master mode

    UCB0I2CSA = addr;       // slave adress

    UCB0CTLW1 |= UCASTP_2;  // auto stop mode based on byte counter UCB0TBCNT

    //-- configure ports
    P1SEL1 &= ~BIT3;        // P1.3 = SCL
    P1SEL0 |= BIT3;

    P1SEL1 &= ~BIT2;        // P1.2 = SDA
    P1SEL0 |= BIT2;

    PM5CTL0 &= ~LOCKLPM5;   // turns on I/O

    UCB0CTLW0 &= ~UCSWRST;  // take out of SW reset

    //-- enable IRQs
    UCB0IE |= UCRXIE0 | UCTXIE0 | UCSTPIE;      // enable READ, WRITE, PAUSE Interrupts
    __bis_SR_register(GIE);                     // global interrupt enable
}

void delay_ms(uint16_t ms) {
    while (ms--) {
        __delay_cycles(1000); // Assuming 1 MHz clock, 1000 cycles = 1 ms
    }
}

void delay_s(uint16_t seconds) {
    while (seconds--) {
        delay_ms(1000); // 1000 ms = 1 second
    }
}

void set_servo_position(uint8_t channel, uint16_t position) {
    if(channel > 15){
        return;
    }

    char servo_data[] = {LED_ON_L(channel), 0x00, 0x00, (position & 0xFF), (position >> 8)};
    write_I2C(servo_data, 5);
    LPM3;
}

int main(void) {
    P1DIR |= BIT0;              // P1.0 (LED) as output
    P1OUT &= ~BIT0;             // LED off

    P1DIR &= ~BIT1;             // P1.1 as input (Button)
    P1REN |= BIT1;              // Enable pull resistor
    P1OUT |= BIT1;              // Pull-up resistor

    P1IES |= BIT1;              // High-to-low transition
    P1IFG &= ~BIT1;             // Clear any pending interrupt
    P1IE  |= BIT1;              // Enable interrupt on P1.1
    
    init_I2C(PCA9685_ADDR);

    //-- Set PWM frequency to 50hz
    // https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf page 25
    // round(25,000,000/(4096 × 50)) - 1 = 121 (0x79)
    // has to happen while in SLEEP Mode
    char PRESCALE_DATA[] = {0xFE, 0x79}; 
    write_I2C(PRESCALE_DATA, 2);

    LPM3;

    char MODE1_AI_ALLCALL_DATA[] = {0x00, 0x21};
    write_I2C(MODE1_AI_ALLCALL_DATA, 2);

    LPM3;

    // read_I2C(0x00);

    // LPM3;

    set_servo_position(0, 205); // Min position (adjust as needed)
    set_servo_position(15, 205); // Min position (adjust as needed)

   while (1) {
    if (!(P1IN & BIT1)) { // Button pressed (active low)
        if (!(P1OUT & BIT0)) { // Check if LED is off
            P1OUT |= BIT0;    // Turn on LED
            set_servo_position(0, 460); // Max position
            set_servo_position(15, 460); // Max position
        }
    } else { // Button released
        if (P1OUT & BIT0) { // Check if LED is on
            P1OUT &= ~BIT0;   // Turn off LED
            set_servo_position(0, 205); // Min position
            set_servo_position(15, 205); // Min position
            LPM3;
        }
    }
    delay_ms(50); // Small delay to debounce the button
    }
}

//--------------------------------------------
//-- ISRs
#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void)
{
      switch (__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG)) {
        // case USCI_NONE: break;
        // case USCI_I2C_UCALIFG: break;       // Arbitration lost
        // case USCI_I2C_UCNACKIFG: break;     // NACK received
        // case USCI_I2C_UCSTTIFG: break;      // START condition received
         case USCI_I2C_UCSTPIFG:                  // STOP condition received
            // bic = bit clear
            __bic_SR_register_on_exit(LPM3_bits);
            break;      
        // case USCI_I2C_UCRXIFG3: break;
        // case USCI_I2C_UCTXIFG3: break;
        // case USCI_I2C_UCRXIFG2: break;
        // case USCI_I2C_UCTXIFG2: break;
        // case USCI_I2C_UCRXIFG1: break;
        // case USCI_I2C_UCTXIFG1: break;
        case USCI_I2C_UCRXIFG0:     
            data_in = UCB0RXBUF;
            break;
        case USCI_I2C_UCTXIFG0:
            if (data_cnt == packet_length - 1) {
                char data = packet[data_cnt];
                UCB0TXBUF = data;
                data_cnt = 0;
            }else {
                UCB0TXBUF = packet[data_cnt];
                data_cnt++;
            }
            break;
        // case USCI_I2C_UCBCNTIFG: break;
        // case USCI_I2C_UCCLTOIFG: break;
        // case USCI_I2C_UCBIT9IFG: break;
        default: break;
    }
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    __bic_SR_register_on_exit(LPM3_bits);

    P1IFG &= ~BIT1;         
}
