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

unsigned int ADC_Result_1;
unsigned int ADC_Result_2;
unsigned int ADC_Result_3;
unsigned int ADC_Result_4;   

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

    LPM0;

    UCB0CTLW0 &= ~UCTR; 

    UCB0TBCNT = 1;

    UCB0CTLW0 |= UCTXSTT; // generate start bit
}


// init, start in write mode
void init_I2C(uint8_t addr) {
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

    UCB0CTLW0 &= ~UCSWRST;  // take out of SW reset

    //-- enable IRQs
    UCB0IE |= UCRXIE0 | UCTXIE0 | UCSTPIE;      // enable READ, WRITE, PAUSE Interrupts
    __bis_SR_register(GIE);                     // global interrupt enable
}

void init_ADC(void) {
    P1SEL0 |= BIT1; // P1.1 as ADC input (A1)
    P1SEL1 |= BIT1;

    P1SEL0 |= BIT5; // P1.5 as ADC input (A5)
    P1SEL1 |= BIT5;

    P1SEL0 |= BIT4;  // P1.4 as ADC input (A5)
    P1SEL1 |= BIT4; 

    // P1.6 as ADC input (A6)
    P1SEL0 |= BIT6;
    P1SEL1 |= BIT6;

    // Configure ADC
    ADCCTL0 |= ADCSHT_2 | ADCON; // ADC ON, S&H=16 ADC clks
    ADCCTL1 |= ADCSHP;           // Use sampling timer
    ADCCTL2 &= ~ADCRES;          // Clear resolution bits
    ADCCTL2 |= ADCRES_2;         // 12-bit resolution
    ADCMCTL0 |= ADCINCH_1;       // Input channel A1, Vref=AVCC
    ADCIE |= ADCIE0;             // Enable ADC conversion complete interrupt
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
    LPM0;
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;  

    P1DIR |= BIT0;  // LED-Pin auf Ausgang setzen
    P1OUT &= ~BIT0; // LED ausschalten zu Beginn

    P1DIR &= ~BIT7;             // P1.7 as input (Button)
    P1REN |= BIT7;              // Enable pull resistoir
    P1OUT |= BIT7;              // Pull-up resistor

    P1IES |= BIT7;              // High-to-low transition
    P1IFG &= ~BIT7;             // Clear any pending interrupt
    P1IE  |= BIT7;              // Enable interrupt on P1.7

    init_I2C(PCA9685_ADDR);
    init_ADC();
    

    //-- Set PWM frequency to 50hz
    // https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf page 25
    // round(25,000,000/(4096 × 50)) - 1 = 121 (0x79)
    // has to happen while in SLEEP Mode
    char PRESCALE_DATA[] = {0xFE, 0x79}; 
    write_I2C(PRESCALE_DATA, 2);

    LPM0;

    char MODE1_AI_ALLCALL_DATA[] = {0x00, 0x21};
    write_I2C(MODE1_AI_ALLCALL_DATA, 2);

    LPM0;

    // read_I2C(0xFE);

    // LPM0;

    while (1) {
        // // First: Read Pot 1 (A1)
        ADCCTL0 &= ~ADCENC;  
        ADCMCTL0 = ADCINCH_1; // Select A1
        ADCCTL0 |= ADCENC | ADCSC;
        __bis_SR_register(LPM0_bits | GIE);
        uint16_t servo_position1 = ((uint32_t)(4095 - ADC_Result_1) * 700) / 4095 + 60 ;
        set_servo_position(0, servo_position1);

        // // Second: Read Pot 2 (A5)
        ADCCTL0 &= ~ADCENC;  
        ADCMCTL0 = ADCINCH_5; // Select A5
        ADCCTL0 |= ADCENC | ADCSC;
        __bis_SR_register(LPM0_bits | GIE);
        uint16_t servo_position2 = ((uint32_t)(4095 - ADC_Result_2) * 700) / 4095 + 60;
        set_servo_position(4, servo_position2); 

        ADCCTL0 &= ~ADCENC;  
        ADCMCTL0 = ADCINCH_4; // Select A4
        ADCCTL0 |= ADCENC | ADCSC;
        __bis_SR_register(LPM0_bits | GIE);
        uint16_t servo_position3 = ((uint32_t)(4095 - ADC_Result_3) * 700) / 4095 + 60;
        set_servo_position(8, servo_position3); 

        // ––– Fourth pot: P1.6/A6 –––
        ADCCTL0 &= ~ADCENC;                // disable
        ADCMCTL0  = ADCINCH_6;             // select channel A6
        ADCCTL0 |= ADCENC | ADCSC;         // start conversion
        __bis_SR_register(LPM0_bits | GIE);

        uint16_t servo_position4 = ((uint32_t)(4095 - ADC_Result_4) * 700) / 4095 + 60;
        set_servo_position(11, servo_position4); 

        if (!(P1IN & BIT7)) { // Button pressed (active low)
            if (!(P1OUT & BIT0)) { // Check if LED is off
                P1OUT |= BIT0;    // Turn on LED
                // set_servo_position(0, 460); // Max position
                set_servo_position(3, 350); // Max position
            }
            } else { // Button released
                if (P1OUT & BIT0) { // Check if LED is on
                    P1OUT &= ~BIT0;   // Turn off LED
                    // set_servo_position(0, 210); // Min position
                    set_servo_position(3, 210); // Min position
            }
        }
        // delay_ms(10);
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
            __bic_SR_register_on_exit(LPM0_bits);
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


// ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
{
    switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
    {
        case ADCIV_NONE:
            break;
        case ADCIV_ADCOVIFG:
            break;
        case ADCIV_ADCTOVIFG:
            break;
        case ADCIV_ADCHIIFG:
            break;
        case ADCIV_ADCLOIFG:
            break;
        case ADCIV_ADCINIFG:
            break;
        case ADCIV_ADCIFG:
             if ((ADCMCTL0 & 0x0F) == ADCINCH_1) {
                ADC_Result_1 = ADCMEM0;
            } else if ((ADCMCTL0 & 0x0F) == ADCINCH_5) {
                ADC_Result_2 = ADCMEM0;
            } else if ((ADCMCTL0 & 0x0F) == ADCINCH_4) {
                ADC_Result_3 = ADCMEM0;
            } else if ((ADCMCTL0 & 0x0F) == ADCINCH_6) {
                ADC_Result_4 = ADCMEM0;
            }
            __bic_SR_register_on_exit(LPM0_bits);
            break;
        default:
            break;
    }
}


#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    __bic_SR_register_on_exit(LPM0_bits);

    P1IFG &= ~BIT7;         
}
