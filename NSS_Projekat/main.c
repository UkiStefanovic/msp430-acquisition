/**
 * @file main.c
 * @brief MSP430F5529LP-based application for dual-channel ADC data acquisition using interrupts.
 *        UART communication is established for serial data transmission.
 *
 *        This code configures the micro-controller to perform analog-to-digital conversions
 *        on two channels alternately. The ADC conversions are triggered by pressing a button (SW1).
 *        Conversions are performed on channels 0 and 1, and the results are stored in arrays.
 *        The code uses Timer A0 for debouncing the button and Timer A1 for periodic ADC conversions.
 *
 *        UART communication is established using 8N1 format at a baud rate of 9600 to send data over
 *        serial communication. LED indicators (P2.4 and P2.5) are used to signal the start and end
 *        of ADC conversions.
 *
 *        The state machine ensures proper sequencing of ADC conversions and button handling.
 *        The program continuously waits for the user to press SW1 to initiate ADC conversions.
 *        After ten conversions on each channel, the program stops and waits for a new button press.
 *
 *        Additionally, SW2 can be pressed to prematurely stop the data acquisition process.
 *
 *        Author: Uros Stefanovic
 *        Date: 19th January 2024
 */

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "ETF_5529_HAL/hal_7seg.h"

/**
 * @brief Length of sequence
 */
const uint8_t n = 10;

/**
 * @brief Arrays for storing values in RAM memory
 *
 * Counters used for adding values and checking end of acquisition
 */

volatile uint8_t first_channel[n] = { 0 };
volatile uint8_t adc_counter1 = 0;
volatile uint8_t second_channel[n] = { 0 };
volatile uint8_t adc_counter2 = 0;

/**
 * @brief State machine parameters
 *
 * Each boolean parameter is used for a state in the sequence
 */

volatile bool start_acquisition = false;
volatile bool read_first_channel = false;
volatile bool send_first_channel = false;
volatile bool read_second_channel = false;
volatile bool send_second_channel = false;

/**
 * @brief Macros for converting between ASCII and binary coding
 */
#define ASCII2DIGIT(x)      (x - '0')
#define DIGIT2ASCII(x)      (x + '0')

/**
 * @brief Frequency of alternate channel ADC conversion
 *
 * Timer A0 uses ACLK (32,768Hz).
 * Change only the desired frequency.
 */

#define DESIRED_FREQUENCY 6         // Desired interrupt frequency in Hz
#define ACLK_FREQUENCY 32768        // ACLK frequency in Hz

/**
 * @brief Timer A1 period
 *
 * Timer is clocked by ACLK (32768Hz).
 * We want ~32ms period, so use 1023 for CCR0
 */
#define TIMER_WAIT_PERIOD        (1048)  /* ~32ms (31.25ms) */

/**
 * @brief Send byte of data with UART and specify the channel
 *
 * @param channel The channel number (1 or 2) indicating the source of the data.
 * @param data The data value to be sent over UART.
 */
void send(uint8_t channel, uint8_t data)
{
    // Irregular inputs
    if(channel<1 || channel>2)
        return;

    // Send channel from which the data is
    UCA1TXBUF = DIGIT2ASCII(channel);
    while ((UCA1IFG & UCTXIFG) == 0);    // wait until byte is sent
    UCA1TXBUF = ':';
    while ((UCA1IFG & UCTXIFG) == 0);    // wait until byte is sent
    UCA1TXBUF = ' ';
    while ((UCA1IFG & UCTXIFG) == 0);    // wait until byte is sent

    // Send data in 3 digit format (000-255)
    UCA1TXBUF = DIGIT2ASCII((data / 100) % 10);
    while ((UCA1IFG & UCTXIFG) == 0);    // wait until byte is sent
    UCA1TXBUF = DIGIT2ASCII((data / 10) % 10);
    while ((UCA1IFG & UCTXIFG) == 0);    // wait until byte is sent
    UCA1TXBUF = DIGIT2ASCII(data % 10);
    while ((UCA1IFG & UCTXIFG) == 0);    // wait until byte is sent

    // Next line
    UCA1TXBUF = '\n';
    while ((UCA1IFG & UCTXIFG) == 0);    // wait until byte is sent
    UCA1TXBUF = '\r';
    while ((UCA1IFG & UCTXIFG) == 0);    // wait until byte is sent
}

/**
 * @brief Sends a start-of-transmission character sequence over UART.
 */
void sendStartCharacter()
{
    UCA1TXBUF = '@';
    while ((UCA1IFG & UCTXIFG) == 0);    // wait until byte is sent
    UCA1TXBUF = '\n';
    while ((UCA1IFG & UCTXIFG) == 0);    // wait until byte is sent
    UCA1TXBUF = '\r';
    while ((UCA1IFG & UCTXIFG) == 0);    // wait until byte is sent
}

/**
 * @brief Sends an end-of-transmission character sequence over UART.
 */
void sendEndCharacter()
{
    UCA1TXBUF = '#';
    while ((UCA1IFG & UCTXIFG) == 0);    // wait until byte is sent
    UCA1TXBUF = '\n';
    while ((UCA1IFG & UCTXIFG) == 0);    // wait until byte is sent
    UCA1TXBUF = '\r';
    while ((UCA1IFG & UCTXIFG) == 0);    // wait until byte is sent
}

/**
 * @brief Initializes the LED pins and turns them off.
 *
 * This function configures the LED pins as output and ensures
 * that both LEDs (P2.4 and P2.5) are turned off.
 */
void initializeLEDs()
{
    P2DIR |= BIT4;              // Configure P2.4 as out
    P2DIR |= BIT5;              // Configure P2.5 as out
    P2OUT &= ~(BIT4);           // Turn off P2.4
    P2OUT &= ~(BIT5);           // Turn off P2.5
}

/**
 * @brief Initializes buttons for interrupts.
 *
 * This function configures buttons on P1.4 and P1.5 for interrupts.
 */
void initializeButtons()
{
    // configure SW1 to generate interrupt
    P1DIR &= ~(BIT4);             // set P1.4 as input pins
    P1REN |= (BIT4); // This is important because there is no PullUp Resistor on the board
    P1OUT |= (BIT4); // This is important because there is no PullUp Resistor on the board
    P1IES |= (BIT4);              // set P1.4 irq as h->l transition
    P1IFG &= ~(BIT4);             // clear P1.4  IFG
    P1IE |= (BIT4);               // enable P1.4 irq

    // configure SW2 to generate interrupt
    P1DIR &= ~(BIT5);             // set P1.5 as input pins
    P1REN |= (BIT5); // This is important because there is no PullUp Resistor on the board
    P1OUT |= (BIT5); // This is important because there is no PullUp Resistor on the board
    P1IES |= (BIT5);              // set P1.5 irq as h->l transition
    P1IFG &= ~(BIT5);             // clear P1.5  IFG
    P1IE |= (BIT5);               // enable P1.5 irq
}

/**
 * @brief Initializes Timer A0 for timing operations.
 *
 * This function configures Timer A0 with the desired frequency and set/reset mode.
 */
void initializeTimerA0()
{
    TA0CCR0 = ACLK_FREQUENCY / DESIRED_FREQUENCY;           // period is 0.333s
    TA0CCTL1 = OUTMOD_3;                                    // use set/reset mode
    TA0CCR1 = TA0CCR0 / 2;                                  //
    TA0CTL = TASSEL__ACLK | MC__UP;                         // ACLK source, UP mode
}

/**
 * @brief Initializes Timer A1 for debouncing button presses.
 *
 * This function configures Timer A1 for debouncing buttons with the specified period.
 */
void initializeTimerA1()
{
    TA1CCR0 = TIMER_WAIT_PERIOD;
    TA1CCTL0 = CCIE;            // enable CCR0 interrupt
    TA1CTL = TASSEL__ACLK;
}

/**
 * @brief Initializes the Analog-to-Digital Converter (ADC).
 *
 * This function configures the ADC for channel selection and enables ADC conversion.
 */
void initializeADC()
{
    P6SEL |= BIT0;                          // set P6.0 for ADC
    P6SEL |= BIT1;                          // set P6.1 for ADC
    ADC12CTL0 = ADC12ON;                    // turn on ADC
    ADC12CTL1 = ADC12SHS_1 | ADC12CONSEQ_3; // set SHS = 1 (TA0.1 used for SAMPCON) and repeat-single-channel mode
    ADC12MCTL0 = ADC12INCH_0;               // select channel 0
    ADC12MCTL1 = ADC12INCH_1 | ADC12EOS;    // select channel 1 and end of sequence
    ADC12CTL0 |= ADC12ENC;                  // enable conversion
    ADC12IE |= ADC12IE0;                    // enable interrupt for MEM0
    ADC12IE |= ADC12IE1;                    // enable interrupt for MEM0
}

/**
 * @brief Initializes UART communication.
 *
 * This function configures UART communication with 8N1 format and a baud rate of 9600.
 */
void initializeUART()
{
    P4SEL |= BIT4 | BIT5;       // configure P4.4 and P4.5 for uart
    UCA1CTL1 |= UCSWRST;        // enter software reset
    UCA1CTL1 |= UCSSEL__SMCLK;
    UCA1BRW = 109;
    UCA1MCTL = UCBRS_2;         // configure 9600 bps
    UCA1CTL1 &= ~UCSWRST;       // release software reset
    UCA1IFG = 0;                // clear IFG
    UCA1IE |= UCRXIE;           // enable RX and TX interrupt
}



/**
 * @brief Main function for MSP430F5529LP application.
 *
 * This program demonstrates analog data acquisition using the ADC module and
 * communicates the acquired data through UART. It utilizes LEDs, buttons, timers,
 * ADC, and UART peripherals to accomplish the task. The application acquires
 * data alternately from two channels, sending the results through UART. SW2 is
 * used to stop the data acquisition process.
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    // Initialize components
    initializeLEDs();
    initializeButtons();
    initializeTimerA0();
    initializeTimerA1();
    initializeADC();
    initializeUART();

    __enable_interrupt();       // GIE

    while (1)
    {
        // State 1: read data from first channel
        if (start_acquisition && read_first_channel && adc_counter1 < n)
        {
            // Turn on blue LED
            P2OUT |= BIT4;
            // Turn off red LED
            P2OUT &= ~(BIT5);
        }

        // State 2: Send data from first channel
        if (start_acquisition && send_first_channel && adc_counter1 <= n)
        {
            // Turn off blue LED
            P2OUT &= ~(BIT4);
            // Turn on red LED
            P2OUT |= BIT5;

            // Signal start of sequence
            if (adc_counter1 == 1)
            {
                sendStartCharacter();
            }

            // Send data to PC terminal
            send(1, first_channel[adc_counter1-1]);

            // Next state: Read data from second channel
            send_first_channel = false;
            read_second_channel = true;
        }

        // State 3: Read data from second channel
        if (start_acquisition && read_second_channel && adc_counter2 < n)
        {
            // Turn off blue LED
            P2OUT &= ~(BIT4);
            // Turn on red LED
            P2OUT |= BIT5;
        }

        // State 4: Send data from second channel
        if (start_acquisition && send_second_channel && adc_counter2 <= n)
        {
            // Send data to PC terminal
            send(2, second_channel[adc_counter2-1]);

            // Next state: Read data from first channel
            read_first_channel = true;
            send_second_channel = false;

            // Signal end of sequence and stop acquisition
            if (adc_counter2 == n)
            {
                start_acquisition = false;
                sendEndCharacter();

                // Turn off LEDs
                P2OUT &= ~(BIT4);
                P2OUT &= ~(BIT5);
            }
        }

    }
}

/**
 * @brief PORT1 ISR
 *
 * Debounces button press
 */
void __attribute__ ((interrupt(PORT1_VECTOR))) P2ISR(void)
{
    if (((P1IFG & BIT4) != 0) || ((P1IFG & BIT5) != 0))
    {
        // start timerA1
        TA1CTL |= MC__UP;
        P1IFG &= ~BIT4;             // clear P1.4 flag
        P1IFG &= ~BIT5;             // clear P1.5 flag
    }
}

/**
 * @brief TIMERA0 Interrupt service routine
 *
 * ISR debounces P1.4 and P1.5.
 * If P1.4 (SW1) is pressed, start sequence
 * If P1.5 (SW2) is pressed, end sequence
 */
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) CCR1ISR(void)
{
    // SW1 pressed
    if ((P1IN & BIT4) == 0) // check if still pressed
    {
        // Initialize state machine
        start_acquisition = true;
        read_first_channel = true;
        send_first_channel = false;
        read_second_channel = false;
        send_second_channel = false;

        // Initialize counters
        adc_counter1 = 0;
        adc_counter2 = 0;

        // Enable only SW2 press and start acquisition
        P1IE &= ~BIT4;              // disable P1.4 interrupt
        P1IE |= BIT5;           // enable P1.5 interrupt
        ADC12CTL0 |= ADC12ON;  // Enable ADC
        ADC12CTL0 |= ADC12SC;  // Start ADC conversion

    }
    // SW2 pressed
    if ((P1IN & BIT5) == 0) // check if still pressed
    {
        // revert back to initial state
        start_acquisition = false;
        read_first_channel = false;
        send_first_channel = false;
        read_second_channel = false;
        send_second_channel = false;

        // Enable only SW1 press and stop acquisition
        P1IE |= BIT4;           // enable P1.4 interrupt
        P1IE &= ~BIT5;              // disable P1.5 interrupt
        ADC12CTL0 &= ~ADC12ON;  // Disable ADC

        // Signal end of sequence
        sendEndCharacter();
    }

    P1IFG &= ~BIT4;         // clear ifg
    P1IFG &= ~BIT5;         // clear ifg
    TA1CTL &= ~(MC0 | MC1); // stop and clear timer
    TA1CTL |= TACLR;
    return;
}

/**
 * @brief ADC12 Interrupt service routine
 *
 * Read conversion result and handle different states for first and second channels
 */
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12ISR(void)
{
    // Check if ADC12 interrupt is for channel 0 (first channel)
    if (ADC12IV == ADC12IV_ADC12IFG0)
    {
        // Clear interrupt flags
        ADC12IFG &= ~(ADC12IFG0);

        // State 1: Read data from the first channel
        if (start_acquisition && read_first_channel && adc_counter1 < n)
        {
            // Read the higher 8 bits of the ADC conversion result
            first_channel[adc_counter1++] = (ADC12MEM0 >> 4) & 0xff;

            // Next state: send data from the first channel
            read_first_channel = false;
            send_first_channel = true;

            P1IE |= BIT5; // Enable P1.5 interrupt (SW2)
        }
    }

    // Check if ADC12 interrupt is for channel 1 (second channel)
    if (ADC12IV == ADC12IV_ADC12IFG1)
    {
        // Clear interrupt flags
        ADC12IFG &= ~(ADC12IFG1);

        // State 3: Read data from the second channel
        if (start_acquisition && read_second_channel && adc_counter2 < n)
        {
            // Read the higher 8 bits of the ADC conversion result
            second_channel[adc_counter2++] = (ADC12MEM1 >> 4) & 0xff;

            // Next state: Send data from the second channel
            read_second_channel = false;
            send_second_channel = true;

            // Last acquisition in sequence? Enable SW1 button and disable SW2 button and ADC
            if (adc_counter2 == n)
            {
                P1IE |= BIT4;               // Enable P1.4 interrupt (SW1)
                ADC12CTL0 &= ~ADC12ON;      // Disable ADC
                P1IE &= ~BIT5;              // Disable P1.5 interrupt (SW2)
            }
        }
    }
}
