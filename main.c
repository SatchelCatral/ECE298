#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"

int main(void) {
    //Turn off interrupts during initialization
    __disable_interrupt();

    WDT_A_hold(WDT_A_BASE);

    Init_GPIO();    //Sets all pins to output low as a default
    Init_Clock();   //Sets up the necessary system clocks
    Init_LCD();     //Sets up the LaunchPad LCD display

    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    unsigned int threshold_GY;
    unsigned int threshold_YO;
    unsigned int threshold_OR;

    unsigned int threshold_DB;
    unsigned int threshold_QB;

    __enable_interrupt();

    Setup_Mode(&threshold_GY, &threshold_YO, &threshold_OR, &threshold_DB, &threshold_QB);

    // displayScrollText("DOUBLE BEEP");

    // Buzzer(2);

    // displayScrollText("QUAD BEEP");

    // Buzzer(4);

    return (0);
}

/* ASSUMPTION: MSG IS 6 CHARACTERS LONG */
void Display_Text(char *msg)
{
    clearLCD();
    if (msg[0] != ' ')
        showChar(msg[0], pos1);
    if (msg[1] != ' ')
        showChar(msg[1], pos2);
    if (msg[2] != ' ')
        showChar(msg[2], pos3);
    if (msg[3] != ' ')
        showChar(msg[3], pos4);
    if (msg[4] != ' ')
        showChar(msg[4], pos5);
    if (msg[5] != ' ')
        showChar(msg[5], pos6);
}

void Setup_Mode(unsigned int *threshold_GY, unsigned int *threshold_YO, unsigned int *threshold_OR, unsigned int *threshold_DB, unsigned int *threshold_QB)
{
    Display_Text("SETUP");
    __delay_cycles(800000);
    Display_Text("MODE");
    __delay_cycles(800000);
    clearLCD();

    char buttonState1 = 0; //Current button press state (to allow edge detection)
    char buttonState2 = 0; //Current button press state (to allow edge detection)

    int setup = 5;
    while (setup > 0)
    {
        //Buttons SW1 and SW2 are active low (1 until pressed, then 0)
        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 1) & (buttonState1 == 0)) //Look for rising edge
        {
            buttonState1 = 1;                //Capture new button state
        }
        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0) & (buttonState1 == 1)) //Look for falling edge
        {
            switch(setup)
            {
                case 5:
                    *threshold_GY = 0; //arbitrary number - set to read sensor value
                    Display_Text("GY SET");
                    __delay_cycles(1600000);
                    clearLCD();
                    break;
                case 4:
                    *threshold_YO = 0; //arbitrary number - set to read sensor value
                    Display_Text("YO SET");
                    __delay_cycles(1600000);
                    clearLCD();
                    break;
                case 3:
                    *threshold_OR = 0; //arbitrary number - set to read sensor value
                    Display_Text("OR SET");
                    __delay_cycles(1600000);
                    clearLCD();
                    break;
                case 2:
                    *threshold_DB = 0; //arbitrary number - set to read sensor value
                    Display_Text("DB SET");
                    __delay_cycles(1600000);
                    clearLCD();
                    break;
                case 1:
                    *threshold_QB = 0; //arbitrary number - set to read sensor value
                    Display_Text("QB SET");
                    __delay_cycles(1600000);
                    clearLCD();
                    break;
                default:
                    break;
            }
            setup--;
            buttonState1 = 0;                            //Capture new button state
        }

        if ((GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 1) & (buttonState2 == 0)) //Look for rising edge
        {
            buttonState2 = 1;                //Capture new button state
        }
        if ((GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 0) & (buttonState2 == 1)) //Look for falling edge
        {
            switch(setup)
            {
                case 5:
                    Display_Text("GY    ");
                    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
                    __delay_cycles(1600000);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
                    clearLCD();
                    break;
                case 4:
                    Display_Text("YO    ");
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
                    __delay_cycles(1600000);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
                    clearLCD();
                    break;
                case 3:
                    Display_Text("OR    ");
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
                    __delay_cycles(1600000);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
                    clearLCD();
                    break;
                case 2:
                    Display_Text("DB    ");
                    Buzzer(2);
                    __delay_cycles(1600000);
                    clearLCD();
                    break;
                case 1:
                    Display_Text("QB    ");
                    Buzzer(4);
                    __delay_cycles(1600000);
                    clearLCD();
                    break;
                default:
                    break;
            }
            buttonState2 = 0;                            //Capture new button state
        }
    }
}

void Buzzer(int beeps)
{
    int i;
    int j;
    for (i = 0; i < beeps; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
            __delay_cycles(10);
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);
            __delay_cycles(10);
        }
        __delay_cycles(150000);
    }
}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}
