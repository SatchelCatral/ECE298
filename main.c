#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include <stdio.h>

int main(void) {
    //Turn off interrupts during initialization
    __disable_interrupt();

    WDT_A_hold(WDT_A_BASE);

    Init_GPIO();    //Sets all pins to output low as a default
    Init_Clock();   //Sets up the necessary system clocks
    Init_LCD();     //Sets up the LaunchPad LCD display

    PMM_unlockLPM5();   //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    unsigned int threshold_GY;  //Threshold value from Green to Yellow
    unsigned int threshold_YO;  //Threshold value from Yellow to Orange
    unsigned int threshold_OR;  //Threshold value from Orange to Red

    unsigned int threshold_DB;  //Threshold value to determine if Buzzer beeps 2 times
    unsigned int threshold_QB;  //Threshold value to determine if Buzzer beeps 4 times

    char *textDisplay = malloc(6 * sizeof(char)); //Only 6 characters can show up on the LCD Display at a time

    __enable_interrupt();

    Setup_Mode(&threshold_GY, &threshold_YO, &threshold_OR, &threshold_DB, &threshold_QB, textDisplay);

    User_Mode(&threshold_GY, &threshold_YO, &threshold_OR, &threshold_DB, &threshold_QB, textDisplay);

    return (0);
}

void Setup_Mode(unsigned int *threshold_GY, unsigned int *threshold_YO, unsigned int *threshold_OR, unsigned int *threshold_DB, unsigned int *threshold_QB, char *textDisplay)
{
    ///* Let's the user know that Bike Sensor is in SETUP MODE
    Display_Text("SETUP");
    __delay_cycles(800000);
    Display_Text("MODE");
    __delay_cycles(800000);
    clearLCD();
    //*/

    char buttonState1 = 0;  //Current button press state (to allow edge detection)
    char buttonState2 = 0;  //Current button press state (to allow edge detection)
    unsigned int capture = 0;   //Holds current sensor reading (used by both front and back sensors)

    int setup = 5;  //The user needs to set up 5 values for their bike sensor
    while (setup > 0)
    {
        Get_Sensor_Data(&capture, textDisplay, setup > 2 ? BACK_SENSOR : FRONT_SENSOR); //continuously capture new sensor readings
        //Buttons SW1 and SW2 are active low (1 until pressed, then 0)

        //Button SW1 Sets the threshold values
        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 1) & (buttonState1 == 0)) //Look for rising edge
        {
            buttonState1 = 1;   //Capture new button state
        }
        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0) & (buttonState1 == 1)) //Look for falling edge
        {
            switch(setup)
            {
                case 5:
                    *threshold_GY = capture;
                    setup--;
                    Display_Text("GY SET");
                    __delay_cycles(800000);
                    Convert_To_String(*threshold_GY, textDisplay);
                    Display_Text(textDisplay);
                    __delay_cycles(800000);
                    clearLCD();
                    break;
                case 4:
                    if (capture < *threshold_GY)
                    {
                        *threshold_YO = capture;
                        setup--;
                        Display_Text("YO SET");
                        __delay_cycles(800000);
                        Convert_To_String(*threshold_YO, textDisplay);
                        Display_Text(textDisplay);
                        __delay_cycles(800000);
                        clearLCD();
                    }
                    else
                    {
                        displayScrollText("INVALID");
                    }
                    break;
                case 3:
                    if (capture < *threshold_YO)
                    {
                        *threshold_OR = capture;
                        setup--;
                        Display_Text("OR SET");
                        __delay_cycles(800000);
                        Convert_To_String(*threshold_OR, textDisplay);
                        Display_Text(textDisplay);
                        __delay_cycles(800000);
                        clearLCD();
                    }
                    else
                    {
                        displayScrollText("INVALID");
                    }
                    break;
                case 2:
                    *threshold_DB = capture;
                    setup--;
                    Display_Text("DB SET");
                    __delay_cycles(800000);
                    Convert_To_String(*threshold_DB, textDisplay);
                    Display_Text(textDisplay);
                    __delay_cycles(800000);
                    clearLCD();
                    break;
                case 1:
                    if (capture < *threshold_DB)
                    {
                        *threshold_QB = capture;
                        setup--;
                        Display_Text("QB SET");
                        __delay_cycles(800000);
                        Convert_To_String(*threshold_QB, textDisplay);
                        Display_Text(textDisplay);
                        __delay_cycles(800000);
                        clearLCD();
                    }
                    else
                    {
                        displayScrollText("INVALID");
                    }
                    break;
                default:
                    break;
            }
            buttonState1 = 0;   //Capture new button state
        }

        //Button SW2 Indicates to the user what threshold value they are currently setting
        if ((GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 1) & (buttonState2 == 0)) //Look for rising edge
        {
            buttonState2 = 1;   //Capture new button state
        }
        if ((GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 0) & (buttonState2 == 1)) //Look for falling edge
        {
            switch(setup)
            {
                case 5:
                    Display_Text("GY    ");
                    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
                    __delay_cycles(800000);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
                    clearLCD();
                    break;
                case 4:
                    Display_Text("YO    ");
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
                    __delay_cycles(800000);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
                    clearLCD();
                    break;
                case 3:
                    Display_Text("OR    ");
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
                    __delay_cycles(800000);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
                    clearLCD();
                    break;
                case 2:
                    Display_Text("DB    ");
                    Buzzer(2);
                    __delay_cycles(800000);
                    clearLCD();
                    break;
                case 1:
                    Display_Text("QB    ");
                    Buzzer(4);
                    __delay_cycles(800000);
                    clearLCD();
                    break;
                default:
                    break;
            }
            buttonState2 = 0;   //Capture new button state
        }
    }
}

void User_Mode(unsigned int *threshold_GY, unsigned int *threshold_YO, unsigned int *threshold_OR, unsigned int *threshold_DB, unsigned int *threshold_QB, char *textDisplay)
{
    unsigned int capture = 0;
    //Enable User Mode until board is reset
    while (1)
    {
        //Since we are using capture for both front and back sensors, we need to reset the capture value
        capture = 0;

        //Busy wait until we get a valid reading from the front sensor
        Get_Sensor_Data(&capture, textDisplay, BACK_SENSOR);
        //Sequential logic to see what LED the sensor should display
        if (capture > *threshold_GY)
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
        }
        else if ((capture < *threshold_GY) && (capture > *threshold_YO))
        {
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
        }
        else if ((capture < *threshold_YO) && (capture > *threshold_OR))
        {
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
        }
        else if ((capture < *threshold_OR))
        {
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
        }

        //Reset capture
        capture = 0;

        //Enter another Busy Wait for a reading from the back sensor
        Get_Sensor_Data(&capture, textDisplay, FRONT_SENSOR);
        if ((capture < *threshold_DB) && (capture > *threshold_QB))
        {
            Buzzer(2);
        }
        else if (capture < *threshold_QB)
        {
            Buzzer(4);
        }
    }
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

void Get_Sensor_Data(unsigned int *capture, char *dest, int sensor)
{
    unsigned int time_a = Read_Sensor(sensor);
    while (time_a == 0) { time_a = Read_Sensor(sensor); }
    unsigned int time_b = Read_Sensor(sensor);
    while (time_b == 0) { time_b = Read_Sensor(sensor); }
    unsigned int time_c = Read_Sensor(sensor);
    while (time_c == 0) { time_c = Read_Sensor(sensor); }

    unsigned int time_avg = (time_a / 6) + ((time_b * 2) / 3) + (time_c / 6);

    Convert_To_String(time_avg, dest);

    Display_Text(dest);
    //__delay_cycles(500000);
    __delay_cycles(250000);

    *capture = time_avg;
}

unsigned int Read_Sensor(int sensor)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
    __delay_cycles(10);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
    __delay_cycles(20);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);

    unsigned int time = 0;
    unsigned int tmp = GPIO_getInputPinValue(sensor == FRONT_SENSOR ? GPIO_PORT_P1 : GPIO_PORT_P2, GPIO_PIN7);

    if (tmp == 1)
    {
        //Start timer in continuous mode sourced by SMCLK
        Timer_A_initContinuousModeParam initContParam = {0};
        initContParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
        initContParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
        initContParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
        initContParam.timerClear = TIMER_A_DO_CLEAR;
        initContParam.startTimer = false;
        Timer_A_initContinuousMode(TIMER_A1_BASE, &initContParam);

        Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);

        while(tmp != 0)
        {
            tmp = GPIO_getInputPinValue(sensor == FRONT_SENSOR ? GPIO_PORT_P1 : GPIO_PORT_P2, GPIO_PIN7);
        }
        Timer_A_stop(TIMER_A1_BASE);
        time = Timer_A_getCounterValue(TIMER_A1_BASE);
    }
    return time;
}

// USE PWM IN THE FUTURE
void Buzzer(int beeps)
{
    int i;
    int j;
    for (i = 0; i < beeps; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
            if (beeps == 2) { __delay_cycles(10); }
            else { __delay_cycles(5); }
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);
            if (beeps == 2) { __delay_cycles(10); }
            else { __delay_cycles(5); }
        }
        __delay_cycles(150000);
    }
}

void Convert_To_String(unsigned int src, char *dest)
{
    memset(dest, 0, 6);
    sprintf(dest, "%d", src);
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

    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN7);
    GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6);
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
