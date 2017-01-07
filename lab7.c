
/*******************************
 * Name: Suman Shrestha
 * Student ID#: 1001162735
 * Lab Day: Monday
 * CSE 3442/5442 - Embedded Systems 1
 * Lab 7 (ABET): Building a PIC18F4520 Standalone Alarm System with EUSART Communication 
 ********************************/

/**
NOTE:     
 *Your comments need to be extremely detailed and as professional as possible
 *Your code structure must be well-organized and efficient
 *Use meaningful naming conventions for variables, functions, etc.
 *Your code must be cleaned upon submission (no commented out sections of old instructions not used in the final system)
 *Your comments and structure need to be detailed enough so that someone with a basic 
           embedded programming background could take this file and know exactly what is going on
 **/


#include <p18F4520.h>
#define _XTAL_FREQ 20000000   //Frequency of oscillator
//other includes
#include <xc.h>
#include <eeprom_routines.h>
#include <stdlib.h>
#include <stdio.h>

//Configuration bits settings code goes here
// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)


// definition of variables to be used throughout the class
unsigned int temp1; // variable to store the not converted temperature reading
double temperature; // double attribute to store the updated temperature as ADIF is triggered
char *password;     // pointer attribute to store the password registered by the user
char *tempPass;     // pointer attribute to store the password entered by the user
char currentTemp[6];    // char array to store the current temperature
int i, status = 0, valid;   // integer vareables
unsigned char *pirState = "INACTIVE";   // initially sets the PIR sensor to 'INACTIVE'
unsigned char *tempState = "INACTIVE";  // initially sets the Tenperature sensor to 'INACTIVE' 
unsigned char *inputType = "Keyboard";  // initially sets the input type ot 'Keyboard'
char degree[] = {176, 'F', '\0'};   // char array that hold degree symbol
char option;        // character variable that stores the input fromm user
char keypadValue;   // character variable that stores input from keypad
char tempThreshold[3];  // character array to hold the temperature threshold
int thresholdTemp = 80; // initially sets the temperature threshold to 80 degree fahrenheit
unsigned int address;  // variable to the address in EEPROM
unsigned int PIRStatusAddress = 0x00;   // address in EEPROM to store the PIR morion sensor status
unsigned int tempStatusAddress = 0x01;  // address in EEPROM to store the temperature sensor status
unsigned int tempThresholdAddress = 0x10;   // address in EEPROM to store temperature threshold
unsigned int inputMethodAddress = 0x03;     // address in EEPROM to store the input method


/*******************************
 * Function prototypes
 ********************************/
void Initial();
void printChar(char *msg);
void getTemperature();
char reception();
char * getPasswordInput();
char * getTempPassword();
void mainScreen();
void setPassword();
void mainMenu();
void passcodeMenu();
void componentStatus();
void interrupt My_ISR_High(void);
void interrupt low_priority My_ISR_Low(void);
void resetScreen();
void temperatureMenu();
void delay();
void tempSetting();
void resetPIR();
void settingsPIR();
void settingsCom();
void settingsTimer();
void pirMenu();
void keypadOnly();
void login();
void write_EEPROM(int, char);
char read_EEPROM(int);
void checkPassword();
void checkStatus();
void getTempThreshold();
void getSingleInput();

/******************************************************************************
 *  void main()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This is the main function where the program initiates.
 ******************************************************************************/
void main() {
    Initial();          // calls the initial function to laod the required settings
    settingsPIR();      // loads the settings for PIR motion sensor alarm   
    settingsCom();      // laods the settings for USB to serial communication
    tempSetting();      // loads the settings for Temperature sensor alarm
    settingsTimer();    // loads the settings for Timer interrupt
    getTemperature();   // gets the current temperature

    // reads the passcode stored in the EEPROM and 
    // sets it to a variable of char *
    for (i = 0, address = 0xE0; i < 4; address++, i++) {
        password[i] = read_EEPROM(address);
    }
    mainScreen();

    while (1) {                 // maintains the loop until the program is halted
        //Main routine code
        if (read_EEPROM(0xE0) == (char) 255) {  // checks if the passcode exists in the EEPROM
            setPassword();                      // if new user ask to set up the passcode
        } else {
            login();                            // else direct the user to login screen
        }

    } //end of while(1)


} //end of void main()

/******************************************************************************
 *  void setttempSettingingsPIR()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method sets the configuration of all the registers needed for Temperature
 *  sensor alarm.
 ******************************************************************************/
void tempSetting() {
    RCONbits.IPEN = 1; // enabling the high and low priority interrupt
    PIR1bits.ADIF = 0; //Clear ADIF flag bit
    IPR1bits.ADIP = 0; //ADC is low Priority
    PIE1bits.ADIE = 0; //Set ADIE enable bit
    INTCONbits.PEIE = 1; //Set PEIE enable bit
    INTCONbits.GIE = 1; //Set GIE enable bit
    PORTBbits.RB5 = 0;  // clears the RB5 bit
}

/******************************************************************************
 *  void settingsCom()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method sets the configuration of all the registers needed for USB to
 *  serial communication.
 ******************************************************************************/
void settingsCom() {
    TRISCbits.RC6 = 0;  //TX is output
    TRISCbits.RC7 = 1;  // RX is input
    SPBRG = 15;         // Low speed 19,200 BAUD for Fosc = 20MHz
    TXSTAbits.SYNC = 0; // Async mode
    TXSTAbits.BRGH = 0; // low speed Baud Rate
    TXSTAbits.TX9 = 0;  // 8-bit transmission
    RCSTAbits.SPEN = 1; // serial port enabled (RX and TX active)
    TXSTAbits.TXEN = 1; // enable transmitter
    RCSTAbits.CREN = 1; // enable continuous receiver
}

/******************************************************************************
 *  void settingsTimer()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method sets the configuration of all the registers needed for setting
 *  Timer interrupt.
 ******************************************************************************/
void settingsTimer() {
    INTCONbits.TMR0IF = 0;  // clears the Timer0 interrupt flag bit
    INTCONbits.TMR0IE = 1;  // sets the Timer0 interrupt enable bit
    INTCON2bits.TMR0IP = 0; // sets the low priority to Timer0 interrupt bit
    T0CON = 0b00000110;     // sets the configuration of Timer0 with the required settings
    
    /* For calculation of preload value
         * d = 1sec (time period)
         * Fosc = 20MHz
         * Fin = Fosc/4 = 5 MHz
         * 16-bit Timer: 0 - 65,536
         * X = d*Fin = 1s*5MHz = 5000000 i.e. 5000000 cycles/ticks occur in a 1s time span
         * Using prescaler to bring down X to fit into the 16-bit Timer register(0 - 65,536)
         * Using Prescaler of 1:128
         * Preload = 65,536 - 5000000/128
         *         = 65,536 - 39063
         *         = 26473
         * dec = 0x6769
         */
    TMR0H = 0x67;       // load TMR0H with high bits of preload value
    TMR0L = 0x6A;       // load TMR0L with low bits of preload value
}

/******************************************************************************
 *  void settingsPIR()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method sets the configuration of all the registers needed for PIR
 *  motion sensor alarm.
 ******************************************************************************/
void settingsPIR() {

    //Interrupt Configuration for motion sensor

    RCONbits.IPEN = 1; //Enabling both high and low priority Interrupts

    INTCONbits.GIE = 1; // enables the global interrupt bit
    INTCONbits.INT0IE = 0; //disable the INT0 interrupt
    INTCON2bits.INTEDG0 = 0; //INT0 on falling edge
    INTCONbits.INT0IF = 0; // clears the INT0IF flag bit
    TRISBbits.RB2 = 0; //PORTB pin 2 as an output to light up Red led
    TRISBbits.RB0 = 1; //PORTB pin 0 as in input
    PORTBbits.RB2 = 0; //RB2 pin to 0
    //For green LED to show alarm system is running
    TRISBbits.RB3 = 0; //set up as output

}

/******************************************************************************
 *  void interrupt My_ISR_High(void)
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method is the interrupt service routine for High priority. For this 
 *  program, PIR sensor Alarm has the highest priority. i.e. INTCONbits.INT0IF.
 *  
 ******************************************************************************/
void interrupt My_ISR_High(void) {
    int check = 0;
    if (INTCONbits.INT0IF == 1 && INTCONbits.INT0IE == 1) { // checks if the INT0IF and INT0IE is set high
        PORTBbits.RB2 = 1;     //Turns on the red LED to indicate the PIR sensor triggered
        INTCONbits.INT0IF = 0; //Reset the interrupt flag 
        resetPIR();            // calls the resetPIR() function)
    }

    do {
        if (valid == 4) {
            check = 0;
            PORTBbits.RB2 = 0;
            printChar("\n\rDisable or Keep enabled the PIR Sensor Alarm?");
            printChar("\n\rEnable = 1 and Disable = 0");
            printChar("\n\rInput Here: ");
            delay();
            // checks if the input from the keypad only mode is on
            if (status == 1 && PORTBbits.RB4 == 1) {
                keypadOnly();               // calls the keypadOnly() function to get the key pressed
                while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done
                TXREG = keypadValue;         // send one 8-byte
                option = keypadValue;        // sets the char variable option with the key pressed
                delay();                     // calls a delay function for 1 sec delay
                while (keypadValue != 'D') { // checks if the 'D' is pressed in keypad, here pressing 'D' functions as enter
                    keypadOnly();
                }
            } else // if not keypad only mode, take input from keyboard
            {
                option = reception();        // calls reception function to get the character input and store it to option
                while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done 
                TXREG = option;              // send one 8-byte
                while (RCREG != 13);         // checks if the user hit 'Enter'
            }

            switch (option) {
                case '1':                   // if the user selects to keep the PIR sensor alarm enabled
                    settingsPIR();          // load the settings for PIR sensor alarm
                    pirState = "ACTIVE";    // sets the status of the PIR to variable of char *
                    INTCONbits.INT0IE = 1; //Enable the INT0 interrupt
                    printChar("\n\r PIR Sensor Alarm Enabled");
                    write_EEPROM(PIRStatusAddress, 'E'); // writes the status of the PIR to EEPROM
                    printChar("\n\rPlease press '0' and 'Enter' to continue: ");
                    delay();
                    break;

                case '0':
                    pirState = "INACTIVE";  // sets the status of the PIR to variable of char *
                    write_EEPROM(PIRStatusAddress, 'D');    // writes the status of the PIR to EEPROM
                    INTCONbits.INT0IE = 0; //Disable the INT0 interrupt
                    printChar("\n\r PIR Sensor Alarm Disabled");
                    printChar("\n\rPlease press '0' and 'Enter' to continue: ");
                    delay();
                    break;

                default:
                    printChar("\n\rInvalid Selection");
                    printChar("\n\r PIR SENSOR ALARM IS KEPT ENABLED");
                    pirState = "INACTIVE";  // sets the status of the PIR to variable of char *
                    write_EEPROM(PIRStatusAddress, 'E');    // writes the status of the PIR to EEPROM
                    settingsPIR();          // load the settings for PIR sensor alarm
                    INTCONbits.INT0IE = 1; //Enable the INT0 interrupt
                    printChar("\n\rPlease press '0' and 'Enter' to continue: ");
                    delay();
                    break;
            }
        } else {
            printChar("\n\rIncorrect Password.... TRY AGAIN!!\n\r");
            printChar("\n\r");
            check = 1;
            delay();
            resetPIR();     //calls the resetPIR function

        }
    } while (check == 1);
}

/******************************************************************************
 *  void resetPIR()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method displays the "PIR sensor motion detected" message to the user
 *  asks the user to enter the passcode to reset the alarm. Once the password
 *  is received, it calls the checkPassword method to verify the password.
 *  
 ******************************************************************************/
void resetPIR() {
    mainScreen();
    printChar("\n\r!!!!!!!!!!!!! PIR SENSOR HAS DETECTED MOTION !!!!!!!!!!!!!!");

    printChar("\r\nEnter the password to reset the alarm: ");
    delay();
    for (i = 0, address = 0xE0; i < 4; address++, i++) {
        password[i] = read_EEPROM(address);
    }

    tempPass = getTempPassword();

    checkPassword();

}

/******************************************************************************
 *  void interrupt low_priority My_ISR_Low(void)
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method is the interrupt service routine for low priority interrupts.
 *  For this program the, Timer0 and AD is the lower priority interrupt. 
 *  If INTCONbits.TMR0IF flag bit or PIR1bits.ADIF flag bit is set to high the
 *  program counter enters this interrupt service routine. If the TMR0IF flag
 *  bit is set it stops the Timer0 and calls the getTemperature function 
 *  to get the current temperature reading from the temperature sensor. Then
 *  starts the timer again.
 *  Once the user enables the temperature sensor,
 *  ADIF flag is set high, it gets the current temperature reading and 
 *  compares it with the temperature threshold. If greater than threshold the
 *  Temperature sensor alarm will be triggered and is indicated by steady Yellow
 *  light. Otherwise, Yellow LED is toggled every time it gets the new
 *  temperature reading.
 ******************************************************************************/
void interrupt low_priority My_ISR_Low(void) {

    if (INTCONbits.TMR0IF == 1) // checks if the TMR0IF is set high
    {
        T0CONbits.TMR0ON = 0; // stops the Timer0
        getTemperature(); // gets current temperature reading
        INTCONbits.TMR0IF = 0; // clears the TMR0IF flag bit
        /* For calculation of preload value
         * d = 1sec (time period)
         * Fosc = 20MHz
         * Fin = Fosc/4 = 5 MHz
         * 16-bit Timer: 0 - 65,536
         * X = d*Fin = 1s*5MHz = 5000000 i.e. 5000000 cycles/ticks occur in a 1s time span
         * Using prescaler to bring down X to fit into the 16-bit Timer register(0 - 65,536)
         * Using Prescaler of 1:128
         * Preload = 65,536 - 5000000/128
         *         = 65,536 - 39063
         *         = 26473
         * dec = 0x6769
         */
        TMR0H = 0x67;
        TMR0L = 0x69;
        T0CONbits.TMR0ON = 1; // starts the Timer0
    }

    if (PIR1bits.ADIF == 1) // checks if the ADIF flag bit is set high
    {
        //ADC conversion done
        //Get result from ADRESH/L
        getTemperature(); // gets the current temperature reading
        if (temperature > thresholdTemp) { // compares the current temperature with the threshold
            PORTBbits.RB5 = 1; // if true, turns on the Yellow in steady state
            T0CONbits.TMR0ON = 0; // stops the timer
            mainScreen();
            printChar("\n\r!!!!!!!!!!!!! Temperature SENSOR HAS DETECTED !!!!!!!!!!!!!!");
            printChar("\n\r");
            printChar("\n\r");
            printChar("\n\rEnter the passcode to reset the alarm and return to main menu.");
            printChar("\n\r");
            printChar("\n\rInput Here: ");
            delay();
            tempPass = getTempPassword(); // gets the passcode to reset alarm 
            checkPassword(); // verifies the passcode

            while (valid != 4) // untial valid maintain loop and keep asking the passcode
            {
                printChar("\n\r Invalid Passcode.. Try Again!!");
                printChar("\n\rEnter passcode: ");
                delay();
                tempPass = getTempPassword();
                checkPassword();
            }

            if (valid == 4) { // if the passcode is valid
                printChar("\n\rDo you want to disable the alarm or keep enabled?");
                printChar("\n\rKeep Enabled = 1    |    Disable = 0");
                printChar("\n\rInput Here: ");
                delay();
                getSingleInput();
                switch (option) {
                    case '1': // if the user selects to keep the Temperature sensor enabled
                        tempState = "ACTIVE";
                        write_EEPROM(tempStatusAddress, 'E'); // writes the temperature sensor status to EEPROM
                        T0CONbits.TMR0ON = 1; // stars the Timer0
                        PORTBbits.RB5 = 0; // turns of the Yellow LED by clearing RB5 bit
                        printChar("\n\rTemperature Sensor has been kept enabled.");
                        break;

                    case '0': // if the user selects to disable the Temperature sensor
                        tempState = "INACTIVE";
                        write_EEPROM(tempStatusAddress, 'D'); // writes the temperature sensor status to EEPROM
                        T0CONbits.TMR0ON = 0; // stops the timer
                        PIE1bits.ADIE = 0; //Set ADIE enable bit
                        PORTBbits.RB5 = 0; // turns of the Yellow LED by clearing RB5 bit
                        printChar("\n\rTemperature Sensor has been disabled.");
                        break;

                    default: // if the user selects nothing
                        printChar("\n\rNo Selection has been made..");
                        tempState = "ACTIVE";
                        write_EEPROM(tempStatusAddress, 'E'); // writes the temperature sensor status to EEPROM
                        T0CONbits.TMR0ON = 1; // stars the Timer0
                        PORTBbits.RB5 = 0; // turns of the Yellow LED by clearing RB5 bit
                        printChar("\n\rTemperature Sensor has been kept enabled.");
                        break;
                }

                delay();
                printChar("\n\r");
                printChar("\n\rDo you want to change the Temperature Threshold.");
                printChar("\n\r");
                printChar("\n\rYes = 1  |   No = 0\n\rInput Here: ");
                delay();
                // checks if the input from the keypad only mode is on
                if (status == 1 && PORTBbits.RB4 == 1) {
                    keypadOnly(); // calls the keypadOnly() function to get the key pressed
                    while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done
                    TXREG = keypadValue; // send one 8-byte
                    option = keypadValue; // sets the char variable option with the key pressed
                    delay(); // calls a delay function for 1 sec delay
                    while (keypadValue != 'D') { // checks if the 'D' is pressed in keypad, here pressing 'D' functions as enter
                        keypadOnly();
                    }
                } else // if not keypad only mode, take input from keyboard
                {
                    option = reception(); // calls reception function to get the character input and store it to option
                    while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done 
                    TXREG = option; // send one 8-byte
                    while (RCREG != 13); // checks if the user hit 'Enter'
                }

                switch (option) {
                    case '1':
                        getTempThreshold();
                        printChar("\n\rPlease press '0' and 'Enter' to continue: ");
                        delay();
                        break;

                    case '0':
                        printChar("\n\rPlease press '0' and 'Enter' to continue: ");
                        delay();
                        break;

                    default:
                        printChar("\n\rNo selection made.");
                        delay();
                        printChar("\n\rPlease press '0' and 'Enter' to continue: ");
                        delay();
                        break;
                }
            }
        } else {
            PORTBbits.RB5 = PORTBbits.RB5^1; // toggles the yellow LED
            __delay_ms(20); // sets the delay of 20 milli seconds
        }

        PIR1bits.ADIF = 0; //clear flag
    }
}

/******************************************************************************
 *  void getSingleInput()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method takes the single character input from the user and displays it
 *  to the user. It will also wait until the user hits 'ENTER"
 ******************************************************************************/
void getSingleInput() {
    // checks if the input from the keypad only mode is on
    if (status == 1 && PORTBbits.RB4 == 1) {
        keypadOnly(); // calls the keypadOnly() function to get the key pressed
        while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done
        TXREG = keypadValue; // send one 8-byte
        option = keypadValue; // sets the char variable option with the key pressed
        delay(); // calls a delay function for 1 sec delay
        while (keypadValue != 'D') { // checks if the 'D' is pressed in keypad, here pressing 'D' functions as enter
            keypadOnly();
        }
    } else // if not keypad only mode, take input from keyboard
    {
        option = reception(); // calls reception function to get the character input and store it to option
        while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done 
        TXREG = option; // send one 8-byte
        while (RCREG != 13); // checks if the user hit 'Enter'
    }
}

/******************************************************************************
 *  void login()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method is displayed when the user registers the passcode for the first
 *  time after programming the PIC or when PIC resets. It asks the user the enter
 *  the 4-digit passcode and calls the checkPassword method to verify it. Once,
 *  the passcode is verified, it allows the user to access the alarm system.
 ******************************************************************************/
void login() {
    printChar("\r\nEnter the password to login: ");
    delay();
    tempPass = getTempPassword(); // gets the passcode input from the user
    checkPassword(); // verifies the passcode]

    if (valid == 4) // if the passcode is valid
    {
        checkStatus(); // calls the checkStatus funtion
        PORTBbits.RB3 = 1; // turns on the green LED by setting it to 1
        resetScreen();
        mainMenu();

    } else // if the passcode is invalid ask user to try again
    {
        printChar("\n\rIncorrect Password Entered!!\n\rTry Again!!");
        delay();
        delay();
        login();
    }

}

/******************************************************************************
 *  void checkPassword()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method verifies the passcode entered by the user. It takes the 4-digit
 *  passcode from the user one by one and checks each of them. if the all the 
 *  digit matches then the passcode is verified.
 ******************************************************************************/
void checkPassword() {
    valid = 0;

    for (i = 0, address = 0xE0; i < 4; address++, i++) {
        //password[i] = read_EEPROM(address);
        if (password[i] == tempPass[i]) {
            valid++;
        }
    }
    if (tempState == "ACTIVE" && valid == 0) {
        valid = 4;
    }
    //valid =4;
}

/******************************************************************************
 *  void checkStatus()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method checks the status of the PIR sensor alarm, temperature sensor
 *  alarm, input method, and temperature threshold once the PIC resets. 
 *  It also loads the data from EEPROM into 
 *  specific variables to display it in the component status screen.
 ******************************************************************************/
void checkStatus() {

    //Checking the values in EEPROM
    if (read_EEPROM(PIRStatusAddress) != (char) 255) {

        if (read_EEPROM(PIRStatusAddress) == 'E') {
            pirState = "ACTIVE";
            settingsPIR();
            INTCONbits.INT0IE = 1; //Enable the INT0 interrupt
        } else if (read_EEPROM(PIRStatusAddress) == 'D') {
            pirState = "INACTIVE";
            INTCONbits.INT0IE = 0; //Disable the INT0 interrupt
        }
    }

    if (read_EEPROM(tempStatusAddress) != (char) 255) {
        if (read_EEPROM(tempStatusAddress) == 'E') {
            tempState = "ACTIVE";
            tempSetting();
            T0CONbits.TMR0ON = 1;
        } else if (read_EEPROM(tempStatusAddress) == 'D') {
            tempState = "INACTIVE";
            T0CONbits.TMR0ON = 0;
            PIE1bits.ADIE = 0; //Set ADIE enable bit
            PORTBbits.RB5 = 0;
        }
    }
    //check stored method of input
    if (read_EEPROM(inputMethodAddress) != (char) 255) {
        if (read_EEPROM(inputMethodAddress) == 'P') //for keypad
        {

            PORTBbits.RB4 = 1;
            status = 1;
            inputType = "Keypad";
        } else if (read_EEPROM(inputMethodAddress) == 'K') //for keyboard
        {
            PORTBbits.RB4 = 0;
            inputType = "Keyboard";
        } else if (read_EEPROM(inputMethodAddress) == 'B') //for keyboard and Keypad
        {
            PORTBbits.RB4 = 1;
            status = 0;
            inputType = "Keyboard AND Keypad";
        }
    }

    if (read_EEPROM(tempThresholdAddress) != (char) 255) //Initially setting up temp threshold to 70
    {
        //write_EEPROM(tempThresholdAddress, 80.0);
        for (i = 0, address = 0x10; i < 4; address++, i++) {
            tempThreshold[i] = read_EEPROM(address);
        }
        thresholdTemp = atoi(tempThreshold);
    }


}

/******************************************************************************
 *  void write_EEPROM()
 *
 *  Input: Integer add, Character c
 *
 *  No returns 
 *
 *  This method takes the EEPROM address as an input and writes the character 
 *  to the address provided one by one.
 ******************************************************************************/
void write_EEPROM(int add, char c) {
    EEADR = add; // load EEADR with EEPROM location destination
    EEDATA = c; // load EEDATA with the data user wants to write
    // set EECON1 configurations for writing
    EEPGD = 0;
    CFGS = 0;
    WREN = 1;
    EECON2 = 0x55; // write 0x55 to EECON2
    EECON2 = 0xAA; // 0xAA to EECON2
    EECON1bits.WR = 1; // sets the WR bit to 1
    while (EECON1bits.WR == 1); // wait until WR bit clears to 0
}

/******************************************************************************
 *  char read_EEPROM()
 *
 *  Input: integer add
 *
 *  returns character
 *
 *  This method reads one character at a time from EEPROM
 ******************************************************************************/
char read_EEPROM(int add) {
    EEADR = add; // load EEADR with EEPROM location source
    // sets EECON1 configurations for reading
    EEPGD = 0;
    CFGS = 0;
    RD = 1;
    return EEDATA; // data is fetched and put into EEDATA and returned
}

/******************************************************************************
 *  void mainScreen()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method displays header to the of the Stanadalone Alarm system
 *  interface.
 ******************************************************************************/
void mainScreen() {
    printChar("\n\r************************************************************");
    printChar("\n\r*                  Alarm System is Connected               *");
    printChar("\n\r*               CSE 3442: Embedded Systems I               *");
    printChar("\n\r*     Lab 7 (ABET) - Standalone PIC with Communication     *");
    printChar("\n\r*                      Suman Shrestha                      *");
    printChar("\n\r************************************************************");

    // reads the stored passcode form EEPROM
    for (i = 0, address = 0xE0; i < 4; address++, i++) {
        password[i] = read_EEPROM(address);
    }
}

/******************************************************************************
 *  void mainMenu()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method displays the main menu to the user once the is able to login to
 *  the standalone alarm system. It allows the user to choose between 
 *  'Passcode options', 'PIR sensor Alarm Options', 'Temperature Sensor
 *  Alarm Options', 'Use only Keypad as input', Use only keyboard as input 
 *  method', 'use both keypad and keyboard as input method' and return to main
 *  menu.
 ******************************************************************************/
void mainMenu() {
    printChar("\n\r");
    printChar("\n\r************************MAIN MENU***************************");
    printChar("\n\r");
    printChar("\n\rSelect one of the following:");
    printChar("\n\r");
    printChar("\n\r         1. Passcode Options");
    printChar("\n\r         2. PIR Sensor Alarm Options");
    printChar("\n\r         3. Temperature Sensor Alarm Options");
    printChar("\n\r         4. Use Keyboard (Terminal) As The Only Input");
    printChar("\n\r         5. Use Keypad As The Only Input ('D' = Enter Key)");
    printChar("\n\r         6. Use BOTH The Keyboard and Keypad As The Input");
    printChar("\n\r");
    printChar("\n\r         0. Refresh Main Menu");
    printChar("\n\r");
    printChar("\n\rInput: ");

    delay();
    // checks if the input from the keypad only mode is on
    if (status == 1 && PORTBbits.RB4 == 1) {
        keypadOnly(); // calls the keypadOnly() function to get the key pressed
        while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done
        TXREG = keypadValue; // send one 8-byte
        option = keypadValue; // sets the char variable option with the key pressed
        delay(); // calls a delay function for 1 sec delay
        while (keypadValue != 'D') { // checks if the 'D' is pressed in keypad, here pressing 'D' functions as enter
            keypadOnly();
        }
    } else // if not keypad only mode, take input from keyboard
    {
        option = reception(); // calls reception function to get the character input and store it to option
        while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done 
        TXREG = option; // send one 8-byte
        while (RCREG != 13); // checks if the user hit 'Enter'
    }

    // switch statements to redirect the user to the functions as selected
    switch (option) {
        case '1':
            passcodeMenu(); // calls passcodeMenu() function
            break;

        case '2':
            pirMenu(); // calls the pirMenu function
            break;

        case '3':
            temperatureMenu(); // calls the temperatureMenu function
            break;

        case '4':
            inputType = "Keyboard"; // sets the char * variable inputType to 'Keyboard'
            write_EEPROM(inputMethodAddress, 'K'); // stores the input method to EEPROM
            PORTBbits.RB4 = 0; // clears the bit RB4 
            status = 0; // sets the integer variable status to 0
            resetScreen(); // resets the screen
            mainMenu(); // returns to main menu
            break;

        case '5':
            inputType = "Keypad"; // sets the char * variable inputType to 'Keypad'
            write_EEPROM(inputMethodAddress, 'P'); // stores the input method to EEPROM
            PORTBbits.RB4 = 1; // sets the bit RB4 to turn on the LED
            status = 1; // sets the integer variable status to 1
            resetScreen(); // resets the screen
            mainMenu(); // returns to main menu
            break;

        case '6':
            inputType = "Keyboard and Keypad"; // sets the char * variable inputType to 'Keyboard and Keypad'
            write_EEPROM(inputMethodAddress, 'B'); // stores the input method to EEPROM
            PORTBbits.RB4 = 1; // sets the bit RB4 to turn on the LED
            status = 0; // sets the integer variable status to 0
            resetScreen(); // resets the screen
            mainMenu(); // returns to main menu
            break;

        case '0':
            resetScreen(); // resets the screen
            mainMenu(); // returns to main menu
            break;

        default:
            printChar("\n\r");
            printChar("\n\r         Incorrect Input options selected!!!");
            delay();
            resetScreen(); // resets the screen
            mainMenu(); // returns to main menu
            break;
    }
}

/******************************************************************************
 *  void passcodeMenu()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method displays the menu related to the passcode once 
 *  the user selects the 'Passcode Options' from the main menu.
 *  This method will allow the user to choose if they want to change the passcode,
 *  and return to main menu. This method calls the checkPassword function to
 *  verify the current passcode before letting the user to enter the new passcode.
 ******************************************************************************/
void passcodeMenu() {
    resetScreen();
    printChar("\n\r");
    printChar("\n\r********************PASSCODE MENU************************");
    printChar("\n\r");
    printChar("\n\r     1. Change Passcode");
    printChar("\n\r");
    printChar("\n\r     0. Return to Main Menu");
    printChar("\n\r");
    printChar("\n\rInput: ");

    delay();
    // checks if the input from the keypad only mode is on
    if (status == 1 && PORTBbits.RB4 == 1) {
        keypadOnly(); // calls the keypadOnly() function to get the key pressed
        while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done
        TXREG = keypadValue; // send one 8-byte
        option = keypadValue; // sets the char variable option with the key pressed
        delay(); // calls a delay function for 1 sec delay
        while (keypadValue != 'D') { // checks if the 'D' is pressed in keypad, here pressing 'D' functions as enter
            keypadOnly();
        }
    } else // if not keypad only mode, take input from keyboard
    {
        option = reception(); // calls reception function to get the character input and store it to option
        while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done 
        TXREG = option; // send one 8-byte
        while (RCREG != 13); // checks if the user hit 'Enter'
    }

    switch (option) {
        case '0': // users selects to return to main menu
            resetScreen();
            mainMenu();
            break;

        case '1': // user selects to change passcode
            resetScreen();
            printChar("\n\r********************PASSCODE MENU************************");
            printChar("\n\r*******************Change Passcode***********************");
            printChar("\n\r");
            printChar("\n\r");
            printChar("\n\rEnter Current Passcode:  ");
            delay();
            tempPass = getTempPassword(); // get current passcode from the user
            checkPassword(); // verifies the passcode

            while (valid != 4) // maintains the loop until the user enters the correct passcode
            {
                printChar("\n\rIncorrect Passcode Entered!!");
                delay();
                printChar("\n\rEnter Current Passcode:  ");
                delay();
                tempPass = getTempPassword(); // asks the user to try again for the correct passcide
                checkPassword(); // verifies the passcode
            }
            if (valid == 4) // if the passcode is verified
            {
                printChar("\n\rEnter New Passcode: ");
                delay();
                password = getPasswordInput(); // gets the new passcode from the user
                printChar("\n\r");
                printChar("\n\rPasscode has been changed successfully.");
                resetScreen(); // resets the screen
                mainMenu(); // returns to main menu
            }
            break;

        default:
            printChar("\n\r");
            printChar("\n\r     Incorrect Input option selected!!!");
            delay();
            resetScreen(); // resets the screen
            mainMenu(); // returns to main menu
            break;
    }

}

/******************************************************************************
 *  void setPassword()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method is called when the PIC18F4520 is programmed for the very first
 *  time. It allows the user to set passcode to access the standalone 
 *  alarm system.
 ******************************************************************************/
void setPassword() {
    printChar("\n\r");
    printChar("\n\rWelcome to the Alarm System");
    printChar("\n\rThis is your first time. So, Let's set up your passcode.");
    printChar("\n\rPlease enter 4-digit passcode: ");
    delay();
    *password = getPasswordInput(); // gets the 4-digit passcode input from the user and stores it to 
    // character pointer 'password'
    while (RCREG != 13); // wait until user hits 'ENTER'
    printChar("\n\rPassword has been registered successfully");
    delay(); // 0.5 sec delay
    mainScreen(); // returns to main menu
    login(); // sends the user to login screen once the password is set
    resetScreen(); // resets the screen
    mainMenu(); // returns to main menu
    componentStatus(); // loads the component status and displays it

    status = 0; // sets the variable integer status to 0;
}

/******************************************************************************
 *  void pirMenu()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method displays the menu related to the PIR sensor alarm once 
 *  the user selects the 'PIR Sensor Alarm Options' from the main menu.
 *  This method will allow the user to choose if they want to enable the 
 *  PIR sensor alarm, disable the V sensor alarm, change the 
 *  temperature alarm threshold and return to main menu. This method will also
 *  perform the selected operations.
 ******************************************************************************/
void pirMenu() {
    resetScreen();
    printChar("\n\r*********************PIR Sensor Alarm ***********************");
    printChar("\n\r");
    printChar("\n\rPlease select your option:");
    printChar("\n\r");
    printChar("\n\r     1. Enable PIR Sensor Alarm ");
    printChar("\n\r     2. Disable PIR Sensor Alarm");
    printChar("\n\r");
    printChar("\n\r     0. Return to main menu");
    printChar("\n\r");
    printChar("\n\rInput Here: ");

    delay();
    // checks if the input from the keypad only mode is on
    if (status == 1 && PORTBbits.RB4 == 1) {
        keypadOnly(); // calls the keypadOnly() function to get the key pressed
        while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done
        TXREG = keypadValue; // send one 8-byte
        option = keypadValue; // sets the char variable option with the key pressed
        delay(); // calls a delay function for 1 sec delay
        while (keypadValue != 'D') { // checks if the 'D' is pressed in keypad, here pressing 'D' functions as enter
            keypadOnly();
        }
    } else // if not keypad only mode, take input from keyboard
    {
        option = reception(); // calls reception function to get the character input and store it to option
        while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done 
        TXREG = option; // send one 8-byte
        while (RCREG != 13); // checks if the user hit 'Enter'
    }

    switch (option) {
        case '1': // users selects to enable the PIR sensor alarm
            write_EEPROM(PIRStatusAddress, 'E'); // writes the status of PIR sensor to EEPROM
            pirState = "ACTIVE"; // sets the PIR status to "ACTIVE"
            settingsPIR(); // calls settingsPIR functon to load the PIR settings
            INTCONbits.INT0IE = 1; // enables the high priority interrupt
            printChar("\n\rPIR Sensor has been enabled.");
            delay();
            resetScreen();
            mainMenu();
            break;

        case '2': // users selects to disable the PIR sensor alarm
            write_EEPROM(PIRStatusAddress, 'D'); // writes the status of PIR sensor to EEPROM
            pirState = "INACTIVE"; // sets the PIR status to "INACTIVE"
            INTCONbits.INT0IE = 0; // disables the high priority interrupt
            PORTBbits.RB3 = 0; // clears of RB3 to turn off the LED
            printChar("\n\rPIR Sensor has been disabled.");
            delay();
            resetScreen();
            mainMenu();
            break;

        case '0': // users selects to return to main menu
            resetScreen();
            mainMenu();
            break;

        default: // users selects an invalid option
            printChar("\n\r");
            printChar("\n\rIncorrect input option!!!");
            delay();
            resetScreen();
            mainMenu();
    }
}

/******************************************************************************
 *  void temperatureMenu()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method displays the menu related to the temperature sensor alarm once 
 *  the user selects the 'Temperature Sensor Alarm Options' from the main menu.
 *  This method will allow the user to choose if they want to enable the 
 *  temperature sensor alarm, disable the temperature sensor alarm, change the 
 *  temperature alarm threshold and return to main menu.
 ******************************************************************************/
void temperatureMenu() {
    resetScreen();
    printChar("\n\r*****************Temperature Sensor Alarm *******************");
    printChar("\n\r");
    printChar("\n\rPlease select you option:");
    printChar("\n\r");
    printChar("\n\r     1. Enable Temperature Sensor Alarm ");
    printChar("\n\r     2. Disable Temperature Alarm");
    printChar("\n\r     3. Change Temperature alarm Threshold");
    printChar("\n\r");
    printChar("\n\r     0. Return to main menu");
    printChar("\n\r");
    printChar("\n\rInput Here: ");

    delay(); // calls delay function for 0.5 sec delay
    // checks if the input from the keypad only mode is on
    if (status == 1 && PORTBbits.RB4 == 1) {
        keypadOnly(); // calls the keypadOnly() function to get the key pressed
        while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done
        TXREG = keypadValue; // send one 8-byte
        option = keypadValue; // sets the char variable option with the key pressed
        delay(); // calls a delay function for 1 sec delay
        while (keypadValue != 'D') { // checks if the 'D' is pressed in keypad, here pressing 'D' functions as enter
            keypadOnly();
        }
    } else // if not keypad only mode, take input from keyboard
    {
        option = reception(); // calls reception function to get the character input and store it to option
        while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done 
        TXREG = option; // send one 8-byte
        while (RCREG != 13); // checks if the user hit 'Enter'
    }

    switch (option) {
        case '1': // actions to perform if user selects to enable temperature sensor alarm
            tempState = "ACTIVE"; // sets the temperature alarm status to 'ACTIVE'
            write_EEPROM(tempStatusAddress, 'E'); // writes the status of the temperature alarm to EEPROM
            T0CONbits.TMR0ON = 1; // starts the Timer0
            ADIE = 1; // enables the AD conversion
            printChar("\n\rTemperature Sensor has been enabled.");
            delay(); // calls delay function for 0.5 sec delay
            break;

        case '2': // actions to perform if user selects to disable temperature sensor alarm
            tempState = "INACTIVE"; // sets the temperature alarm status to 'INACTIVE'
            write_EEPROM(tempStatusAddress, 'D'); // writes the status of the temperature alarm to EEPROM
            T0CONbits.TMR0ON = 0; // stops the Timer0
            PIE1bits.ADIE = 0; //Set ADIE enable bit to 0
            PORTBbits.RB5 = 0; //Set RB5 bit to 0
            printChar("\n\rTemperature Sensor has been disabled.");
            delay();
            break;

        case '3': // actions to perform if user selects to change the temperature alarm threshold
            getTempThreshold(); // calls getTempThreshold() to get the input for temperature threshold from the user
            break;

        case '0': // actions to perform if user selects to return to main menu
            break;

        default: // actions to perform if user selects an invalid menu option
            printChar("\n\r");
            printChar("\n\rIncorrect input option!!!");
            delay();
            resetScreen(); // calls the resetFuntion())
            mainMenu(); // returns to main menu
    }
    resetScreen(); //calls resetFunction to display header
    mainMenu(); // returns to main menu
}

/******************************************************************************
 *  void getTempThreshold()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method takes the two digit user input from the user. It takes the input
 *  as the character, stores them into and character array and converts it to 
 *  integer to store it in an integer variable.
 ******************************************************************************/
void getTempThreshold() {
    printChar("\n\rPlease enter the new Temperature Sensor Alarm Threshold (2 digits): ");
    delay(); // calls delay function for 0.5 sec delay
    tempThresholdAddress = 0x10; // sets the address in EEPROM for storing temperature threshold
    if (status == 1 && PORTBbits.RB4 == 1) // checks if the integer variable status and RB4 is high to take the input from keypad 
    {
        for (i = 0; i < 2; i++) {
            keypadOnly(); // calls the keypadOnly() function to get the key pressed
            tempThreshold[i] = keypadValue; // sets the char variable option with the key pressed
            write_EEPROM(tempThresholdAddress, tempThreshold[i]); // calls the write_EEPROM method to write to EEPROM
            while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done 
            TXREG = keypadValue; // send one 8-byte
            tempThresholdAddress++; // moves the pointer to the next address
            delay(); // calls a delay function for 1 sec delay
        }
        while (keypadValue != 'D') { // checks if the 'D' is pressed in keypad, here pressing 'D' functions as enter
            keypadOnly();
        }
    } else // if not keypad only mode, take input from keyboard
    {
        for (i = 0; i < 2; i++) {
            tempThreshold[i] = reception(); // calls reception function to get the character input and store it to option
            write_EEPROM(tempThresholdAddress, tempThreshold[i]);
            while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done 
            TXREG = tempThreshold[i]; // send one 8-byte
            tempThresholdAddress++; // moves the pointer to the next address
        }
        while (RCREG != 13); // checks if the user hit 'Enter'
    }
    thresholdTemp = atoi(tempThreshold); // converts the temperature in character array to integer
    printChar("\n\r");
    printChar("\n\rNew Temperature Sensor Alarm Threshold has been set..");
    delay(); // calls delay function for 0.5 sec delay
}

/******************************************************************************
 *  void keypadOnly()
 *
 *  No input parameters
 *
 *  No returns 
 *
 *  This method takes the input from the 4X4 matrix numeric keypad and stores
 *  input character in a character varibale named keypadValue. In order to do so,
 *  first of all, all the rows of TRISD ports are set as output and all the 4 
 *  columns are set as input. Then, one row of of the keypad is set high (i.e.
 *  one of the least significant bits of PORTD). Now, the each column is checked
 *  if it is set high(i.e. Most significant bits of PORTD). Once, the cloumn is 
 *  found, the element at the corresponding row and column would be the key
 *  pressed.
 ******************************************************************************/
void keypadOnly() {
    TRISD = 0b11110000; // sets the 4 Most significant bits of TRISD ports as input(for columns)
    // and remaining ports as output (for rows)

    while (1) { // maintains the loop until a key pressed is identified
        PORTD = 1;
        ; // sets the PORTDbits.RD0 to high (i.e. for 1st row)
        if (PORTDbits.RD4 == 1) // checks if RD4 is high
        {
            keypadValue = '1'; // if true the key pressed is '1'
            break;
        } else if (PORTDbits.RD5 == 1) // checks if RD5 is high
        {
            keypadValue = '2'; // if true the key pressed is '2'
            break;
        } else if (PORTDbits.RD6 == 1) // checks if RD6 is high
        {
            keypadValue = '3'; // if true the key pressed is '3'
            break;
        } else if (PORTDbits.RD7 == 1) // checks if RD7 is high
        {
            keypadValue = 'A'; // if true the key pressed is 'A'
            break;
        }
        PORTD = 2;
        if (PORTDbits.RD4 == 1) // checks if RD4 is high
        {
            keypadValue = '4'; // if true the key pressed is '4'
            break;
        } else if (PORTDbits.RD5 == 1) // checks if RD5 is high
        {
            keypadValue = '5'; // if true the key pressed is '5'
            break;
        } else if (PORTDbits.RD6 == 1) // checks if RD6 is high
        {
            keypadValue = '6'; // if true the key pressed is '6'
            break;
        } else if (PORTDbits.RD7 == 1) // checks if RD7 is high
        {
            keypadValue = 'B'; // if true the key pressed is 'B'
            break;
        }
        PORTD = 4;
        if (PORTDbits.RD4 == 1) // checks if RD4 is high
        {
            keypadValue = '7'; // if true the key pressed is '7'
            break;
        } else if (PORTDbits.RD5 == 1) // checks if RD5 is high
        {
            keypadValue = '8'; // if true the key pressed is '8'
            break;
        } else if (PORTDbits.RD6 == 1) // checks if RD6 is high
        {
            keypadValue = '9'; // if true the key pressed is '9'
            break;
        } else if (PORTDbits.RD7 == 1) // checks if RD7 is high
        {
            keypadValue = 'C'; // if true the key pressed is 'C'
            break;
        }
        PORTD = 8;
        if (PORTDbits.RD4 == 1) // checks if RD4 is high
        {
            keypadValue = '*'; // if true the key pressed is '*'
            break;
        } else if (PORTDbits.RD5 == 1) // checks if RD5 is high
        {
            keypadValue = '0'; // if true the key pressed is '0'
            break;
        } else if (PORTDbits.RD6 == 1) // checks if RD6 is high
        {
            keypadValue = '#'; // if true the key pressed is '#'
            break;
        } else if (PORTDbits.RD7 == 1) // checks if RD7 is high
        {
            keypadValue = 'D'; // if true the key pressed is 'D'
            break;
        }
    }
    PORTBbits.RB4 = PORTBbits.RB4^1; // toggles the led connected to RB4 when any key in the keypad is pressed
}

/******************************************************************************
 *  char reception()
 *
 *  No input parameters
 *
 *  Returns: character variable 
 *
 *  This method takes the single character input from the user and returns the
 *  character.
 ******************************************************************************/
char reception() {
    while (PIR1bits.RC1IF == 0); // wait for incoming data
    char val = RCREG; // Reading RCREG clears RCIF flag
    return val; // returns the character val received through
    // serial communication
}

/******************************************************************************
 *  char * getPasswordInput()
 *
 *  No input parameters
 *
 *  returns char *
 *
 *  This method takes the 4-digit passcode as input from the user. Since, it takes
 *  passcode as input, it hides the passcode and displays '*' character as it is
 *  typed. This method does not write the passcode to the EEPROM. It just reads
 *  the passcode entered by the user in the terminal.
 ******************************************************************************/
char * getTempPassword() {
    char inputMsg[4];
    int i;

    // checks if the input from the keypad only mode is on
    if (status == 1 && PORTBbits.RB4 == 1) // maintains loop 4 times to get the 4-digit passcode
    {
        for (i = 0; i < 4; i++) {
            keypadOnly(); // calls the keypadOnly() function to get the key pressed from the keypad
            inputMsg[i] = keypadValue; // sets the char variable option with the key pressed
            delay(); // calls a delay function for 1 sec delay
            printChar("*"); // prints '*' character as passcode is typed by the user
        }
        while (keypadValue != 'D') { // checks if the 'D' is pressed in keypad, here pressing 'D' functions as enter
            keypadOnly(); // calls the keypadOnly() function to get the input from the keypad
        }
    } else // if not keypad only mode, take input from keyboard
    {
        for (i = 0; i < 4; i++) // maintains loop 4 times to get the 4-digit passcode
        {
            inputMsg[i] = reception(); // calls reception function to get the character input and store it to option
            printChar("*"); // prints '*' character as passcode is typed by the user
        }
        while (RCREG != 13); // checks if the user hit 'Enter'
    }
    return inputMsg; // returns the character array with the input passcode
}

/******************************************************************************
 *  char * getPasswordInput()
 *
 *  No input parameters
 *
 *  returns char *
 *
 *  This method takes the 4-digit passcode as input from the user. It also writes
 *  the passcode to the address in EEPROM as it is entered. Since, it takes
 *  passcode as input, it hides the passcode and displays '*' character as it is
 *  typed.
 ******************************************************************************/
char * getPasswordInput() {
    int i;
    address = 0xE0; //sets the address to write the user passcode
    // checks if the input from the keypad only mode is on
    if (status == 1 && PORTBbits.RB4 == 1) {
        for (i = 0; i < 4; i++) // maintains loop 4 times to get the 4-digit passcode
        {
            keypadOnly(); // calls the keypadOnly() function to get the key pressed
            password[i] = keypadValue; // sets the char variable option with the key pressed
            _EEREG_EEPROM_WRITE(address, password[i]); // writes the individual character to the address specified in EEPROM
            delay(); // calls a delay function for 0.5 sec delay
            printChar("*"); // prints '*' character as passcode is typed by the user
            address++; // moves the pointer to the next address
        }
        while (keypadValue != 'D') { // checks if the 'D' is pressed in keypad, here pressing 'D' functions as enter
            keypadOnly(); // calls the keypadOnly() function to get the input from the keypad
        }
    } else // if not keypad only mode, take input from keyboard
    {
        for (i = 0; i < 4; i++) // maintains loop 4 times to get the 4-digit passcode
        {
            password[i] = reception(); // calls reception function to get the character input and store it to option
            _EEREG_EEPROM_WRITE(address, password[i]); // writes the individual character to the address specified in EEPROM
            printChar("*"); // prints '*' character as passcode is typed by the user
            address++; // moves the pointer to the next address 
        }
        while (RCREG != 13); // checks if the user hit 'Enter'
    }
    return password; // returns the passcode as a character array
}

/******************************************************************************
 *  void printChar()
 *
 *  input: char *
 *
 *  no returns
 *
 *  This method takes the character pointer as input and prints the each
 *  character one by one to the terminal via EUSART serial communication
 ******************************************************************************/
void printChar(char *msg) {
    while (*msg) // iterates the loop while there is a chracter remaining in the array
    {
        while (TXSTAbits.TRMT == 0); // wait until possible previous transmission data is done
        TXREG = *msg; // send 8-bit byte
        while (TXSTAbits.TRMT == 1); // wait until transmit shift register status bit is empty
        msg++; // increments the message pointer to next address
    }
}

/******************************************************************************
 *  void componentStatus()
 *
 *  No input parameters
 *
 *  No returns
 *
 *  This method displays the components of the standalone alarm system with its
 *  status. This status is hold and displayed even after the PIC resets
 ******************************************************************************/
void componentStatus() {
    printChar("\n\r");
    printChar("\n\rComponent Statuses");
    printChar("\n\r************************************************************");
    // prints the status of the PIR sensor alarm
    printChar("\n\rPIR Sensor Alarm State:          ");
    printChar(pirState);
    // prints the status of the Temperature sensor alarm
    printChar("\n\rTemperature Alarm State:         ");
    printChar(tempState);
    // prints the current temperature reading in degree Fahrenheit
    printChar("\n\rCurrent Temperature Reading:     ");
    printChar(currentTemp);
    printChar(degree);
    // prints the Temperature Alarm Threshold in degree Fahrenheit
    printChar("\n\rTemperature Alarm Threshold:     ");
    char arr[sizeof (thresholdTemp)]; // initializes a new character array with the size of unsigned int thresholdTemp value
    sprintf(arr, "%d", thresholdTemp); // stores the thresholdTemp to arr
    for (i = 0; i < 2; i++) { // copies the thresholdTemp to character pointer named tempThreshold
        tempThreshold[i] = arr[i];
    }
    tempThreshold[2] = '\0'; // added as last element to the array to check for end of the array
    printChar(tempThreshold);
    printChar(degree);
    // prints the current input method(i.e. Keyboard or Keypad))
    printChar("\n\rCurrent Input Method:            ");
    printChar(inputType);
    printChar("\n\r");
    printChar("\n\r************************************************************");
    printChar("\n\r");
}

/******************************************************************************
 *  void Initial()
 *
 *  No input parameters
 *
 *  No returns
 *
 *  This method initializes the registers to enable different ports for input
 *  and output
 ******************************************************************************/
void Initial() {
    ADCON1 = 0b10001110; // Enable PORTA & PORTE digital I/O pins
    TRISA = 0b11100001; // Set I/O for PORTA
    TRISB = 0b11000000; // Set I/O for PORTB
    TRISC = 0b11010000; // Set I/0 for PORTC
    TRISD = 0b11111111; // Set I/O for PORTD
    TRISE = 0b00000100; // Set I/O for PORTE
    PORTA = 0b00010000; // Turn off all four LEDs driven from PORTA
    PORTBbits.RB4 = 0;

}

/******************************************************************************
 *  void getTemperature()
 *
 *  No input parameters
 *
 *  No returns
 *
 *  This method reads the analog voltage from the temperature sensor TMP36 and
 *  converts it to a temperature of degree degree celsius [10 mV = 1 degree celsius]
 *  and then converts it to degree fahrenheit
 ******************************************************************************/
void getTemperature() {
    ADCON0 = 0b00000001; // sets the configuration of ADCON0 register to take the
    // input from port A0/channel AN0 for FOSC/4
    ADCON1 = 0b00001110; // setting the A/D port configuration to take analog input from AN0
    ADCON2 = 0b10101100; // sets the configuration of ADCON2 register to select
    // the result format as right justified, set the A/D Acquistion time selcet bits
    // and A/D conversion clock select bits
    TRISAbits.TRISA0 = 1; // sets the PORT A0 as input
    ADCON0bits.GO = 1; // sets the current status of A/D conversion as in progress
    while (ADCON0bits.DONE == 1); // do nothing while the A/D conversion as in progress
    temp1 = ADRESH; // stores the right justified high bit of the conversion result
    temp1 = temp1 << 8; // left shifts the value of the external potentiometer by 8
    temp1 += ADRESL; // adds the result with the low bit of the conversion result
    // to make it a 10-bit number
    temperature = temp1;
    temperature = (double) ((temperature / 1023)*5000); //converts the ADC result to a range of 0 to 5000 mV  
    temperature = temperature - 500; // subtracts the offset of 500 mV for TMP36
    temperature = temperature / 10; // converts the mV to degree celsius [10 mV = 1 degree celsius]
    temperature = (temperature)*1.8 + 32; // converts degree celsius to farhenheit, [F = C*1.8 + 32]

    char arr[sizeof (temperature)]; // initializes a new character array with the size of double temperature value
    sprintf(arr, "%2.2f", temperature); // converts the double temperature to character array of length 5
    for (i = 0; i < 5; i++) { // loop to store the temperature in a new character pointer
        currentTemp[i] = arr[i];
    }
    currentTemp[5] = '\0'; // added as last element to the array to check for end of the array
}

/******************************************************************************
 *  void resetScreen()
 *
 *  No input parameters
 *
 *  No returns
 *
 *  This method refreshes the screen to show the new page in the terminal.
 *  It also loads the main screen with header and component statuses.
 ******************************************************************************/
void resetScreen() {
    printChar("\033[2J"); // refresh the screen to new page
    mainScreen(); // displays the main heading
    componentStatus(); // calls componentStatus function to to display
    // statuses of the component
}

/******************************************************************************
 *  void delay()
 *
 *  No input parameters
 *
 *  No returns
 *
 *  This method creates a 0.5 seconds delay
 ******************************************************************************/
void delay() {
    for (i = 0; i < 25; i++) {
        __delay_ms(20); // calls delay function in XC.h to creates 20 milliseconds delay
    }
}
//Be sure to have a blank line at the end of your program
