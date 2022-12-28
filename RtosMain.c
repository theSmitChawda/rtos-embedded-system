//SmitChawda//12062022
//RTOS Project

#include <xc.h>
#include <salvo.h>
#include <stdint.h>
#include <stdbool.h>


#define _XTAL_FREQ  8000000UL

// CONFIG2 WORD
#pragma config POSCMOD = XT         // Primary Oscillator Select->XT Oscillator mode selected
#pragma config OSCIOFNC = OFF       // Primary Oscillator Output Function->OSC2/CLKO/RC15 functions as CLKO (FOSC/2)
#pragma config FCKSM = CSDCMD       // Clock Switching and Monitor->Clock switching and Fail-Safe Clock Monitor are disabled
#pragma config FNOSC = PRI          // Oscillator Select->Primary Oscillator (XT, HS, EC)
#pragma config IESO = ON            // Internal External Switch Over Mode->IESO mode (Two-Speed Start-up) enabled

// CONFIG1 WORD
#pragma config WDTPS = PS32768      // Watchdog Timer Postscaler->1:32768
#pragma config FWPSA = PR128        // WDT Prescaler->Prescaler ratio of 1:128
#pragma config WINDIS = ON          // Watchdog Timer Window->Standard Watchdog Timer enabled,(Windowed-mode is disabled)
#pragma config FWDTEN = OFF         // Watchdog Timer Enable->Watchdog Timer is disabled
#pragma config ICS = PGx2           // Comm Channel Select->Emulator/debugger uses EMUC2/EMUD2
#pragma config BKBUG = OFF          // Background Debug->Device resets into Operational mode
#pragma config GWRP = OFF           // General Code Segment Write Protect->Writes to program memory are allowed
#pragma config GCP = OFF            // General Code Segment Code Protect->Code protection is disabled
#pragma config JTAGEN = OFF         // JTAG Port Enable->JTAG port is disabled

////////////////////////////////////////////////////////////////////////////////
//Function Proto type
void SYSTEM_Initialize(void);
void OSCILLATOR_Initialize(void);
void PIN_MANAGER_Initialize(void);
void TMR1_Initialize (void);
void TMR2_Initialize (void);
void UART2_Initialize (void);
void ADC_Initialize (void);
void delayFunc(void);
void U2_TxByte(char value);
void U2_TxString(char *s);
//Interrupt Prototype
void __attribute__ ((interrupt, no_auto_psv)) _T1Interrupt(void);
void __attribute__ ((interrupt, no_auto_psv)) _T2Interrupt(void);
void __attribute__ ((interrupt, no_auto_psv)) _U2RXInterrupt(void); 

////////////////////////////////////////////////////////////////////////////////
//Code variables
unsigned int counter = 0;
unsigned int count1 = 150;
unsigned int count2 = 150;
int rcvByte; //we dont need this for this program - as we are not receiving anything
float temperature =0.0f;
float potVoltage = 0.0f;
/**
 * unsigned to get the full positive range of the buffer_variable
 * and long as normal int cannot go beyond 65000 either but long can
 * go approx 4.2 million before overflowing
 */
unsigned long int buffer_variable = 0;

#define TASK_READ_TEMP_POT_P              OSTCBP(1) 	//Task #1
#define TASK_KNIGHT_RIDER_P               OSTCBP(2) 	//Task #2
#define TASK_RUN_LED_P                    OSTCBP(3) 	//Task #3
#define TASK_PRINT_TEMP_POT               OSTCBP(4) 	//Task #4

#define PRIO_READ_TEMP_POT                10        	//Task1 priorities
#define PRIO_KNIGHT_RIDER                 10        	//Task2   ,,
#define PRIO_RUN_LED                      10         	//Task3   ,,
#define PRIO_PRINT_TEMP_POT               10         	//Task4   ,,

#define BINSEM_SIGNAL_BEGIN               OSECBP(1) 	//Semaphore
#define BINSEM_SIGNAL_READ_COMPLETE       OSECBP(2) 	//Semaphore

////////////////////////////////////////////////////////////////////////////////
void TaskReadTempAndPot (void)
{
  while(1) 
  {
     OSSignalBinSem(BINSEM_SIGNAL_BEGIN);
     temperature = ADC_Operate(4); //using ANI4
     temperature = (((temperature *0.00322) - 0.5)/0.01);
     
     potVoltage = ADC_Operate(5);  //using ANI5
     potVoltage = (potVoltage * 0.00322);  
     OSSignalBinSem(BINSEM_SIGNAL_READ_COMPLETE);
     OS_Yield();
  }
}
////////////////////////////////////////////////////////////////////////////////
void TaskKnightRider (void)
{
 while(1) 
 {
    OS_WaitBinSem(BINSEM_SIGNAL_BEGIN, OSNO_TIMEOUT);
    
    if (PORTDbits.RD6 == 0)
    {  
        /**
        * Run Straight ProjectKnightRider
        */
        PORTAbits.RA0 = 1;
        delayFunc();

        PORTAbits.RA1 = 1;
        delayFunc();

        PORTAbits.RA2 = 1;
        delayFunc();

        PORTAbits.RA3 = 1;
        delayFunc();

        PORTAbits.RA4 = 1;
        delayFunc();

        PORTAbits.RA5 = 1;
        delayFunc();

        PORTAbits.RA6 = 1;
        delayFunc();
        PORTA &=0x0080;
    }
    else
    {    
        /**
         * Run reverse ProjectKnightRider
         */
        PORTAbits.RA6 = 1;
        delayFunc();

        PORTAbits.RA5 = 1;
        delayFunc();

        PORTAbits.RA4 = 1;
        delayFunc();

        PORTAbits.RA3 = 1;
        delayFunc();

        PORTAbits.RA2 = 1;
        delayFunc();

        PORTAbits.RA1 = 1;
        delayFunc();

        PORTAbits.RA0 = 1;
        delayFunc();

        PORTA &=0x0080;
    }
 }
}

void TaskPrintTempAndPot (void)
{
    while(1)
    {
        OS_WaitBinSem(BINSEM_SIGNAL_READ_COMPLETE, OSNO_TIMEOUT);
        char TxString[100] = "";
        sprintf(TxString,"\n\n#> Voltage: %4.2f       Temperature: %4.2f", potVoltage, temperature);
        U2_TxString(TxString);
    }
    
}
////////////////////////////////////////////////////////////////////////////////
void TaskRunLED (void) 
{
    while(1) 
    {
       /**
        * Flip LED
        */
       if(PORTAbits.RA7 == 0)
        {
            PORTAbits.RA7 = 1;
        }
        else 
        {
            PORTAbits.RA7 = 0;
        }
       OS_Delay(100);
    }
}
////////////////////////////////////////////////////////////////////////////////
int main (void)
{
 OSInit();
    
 OSCreateTask(TaskReadTempAndPot, TASK_READ_TEMP_POT_P, PRIO_READ_TEMP_POT);
 OSCreateTask(TaskKnightRider,  TASK_KNIGHT_RIDER_P,  PRIO_KNIGHT_RIDER );
 OSCreateTask(TaskRunLED, TASK_RUN_LED_P, PRIO_RUN_LED);
 OSCreateTask(TaskPrintTempAndPot, TASK_PRINT_TEMP_POT, PRIO_PRINT_TEMP_POT);
// OSCreateTask(TaskRead, TASK_Read, PRIO_Read);
    
 OSCreateBinSem(BINSEM_SIGNAL_BEGIN, 0);
 OSCreateBinSem(BINSEM_SIGNAL_READ_COMPLETE, 0);
 
 SYSTEM_Initialize();           //PIC initialization
	
// start multitasking/scheduler/despatcher.

 while(1){
  OSSched();        //pass control to the scheduler(kernel))
 }
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _T1Interrupt (  )
{ 	
   static uint8_t i = 10;       //counter = 3 ms x 10 = 30ms
	
  if(IEC0bits.T1IE && IFS0bits.T1IF )
  {
   IFS0bits.T1IF = false;
   if ( !(--i) )
   {
	i = 10;
			
	OSTimer();          //call salvo service at 30 ms interval, Tick Rate = 10 mS
   }
  } 
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _T2Interrupt (  )
{ 
//    if(buffer_variable < 100)
//    {
//        buffer_variable++;
//    }
//    if(buffer_variable >= 100)
//    {
//        buffer_variable = 0;
//        // PORTA ^= 0x80; //toggling bit 7 of register A - RA7 - LED10
//        if(PORTAbits.RA7 == 0)
//        {
//            PORTAbits.RA7 = 1;
//        }
//        else 
//        {
//            PORTAbits.RA7 = 0;
//        }
//    }
//    IFS0bits.T2IF = 0;
}

void __attribute__ ((interrupt, no_auto_psv)) _U2RXInterrupt(  )
{
    
    if(IEC1bits.U2RXIE == 1)
    {
        if(IFS1bits.U2RXIF == true)       //check the status flag
        {
          rcvByte = U2RXREG;                //Read the received byte 
        }
    }
    IFS1bits.U2RXIF = false;                 //reset the status flag  
}
//////////////////////////////////////////////////////////////////////
void SYSTEM_Initialize(void)
{
    PIN_MANAGER_Initialize();
    OSCILLATOR_Initialize();
    TMR1_Initialize();
    TMR2_Initialize ();
    UART2_Initialize();
    ADC_Initialize();
}
////////////////////////////////////////////////////////////////////////////////
void OSCILLATOR_Initialize(void)
{
// NOSC PRI; SOSCEN disabled; OSWEN Switch is Complete; 
  __builtin_write_OSCCONL((uint8_t) (0x0200 & 0x00FF));
// RCDIV FRC/2; DOZE 1:8; DOZEN disabled; ROI disabled; 
  CLKDIV = 0x3100;
// TUN Center frequency; 
  OSCTUN = 0x0000;
// WDTO disabled; TRAPR disabled; SWDTEN disabled; EXTR disabled; POR disabled; SLEEP disabled; BOR disabled; IDLE disabled; IOPUWR disabled; VREGS disabled; CM disabled; SWR disabled; 
  RCON = 0x0000;
}
////////////////////////////////////////////////////////////////////////////////
void PIN_MANAGER_Initialize(void)
{
/**********
* Setting the Output Latch SFR(s)
*********/
 LATA = 0x0000;
 LATB = 0x0000;
 LATC = 0x0000;
 LATD = 0x0000;
 LATE = 0x0000;
 LATF = 0x0000;
 LATG = 0x0000;

/**********
* Setting the GPIO Direction SFR(s)
*********/
TRISA = 0x0000;
TRISB = 0xFFFF;
TRISC = 0xF01E;
TRISD = 0xFFFF;
TRISE = 0x03FF;
TRISF = 0x31FF;
TRISG = 0xF3CF;
/**********
* Setting the Weak Pull Up and Weak Pull Down SFR(s)
*********/
CNPU1 = 0x0000;
CNPU2 = 0x0000;

/**********
* Setting the Open Drain SFR(s)
*********/
ODCA = 0x0000;
ODCB = 0x0000;
ODCC = 0x0000;
ODCD = 0x0000;
ODCE = 0x0000;
ODCF = 0x0000;
ODCG = 0x0000;

/**********
* Setting the Analog/Digital Configuration SFR(s)
*********/
 AD1PCFG = 0x00C0;
}
////////////////////////////////////////////////////////////////////////////////
void TMR1_Initialize (void)
{
       TMR1 = 0x0000;
      //Instructions from Manual
//    1. Set the TON bit (= 1).
      /**
       * Timer setting done at the last
       */
//    2. Select the timer pre scaler ratio using the
      T1CONbits.TCKPS = 0b00;  //1:1 ratio selected
      /**
       * We are using the lowest possible pre scaler to get
       * a more precis time interval facilitating more control
       * over the time
       */
//    3. Set the Clock and Gating modes using the TCS
      T1CONbits.TCS = 0; //using internal clock so TCS=0
      T1CONbits.TGATE = 0; //TGATE is set to 0 as we want the output from the PreScaler to generate out interrupts
      
//    4. Set or clear the TSYNC bit to configure synchronous or asynchronous operation.
      T1CONbits.T1SYNC = 1; //turning on synchronization of the clock
            
//    5. Load the timer period value into the PR1
      PR1 = (uint16_t) 0x2EE0; //The value of the pre scaler goes here register.
      
      IFS0bits.T1IF = 0; 
      
      // Interrupt priority setting
      // 7 = 0b0111
      IPC0bits.T1IP2 = 1; 
      IPC0bits.T1IP1 = 1; 
      IPC0bits.T1IP0 = 1;
      
      IEC0bits.T1IE = 1;
      T1CONbits.TON = 1; 
//    Ans: Check the Interrupt_Initialize for more details
}

void TMR2_Initialize (void)
{
//       TMR2 = 0x0000;
//       T2CONbits.T32 = 1;
//      //Instructions from Manual
////    1. Set the TON bit (= 1).
//      /**
//       * Timer setting done at the last
//       */
////    2. Select the timer pre scaler ratio using the
//      T2CONbits.TCKPS = 0b00;  //1:1 ratio selected
//      /**
//       * We are using the lowest possible pre scaler to get
//       * a more precis time interval facilitating more control
//       * over the time
//       */
////    3. Set the Clock and Gating modes using the TCS
//      T2CONbits.TCS = 0; //using internal clock so TCS=0
//      T2CONbits.TGATE = 0; //TGATE is set to 0 as we want the output from the PreScaler to generate out interrupts
//      
////    4. Set or clear the TSYNC bit to configure synchronous or asynchronous operation.
////      T2CONbits.T2SYNC = 1; //turning on synchronization of the clock
//            
////    5. Load the timer period value into the PR1
//      PR2 = (uint16_t) 0x04B0; //The value of the pre scaler goes here register.
//      //58000 = E290
//      
//      IFS0bits.T2IF = 0; 
//      
//      // Interrupt priority setting
//      // 7 = 0b0111
//      IPC1bits.T2IP2 = 1; 
//      IPC1bits.T2IP1 = 1; 
//      IPC1bits.T2IP0 = 1;
//      
//      IEC0bits.T2IE = 1;
//      T2CONbits.TON = 1; 
////    Ans: Check the Interrupt_Initialize for more details
}

////////////////////////////////////////////////////////////////////////////////
void UART2_Initialize (void)
{
// STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; RXINV disabled; UEN TX_RX; 
 U2MODE = 0x8002;     //0x8008;
// OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; UTXISEL0 TX_ONE_CHAR; UTXINV disabled; 
 U2STA = 0x0000;
// U2TXREG 0; 
 U2TXREG = 0x0000;
// BaudRate = 9600; Frequency = 2000000 Hz; BRG 51; 
 U2BRG = 0x0019;   
 IEC1bits.U2RXIE = 1;    //Rx interrupt Enabled
 U2STAbits.UTXEN = 1;
 IEC1bits.U2TXIE = 0;    //Tx interrupt disabled
 IFS1bits.U2RXIF = 0;
 IPC7bits.U2RXIP2 = 1;
 IPC7bits.U2RXIP1 = 0;
 IPC7bits.U2RXIP0 = 1;
}

void ADC_Initialize ( void )
{
    
    /**
     * Setting pin4 and pin5 as a analog input
     */
    AD1PCFG = 0xFFCF;
    
    /**
     * Start the sample enable bit 
     * from bit2 and set the SSRC<2:0>
     * to 111 - (Internal counter ends sampling 
     * and starts conversion (auto-convert))
     */
    AD1CON1 = 0x00E2;
    
    
    AD1CON2 = 0x0000;
    
    /**
     * AD1CON3 value bit7 to bit0 has been set to 0 so
     * that ADCS is set to zero and TAD = Tcy
     */
    AD1CON3 = 0x1100;
    AD1CSSL = 0x0000;
    /**
     * Setting the initial value for AD1CHS to 0x0004
     */
    AD1CHS = 0x0004;
    AD1CON1bits.ADON = 1;
}

int ADC_Operate (int value)
{
    while (1) {
        AD1CHS = value;
        AD1CON1bits.SAMP = 1;
        while (!AD1CON1bits.DONE);
        return ADC1BUF0;
    }
}

int ADC_Temp(int value)
{  
        AD1CHS = value;
        AD1CON1bits.SAMP = 1;
        while (!AD1CON1bits.DONE);
        if (IFS0bits.AD1IF == 1) {
            return ADC1BUF0;
        }
}

void U2_TxByte(char value)
{
  while (!U2STAbits.TRMT);    
  U2TXREG = value;   
  Nop();
  Nop();
  Nop();
}

void U2_TxString(char *s)
{
  char nextChar;
  while(*s != '\0')
  {
    nextChar = *s++;  
    U2_TxByte(nextChar);
  }
  U2_TxByte(0x0d);    //CR
  U2_TxByte(0x0a);    //LF
  Nop();
  Nop();
}

void delayFunc(void)
{
  int j,k;
  int a;
  
  for(j = 0; j < count1; j++)
  {
      for(k=0; k < count2; k++)
      {
          a = 0; //How many times (frequency) this instruction getting executed?
      }
  }
    
}

////////////////////////////////////////////////////////////////////////////////
//Revision History:
//Dec 19/17: original build up.