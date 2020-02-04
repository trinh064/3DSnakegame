
#include <p24FJ64GA002.h>


// CW1:FLASH CONFIGURATION WORD 1 (see PIC24 Family Reference Manual 24.1)

#pragma config ICS = PGx1       // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config FWDTEN = OFF     // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF       // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF        // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF     // JTAG Port Enable (JTAG port is disabled)

// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config I2C1SEL = PRI    // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF    // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON    // Primary Oscillator I/O Function (CLKO/RC15 functions as I/O pin)
#pragma config FCKSM = CSECME   // Clock Switching and Monitor (Clock switching is enabled,
                                // Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL   // Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))

int pos0[3]={1,1,2};
int pos1[3]={2,1,2};
int pos2[3]={3,1,2};
int  *position[3]={pos0,pos1,pos2};
int  apple[3]={2,2,2};
int direction=1;

void display(){
    int x=0b00001;
    int y=0b00001;
    int z=0b00001;
    
    int count=0;
    // display snake
    for (count=0;count<3;count++){
        int X=x<<position[count][0];
        int Y=y<<position[count][1];
        int Z=(z<<position[count][2])^0b11111;
        LATB=0;
        LATB=X+(Y<<5)+(Z<<10);
        asm ("REPEAT #15");
        asm ("NOP");
        
    }
    // display food
    int X=x<<apple[0];
    int Y=y<<apple[1];
    int Z=(z<<apple[2])^0b11111;
    LATB=X+(Y<<5)+(Z<<10);
        asm ("REPEAT #15");
        asm ("NOP");
    
    
    
}

void food()
{
    unsigned int random = TMR4%125;
    apple[0] = random/25;
    apple[1] = (random%25)/5;
    apple[2] = random%5;
  
}

void move()
{
    unsigned static int prev_direc = 1;
    if((((1+prev_direc)/2)==((1+direction)/2))||(direction == 0))
    {
        direction = prev_direc;
    }
    
    else
    {
        prev_direc = direction;
    }
    

    position[0][0]=position[1][0];
     position[0][1]=position[1][1];
      position[0][2]=position[1][2];
      
      position[1][0]=position[2][0];
     position[1][1]=position[2][1];
      position[1][2]=position[2][2];
      
 
    
    switch(direction)
    {
        case 1:
            position[2][0] = (position[2][0]+1)%5;
            break;
        
        case 3:
            position[2][1] = (position[2][1]+1)%5;
            break;
        case 5:
            position[2][2] = (position[2][2]+1)%5;
            break;
        case 2:
            position[2][0] = (position[2][0]+4)%5;
            break;
        case 4:
            position[2][1] = (position[2][1]+4)%5;
            break;
        case 6:
            position[2][2] = (position[2][2]+4)%5;
            break;
    }
    
    if(position[2][0]==apple[0] &&
     position[2][1]==apple[1]    &&
      position[2][2]==apple[2])
    {
        food();
    }
}
void getDirection()
{
    int xHigh;
    int yHigh;
    int xLow; //The low values keep track of how low it is. Inverse of the high values
    int yLow;
    int threshold = 0x300; //Threshhold is at the 3/4 point
    
    AD1CHSbits.CH0SA = 0; //set AN0 as input to MUXA
    AD1CON1bits.SAMP = 1;
    asm("nop");
    asm("repeat #10");
    asm("nop");
    AD1CON1bits.SAMP = 0;
    while(!AD1CON1bits.DONE);
    xHigh=ADC1BUF0;
    xLow=0x3ff-xHigh; //Max value - the high value
    
    
    AD1CHSbits.CH0SA = 1; //set AN1 as input to MUXA
    AD1CON1bits.SAMP = 1;
    asm("nop"); //Two cycles to get the sample
    asm("repeat #10");
    asm("nop");
    AD1CON1bits.SAMP = 0;
    while(!AD1CON1bits.DONE);
    yHigh=ADC1BUF0;
    yLow=0x3ff-yHigh;
    /*
    Comparisons test to see if each direction is the most
    extreme and meets the threshold */
    
    if((xHigh>yHigh)&&(xHigh>xLow)&&(xHigh>yLow)&&(xHigh>threshold))
    {
        direction = 1;
    }
    
    else if((yHigh>xLow)&&(yHigh>yLow)&&(yHigh>threshold))
    {
        direction = 3;
    }
    
    else if((xLow>yLow)&&(xLow>threshold))
    {
        direction = 2;
    }
    
    else if(yLow>threshold)
    {
        direction = 4;
    }
    
    //buttons override everything
    
    if(!PORTAbits.RA2)
    {
        direction = 5;
    }
    
    if(!PORTAbits.RA3)
    {
        direction = 6;
    }
    

}




// use bitshifting to toggle the desire bit based on the snake array
void setup(){
    CLKDIVbits.RCDIV=0; 
     AD1PCFG = 0xFFFC;      //configure pins to have AN0 and AN1 set to analog
    TRISB=0;
   LATB=0;
   
    T1CON = 0; // turn off timer, prescalar 1:1, use Tcy as clk source
    T1CONbits.TCKPS=0b11;
    PR1 = 32765;
    TMR1 = 0;
    _T1IF = 0; // equivalent to IFS0bits.T1IF = 0;
    T1CONbits.TON = 1; // now turn on the timer
    T4CONbits.TON = 1;
    
   
    _TRISA2=1;
    _TRISA3=1;
    CNPU2bits.CN30PUE=1;
    CNPU2bits.CN29PUE=1;
    
    AD1CON1bits.ADON = 0; //turn off
    AD1CON1bits.ADSIDL = 0; //stop in idle
    AD1CSSL = 0; //per example, pg18
    AD1CON1bits.FORM = 0; //integer number
    AD1CON1bits.ASAM = 0; //wait til SAMP bit it set to start sampling
    AD1CON1bits.SSRC = 0; //wait til user sets SAMP to 0 to end sampling
    AD1CON2 = 0; //per example
    AD1CON3 = 0x0003; //per example
    AD1CON1bits.ADON = 1; //turn on

}


int main(void){
    setup();
   
    while (1){
        display();
         getDirection();
        
        if(_T1IF){
            _T1IF=0;
            //getDirection();
        move();}
    }
return 0;}
