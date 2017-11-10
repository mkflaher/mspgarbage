#include <msp430.h>
 
// PWM channels duty cycle array
int pw[] = {3000,1300,3000,2500};
int n = 0;
int main( void )
{
	WDTCTL = WDTPW + WDTHOLD; // Disable watchdog timer
 
	P1DIR = 0x0F; // Make P1.0-3 outputs
 
	// Configure Timer A0 Compare interrupts
	TA0CTL = TASSEL_2 + MC_1 + ID_0; // up mode
	TA0CCR0 = 5000;              	// set timer period to 5ms
	TA0CCTL0 = CCIE;             	// enable CC0 interrupt
	TA0CCR1 = 771;              	// set pulse width to .771ms
	TA0CCTL1 = CCIE;             	// enable CC1 interrupt
	_BIS_SR(GIE);                	// global interrupt enable
 

	//  
	//changing pulse width for all channels
	//  
	int channel;
	while(1)
	{
   		pw[3] -= 1;
		if(pw[3] < 2000){
			pw[3] = 3000;
		}
    		__delay_cycles(5000);
	}
 

}
 
//
// Timer A0 CC0 interrupt service routine.
// Switches output between pins p1.0-p1.5
// Sets pulse width with respect to pins
// Triggered when Timer A0 matches TA0CCR0
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0_CC0(void)
{
 
	P1OUT = 1 << n; 	// Set P1.n high
	TA0CCR1 = pw[n];	// Set timer for current pin's pulse width
 
	n = (n+1)%4 ;    	// Move on to next PWM channel
	TA0CCTL0 &= ~CCIFG; // Reset CC interrupt flag
}
 
//
// Timer A0 CC1 interrupt service routine.
// responsible for ending each PWM
// pulse on all channels. It is triggered when
// Timer A0 matches TA0CCR1, which is at a
// different time for each PWM channel.
//
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer_A1_CC1(void)
{
	P1OUT = 0;
	TA0CCTL1 &= ~CCIFG;
}
