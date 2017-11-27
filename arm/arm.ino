#include <msp430g2553.h>
#include <math.h> //for trig functions
//Uses a combination of code from the original sensor unit and the servo multiplexing code for the robot arm.
unsigned int echo;
int mode = 0;
int pw[4] = {1500, 1500, 2850, 1500};
int main(void){
	WDTCTL = WDTPW + WDTHOLD; //stop wdt
	P1IE = 0; //prevent premature interrupts
	P2DIR = 0x0F; //P2.0 to P2.3 (four servos) - this should always be on
	TA1CTL = TASSEL_2 + MC_1 + ID_0;
	  TA1CCTL0 = CCIE;
	  TA1CCTL1 = CCIE;
	TA1CCR0 = 5000; //period of PWM
	TA1CCR1 = 1000; //arbitrary value for PWM
	_BIS_SR(GIE);
	__delay_cycles(1000000);
	for(;;){
		switch(mode){
			case 0: sweep();
			break;
			case 1: reach();
			break;
			case 2: clamp();
			break;
			case 3: lift();
			break;
			case 4: wait();
			break;
			case 5: rel();
			break;
		}
	if(mode!= 0){
	P1DIR |= BIT1;
    	P1OUT |= BIT1;
   }
	}

	return 0;
}

//subroutines

void sweep(void){
	//in this subroutine, base of arm moves back and fourth while sensor is scanning for objects
	//this should be the only subroutine where P1 and TA0 interrupts are enabled
	P1DIR = BIT1 + BIT6; //P1.1 for TRIG, 1.6 for LED2
	P1REN |= BIT3 + BIT4; //P1.3 for BTN, P1.4 for ECHO
  	P1OUT = BIT3; //get rid of this when using the 555
	P1IE |= BIT3; //enable interrupt for P1.3, P1.4
	P1IES &= ~BIT4; //make sure this interrupts on rising edge
	P1IFG &= ~(BIT2 + BIT3 + BIT4); //clear P1.2, P1.3, P1.4 interrupt flag
	P1SEL = BIT2;
	P1SEL2 &= ~BIT2;

	//idle position of the arm
	pw[1] = 1550;
	pw[2] = 2850;
 	pw[3] = 1000; //clamp open

	int inc = 1; //decide whether to increment or decrement
	while(mode==0){
		//things to do in sweep mode: only thing to go into this loop should be moving base servo back and forth
		//loop broken by mode change in interrupt routine

		//sweep back and forth 90 degrees - corner to coner of the base
		pw[0] += inc;
		if(pw[0] > 2850){
			inc = -1;
		}
		if(pw[0] < 1850){
			inc = 1;
		}
		__delay_cycles(5000);
	}
	
}

void reach(void){
	//in this subroutine, turn off P1 and TA0 interrupts
	//keep TA1 interrupts on
	TA0CTL = MC_0; //make sure TA0 is turned off
	TA0CCTL1 = OUTMOD_0; //the capture/compare as well
	P1IE = 0; //disable all P1 interrupts
	//in this mode, move first joint, wait, move second joint, and proceed to clamp subroutine

	float l1 = 11.5; //first segment (cm)
	float l2 = 23; //second segment (cm)
	float len = echo / 58; //distance of object from sensor (cm)

	float theta_2 = acos((len*len - l1*l1 - l2*l2)/(-2*l1*l2))*180.0/M_PI; //angle of second joint from straight wrt 1st seg. note: need to convert from rads to deg
	float phi_2 = 180.0 - theta_2; //angle wrt the vertical (use this one for second joint)
	float theta_1 = acos((l2*l2 - l1*l1 - len*len)/(-2*l1*len))*180.0/M_PI; //Triangle is scalene, not isoceles - can't just do (180-theta_2)/2


	pw[3] = 1000; //make sure clamp is open if not already.
	pw[1] = (int)(theta_1*1000/90); //round to int for valid pulse width
	__delay_cycles(1000000);

  if(pw[1] > 30000){
    P1OUT |= BIT6;
  }

	pw[2] = 2350 + (int)(phi_2*1000/90); //still not quite sure how to get this right
 	if(pw[2] > 3000){
    		pw[3] = 3000; //prevent ta1ccr1 overflow
  	}
  
 	__delay_cycles(1000000);

	mode = 2; //once done moving to position, advance to clamping
	
}

void clamp(void){
	pw[3] = 3000; //close the clamp
	__delay_cycles(1000000);

	mode = 3;
}
void lift(void){
	//this subroutine will bring the arm to a position to drop in the cart
	pw[0] = 1350; //take arm to dumping sector
	pw[1] = 1350; //extend arm
	pw[2] = 2350; //straighten second segment
	__delay_cycles(1000000);

	mode = 4;
}
void wait(void){
	//once we know the cart is there, release
	//if we use the 30 degree dumping sector, we can use the ultrasonic sensor to determine if the cart is available
	__delay_cycles(1000000);
	while(mode==3){
		
	}
	mode  = 5;
}
void rel(void){
	//let go of object, go back to sweeping phase
	pw[3] = 1000; //release clamp
	 __delay_cycles(1000000);
	mode = 0;
}

//port 1 ISR
#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR(void){
	if(P1IFG & BIT3){ //interrupt from input to set off trigger pulse
		P1OUT |= BIT1; //turn on P1.2 until timer is done
		TA0CTL = TASSEL_2 + MC_1 + TAIE;
  		TA0CCR0 = 10;
  		TA0CCTL0 = CCIE;
		P1IFG &= ~BIT3;
		P1IE &= ~BIT3;
		P1IE |= BIT4;
	}
	else if(P1IFG & BIT4){ //interrupt from echo input
   		TA0CTL = TASSEL_2 + TACLR + MC_2; //continous mode to prevent overflow from TA0CCR0
		TA0CCR1 = 0x0FFFF;
		TA0CCR0 = 0x0FFFF; //again, to prevent overflow
		TA0CCTL1 = OUTMOD_0 + CM_2 + CCIS_0 + SCCI + CAP + CCIE; //capture from CCI1A: synchronous, falling edge
		P1IFG &= ~BIT4;
		P1IE &= ~BIT4;
		P1IE |= BIT3;
	}
	P1IFG &= ~(BIT3 + BIT4);
}

void overflow(void){
	P1OUT &= ~BIT1; //turn off P1.2
	//P1OUT ^= BIT6;
	TA0CTL = MC_0; //turn off timer
}

//Timer A0 ISR
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TA0_ISR(void){
	if(TA0IV &  0x0a){
    		overflow(); //hooray, this works!
	}
	TA0CTL = 0;
	TA0CCTL0 = 0;
}
//An interrupt occurs at this vector for capture/compare block 1
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TA0_CCI1A_ISR(void){
	echo = TA0CCR1;
	if(echo<2000){ //2 mS is about 1 ft
		if(mode==0){ //if looking for trash
			mode = 1; //advance to move into position
		}
		
	}
	TA0CCTL1 = 0;
	TA0CTL = MC_0;
}

//idea for running servos off timer comes from Ted Burke
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TA1_ISR(void){
	//overflow interrupt for Timer A1
	//starts the pulse at the current pin selected, then goes to the next
	static int n = 0;
	P2OUT = 1 << n;
	TA1CCR1 = pw[n];
	
	n = (n + 1) % 4;
	TA1CCTL0 &= ~CCIFG;
}
#pragma vector=TIMER1_A1_VECTOR
__interrupt void TA1_CC1_ISR(void){
	P2OUT = 0;
	TA1CCTL1 &= ~CCIFG;
}
