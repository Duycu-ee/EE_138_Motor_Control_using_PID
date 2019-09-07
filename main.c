////////////////////////////////////////////////////////////////////////////////////// 
////	Lab 4.2 - PWM motor control + displaying motor speed 
////		
////		
////			
//////////////////////////////////////////////////////////////////////////////////////

#include <asf.h>


// default system declarations
void Simple_Clk_Init(void);
void wait(int t);

// function declarations for enabling peripheral clocks
void enable_tc_clocks(void);
void enable_eic_clocks(void);

// function declarations for initializing peripherals
void init_tc2(void);
void init_tc4(void);
void init_tc6(void);
void init_eic(void);

// function declarations for display
void num_to_segment(int number);
void display_digits_and_check_key(void);
void process_digits(int num);
void check_key(void);
void verify_key_pressed(void);
unsigned int read_adc(void);



Adc *adcPtr = (Adc *) ADC;							// define a pointer for the ADC

TcCount16 *tcPtr2 = &(TC2->COUNT16);				// define a pointer for TC4 (POT control motor)

TcCount8 *tcPtr_pwm = &(TC4->COUNT8);				// define a pointer for TC6 (measure motor speed)

TcCount16 *tcPtr6 = &(TC6->COUNT16);				// define a pointer for TC4 (POT control motor)

Eic *eicPtr = (Eic *) EIC;


// global variables for display 
int disp_digits[4] = {12,12,12,12};		// used for displaying each digits
static int digit_place = 4;

// global variables for CC[] values and RPM
int cc_value = 0;					// used to adjust CCx
int counter = 0;
int rpm_value = 0;					// rpm is: revolution / minute
int rpm = 0;
int neg_value = 0;					// used for indicating CCW rotation (PB09 pin)
int pending = 0;					// pending is for 0 RPM is reached before entering IDLE state

// global variables for FSM for PID, speed control, and position control
int PID_state = 0;
int PID = 0;
float PID_p = 0;
float PID_i = 0;
float PID_d = 0;
int PID_p_neg = 0;
int PID_i_neg = 0;
int PID_d_neg = 0;
float error_value = 0;
float prev_error_value = 0;
int desired_speed = 0;
int motor_control = 0;
int position_control = false;
unsigned int desired_position = 0;	
double position = 0;	
int direction = 0;	

// PID state definition
#define IDLE 0
#define ACCEL 1
#define DECEL 2
#define SPEED_CTRL 3
#define POS_CTRL 4

// motor speed definition
#define MOTOR_0 1
#define MOTOR_ACCEL 2
#define MOTOR_DECEL 3
#define MOTOR_CW 4					// used for position clock-wise direction
#define MOTOR_CCW 5					// used for position counter-clock-wise direction

#define LEFT 1						// used to determine which direction its offset from
#define RIGHT 2						// used to determine which direction its offset from

// global variables for keypad
int digits[4] = {12,12,12,12};
unsigned int disp_value;
int key = 10;						// "key" used for de-bouncing stage
int key_copy = 10;
int  key_last = 10;					// "key last" used for de-bouncing
int key_pressed = 10;				// "key pressed" used for storing into array
int key_counter_start = 0;			// "key counter" used for counter for de-bouncing
int key_counter_end = 0;
int key_state = 0;

unsigned int x[3] = {0,0,0};		// input for IIR LPF
float y[3] = {0,0,0};				// output for IIR LPF

// define constants for digital filter (LPF filter)
#define a0 0
#define a1 0.00049
#define a2 0.00048

#define b0 1
#define b1 1.968
#define b2 0.9691

// define constants for PID speed controller
#define Kp 0.15
#define Ki 0.4
#define Kd 2.5

// define constants for PID position controller
#define Kp_pos 0.22
#define Ki_pos 0.35
#define Kd_pos 10



// converts number into seven-segments
void num_to_segment(int num)
{
	// setup all ports of A
	Port *por = PORT_INSTS;
	PortGroup *porB = &(por->Group[1]);
	
	// initialize all the display (seven segment) as output
	porB->DIRSET.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05 | PORT_PB06 | PORT_PB07;
	porB->OUTSET.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05 | PORT_PB06 | PORT_PB07;
	
	// seven segment display:  PB07 PB06 PB05 PB04 PB03 PB02 PB01 PB00
	//							DP	 G	   F	E 	D	 C	  B	    A
	switch(num)
	{
		case 1:
		{
			porB->OUTCLR.reg = PORT_PB01 | PORT_PB02;
		}break;
		case 2:
		{
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB03 | PORT_PB04 | PORT_PB06;
		}break;
		case 3:
		{
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB06;
		}break;
		case 4:
		{
			porB->OUTCLR.reg = PORT_PB01 | PORT_PB02 | PORT_PB05 | PORT_PB06;
		}break;
		case 5:
		{
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB02 | PORT_PB03 | PORT_PB05 | PORT_PB06;
		}break;
		case 6:
		{
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05 | PORT_PB06;
		}break;
		case 7:
		{
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02;
		}break;
		case 8:
		{
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05 | PORT_PB06;
		}break;
		case 9:
		{
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB05 | PORT_PB06;
		}break;
		case 0:
		{
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05;
		}break;
		default:
		{
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05;
		}
	}
}



// display digits while detecting keyboard in each row 
void display_digits_and_check_key(void)
{
	// setup all ports of A
	Port *por = PORT_INSTS;
	PortGroup *porA = &(por->Group[0]);
	
	
	porA->DIRSET.reg = PORT_PA04 | PORT_PA05 | PORT_PA06 | PORT_PA07;
		
	switch(digit_place)
	{
		// display PA04 and check fourth row for key pressed
		case 4:
		{
			porA->OUTSET.reg = PORT_PA05 | PORT_PA06 | PORT_PA07;
			num_to_segment(disp_digits[0]);
			porA->OUTCLR.reg = PORT_PA04;
			check_key();
			if (rpm < 2200)
			{
				wait(2);
			}
			else
			{
				wait(1);
			}
			digit_place++;
		}
		// display PA05 and check fifth row for key pressed
		case 5:
		{
			porA->OUTSET.reg = PORT_PA04 | PORT_PA06 | PORT_PA07;
			num_to_segment(disp_digits[1]);
			porA->OUTCLR.reg = PORT_PA05;
			check_key();
			wait(1);
			if (rpm < 2200)
			{
				wait(2);
			}
			else
			{
				wait(1);
			}
			digit_place++;
		}
		// display PA06 and check sixth row for key pressed
		case 6:
		{
			porA->OUTSET.reg = PORT_PA04 | PORT_PA05 | PORT_PA07;
			num_to_segment(disp_digits[2]);
			porA->OUTCLR.reg = PORT_PA06;
			check_key();
			wait(1);
			if (rpm < 2200)
			{
				wait(2);
			}
			else
			{
				wait(1);
			}
			digit_place++;
		}
		// display PA07 and check seventh row for key pressed
		case 7:
		{
			porA->OUTSET.reg = PORT_PA04 | PORT_PA05 | PORT_PA06;
			num_to_segment(disp_digits[3]);
			porA->OUTCLR.reg = PORT_PA07;
			check_key();
			wait(1);
			if (rpm < 2200)
			{
				wait(2);
			}
			else
			{
				wait(1);
			}
			porA->OUTSET.reg = PORT_PA04 | PORT_PA05 | PORT_PA06 | PORT_PA07;
			digit_place = 4;		
		}
	}
}



// takes a number and break it into 4 display value array
void process_digits(int num)
{
	for (int i = 0; i < 4; i++)
	{
		disp_digits[i] = num % 10;
		
		num = num / 10;
	}
}



// calls the ADC peripheral and output 12-bit value
unsigned int read_adc(void)
{
	// start the conversion, see 0x0C in the table in Section 28.7 (pg 534)
	adcPtr->SWTRIG.bit.START = 1;
	
	// wait for conversion to be available
	while(!(adcPtr->INTFLAG.bit.RESRDY));
	
	// insert register where ADC store value			
	return(adcPtr->RESULT.reg); 					
}



// check if key was pressed
void check_key(void)
{
	Port *por = PORT_INSTS;
	PortGroup *porA = &(por->Group[0]);
	
	for (int col = 19; col > 15; col--)
	{
		// setup port A columns to read one at a time
		unsigned int test = 1u << col;
	
		// check individual one row and 4 columns
		if (porA->IN.reg & test)
		{
			// first row
			if ((digit_place == 7) & (col == 19))
			{
				key = 1;
			}
			else if ((digit_place == 7) & (col == 18))
			{
				key = 2;
			}
			else if ((digit_place == 7) & (col == 17))
			{
				key = 3;
			}
			else if ((digit_place == 7) & (col == 16))
			{
				key = 14;
			}
			// second row
			else if ((digit_place == 6) & (col == 19))
			{
				key = 4;
			}
			else if ((digit_place == 6) & (col == 18))
			{
				key = 5;
			}
			else if ((digit_place == 6) & (col == 17))
			{
				key = 6;
			}
			else if ((digit_place == 6) & (col == 16))
			{
				key = 24;
			}
			// third row
			else if ((digit_place == 5) & (col == 19))
			{
				key = 7;
			}
			else if ((digit_place == 5) & (col == 18))
			{
				key = 8;
			}
			else if ((digit_place == 5) & (col == 17))
			{
				key = 9;
			}
			else if ((digit_place == 5) & (col == 16))
			{
				key = 34;
			}
			// fourth row
			else if ((digit_place == 4) & (col == 19))
			{
				key = 41;
			}
			else if ((digit_place == 4) & (col == 18))
			{
				key = 0;
			}
			else if ((digit_place == 4) & (col == 17))
			{
				key = 43;
			}
			else if ((digit_place == 4) & (col == 16))
			{
				key = 44;
			}
			else
			{
				key = 10;
			}
		}
	}
	if (key != 10)
	{
		key_copy = key;
	}
}



// update key pressed if de-bounce start + de-bounce end are met
void check_debounce(void)
{
	// check at start of debouncing effect and update counter_start
	if (key != 10)
	{
		key_counter_start++;
		
	}
	
	// check at end of debouncing effect and update counter_end
	if ((key == 10) && (key_counter_start > 10))
	{
		key_counter_end++;
	}
	
	// copy key pressed after counter is 40
	if ((key_counter_end > 10) && (key != 10))
	{
		key_copy = key;
	}
}



// update key_pressed and reset everything
void verify_key_pressed(void)
{
	if (key_copy != 10)
	{
		key_pressed = key_copy;
		key_copy = 10;
		key_counter_start = 0;
		key_counter_end = 0;
	}
}



// TC2 handler to get system timer at 60 Hz (Ts = 0.017 s)
void TC2_Handler(void)
{
	tcPtr2->COUNT.reg = 0;
	tcPtr2->INTFLAG.bit.MC0 = 1;				// reset interrupt flag

	Port *ports = PORT_INSTS;
	//PortGroup *porA = &(ports->Group[0]);
	PortGroup *porB = &(ports->Group[1]);
	
	//// testing clock time when running TC6 interrupt handler
	//porA->OUTTGL.reg = PORT_PA12;
	
	if (neg_value == 1)
	{
		porB->OUTCLR.reg = PORT_PB09;
	}
	if (neg_value == 0)
	{
		porB->OUTSET.reg = PORT_PB09;
	}
	
	
	
	// keypad FSM for detecting key + updating desired speed + displaying
	switch(key_state)
	{
		// state 1: check key pressed. next state if detected
		case (0):
		{
			display_digits_and_check_key();
			check_debounce();
			key = 10;
			
			key_state = (key_counter_end > 20) ? 1 : 0;
		}break;

		// state 2: verify if key was pressed. next state if it is
		case (1):
		{
			verify_key_pressed();
			
			// set desired speed according to key pressed
			switch (key_pressed)
			{
				case (0):
				{
					desired_speed = 0;
					//if ((desired_speed == 1500) && (error_value > 200))
					//{
						//desired_speed = 1500;
					//}
					//else
					//{
						//desired_speed = 0;
					//}
					position_control = false;
				}break;
				case (1):
				{
					desired_speed = 1500;
					position_control = false;
				}break;
				case (2):
				{
					if ((desired_speed == 1500) && (error_value > 200))
					{
						desired_speed = 1500;
					}
					else
					{
						desired_speed = 0;
					}
				}break;
				// 'A' to increment desired speed
				case (14):
				{
					desired_speed += 200;
					position_control = false;
				}break;
				// 'B' to increment desired speed
				case (24):
				{
					desired_speed -= 200;
					position_control = false;
				}break;
				// 'C' to increment desired position
				case (34):
				{
					desired_speed -= 200;
					position_control = false;
				}break;
				case (44):
				{
					desired_speed -= 200;
					position_control = false;
				}break;
				
			}
			key_state = 0;
		}break;
	}
	
	
	
	// PID FSM for controlling motor speed
	switch (PID_state)
	{
		case (IDLE):
		{
			motor_control = MOTOR_0;
			error_value = 0;
			prev_error_value = 0;
			PID_i = 0;
			if ((key_pressed == 2) && (pending > 150))
			{
				pending = 0;
				PID_state = POS_CTRL;
				position_control = true;
				position = 0;
			}
			else
			{
				PID_state = (desired_speed == 0) ? IDLE : ACCEL;
			}
			pending++;		
		}break;
		
		case (ACCEL):
		{
			if (error_value > 6)
			{
				motor_control = MOTOR_ACCEL;
			}
			else
			{
				PID_state = SPEED_CTRL;
			}
			pending = 0;
		}break;
		
		case (DECEL):
		{
			if ((rpm < 90) && (desired_speed == 0))
			{
				motor_control = MOTOR_0;
				PID_state = IDLE;			
			}
			if (PID < 1)
			{
				PID = 0;
			}
			else
			{
				motor_control = MOTOR_DECEL;
			}
			pending = 0;
		}break;
		
		case (SPEED_CTRL):
		{
			if (abs(error_value) < 5)
			{
				return;
			}		
			else if (error_value < -5)
			{
				PID_state = DECEL;
			}
			else
			{
				PID_state = ACCEL;
			}
		}break;
		
		case (POS_CTRL):
		{
			porB->OUTCLR.reg = PORT_PB09;
			if (key_pressed != 2)
			{
				PID_state = IDLE;
				position_control = false;
			}
			if (position != 0)
			{
				if (direction == LEFT)
				{
					motor_control = MOTOR_CW;
				}
				else if (direction == RIGHT)
				{
					motor_control = MOTOR_CCW;
				}
			}
			else
			{
				motor_control = MOTOR_0;
				if (error_value == prev_error_value)
				{
					PID_i = 0;
				}
			}
		}break;
		
	}
}



// TC4 handler to get system timer at 200Hz (Ts = 0.005 s)
void TC6_Handler(void)
{
	tcPtr6->COUNT.reg = 0;
	tcPtr6->INTFLAG.bit.MC0 = 1;				// reset interrupt flag
	
	//// testing clock time when running TC6 interrupt handler
	//Port *ports = PORT_INSTS;
	//PortGroup *porA = &(ports->Group[0]);
	//porA->OUTTGL.reg = PORT_PA12;
	
	if (counter < 0)
	{
		neg_value = 1;
	}
	if (counter >= 0)
	{
		neg_value = 0;
	}

	if (position < 0)
	{
		direction = LEFT;
	}
	if (position > 0)
	{
		direction = RIGHT;
	}
	

	//  60 / 400 * 200 * 1.3
	rpm_value = counter * 45;		// 1.3 is correction value for more accurate reading	
	counter = 0;

	// low pass filter
	y[2] = y[1];
	y[1] = y[0];
	
	x[2] = x[1];
	x[1] = x[0];
	
	if (PID_state == IDLE)
	{
		y[2] = 0;
		y[1] = 0;
		y[0] = 0;
		x[2] = 0;
		x[1] = 0;
		x[0] = 0;
	}
		
	x[0] = rpm_value;
	
	if (position_control == true)
	{
		x[0] = position;
	}

	y[0] = (a1*x[1] + a2*x[2] + b1*y[1] - b2*y[2]);
	rpm = y[0];
	
	if (position_control == true)
	{
		y[0] = position;
	}
	
	// output into array for display
	if (PID_state == POS_CTRL)
	{
		process_digits(abs(position));
	}
	else
	{
		process_digits(rpm_value);				
	}
	
	
	// PID for speed control
	if ((PID_state != IDLE) && (PID_state != POS_CTRL))
	{
		error_value = desired_speed - rpm;

		PID_p = Kp*error_value;
		PID_i = PID_i + (error_value/200);
		PID_d = Kd*(error_value - prev_error_value);
		
		PID = PID_p + Ki*PID_i + PID_d;
		PID /= 10;
		if (PID < -35)
		{
			PID = -35;
		}
		else if (PID > 200)
		{
			PID = 200;
		}
	}
	
	// PID for position control
	if (PID_state == POS_CTRL)
	{
		error_value = desired_position - position;

		PID_p = Kp_pos*error_value;
		PID_i = PID_i + (error_value/200);
		if (PID_i > 62)
		{
			PID_i = 62;
		}
		if (PID_i < -62)
		{
			PID_i = -62;
		}
		PID_d = Kd_pos*(error_value - prev_error_value);
		
		PID = PID_p + Ki_pos*PID_i + PID_d;
		if (PID < -13)
		{
			PID = -13;
		}
		if (PID > 13)
		{
			PID = 13;
		}
	}
	
	

	switch (motor_control)
	{
		case MOTOR_ACCEL:
		{
			if (tcPtr_pwm->CC[1].reg - PID > 0)
			{
				tcPtr_pwm->CC[1].reg = 220 - PID;
			}
			else
			{
				tcPtr_pwm->CC[1].reg = 1;
			}			
		}break;
		case MOTOR_DECEL:
		{
			if (tcPtr_pwm->CC[1].reg - PID < 255)
			{
				tcPtr_pwm->CC[1].reg = 220 - PID;
			}
			else
			{
				tcPtr_pwm->CC[1].reg = 255;
			}
		}break;
		case MOTOR_0:
		{
			tcPtr_pwm->CC[1].reg = 220;
		}break;
		default:
		{
			return;
		}
		// motor control for position
		case MOTOR_CCW:
		{
			if (tcPtr_pwm->CC[1].reg > 0)
			{
				tcPtr_pwm->CC[1].reg = 228 - PID;
			}
		}break;
		case MOTOR_CW:
		{
			if (tcPtr_pwm->CC[1].reg < 255)
			{
				tcPtr_pwm->CC[1].reg = 212 - PID;
			}
		}break;
	}
	prev_error_value = error_value;	
}



// EIC handler to get count
void EIC_Handler()
{	
	eicPtr->INTFLAG.bit.EXTINT14 = 1;				// reset flag
	Port *ports = PORT_INSTS;
	//PortGroup *porA = &(ports->Group[0]);
	PortGroup *porB = &(ports->Group[1]);

	//porA->OUTTGL.reg = PORT_PA12;
	
	if (porB->IN.reg & PORT_PB14)
	{
		--counter;
		if (position_control == true)
		{
			--position;
		}
	}
	else
	{
		++counter;
		if (position_control == true)
		{
			++position;
		}
	}	
}




///////////////////////////////////////////////////
///////////    MAIN FUNCTION    ///////////////////
int main (void)
{
	Simple_Clk_Init();
	enable_tc_clocks();
	enable_eic_clocks();
	
	init_tc2();
	init_tc4();
	init_tc6();
	init_eic();
	
	// setup port structure
	Port *por = PORT_INSTS;
	PortGroup *porA = &(por->Group[0]);				// setup port A
	PortGroup *porB = &(por->Group[1]);				// setup port A
	
	porA->DIRSET.reg = PORT_PA04 | PORT_PA05 | PORT_PA06 | PORT_PA07;
	porA->OUTSET.reg = PORT_PA04 | PORT_PA05 | PORT_PA06 | PORT_PA07;
	
	// setup port A pins configuration for keypad (in-enable)
	porA->PINCFG[16].reg = PORT_PINCFG_INEN;
	porA->PINCFG[17].reg = PORT_PINCFG_INEN;
	porA->PINCFG[18].reg = PORT_PINCFG_INEN;
	porA->PINCFG[19].reg = PORT_PINCFG_INEN;
	
	// set PA13 as output (3.3V source)
	porA->DIRSET.reg = PORT_PA13;
	porA->OUTSET.reg = PORT_PA13;	
	
	// set PB09 as output (LED)
	porB->DIRSET.reg = PORT_PB09;
	porB->OUTSET.reg = PORT_PB09;
	porA->DIRSET.reg = PORT_PA12;

	
	porA->PINCFG[28].bit.INEN = 1;
	porB->PINCFG[14].bit.INEN = 1;
	
	while(1)
	{
		
	}
}



// enable TC2 and TC4 with generic clock 0
void enable_tc_clocks(void)
{
	// selecting TC2 clock and enable
	PM->APBCMASK.bit.TC2_ = 1;						// enable TC2 peripheral on PM
	uint32_t temp1 = 0x14;							// enabling the TC2 clock for GCLK (page 116)
	temp1 |= 0 << 8;								// select GCLK 0
	GCLK->CLKCTRL.reg = temp1;
	GCLK->CLKCTRL.bit.CLKEN = 0x1;					// enable generic clock clock
	
	// selecting TC4 clock and enable
	PM->APBCMASK.bit.TC4_ = 1;						// enable TC4 peripheral on PM
	uint32_t temp2 = 0x15;							// enabling the TC4 clock for GCLK (page 116)
	temp2 |= 0 << 8;								// select GCLK 0
	GCLK->CLKCTRL.reg = temp2;
	GCLK->CLKCTRL.bit.CLKEN = 0x1;					// enable generic clock clock
	
	// selecting TC4 clock and enable
	PM->APBCMASK.bit.TC6_ = 1;						// enable TC4 peripheral on PM
	uint32_t temp3 = 0x16;							// enabling the TC4 clock for GCLK (page 116)
	temp3 |= 0 << 8;								// select GCLK 0
	GCLK->CLKCTRL.reg = temp3;
	GCLK->CLKCTRL.bit.CLKEN = 0x1;					// enable generic clock clock
}



// enable EIC with generic clock 0
void enable_eic_clocks(void)
{
	PM->APBAMASK.bit.EIC_ = 1;
	
	uint32_t temp1 = 0x03;							// enabling the EIC clock (page 116)
	temp1 |= 0 << 8;
	GCLK->CLKCTRL.reg = temp1;
	GCLK->CLKCTRL.bit.CLKEN = 0x1;
}



// initialize TC2 16-bit mode peripheral (used for FSM + keypad + display)
void init_tc2(void)
{	
	// setup TC2 peripheral
	tcPtr2->CTRLA.reg &= ~(0x2);				// disable TC2 peripheral
	
	tcPtr2->CTRLA.bit.RUNSTDBY = 1;
	tcPtr2->READREQ.bit.RCONT = 1;				// enable continuous sync.
	tcPtr2->CTRLA.bit.MODE = 0x0;
	tcPtr2->CTRLA.bit.WAVEGEN = 0x2;			// match PWM (page 490)
	tcPtr2->CTRLA.bit.PRESCALER = 0x5;			// TC4 clock = 8MHz/64 = 125kHz
	tcPtr2->CTRLA.bit.PRESCSYNC = 0x1;			// counter run on prescale clock
	tcPtr2->CTRLBCLR.bit.DIR = 1;				// count increment
	
	tcPtr2->CC[0].reg = 2040;					// 125 kHz / 60 Hz = 2083 (time = 0.1)
	tcPtr2->CC[1].reg = 2040;
	tcPtr2->INTENSET.bit.MC0 = 1;				// enable interrupt when match for CC[0]
	
	NVIC->ISER[0] |= 1u << 15;
	NVIC->IP[3] |= 0xC0000000;					// priority level 3

	tcPtr2->CTRLA.reg |= 0x2;					// enable TC2 peripheral
}



// initialize TC4 8-bit mode peripheral (used for motor speed control)
void init_tc4(void)
{
	// setup port structure
	Port *ports = PORT_INSTS;
	PortGroup *porA = &(ports->Group[0]);		// setup port A
	
	// setup PA22, PA23 as output
	porA->DIRSET.reg = PORT_PA22 | PORT_PA23;
	porA->OUTSET.reg = PORT_PA22 | PORT_PA23;
	
	// setup TC4 peripheral
	tcPtr_pwm->CTRLA.reg &= ~(0x2);				// disable TC4 peripheral
	
	// 8-bit
	tcPtr_pwm->CTRLA.bit.WAVEGEN = 0x2;			// normal PWM (page 490)
	tcPtr_pwm->CTRLA.bit.MODE = 0x1;			// 8-bit mode
	tcPtr_pwm->CTRLA.bit.RUNSTDBY = 1;
	tcPtr_pwm->CTRLA.bit.PRESCALER = 0x3;		// TC4 clock = 8MHz/8 = 1MHz
	tcPtr_pwm->CTRLA.bit.PRESCSYNC = 0x1;		// counter run on prescale clock
	tcPtr_pwm->PER.reg = 0xFF;					// MAX period = 255
	tcPtr_pwm->READREQ.bit.RCONT = 1;			// enable continuous sync.
	tcPtr_pwm->CTRLBSET.bit.DIR = 1;			// count increment
	
	// setup PA22, PA23 to TC4 peripheral and allow output
	porA->PMUX[11].bit.PMUXE = 0x5;
	porA->PMUX[11].bit.PMUXO = 0x5;
	porA->PINCFG[22].reg = 1;
	porA->PINCFG[23].reg = 1;
	
	//tcPtr_pwm->CTRLBCLR.bit.DIR = 1;
	
	tcPtr_pwm->CC[0].reg = 220;
	tcPtr_pwm->CC[1].reg = 220;

	tcPtr_pwm->CTRLA.reg |= 0x2;				// enable TC4 peripheral	
}




// initialize TC4 16-bit mode peripheral (used for system timer)
void init_tc6(void)
{
	// setup TC4 peripheral
	tcPtr6->CTRLA.reg &= ~(0x2);				// disable TC4 peripheral
	
	tcPtr6->CTRLA.bit.RUNSTDBY = 1;
	tcPtr6->READREQ.bit.RCONT = 1;				// enable continuous sync.
	tcPtr6->CTRLA.bit.MODE = 0x0;
	tcPtr6->CTRLA.bit.WAVEGEN = 0x3;			// match PWM (page 490)
	tcPtr6->CTRLA.bit.PRESCALER = 0x5;			// TC4 clock = 8MHz/64 = 125kHz
	tcPtr6->CTRLA.bit.PRESCSYNC = 0x1;			// counter run on prescale clock
	tcPtr6->CTRLBCLR.bit.DIR = 1;				// count increment
	
	tcPtr6->CC[0].reg = 617;					// 125 kHz / 200 Hz = 625 (time = 0.005)
	tcPtr6->INTENSET.bit.MC0 = 1;				// enable interrupt when match for CC[0]
	
	NVIC->ISER[0] |= 1u << 19;
	NVIC->IP[4] |= 0x40000000;					// priority level 1

	tcPtr6->CTRLA.reg |= 0x2;					// enable TC4 peripheral
}



// initialize EIC peripheral (used for count positive edge of pin PA28 and PB14)
void init_eic(void)
{
	Port *ports = PORT_INSTS;
	PortGroup *porA = &(ports->Group[0]);
	PortGroup *porB = &(ports->Group[1]);
	// setup PA28, PB14 to EIC peripheral and enable PMUX
	porA->PMUX[14].bit.PMUXE = PORT_PMUX_PMUXE_A;
	porB->PMUX[7].bit.PMUXE = PORT_PMUX_PMUXE_A;		
	porA->PINCFG[28].bit.PMUXEN = PORT_PINCFG_PMUXEN;
	porB->PINCFG[14].bit.PMUXEN = PORT_PINCFG_PMUXEN;
	
	eicPtr->CTRL.reg &= ~(0x2);						// disable EIC peripheral
	
	//eicPtr->CONFIG[1].bit.FILTEN0 = 1;			// detect rising edge for EXTINT[8]
	eicPtr->CONFIG[1].bit.SENSE0 =	0x1;			// [n*8 + x] = 8
	//			   n           x
	//eicPtr->CONFIG[1].bit.FILTEN6 = 1;			// detect rising edge for EXTINT[14] 
	//eicPtr->CONFIG[1].bit.SENSE6 =	0x1;
	
	NVIC->ISER[0] |= 1u << 4;
	NVIC->IP[1] |= 0x00000000;						// priority level 0
	
	eicPtr->INTENSET.bit.EXTINT8 = 1;				// enable external interrupt for EXTINT[8]
	
	while (eicPtr->STATUS.bit.SYNCBUSY & EIC_STATUS_SYNCBUSY) {	/* Wait for setup to complete */ }	
	eicPtr->CTRL.reg |= 0x2;						// enable EIC peripheral
}



void wait(int t)
{
	volatile int count = 0;
	while (count < t*800)
	{
		count++;
	}
}



//Simple Clock Initialization
void Simple_Clk_Init(void)
{
	/* Various bits in the INTFLAG register can be set to one at 
	startup. This will ensure that these bits are cleared */
	SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | 
	SYSCTRL_INTFLAG_BOD33DET |
	SYSCTRL_INTFLAG_DFLLRDY;
	system_flash_set_waitstates(0);					// Clock_flash wait state =0
	
	/* for OSC8M initialization  */
	SYSCTRL_OSC8M_Type temp = SYSCTRL->OSC8M;      
	temp.bit.PRESC    = 0;							// no divide, set clock = 8Mhz  (see page 170)
	temp.bit.ONDEMAND = 1;							// On-demand is true
	temp.bit.RUNSTDBY = 0;							// Run on standby is false
	SYSCTRL->OSC8M = temp;
	SYSCTRL->OSC8M.reg |= 0x1u << 1;				// SYSCTRL_OSC8M_ENABLE bit = bit-1 (page 170)
	
	PM->CPUSEL.reg = (uint32_t)0;					// CPU and BUS clocks Divide by 1  (see page 110)
	PM->APBASEL.reg = (uint32_t)0;					// APBA clock 0= Divide by 1  (see page 110)
	PM->APBBSEL.reg = (uint32_t)0;					// APBB clock 0= Divide by 1  (see page 110)
	PM->APBCSEL.reg = (uint32_t)0;					// APBC clock 0= Divide by 1  (see page 110)
	PM->APBAMASK.reg |= 01u << 3;					// Enable Generic clock controller clock (page 149)
	
	/* Software reset Generic clock to ensure it is re-initialized correctly */
	GCLK->CTRL.reg = 0x1u << 0;						// Reset gen. clock (see page 94)
	while (GCLK->CTRL.reg & 0x1u ) {	/* Wait for reset to complete */ }
	
	// Initialization and enable generic clock #0
	*((uint8_t*)&GCLK->GENDIV.reg) = 0;				// Select GCLK0 (page 121)
	GCLK->GENDIV.reg  = 0x0100;						// Divide by 1 for GCLK #0 (page 121)
	GCLK->GENCTRL.reg = 0x030600;					// GCLK#0 enable, Source=6(OSC8M), IDC=1 (page 118)
}
