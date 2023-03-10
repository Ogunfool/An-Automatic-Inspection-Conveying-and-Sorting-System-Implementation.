#include <avr/interrupt.h>
#include <stdlib.h>//
#include <avr/io.h>
#include "lcd.h"
#include "LinkedQueue.h"

//TIMER
void mTimer(int count);
void rampTimer();
//FIFO is in LinkedQueue.h
//STEPPER
void init();
void rtCW(int step, int delay);
void rtCCW(int step, int delay);
//DC MOTOR
void PWM();
void motorRotate();
void motorRotateFWD();
void motorBrake();
void motorStop();

// Global Variables
volatile char STATE;//for goto statements
volatile int ind = 0;//set up "step number" global variable for stepper									STEPPER
volatile int step;//what tray is facing the conveyor 0:black 1:aluminum 2:white 3:steel					STEPPER
volatile int IR_min;//lowest value ready by IR sensor. Reset to 1023 after each part adc				ADC
volatile int min = 1023;//CALIBRATION MIN RL sensor values												ADC
volatile int max = 0;//CALIBRATION MAX																	ADC
volatile int running = 1;//system on/off																Pause
//initialize the FIFO Q		global pointers do NOT need volatile
link *head;			// The ptr to the head of the queue
link *tail;			// The ptr to the tail of the queue
link *newLink;		// A ptr to a link aggregate data type (struct)
link *rtnLink;		// same as the above

//IA=PB0 IB=PB1 EA=PB2 EB=PB3		DC MOTOR wiring
//Stepper	E1	L1	L2	E2	L3	L4 Wiring
//PORTA		0	1	2	3	4	5
//Arrays do NOT need volatile
int array[4] = {27, 29, 45, 43};//double phase //stepper sequence	arrays dont need volatile	//{3, 24, 5, 40};//{0b00000011, 0b00011000, 0b00000101, 0b00101000}; single phase
int sorted[4] = {0,0,0,0};//Number of each object sorted 0:black 1:aluminum 2:white 3:steel
int values[8] = {949,1000,1,262,808,948,263,807};//black, aluminum, white, steel. [min,max]

int main(int argc, char *argv[]){
	CLKPR = 0x80;
	CLKPR = 0x01;// sets system clock to 8MHz
	TCCR1B|=_BV(CS11);//prescale timer 1
	STATE = 0;
	
	
	cli();// Disables all interrupts
	DDRD = 0b11110000;	//set up INT2 & INT3 on PORTD
	DDRC = 0xFF;		// LCD/Red LED output
	DDRA = 0b00111111;//Set bits 0-5 to output 6-7 to input. Stepper and hall effect respectively
	DDRB = 0xff;//B7 used for PWM output										
	DDRL = 0b11110000;//green and yellow LEDs for signaling
	//additionally PF0(ADC0) is used for input from ADC
	
	// Set up the external Interrupts 0-3
	EICRA |= _BV(ISC01) | _BV(ISC00); //rising edge				Pause
	EICRA |= _BV(ISC11);//falling edge							Ramp Down
	EICRA |= _BV(ISC21) | _BV(ISC20);//rising edge				Reflective Laser
	EICRA |= _BV(ISC31);//falling edge							Exit Gate
	EIMSK |= (_BV(INT0)) | (_BV(INT1)) | (_BV(INT2)) | (_BV(INT3)); // enable Interrupts 0,1,2,3			 
	// config ADC
	ADCSRA |= _BV(ADEN); // enable ADC
	ADCSRA |= _BV(ADIE); // enable interrupt of ADC
	ADMUX |= _BV(REFS0); // _BV(ADLAR) | 'ADLAR' left adjusts. REF sets reference voltage. MUST BE RIGHT ADJUSTED!!!

	// Enable all interrupts
	sei();	// Note this sets the Global Enable for all interrupts
	
	//initialize the FIFO
	rtnLink = NULL;
	newLink = NULL;	
	setup(&head, &tail);
	
	//Initialize the stepper motor
	init();
	step = 0;//signals it is on black
	
	//initialize/start the DC motor
	PWM();
	motorRotateFWD();
	OCR0A = 95;//setting PWM. for duty cycle take % of 255

	goto POLLING_STAGE;

	// POLLING STATE
	POLLING_STAGE:
	switch(STATE){
		case (0) :
		goto POLLING_STAGE;
		break;	//not needed but syntax is correct
		case (1) :
		goto MAGNETIC_STAGE;//currently not in use
		break;
		case (2) :
		goto REFLECTIVE_STAGE;//currently not in use
		break;
		case (3) :
		goto BUCKET_STAGE;
		break;
		case(4):
		goto PAUSE;
		break;
		case (5) :
		goto END;
		default :
		goto POLLING_STAGE;
	}//switch STATE
	

	MAGNETIC_STAGE://CURRENTLY NOT USED
	// Do whatever is necessary HERE
	PORTC = 0x01; // Just output pretty lights know you made it here
	//Reset the state variable
	STATE = 0;
	goto POLLING_STAGE;

	REFLECTIVE_STAGE://CURRENTLY NOT USED
	/*Item has entered the reflective sensor section
	1. Read analog data from reflective sensor
	2. Convert into digital signal/data
	3. Add link to queue containing the part data
	4.*/
	

	STATE = 0;//Reset the state variable
	goto POLLING_STAGE;
	
	BUCKET_STAGE:/*Item has hit end of conveyor
	1. Reset state
	2. Check we do not have an empty Q
	3. Remove oldest link from queue
	4. rotate tray to proper position & update position and parts sorted
	5. Free memory
	6. if another part has already been sensed run conveyor for a short time to get first one off.
	7. restart conveyor. speed based on Q size*/
	//STEP 1
	STATE = 0;//Reset the state variable
	//motorBrake();

	//STEP 2
	if(size(&head, &tail) == 0){//if Q is empty do not run stepper sequence
		motorRotateFWD();
		STATE = 0;
		goto POLLING_STAGE;
	}//if
	
	//STEP 3
	dequeue(&head, &tail, &rtnLink);//sets rtnLink to the first item in the list and delete that item from the list
	
	//STEP 4
	mTimer(60);//Pause to ensure stepper in correct position. Alter this duration if the PWM changes
	switch(((step+4)-(rtnLink->e.obj))%4){//determine where the part must go respective of current position
		case(0):
			break;
		case(1):
			rtCCW(50,16);
			break;
		case(2):
			rtCW(100,16);
			break;
		case(3):
			rtCW(50,16);
			break;	
		default:
			break;
	}//switch case
	step = rtnLink->e.obj;//set new step to the sorted object 
	sorted[rtnLink->e.obj]++;//increments the count for the number of that type that have been sorted
	
	//STEP 5
	free(rtnLink);
	
	//STEP 6
	if(STATE == 3){
		PORTL = 0b11110000;//indicator lights
		motorRotateFWD();//
		mTimer(30);//
	}
	
	//STEP 7
	if(size(&head,&tail)==0){
	OCR0A = 95;//once no more are between sensor and exit, turn speed back down
	}
	motorRotateFWD();
		
	goto POLLING_STAGE;
	//END OF BUCKET#########################################################################################################	
	
	
	PAUSE:
	if(running == 0){//if paused
		running = 1;
		motorRotateFWD();
		LCDClear();
		STATE = 0;
		goto POLLING_STAGE;
	}
	else{//assumes running
		running = 0;
		motorBrake();
		InitLCD(LS_BLINK|LS_ULINE);
		LCDClear();
		LCDWriteString("B:  A:  W:  S:");//print out sorted & unsorted part counts
		LCDWriteIntXY(2,0,sorted[0],2);
		LCDWriteIntXY(6,0,sorted[1],2);
		LCDWriteIntXY(10,0,sorted[2],2);
		LCDWriteIntXY(14,0,sorted[3],2);
		LCDWriteStringXY(0,1,"Un-Sorted:")
		LCDWriteIntXY(11,1,size(&head,&tail),1);
		STATE = 0;
		goto POLLING_STAGE;
	}
	//END OF PAUSE##########################################################################################################
	
	END:
	// The closing STATE triggered by timer 3 (Ramp down). Makes system safe 'kill switch'
	cli();//disable interrupts
	return(0);
	//END OF END############################################################################################################

}//main

/* Set up the External Interrupt 0 Vector *///SYSTEM PAUSE:right hand button
ISR(INT0_vect){//rising edge
	mTimer(20);//Wait 20ms for Debouncing
	while((PIND & 0x01) == 0x01);//check if the push button is back up
	mTimer(20);//Wait 20ms for Debouncing	
	STATE = 4;
}//ISR 0

/* Set up the External Interrupt 1 Vector *///SYSTEM RAMP DOWN:left hand button
ISR(INT1_vect){//falling edge
	mTimer(20);//Wait 20ms for Debouncing
	while((PIND & 0x02) != 0x02);//check if the push button is back up
	mTimer(20);//Wait 20ms for Debouncing
	rampTimer();//starts the ramp down timer
}//ISR 1

/* Set up the External Interrupt 2 Vector *///Entrance Sensor
ISR(INT2_vect){//rising edge
	if((PIND & 0x04) == 0x04){//Software filter
		IR_min=1023;//reset value for reflective sensor. will get decreased as part goes through to get min value
		ADCSRA |= _BV(ADSC);//initialize ADC and start ONE conversion
		STATE = 0;// return to state machine
	}
}//ISR 2

/* Set up the External Interrupt 3 Vector *///Exit Sensor
ISR(INT3_vect){//falling edge
	if((PIND & 0x08) != 0x08){//Software filter
		motorBrake();
		OCR0A = 220;//setting PWM. for duty cycle take % of 255
		STATE = 3;
		//STATE = 0;
	}
}//ISR 3

//  interrupt triggered when ADC is done 
ISR(ADC_vect){
	if(ADC<IR_min){//check if ADC result is the lowest seen value.
		IR_min = ADC;//set lowest seen value to the ADC result
	}
	if((PIND & 0x04) == 0x04){//check laser is still broken (ie part is in sensor)
		ADCSRA |= _BV(ADSC);//initialize ADC and start ONE conversion
	}
	else{//if no part is in sensor
		//start a new link
		initLink(&newLink);
		if(IR_min >= values[0] && IR_min <= values[1]){//Black
			newLink->e.obj = 0;
		}
		else if(IR_min >= values[2] && IR_min <= values[3]){//Aluminum
			newLink->e.obj = 1;
		}
		else if(IR_min >= values[4] && IR_min <= values[5]){//White
			newLink->e.obj = 2;
		}
		else if(IR_min >= values[6] && IR_min <= values[7]){//Steel
			newLink->e.obj = 3;
		}		
		enqueue(&head, &tail, &newLink);
		//LCDWriteInt(newLink->e.obj,1);//for debugging
	}//else
}//ADC ISR

//triggered when timer 3 reaches compare value (8 seconds after ramp down is initiated)
ISR(TIMER3_COMPA_vect){//stop motor and print out sorted counts onto LCD then enter END state which disables all interrupts
	motorStop();
	InitLCD(LS_BLINK|LS_ULINE);
	LCDClear();
	LCDWriteString("B:  A:  W:  S:");
	LCDWriteIntXY(2,0,sorted[0],2);
	LCDWriteIntXY(6,0,sorted[1],2);
	LCDWriteIntXY(10,0,sorted[2],2);
	LCDWriteIntXY(14,0,sorted[3],2);
	STATE = 5;//send to the end state
}//ramp timer ISR


// If an unexpected interrupt occurs (interrupt is enabled and no handler is installed,
// which usually indicates a bug), then the default action is to reset the device by jumping
// to the reset vector. You can override this by supplying a function named BADISR_vect which
// should be defined with ISR() as such. (The name BADISR_vect is actually an alias for __vector_default.
// The latter must be used inside assembly code in case <avr/interrupt.h> is not included.
ISR(BADISR_vect){
	// user code here
	while(1){//FLASH LIGHTS SO YOU KNOW SOMETHING IS WRONG
		PORTC = 0b01010101;
		mTimer(1000);
		PORTC = 0x00;
		mTimer(500);
	}
}//BAD_ISR

/**************************************************************************************/
/**************************************************************************************/
/***************************** SUBROUTINES ********************************************/
/**************************************************************************************/
/**************************************************************************************
* DESC: WAITS A SPECIFIC TIME
* INPUT: NUMBER OF MILISECONDS DESIRED TO WAIT
*/
void mTimer(int count){
	
	int i;//keeps track of loop number
	i = 0;//initilizes loop counter to 0
	
	TCCR1B |=_BV(WGM12);//sets to clear timer on compare. mode 4 in datasheet
	
	OCR1A = 0x03E8;//3E8 is 1000. output compare register for 1000cycles = 1ms
	
	TCNT1 = 0x0000;//sets initial timer counter to 0
	
	TIFR1 |=_BV(OCF1A);//clear the timer interrupt flag before beginning new timing
	
	while(i<count){//polls the timer to determine when counter has reached compare value
		
		if((TIFR1 & 0x02) == 0x02){
			
			TIFR1 |= _BV(OCF1A);//clears interrupt flag
			
			i++;//increment loop number
		}//end if statement
	}//end while
	return;
}/*mTimer*/


/**************************************************************************************
* DESC: WAITS A SPECIFIC TIME BEFORE TRIGGERING AN INTURUPT uses timer 3 since it is 16bit
* INPUT: NUMBER OF MILISECONDS DESIRED TO WAIT
*/
void rampTimer(){

TCCR3B |=_BV(WGM32);
TCCR3B|=_BV(CS32) | _BV(CS30);//prescale timer 3 by 1024 

OCR3A = 0xF424;//62500cycles. 8s 

TCNT3 = 0x0000;//set initial timer value to 0

TIMSK3 = TIMSK3 | 0b00000010;//enable output compare interrupt (bit 1)
}/*rampTimer*/
 
/******************************************************************************************	FIFO Q	**************************************************************************************/
/**************************************************************************************
* DESC: initializes the linked queue to 'NULL' status
* INPUT: the head and tail pointers by reference
*/
void setup(link **h,link **t){
	*h = NULL;		/* Point the head to NOTHING (NULL) */
	*t = NULL;		/* Point the tail to NOTHING (NULL) */
	return;
}/*setup*/


/**************************************************************************************
* DESC: This initializes a link and returns the pointer to the new link or NULL if error 
* INPUT: newlink by reference
*/
void initLink(link **newLink){
	//link *l;
	*newLink = malloc(sizeof(link));
	(*newLink)->next = NULL;
	return;
}/*initLink*/


/****************************************************************************************
*  DESC: Accepts as input a new link by reference, and assigns the head and tail		
*  of the queue accordingly				
*  INPUT: the head and tail pointers, and a pointer to the new link that was created 
*/
/* will put an item at the tail of the queue */
void enqueue(link **h, link **t, link **nL){

	if (*t != NULL){
		/* Not an empty queue */
		(*t)->next = *nL;
		*t = *nL; //(*t)->next;
	}/*if*/
	else{
		/* It's an empty Queue */
		//(*h)->next = *nL;
		//should be this
		*h = *nL;
		*t = *nL;//if empty: head=tail
	}/* else */
	return;
}/*enqueue*/


/**************************************************************************************
* DESC : Removes the link from the head of the list and assigns it to deQueuedLink
* INPUT: The head and tail pointers, and a ptr 'deQueuedLink' 
* 		 which the removed link will be assigned to
*/
/* This will remove the link and element within the link from the head of the queue */
void dequeue(link **h, link **t, link **deQueuedLink){
	*deQueuedLink = *h;	// Will set to NULL if Head points to NULL
	/* Ensure it is not an empty queue */
	if (*h != NULL){
		if(*h == *t){//if queue only has one item reset queue (head and tail = NULL)
			*h=NULL;
			*t=NULL;
		}
		else{
			*h = (*h)->next;
		}
	}/*if*/
	
	return;
}/*dequeue*/


/**************************************************************************************
* DESC: Peeks at the first element in the list
* INPUT: The head pointer
* RETURNS: The element contained within the queue
*/
/* This simply allows you to peek at the head element of the queue and returns a NULL pointer if empty */
element firstValue(link **h){
	return((*h)->e);
}/*firstValue*/


/**************************************************************************************
* DESC: deallocates (frees) all the memory consumed by the Queue
* INPUT: the pointers to the head and the tail
*/
/* This clears the queue */
void clearQueue(link **h, link **t){

	link *temp;

	while (*h != NULL){
		temp = *h;
		*h=(*h)->next;
		free(temp);
	}/*while*/
	
	/* Last but not least set the tail to NULL */
	*t = NULL;		

	return;
}/*clearQueue*/


/**************************************************************************************
* DESC: Checks to see whether the queue is empty or not
* INPUT: The head pointer
* RETURNS: 1:if the queue is empty, and 0:if the queue is NOT empty
*/
/* Check to see if the queue is empty *//*
char isEmpty(link **h){
	
	return(*h == NULL);
}*//*isEmpty*/


/**************************************************************************************
* DESC: Obtains the number of links in the queue
* INPUT: The head and tail pointer
* RETURNS: An integer with the number of links in the queue
*/
/* returns the size of the queue*/
int size(link **h, link **t){

	link 	*temp;			/* will store the link while traversing the queue */
	int 	numElements;

	numElements = 0;

	temp = *h;			/* point to the first item in the list */

	while(temp != NULL){
		numElements++;
		temp = temp->next;
	}/*while*/
	
	return(numElements);
}/*size*/




/******************************************************************************************	STEPPER	**************************************************************************************/
/**************************************************************************************
* DESC: Initiate stepper using the following CCW & CW functions. Runs CCW until the hall effect sensor is triggered
* INPUT:
*/
void init(){
	while((PINA & 0x40) == 0x40){//rotate until hall effect triggered	//PORTA06 HE is a falling edge
		rtCCW(1,20);
	}
	rtCCW(10,10);//Bin correction. rtCW or ctCCW a few steps to get bin properly lined up. Number of steps will be station dependent 
}/*init*/


/**************************************************************************************
* DESC: rotate Clockwise, accelerating for the first 10 steps and decelerating for the last 10
* INPUT: int: number of steps to rotate, int: initial delay between steps (in ms)
*/
void rtCW(int step, int delay){
	int i = 0;
	while(i < step){
		ind++;
		if(ind == 4) ind = 0;//ensures value never leave array bounds of 0-3
		PORTA = array[ind];
		mTimer(delay);
		i++;		
		if(step == 100){//for 180degree rotation
			if(i <= 10) delay--;
			if(i >= 90) delay++;
		}
		if(step == 50){//for 90degree rotation
			if(i <= 10) delay--;
			if(i >= 40) delay++;			
		}
	}//while
}/*rtCW*/


/**************************************************************************************
* DESC: rotate CounterClockwise, accelerating for the first 10 steps and decelerating for the last 10
* INPUT: int: number of steps to rotate	int: initial delay between steps (in ms)
*/
void rtCCW(int step, int delay){
	int i = 0;
	while(i < step){
		ind--;
		if(ind == -1) ind = 3;//ensures value never leave array bounds of 0-3
		PORTA = array[ind];
		mTimer(delay);
		i++;		
		if(step == 100){//for 180degree rotation
			if(i <= 10) delay--;
			if(i >= 90) delay++;			
		}
		if(step == 50){//for 90degree rotation
			if(i <= 10) delay--;
			if(i >= 90) delay++;			
		}
	}//while
}/*rtCCW*/


/******************************************************************************************	DC MOTOR **************************************************************************************/
/* ################## PWM SUBROUTINE ################## */
void PWM (){
	DDRB |= (1 << PB7);  //Set the designated PWM pin for Timer 0 to output.

	TCCR0B |= (1 << CS01) | (1 << CS00); //Pre-scaler set to 64  to achieve frequency 500hz on PWM.

	TCCR0A |= (1 << COM0A1) | (1 << WGM01) |(1 << WGM00); //Sets bits to fast PWM mode and COM0A1:0 bits to clear on compare match.

}/*PWM*/

/**************************************************************************************
* DESC: rotates motor counter clockwise. results in motion of conveyor belt from right to left
* INPUT: N/A
*/
void motorRotateFWD(){
	PORTB = 0B00001110; //Writes logic to IN AND EN pin connections for counterclockwise movement
}/*motorRotateFWD*/

/**************************************************************************************
* DESC: Applies an electronic brake to the motor. Will stop and not allow coasting
* INPUT: N/A
*/
void motorBrake(){
	PORTB = 0B00001111; //Brake to high
}/*motorBrake*/

/**************************************************************************************
* DESC: Stops suppling power to the motor (Disables). Will allow coasting
* INPUT: N/A
*/
void motorStop(){
	PORTB = 0B00000000; //removes power from motor controller
}/*motorStop*/