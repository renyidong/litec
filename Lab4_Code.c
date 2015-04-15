
#include <c8051_SDCC.h>
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>
#include <limits.h>

#define PW1MS1		63508
#define PW1MS5		62771
#define PW1MS9		62034

unsigned int SERVO_PW = 2735;
unsigned int SERVO_MAX = 3335;
unsigned int SERVO_MIN = 2185;
unsigned int PW_CENTER = 2735;


unsigned int heading;
unsigned int range;
int compass_adj = 0;
int range_adj = 0;
__bit updateCompass = 0;
__bit updateRanger = 0;
__bit updateLCD = 0;

unsigned int desired_heading;
unsigned int PCACounter = 0;
unsigned int motor_min,motor_max;
int range_gain;
char keypad;

__sbit __at 0xB7 RUN;
#define BATT_ADC_PIN 4
	

void Port_Init(void);
void XBR0_Init(void);
void SMBus_Init(void);
void ADC_Init(void);
void Interrupt_Init(void);
void PCA_Init(void);

void motor_init(void);

	
void PCA_ISR( void ) __interrupt 9;
	
int Update_Value( int Constant, unsigned char incr, int maxval, int minval );
unsigned char read_AD_input( unsigned char n );
int read_compass( void );
void set_servo_PWM( void );
int read_ranger( void );
void set_drive_PWM( void );
unsigned int pick_heading(void);
int pick_gain(void);
void set_motor_speed(signed char speed);
void pause(void);

int compassADJ( void );
	
	
void main(void) {
	//Local Variables
	unsigned char run_stop;
	
	//Initialization Functions
	Sys_Init();
	putchar(' ');
	Port_Init();
	XBR0_Init();
	SMBus_Init();
	ADC_Init();
	Interrupt_Init();
	PCA_Init();
	motor_init();
	
	while ( PCACounter < 50 );	//Waits 50 overflows (1.778 seconds)
	lcd_clear();

	
	while ( 1 ) {
		run_stop = 0;
		while ( !RUN ) {
			if (run_stop == 0) {
				desired_heading = pick_heading() * 10;
				range_gain = pick_gain();
				run_stop = 1;
			}
		}
		
		set_motor_speed( 120 );
		
		if ( updateCompass ) {
			if (range > 12) {
				heading = read_compass();
				set_servo_PWM();
				updateCompass = 0;
			}
			else {
				set_motor_speed(0);
			}
		}
		
		if ( updateRanger ){
			range = read_ranger();
			set_range_adj();
			updateRanger = 0;
			printf("Compass: %d\tRanger:%d\tServo_PW: %u\t",heading,range,SERVO_PW);
		}
	
		if( updateLCD ) {
			//LCD code TODO
			updateLCD = 0;
		}
	
	}
	
}


//Functions

void Port_Init(void) {
	P1MDOUT = 0x05; //set output pin for CEX0 in push-pull mode
					//Open pin 0 and 2
	P3MDOUT &= ~0x80; //set P3.7 to open drain (input)
	P3 |= 0x80; //Set P3.7 to high impedance
}

void XBR0_Init(void) {
	XBR0 = 0x27;
}

void Interrupt_Init(void){
	EIE1 |= 0x08;
	EA = 1;
}

void PCA_Init(void){
	PCA0MD &= 0xF1;
	PCA0MD |= 0x01;
	PCA0CPM0 = 0xC2;
	PCA0CPM2 = 0xC2;
	PCA0CN = 0x40;
	PCA0 = 28671;
}

void SMBus_Init(void){
	SMB0CR = 0x93;
	SMB0CN |= 0x40;
}

void ADC_Init(void){
	REF0CN = 0x03; //
	ADC1CN = 0x80; //
	ADC1CF &= ~0x03; //Clears lowest 2 bits
	ADC1CF |= 0x01; //Gain 1
}

void motor_init(void) {
	char user_input=0;
	PCA0CP2	=PW1MS5;

	PCACounter = 0;
	while (PCACounter<50) {};
	
	motor_min = PW1MS9;
	motor_max = PW1MS1;

	printf("Setting forward speed limit, press d when done.\n\r");
	printf("press f for forward, s for reverse(slower)\n\r");
	while (user_input!='d') {
		user_input = getchar();
		switch(user_input) {
			case 'f':
				if (PCA0CP2 > motor_min) PCA0CP2-=10;
				break;
			case 's':
				if (PCA0CP2 < motor_max) PCA0CP2+=10;
				break;
		}
	}
	motor_min = PCA0CP2;

	PCA0CP2	=PW1MS5;		//1.5ms
	user_input = 0;
	printf("Setting reverse speed limit, press d when done.\n\r");
	printf("press f for forward, s for reverse(slower)\n\r");
	while (user_input!='d') {
		user_input = getchar();
		switch(user_input) {
			case 'f':
				if (PCA0CP2 > motor_min) PCA0CP2-=10;
				break;
			case 's':
				if (PCA0CP2 < motor_max) PCA0CP2+=10;
				break;
		}
	}
	motor_max = PCA0CP2;

	PCA0CP2	=PW1MS5;
	printf("Speed Setting Finish\n\r");
}

void PCA_ISR(void) __interrupt 9 {
	if ( CF ) {
		if ( PCACounter % 2 == 0 ) {
			updateCompass = 1;
		}
		if ( PCACounter % 4 == 0 ) {
			updateRanger = 1;
		}
		if ( PCACounter % 20 == 0 ) {
			updateLCD = 1;
		}
		PCA0 = 28672;
		CF = 0;
		PCACounter++;
	}
}
/*
int Update_Value( int Constant, unsigned char incr, int maxval, int minval ){
	//Local Variables
	int deflt;
	char input;
	deflt = Constant;
	while ( TRUE ) {
		input = getchar();
		if ( input == 'c' ) { 
			Constant = deflt;
		}
		if ( input == 'i' ) {
			Constant += incr;
			if ( Constant > maxval ) {
				Constant = maxval;
			}
		}
		if ( input == 'd' ) {
			Constant -= incr;
			if (Constant < minval) {
				Constant = minval;
			}
		}
		if ( input == 'u' ) {
			return Constant;
		}
	}
}
*/
unsigned char read_AD_input( unsigned char n ) {
	AMX1SL = n;
	ADC1CN = (ADC1CN & ~0x20) | 0x10;
	while (!(ADC1CN & 0x20));
	return ADC1;
}

int read_compass( void ){
	unsigned char addr = 0xC0;
	unsigned char Data[2];
	unsigned int heading;
	i2c_read_data(addr, 2, Data, 2);
	heading =(((unsigned int)Data[0] << 8) | Data[1]);
	return heading;
}

void set_servo_PWM( void ){
	compass_adj = compassADJ();
	range_adj = set_range_adj();
	SERVO_PW = PW_CENTER + compass_adj + range_adj;
	if (SERVO_PW < SERVO_MIN || SERVO_PW > SERVO_MAX ) {
		if (range < 20 ) {
			set_motor_speed(0);
			return;
		}
		if (SERVO_PW < SERVO_MIN) {SERVO_PW = SERVO_MIN;}
		else {SERVO_PW = SERVO_MAX;}
        }
	PCA0CP0 = 0xFFFF - SERVO_PW;
	printf("Compass Adjust: %d\tRanger Adjust:%d\t\r\n",compass_adj,range_adj);
	//set PCACP0 to the correct pulsewidth
	//PCACP0 = 0xFFFF - PW
}

// ----------------------ranger-------------------
int read_ranger (void) {
	unsigned const char command=0x51;
	static unsigned short distance;
	unsigned char raw_data[2];
	i2c_read_data(0xE0,2,raw_data,2);
	if (raw_data[0]!=0xFF&&raw_data[1]!=0xFF){
		distance  += (unsigned short)raw_data[0]<<8 | raw_data[1];
		distance >>= 1;		// use average of old and new to stablize
	}
	i2c_write_data(0xE0,0,&command,1);
	return distance;
}

void set_range_adj(void) {
	const unsigned int MAX_RANGE=50;
	if (range > MAX_RANGE ) range_adj = 0;
	else range_adj = (int)( range_gain * (MAX_RANGE - range) );
}

// ----------------------motor--------------------
void set_motor_speed(signed char speed) {
	unsigned short pcacp;
	if (speed>=0) {
		pcacp = PW1MS5 - (speed * (PW1MS5-motor_min)/SCHAR_MAX);
	}
	else {
		pcacp = PW1MS5 + (speed * (motor_max-PW1MS5)/SCHAR_MIN);
	}
	PCA0CP2 = pcacp;
}

unsigned int pick_heading(void) {
	unsigned int chosenHeading = 0;
	char counter = 0;
	
	while(1){
		lcd_clear();
		lcd_print("Input Desired \nHeading (Under 360)");
		while( counter < 3 ){
			while( read_keypad() == -1){ pause(); }
			keypad = read_keypad();
			lcd_clear();
			pause();
			while(read_keypad() != -1){ pause();}
			if( keypad == '#' ){

				break;
			}
			else if( keypad == '*' ){
				counter = 0;
				chosenHeading = 0;
				lcd_print("Heading: %d", chosenHeading);				
			}
			else {
				chosenHeading *= 10;
				chosenHeading += ( keypad - '0');
				++counter;
				lcd_print("Heading: %d", chosenHeading);
			}
		}		
		lcd_clear();	
		if( chosenHeading < 360 ) {
			break;
		}
	}	
	
	lcd_print("Heading Input Complete");
	return chosenHeading;
}

void pause(void) {
	unsigned int waitCounter;
	if( PCACounter < 65533 ){
		waitCounter = PCACounter;
	}
	else {
		waitCounter = 0;
	}
	while(PCACounter > 65533);
	while( PCACounter - waitCounter != 2 );
}

int pick_gain(void) {
	unsigned int chosenGain = 0;
	char counter = 0;
	
	while(1){
		lcd_clear();
		lcd_print("Input Desired\nGain (Under 999)");
		while( counter < 3 ){
			while( read_keypad() == -1){ pause(); }
			keypad = read_keypad();
			lcd_clear();
			pause();
			if( keypad == '#' ){
				break;
			}
			else if( keypad == '*' ){
				counter = 0;
				chosenGain = 0;
				lcd_print("Gain: %d", chosenGain);				
			}
			else {
				chosenGain *= 10;
				chosenGain += ( keypad - '0');
				++counter;
				lcd_print("Gain: %d", chosenGain);
			}
			while(read_keypad() != -1){ pause();}
		}		
		lcd_clear();	
		if( chosenGain <= 999 ) {
			break;
		}
	}	
	
	lcd_print("Gain Input Complete");
	return chosenGain;
}

int compassADJ( void ){
	int error = heading - desired_heading;		//Calculates the error with the values 
													//shifted towards 0 till desired_heading = 0
	float k = (float)550/(float)1800; //TODO 550 or 600					
	if( error > 1800 ) {
		error = (-1) * ( 3600 - error );			//Calculates the error if actual_heading is between 1800 and 3599
	}
	error *= -1;		//Sets the error into the correct sign
	
	return k * error;
}
