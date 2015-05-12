#include <c8051_SDCC.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <i2c.h>

#define PW1MS1		63508
#define PW1MS5		62771
#define PW1MS9		62034

unsigned int SERVO_PW  = 2735;
unsigned int MOTOR_PW  = 0;
unsigned int SERVO_MAX = 3335; //Value for the wheels to be left
unsigned int SERVO_MIN = 2185; //Value for the wheels to be right
unsigned int PW_CENTER = 2735; //Value for the wheels to be straight
int temp;

__bit flag_lcd = 0;
__bit flag_accl = 0;

unsigned int PCACounter = 0;
unsigned int motor_min,motor_max;
__idata unsigned int drive_p=0,    drive_i=0,    drive_d=0,    drive_t=0;
__idata unsigned int steering_p=0, steering_i=0, steering_d=0, steering_t=0;
signed int accl_x, accl_y;

__sbit __at 0xB7 RUN;
#define BATT_ADC_PIN 6

void Port_Init(void);
void XBR0_Init(void);
void SMBus_Init(void);
void ADC_Init(void);
void Interrupt_Init(void);
void PCA_Init(void);
void motor_init(void);
void PCA_ISR( void ) __interrupt 9;
	
unsigned char read_AD_input( unsigned char n );
void set_motor_speed(signed char speed);
void pause(void);
void get_and_display_status(void);
signed int prompt_input (char * prompt, unsigned char digit);
void update_accl(void);
void set_output(void);	
	
void main(void) {
	//Local Variables
	
	//Initialization Functions
	Sys_Init();
	putchar(' ');
	Port_Init();
	XBR0_Init();
	SMBus_Init();
	ADC_Init();
	Accel_Init();
	Interrupt_Init();
	PCA_Init();
	motor_init();
	
	while ( PCACounter < 50 );	//Waits 50 overflows (1.778 seconds)
	lcd_clear();		//clears the lcd of the bootup message
	set_motor_speed( 0 );
	while ( 1 ) {
		if ( !RUN ) { //Prompts user for gains
			set_motor_speed( 0 );
			drive_p = prompt_input("Input drive proportional gain\n",3);
			drive_d = prompt_input("Input drive derivative gain\n",3);
			steering_p = prompt_input("Input steering proportional gain\n",3);
			steering_d = prompt_input("Input steering derivative gain\n",3);
			while (!RUN){}
		}
		while (RUN) {
			if( flag_accl ) {
				flag_accl = 0;
				update_accl();
				set_output();
			}
			if( flag_lcd ) {//Display status
				get_and_display_status();
				printf("X:%6d\tY:%6d\tServo:%6d\tMotor%6d\ttemp:%d\r\n", accl_x, accl_y, SERVO_PW, MOTOR_PW, temp);
				flag_lcd = 0;
			}
		}
	}
}


//Functions

//============= init ======================
void Port_Init(void) {
	P1MDIN  &= ~0x40;
	P1MDOUT = 0x05; //set output pin for CEX0 in push-pull mode
					//Open pin 0 and 2
	P1 		|= 0x40;
	P3MDOUT &= ~0x80; //set P3.7 to open drain (input)
	P3 |= 0x80; //Set P3.7 to high impedance
}

void XBR0_Init(void) {
	XBR0 = 0x27;
}

void Interrupt_Init(void){
	EIE1 |= 0x08; //enables interrupts
	EA = 1;
}

void PCA_Init(void){
	PCA0MD &= 0xF1;	
	PCA0MD |= 0x01;
	PCA0CPM0 = 0xC2;
	PCA0CPM2 = 0xC2;
	PCA0CN = 0x40;
	PCA0 = 28671; //20 ms preset
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
	//Initialization of motor controller
	motor_min = PW1MS9;
	motor_max = PW1MS1;

}

// ==================isr ==================
void PCA_ISR(void) __interrupt 9 {
	if ( CF ) {
		PCA0 = 28672; //presets the buffer
		CF = 0;
		PCACounter++;
		flag_accl =1;
		if ( PCACounter % 20 == 0 ) {
			flag_lcd = 1;
		}
	}
}

// ============= low level calls =====================
unsigned char read_AD_input( unsigned char n ) { //reads the value on port 0 pin n
	AMX1SL = n;
	ADC1CN = (ADC1CN & ~0x20) | 0x10;
	while (!(ADC1CN & 0x20));
	return ADC1;
}

// ================ acclerometer ====================
void update_accl(void) {
	signed char i2c_buffer[4] = {0,0,0,0};

	i2c_read_data(0x30,0x27,i2c_buffer,1);
	if ( (i2c_buffer[0]&0x03) != 0x03 ) return;
		// data not ready, not updating this times

	i2c_read_data(0x30,(0x28|0x80),i2c_buffer,4);
	//printf("XY\t%d\t%d\n\r",(short)i2c_buffer[1]*2,(short)i2c_buffer[3]*2);
	accl_x  = ((long)accl_x * 7 + (short)i2c_buffer[1]*16)/8; //*16/8
	accl_y  = ((long)accl_y * 7 + (short)i2c_buffer[3]*16)/8;
}

void set_output(void) {
	static int last_x=0,last_y=0;
	static int error_sum_x=0, error_sum_y=0;
	int dx=accl_x-last_x, dy=accl_y-last_y;
	int error_x=accl_x-steering_t, error_y= accl_y-drive_t;
	temp =   drive_p * error_y
	                  + drive_d * dy
	                  + drive_i * error_sum_y
	                  + drive_p * abs(error_x) ;
	//printf("temp: %d", temp);
	error_sum_x +=error_x;
	error_sum_y +=error_y;
	if(temp > 8128){temp = 8128;}
	set_motor_speed( ( temp          // assume drive_px=1 here
	                 ) /64 );
	//set_motor_speed(127);
	//PCA0CP2 = PW1MS9;
	SERVO_PW = PW_CENTER - (  steering_p * error_x
	                        + steering_d * dx
	                        + steering_i * error_sum_y )/8;
	if (SERVO_PW < SERVO_MIN)       SERVO_PW = SERVO_MIN;
	else if (SERVO_PW > SERVO_MAX)  SERVO_PW = SERVO_MAX;
	PCA0CP0 = 0xFFFF - SERVO_PW;
	last_x = accl_x; last_y = accl_y;
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
	MOTOR_PW = 0xFFFF - pcacp;
	PCA0CP2 = pcacp;
}

// --------------------UI-------------------------

signed int prompt_input (char * prompt, unsigned char digit) { 
	char key = 0;
	signed int value=0;
	__bit negative = 0;
	unsigned char digit_left=digit;
	while (digit_left) {
		pause();
		lcd_clear();
		lcd_print(prompt);
		lcd_print("%i",value);
		while( read_keypad() == -1){ pause(); }
		key = read_keypad();
		while(read_keypad() != -1){ pause();}
		switch (key) {
			case '#':
				if (value == 0 && negative == 0) negative = 1;
				else            digit_left = 0;
				break;
			case '*':
				digit_left = digit;
				value = 0;
				negative = 0;
				break;
			default:
				--digit_left;
				if (negative) value = value *10 - (key-'0');
				else          value = value *10 + (key-'0');
				break;
		}
	}
	lcd_clear();
	return value;
}

void get_and_display_status (void) { //Displays the battery voltage, heading, and current ranger value on the lcd
	static unsigned int batt_volt;
	static __bit update_batt=0;
	update_batt = !update_batt;
	if (update_batt) { //Reads from the a/d conversion
		batt_volt = ( (unsigned int)read_AD_input(BATT_ADC_PIN) * 150 ) / UCHAR_MAX;	//15.0 V ~ 255
	}
	lcd_clear();
	lcd_print("BAT:%2u.%1uV",batt_volt/10,batt_volt%10);
	pause();
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
