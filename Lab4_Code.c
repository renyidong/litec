#include <c8051_SDCC.h>
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>

/* 
unsigned int PW_CENTER = 
unsigned int PW_RIGHT = 
unsigned int PW_LEFT = 
*/
unsigned int SERVO_PW = 2735;
unsigned int SERVO_MAX = 3335;
unsigned int SERVO_MIN = 2185;
unsigned char new_heading = 0;
unsigned char new_range = 0;
unsigned char new_lcd = 0;
unsigned int heading;
unsigned int range;
int compass_adj = 0;
int range_adj = 0;
unsigned char lcd_count;
unsigned char r_count;
unsigned char h_count;
__sbit __at 0xB7 RUN;
	

void Port_Init(void)
void XBR0_Init(void)
void SMBus_Init(void)
void ADC_Init(void)
void Interrupt_Init(void)
void PCA_Init(void)
void Output_Init(void)
	
void PCA_ISR( void ) __interrupt 9;
	
int Update_Value( int Constant, unsigned char incr, int maxval, int minval );
unsigned char read_AD_input( unsigned char n );
int read_compass( void );
void set_servo_PWM( void );
int read_ranger( void );
void set_drive_PWM( void );
int pick_heading(void);
int pick_range(void);
	
	
void main(void) {
	//Local Variables
	unsigned char run_stop
	//Initialization Functions
	Sys_Init();
	putchar(' ');
	Port_Init();
	XBR0_Init();
	SMBus_Init();
	ADC_Init();
	Interrupt_Init();
	PCA_Init();
	Output_Init();
	
	r_count = 0;
	h_count = 0;
	while ( 1 ) {
		run_stop = 0;
		while ( !RUN ) {
			if (run_stop == 0) {
				desired_heading = pick_heading();
				desired_range = pick_range();
				run_stop = 1;
			}
		}
		if ( new_heading ) {
			heading = read_compass();
			set_servo_PWM();
			new_heading = 0;
			h_count = 0;
		}
		if (new_range){
			range = read_ranger() ;
			set_range_adj();
			new_range = 0;
			r_count = 0;
		}
	}
}


//Functions

void Port_Init(void) {
	P1MDOUT = 0x05; //set output pin for CEX0 in push-pull mode
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



void PCA_ISR(void) __interrupt 9 {
	if ( CF ) {
		CF = 0;
		h_count++;
		if ( h_count >= 2 ) {
			new_heading = 1;
			h_count = 0;
		}
		r_count++;
		if ( r_count >= 4 ){
			new_range = 1;
			r_count = 0;
		}
		lcd_count++;
		if ( lcd_count >= 20 ){
			new_lcd = 1;
			lcd_count = 0;
		}
		PCA0 = PCA_start;
	}
	PCA0CN &= 0xC0;
}

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

unsigned char read_AD_input( unsigned char n ) {
	AMX1SL = n;
	ADC1CN = ADC1CN & ~0x20;
	ADC1CN = ADC1CN | 0x10;
	while ((ADC1CN & 0x20) == 0x00);
	return ADC1;
}

int read_compass( void ){
	unsigned char addr = 0xC0;
	unsigned char Data[2];
	unsigned int heading;
	i2c_read_data(addr, 2, Data, 2);
	heading =(((unsigned int)Data[0] << 8) | Data[1]);
	return heading
}

void set_servo_PWM( void ){
	set PCACP0 to the correct pulsewidth
	PCACP0 = 0xFFFF - PW
}










