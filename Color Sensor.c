#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define CW 0
#define CCW 1
#define BREAK 0
#define STOP 1
uint8_t dataArray[8];
int i;
unsigned int left = 0;
unsigned int right = 0;
uint16_t C, R, G, B;
int stop;
int start;

void clearStatusFlags() {
    // clear STOPF and Undocumented STARTF
	I2C0->FLT |= (1<<6); // w1c
	I2C0->FLT &= ~(1<<4); // clear bit 4
    // clear ARBL and IICIF
	I2C0->S |= (1<<4); // clear Arbitration loss
	I2C0->S |= (1<<1); // clear IICIF
}

void TCFWait() {
    // Wait for TCF bit to set
	while( !(I2C0->S | 1<<7) ) {}
}

void IICIFWait() {
    // Wait for IICIF bit to set
	while( !(I2C0->S & 1<<1) ) {
	}
}

void SendStart() {
    // set MST for master mode
	// set TX for transmit
	I2C0->C1 |= (1<<5) | (1<<4);
}

void RepeatStart() {
    // set MST for master mode
	// set TX for transmit
	// set RSTA for restart
	I2C0->C1 |= (1<<5) | (1<<4) | (1<<2);
	int i;
	for (i=0; i<10; i++) {
		__asm volatile ("nop");
	}								// Wait 10 cycles
}

void SendStop() {
    // Clear MST, TX and TXAK bits in Control 1 Register
	I2C0->C1 &= ~( (1<<5) | (1<<4) | (1<<3) );
	//I2C0->C1 &= ~(1<<4);
	//I2C0->C1 &= ~(1<<3);
    // Wait for BUSY bit to go low in Status Register
	while( (I2C0->S & (1<<5) ) ) {}
	//printf("Send stop function finished");  // for sanity
}

void clearIICIF() {
	// clear IICIF
	I2C0->S |= (1<<1); // w1c
}

int RxAK() {
    // Return 1 if byte has been ACK'd. (See RXAK in Status Register)
	if( !(I2C0->S) & (0x1) ) {
		return 0;
	}
	return 1;
}

void I2C_write(uint8_t reg_address, uint8_t Data) {
	//printf("Start WRITE\n");
	clearStatusFlags();
	TCFWait();
	//I2C0->C1 |= (1<<3); // TXAK
	SendStart();
	I2C0->D = (0x29 << 1); // color sensor address
	IICIFWait();

	if ( !RxAK() ) {
		SendStop();
		return;
	}

	clearIICIF();
	I2C0->D = (reg_address);
	IICIFWait();

	if ( !RxAK() ) {
		SendStop();
		return;
	}

	TCFWait();
	clearIICIF();
	I2C0->D = (Data);
	IICIFWait();

	if ( !RxAK() ) {
		printf("data NACK");
	}

	clearIICIF();
	SendStop();
	//printf("End WRITE\n");
}

void I2C_read(uint8_t reg_address, uint8_t* data_array, int Length) {
	//printf("Start READ\n");
	unsigned char spare = 0; // avoids a warning
	//int temp = 1;
	clearStatusFlags();
	TCFWait();
	SendStart();
	spare++;
	I2C0->D = (0x29 << 1);
	IICIFWait();

	if ( !RxAK() ) {
		SendStop();
		printf("NACK");
	}

	clearIICIF();
	I2C0->D = (reg_address);
	IICIFWait();

	if ( !RxAK() ) {
		SendStop();
		printf("NACK");
	}

	clearIICIF();
	RepeatStart();
	I2C0->D = (0x29 << 1) | 0x1;
	IICIFWait();

	if ( !RxAK() ) {
		SendStop();
		printf("NACK");
	}

	TCFWait();
	clearIICIF();
	I2C0->C1 &= ~( (1<<3) | (1<<4) );

	if(Length == 1) {
		// set TXAK to NACK; stop requesting data
		I2C0->C1 |= (1<<3);
	}

   spare = I2C0->D; // spare read
   //printf("before FOR\n");
   for(int index = 0; index < Length; index++) {
		IICIFWait();
		//printf("after IICIFWait()\n");
		clearIICIF();
		//printf("after clearIICIF()\n");

		if( (Length - index) == 2) {
			// Set TXAK to NACK in Control 1 - No more data!
			I2C0->C1 |= (0x8);
		}

		if( (Length - index) == 1) {
		   // final byte
		   SendStop();
		}
		// Read Byte from Data Register into Array
		//temp = I2C0->D;
		//printf("temp  %d\n", temp);

		data_array[index] = I2C0->D;

	}
   //printf("after  FOR\n");
}

void init_I2C() {
    // Enable Clock Gating for I2C module
	SIM->SCGC4 |= (1<<6); // I2C0
	PORTC->PCR[8] &= ~(0x700);			// clear mux
	PORTC->PCR[9] &= ~(0x700);			// clear mux
	PORTC->PCR[8] |= (0x700) & (0x200); // set mux as ALT2 (I2C0_SCL)
	PORTC->PCR[9] |= (0x700) & (0x200); // set mux as ALT2 (I2C0_SDA)

    // clear all I2C registers
	I2C0->A1 = 0;
	I2C0->F = 0;
	I2C0->C1 = 0;
	I2C0->S = 0;
	I2C0->D = 0;
	I2C0->C2 = 0;
	I2C0->FLT = 0;
	I2C0->A2 = 0;
	I2C0->RA = 0;
	I2C0->SMB = 0;
	I2C0->SLTH = 0;
	I2C0->SLTL = 0;


	I2C0->FLT |= (0x50); // Write 0x50 to FLT register
	clearStatusFlags();

	I2C0->F &= (0x23); // Set I2C Divider Register
	I2C0->C1 |= (1<<7); // enable I2C module
}

void delay_ms(unsigned short delay_t){
	SIM->SCGC6 |= (1 << 24); //Clock enable TPM0
	SIM->SOPT2 |= (0x2 << 24); //TPM clock source OSCERCLK
	TPM0->CONF |= (0x1 << 17); // Set rest for overflow
	TPM0->SC = (1 << 7) | (0x7);  //Reset TOF, PRescalar 128

	TPM0->MOD = delay_t*62 + delay_t/2;

	TPM0->SC |= 0x01 << 3; //start clock

	while(!(TPM0->SC & 0x80)){}
	return;
}

void motor_control(char motor, char direction, int speed) {

	if(motor == LEFT_MOTOR) {
		TPM2->CONTROLS[0].CnV = speed; //Channel 0 is PTB2 (left motor)
		if(direction == CW) {
			GPIOB->PDOR &= ~(1<<1);
			GPIOB->PDOR |= (1<<0);
		}

		if(direction == CCW) {
			GPIOB->PDOR &= ~0x1;
			GPIOB->PDOR |= 0x2;
		}
	}

	if(motor == RIGHT_MOTOR) {
		TPM2->CONTROLS[1].CnV = speed; //Channel 1 is PTB3 (right motor)
		if(direction == CW) {
			GPIOC->PDOR &= ~0x4;
			GPIOC->PDOR |= 0x2;
		}

		if(direction == CCW) {
			GPIOC->PDOR &= ~0x2;
			GPIOC->PDOR |= 0x4;
		}
	}
}

void motors_setup() {
	SIM->SCGC5 |= (1 << 10 | 1 << 11);

	SIM->SCGC6 |= (1 << 26); // Clock Enable TPM2
	SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK
	TPM2->CONTROLS[0].CnV = 600; //Channel 0 is PTB2
	TPM2->CONTROLS[1].CnV = 600; //Channel 1 is PTB3
	TPM2->MOD = 1000;

	PORTB->PCR[0] &= ~0x703;
	PORTB->PCR[1] &= ~0x703;
	PORTB->PCR[2] &= ~0x703;
	PORTB->PCR[3] &= ~0x703;
	PORTC->PCR[1] &= ~0x703;
	PORTC->PCR[2] &= ~0x703;

	TPM2->CONTROLS[0].CnSC |= (1 << 3) | (1 << 5); //Set to Edge PWM
	TPM2->CONTROLS[1].CnSC |= (1 << 3) | (1 << 5);

	TPM2->SC |= 1 << 3;



	PORTB->PCR[0] |= 1 << 8;
	PORTB->PCR[1] |= 1 << 8;
	PORTC->PCR[1] |= 1 << 8;
	PORTC->PCR[2] |= 1 << 8;

	PORTB->PCR[2] |= 3 << 8;
	PORTB->PCR[3] |= 3 << 8;

	GPIOB->PDDR |= 0xF;
	GPIOC->PDDR |= 0x6;
}

void motor_break(char motor, char stp_brk) {
	if((motor == LEFT_MOTOR) & (stp_brk == BREAK)) {
		GPIOB->PDOR |= 0x3;
	}

	if((motor == RIGHT_MOTOR) & (stp_brk == BREAK)) {
		GPIOC->PDOR |= 0x6;
	}

	if((motor == LEFT_MOTOR) & (stp_brk == STOP)) {
		GPIOB->PDOR &= ~0x3;
	}

	if((motor == RIGHT_MOTOR) & (stp_brk == STOP)) {
		GPIOC->PDOR &= ~0x6;
	}
}

void linesensor_setup(){
	unsigned short cal_v = 0;
	PORTC->PCR[3] |= (1 << 24); // Clear Interrupt Flag!
	GPIOD->PTOR |= (1 << 5); // Toggle!
	//line sensor setup
	SIM->SCGC5 |= (1<<13);
	SIM->SCGC6 |= (1<<27);
	// Setup ADC Clock ( < 4 MHz)
	ADC0->CFG1 = 0; // Default everything.
	// Analog Calibrate
	ADC0->SC3 = 0x07; // Enable Maximum Hardware Averaging
	ADC0->SC3 |= 0x80; // Start Calibration
	// Wait for Calibration to Complete (either COCO or CALF)
	while(!(ADC0->SC1[0] & 0x80)){ }
	// Calibration Complete, write calibration registers.
	cal_v = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 + ADC0->CLP3 + ADC0->CLP4 + ADC0->CLPS;
	cal_v = cal_v >> 1 | 0x8000;
	ADC0->PG = cal_v;
	cal_v = 0;
	cal_v = ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 + ADC0->CLM3 + ADC0->CLM4 + ADC0->CLMS;
	cal_v = cal_v >> 1 | 0x8000;
	ADC0->MG = cal_v;
	ADC0->SC3 = 0; // Turn off Hardware Averaging
}

void trace_line(){
	//Left sensor
	ADC0->SC1[0] = 0x05; // Set Channel, starts conversion.
	while(!(ADC0->SC1[0] & 0x80)){ }
	//delay(1000);
	left = ADC0->R[0]; // Resets COCO
	if(left > 174) {
		motor_control(RIGHT_MOTOR, CCW, 400);
	}else{
		motor_control(RIGHT_MOTOR, CCW, 700);
	}
	//Right sensor
	ADC0->SC1[0] = 0x01; // Set Channel, starts conversion.
	while(!(ADC0->SC1[0] & 0x80)){ }
	//delay(1000);
	right = ADC0->R[0]; // Resets COCO
	if(right > 174) {
		motor_control(LEFT_MOTOR, CW, 400);
	}
	else{
		motor_control(LEFT_MOTOR, CW, 700);
	}
}

int calulateColor() {
    C = dataArray[1] << 8 | dataArray[0];
    R = dataArray[3] << 8 | dataArray[2];
    G = dataArray[5] << 8 | dataArray[4];
    B = dataArray[7] << 8 | dataArray[6];

    C /= 10;
    R /= 10;
    G /= 10;
    B /= 10;

    printf("C:%d, R:%d, G:%d, B:%d ",C,R,G,B);

    if (14 <= C && C <= 16)
        return 1; // Red

    if (24 <= C && C <= 28)
        return 2; // Green

    if (31 < C && C < 50)
        return 3; // Yellow

    if (B<=G && 14 <= C && C <= 16)
        return 4; // Blue
    return -1;
}
int getColor() {
    uint8_t cmdcode = (1 << 7) | (1 << 5) | (0x14);

    I2C_read(cmdcode, dataArray, 8);
    i = calulateColor();
    printf("%i\n", i);
    return i;
}

void SW1_setup_interrupt() {
	SIM->SCGC5 |= (1<<11);  // Enable Port C Clock
	PORTC->PCR[3] &= ~0xF0703; // Clear First
	PORTC->PCR[3] |= 0xF0703 & ((0xA << 16) | (1 << 8) | 0x3 ); // Set MUX bits, enable pullups, interrupt on falling edge
	GPIOC->PDDR &= ~(1 << 3); // Setup Pin 3 Port C as input
	NVIC_EnableIRQ(31);
}

void PORTC_PORTD_IRQHandler(void) {
	delay_ms(2000);
	init_I2C();
	I2C_write(0x80, 0x03);
	delay_ms(3);
	I2C_write( (0x80 | 0x01), 0xFF);
	delay_ms(3);
	start = getColor();
	if(start == 3){
		stop = 1;
	}
	//Straight
	motor_control(LEFT_MOTOR, CW, 700);
	motor_control(RIGHT_MOTOR, CCW, 700);
	while(1){
		int color = getColor();
		while(color == 3){
			color = getColor();
			printf("stop: %d", stop);
			//Turn right
			motor_control(RIGHT_MOTOR, CCW, 700);
			motor_control(LEFT_MOTOR, CW, 300);
			if(color == stop){
				motor_break(LEFT_MOTOR, STOP);
				motor_break(RIGHT_MOTOR, STOP);
			}
		}
		trace_line();
	}
}


int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif
    motors_setup();
    linesensor_setup();
    SW1_setup_interrupt();
    int i = 0;
    while(1){
//    	delay_ms(2000);
//    	init_I2C();
//    	I2C_write(0x80, 0x03);
//    	delay_ms(3);
//    	I2C_write( (0x80 | 0x01), 0xFF);
//    	delay_ms(3);
//    	int color = getColor();
    }
    return 0 ;
}
