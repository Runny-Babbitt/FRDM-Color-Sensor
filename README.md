# FRDM-Color-Sensor
Description: The objective of this project is to learn to properly use and set up the I2C protocol on the FRDM board and allow communication between the board and the color sensor. With that communication, have the robot follow a specific movement pattern given the starting position of the robot.

The I2C required that I used the I2C module in the FRDM board. I first had to enable clock gating for the SCGC 4 bit 6 and SCGC 5 bit 11. I then set port c pin 8 and 9 to use the I2C protocol. After that, to initialize the I2C I cleared all its registers.
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
I then write the register address of the sensor to the I2C module. Use the NACK and other flags to determine if the register address and data was acknowledged. Once that happens, I now am able to send and receive data in bytes separated by acknowledging.

To run the project, use the MCUXpresso IDE. Unzip the file and upload to the FRDM KL46Z  board.
