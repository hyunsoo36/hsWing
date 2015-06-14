#include "srf08.h"

void SRF08::initialize() {
	delay(50);
	I2Cdev::writeByte(SRF08_ADDRESS, SRF08_RANGE, SRF08_MAX_RANGE_4M);
	delay(50);
	I2Cdev::writeByte(SRF08_ADDRESS, SRF08_ANALOGUE_GAIN, SRF08_MAX_ANALOGUE_GAIN_94);
	delay(50);
}

void SRF08::sendRangeCommand() {

	I2Cdev::writeByte(SRF08_ADDRESS, SRF08_CMD, SRF08_RANGE_CENTI);

}

int SRF08::recieveRange() {
	int range;
	uint8_t data[2];

	int count = I2Cdev::readBytes(SRF08_ADDRESS, SRF08_RANGE, 2, data);

	range = (data[0] << 8) + data[1];
	//Serial.print(count);
	//Serial.print("\t");
	//Serial.print(data[0]);
	//Serial.print("\t");
	//Serial.print(data[1]);
	//Serial.println("");

	return range;


}