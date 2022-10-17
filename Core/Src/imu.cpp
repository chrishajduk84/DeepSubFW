/*
 * imu.cpp
 *
 *  Created on: Sep. 17, 2022
 *      Author: Chris Hajduk
 */

#include "imu.h"
#include <stdexcept>


IMU::IMU(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csi_port, uint16_t csi_pin)
{
	IMU::csi_port = csi_port;
	IMU::csi_pin = csi_pin;
	IMU::hspi = hspi;
	// Ensure we are in bank 1
//	write_register(REG_BANK_SEL, 3 << 4); // Put operation back into bank 3
//	write_register(I2C_SLV0_ADDR, 0); // Read the device ID
//	write_register(I2C_SLV0_REG, 0);
//	write_register(I2C_SLV0_CTRL, 0); //Enabling I2C slave read of magnetometer with 1 byte-length of data
//	write_register(REG_BANK_SEL, 1 << 4); // Put operation back into bank 3
	// Check that WHO_AM_I register is as-expected
	uint8_t who_am_i_response = read_register(WHO_AM_I);
	if (who_am_i_response != 0xEA)
	{
		std::string response = std::to_string(who_am_i_response);
		throw std::domain_error("ICM20948 WHO_AM_I response is not expected (value: " + response + ", expected: 0xEA)");
	}

	//Enable I2C master for magnetometer communication, disable I2C slave mode
	write_register(USER_CTRL, 0b00100000);

	//Exit sleep mode
	write_register(PWR_MGMT_1, 1);

	// Check that magnetometer WHO_AM_I register is as-expected
	write_register(REG_BANK_SEL, 3 << 4); // Put operation into bank 3
	write_register(I2C_MST_CTRL, 0b10010000);
	write_register(I2C_SLV0_ADDR, MAG_I2C_ADDRESS | 0x80); // Read the device ID
	write_register(I2C_SLV0_REG, MAG_DEVICE_ID);
	write_register(I2C_SLV0_CTRL, 0b10000000 | 1); //Enabling I2C slave read of magnetometer with 1 byte-length of data

	write_register(REG_BANK_SEL, 0); // Put operation back into bank 0
	uint8_t who_am_i_mag = read_register(EXT_SLV_SENS_DATA_00);
	if (who_am_i_mag != 0x9)
	{
		std::string response = std::to_string(who_am_i_response);
		throw std::domain_error("Magnetometer WHO_AM_I response is not expected for (value: " + response + ", expected: 0x09)");
	}

	// Now that we have established that we can communicate with all components, lets setup the magnetometer read registers
	write_register(REG_BANK_SEL, 3 << 4); // Put operation back into bank 3
	write_register(I2C_SLV0_ADDR, MAG_I2C_ADDRESS | 0x80); // Read the device ID
	write_register(I2C_SLV0_REG, 0); // Auto-increment starting from register 0x0 -> 0x18 (9 bytes)
	write_register(I2C_SLV0_CTRL, 0b10000000 | 9); //Enabling I2C slave read of magnetometer with 9 byte-length of data
	write_register(REG_BANK_SEL, 0); // Put operation back into bank 0

}

uint8_t IMU::read_register(uint8_t address)
{
	HAL_StatusTypeDef status;
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	uint32_t timeout = 1000;

	tx_data[0] = address | 0x80; //READ
	tx_data[1] = 0; //Response goes here

	HAL_GPIO_WritePin(IMU::csi_port, IMU::csi_pin, GPIO_PIN_RESET);
	// TODO: use the HAL_Delay function
	HAL_Delay(1);
	//for (int i = 0; i < 100000; i++){}
	status = HAL_SPI_TransmitReceive(IMU::hspi, tx_data, rx_data, 2, timeout);
	//for (int i = 0; i < 100000; i++){}
	HAL_Delay(1);
	HAL_GPIO_WritePin(IMU::csi_port, IMU::csi_pin, GPIO_PIN_SET);
	uint8_t byte = 0;
	if (status == HAL_OK)
	{
	  byte = rx_data[1];
	}
	return byte;
}
uint8_t* IMU::read_registers(uint8_t start_address, uint8_t* rx_buffer, uint8_t length)
{
	HAL_StatusTypeDef status;
	uint8_t tx_data[length];
	uint32_t timeout = 1000;

	tx_data[0] = 0x00 | 0x80; //READ
	tx_data[1] = 0; //Response goes here

	HAL_GPIO_WritePin(IMU::csi_port, IMU::csi_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	//for (int i = 0; i < 100000; i++){}
	status = HAL_SPI_TransmitReceive(IMU::hspi, tx_data, rx_buffer, length, timeout);
	//for (int i = 0; i < 100000; i++){}
	HAL_Delay(1);
	HAL_GPIO_WritePin(IMU::csi_port, IMU::csi_pin, GPIO_PIN_SET);
	return rx_buffer;
}
void IMU::write_register(uint8_t address, uint8_t data)
{
	HAL_StatusTypeDef status;
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	uint32_t timeout = 1000;

	tx_data[0] = address & 0x7F; //WRITE
	tx_data[1] = data; //data goes here

	HAL_GPIO_WritePin(IMU::csi_port, IMU::csi_pin, GPIO_PIN_RESET);
	// TODO: use the HAL_Delay function
	HAL_Delay(1);
	//for (int i = 0; i < 100000; i++){}
	status = HAL_SPI_TransmitReceive(IMU::hspi, tx_data, rx_data, 2, timeout);
	//for (int i = 0; i < 100000; i++){}
	HAL_Delay(1);
	HAL_GPIO_WritePin(IMU::csi_port, IMU::csi_pin, GPIO_PIN_SET);
}
void IMU::write_registers(uint8_t start_address, uint8_t* data, uint8_t length)
{

}
