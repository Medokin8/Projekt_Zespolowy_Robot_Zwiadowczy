/*
 * camera.c
 *
 */

#include "camera.h"
#include "camera_inittable.h"
#include <stdio.h>
#include <string.h>

#define MAX_PICTURE_BUFF	92160
//extern I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef *_phuart;
I2C_HandleTypeDef *_phi2c;
DCMI_HandleTypeDef *_pdcmi;

int SCCB_Read(uint8_t reg_addrs, uint8_t *data)
{
	if(HAL_I2C_Master_Receive(_phi2c, OV2640_DEVICE_READ_ADDRESS, data, 1, 100) == HAL_OK){
		return 1;
	}
	return 0;
}

int SCCB_Write(uint8_t reg_addrs, uint8_t data)
{
	//uint8_t buffer[2] = {reg_addrs, data};
	HAL_StatusTypeDef connectionStatus;
	connectionStatus = HAL_I2C_Mem_Write(_phi2c, (uint8_t)0x60, reg_addrs, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
	//connectionStatus = HAL_I2C_Master_Transmit(_phi2c, (uint8_t)0x60, buffer, 2, 100);
	if(connectionStatus == HAL_OK){
		HAL_UART_Transmit(_phuart, (uint8_t*)"1\r", strlen("1\r"), 100);
		return 1;
	}
	HAL_UART_Transmit(_phuart, (uint8_t*)"0\r", strlen("0\r"), 100);
	return 0;
}

void camera_Init(I2C_HandleTypeDef *hi2c, DCMI_HandleTypeDef *dcmi)
{
	camera_setI2C(hi2c);
	camera_setDCMI(dcmi);
	camera_Reset();
	camera_StopDCMI();
}

void camera_Reset()
{
	// power the camera
	HAL_GPIO_WritePin(PWDNPG1_GPIO_Port, PWDNPG1_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	// hardware reset
	HAL_GPIO_WritePin(RESETG0_GPIO_Port, RESETG0_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RESETG0_GPIO_Port, RESETG0_Pin, GPIO_PIN_SET);
	HAL_Delay(100);

	// reset registers to default
	SCCB_Write(0xff, 0x01);
	SCCB_Write(0x12, 0x80);
	HAL_Delay(250);
}

void camera_Configuration(const char arr[][2])
{
	uint32_t index = 0;
	while(1){
		if(arr[index][0] == 0xff && arr[index][1] == 0xff){
			break;
		}
		SCCB_Write(arr[index][0], arr[index][1]);
		HAL_Delay(1);
		++index;
	}
}

void camera_configureResolution(const char arr[][2])
{
	camera_StopDCMI();
	camera_Configuration(OV2640_JPEG_INIT);
	camera_Configuration(OV2640_YUV422);
	camera_Configuration(OV2640_JPEG);
	SCCB_Write(0xff, 0x01);
	HAL_Delay(1);
	SCCB_Write(0x15, 0x00);
	HAL_Delay(1);
	camera_Configuration(arr); //configure resolution
}

void camera_ResolutionConfiguration(enum imgResolution res)
{
	switch (res)
	{
	case RES_320x240:
		camera_configureResolution(OV2640_320x240_JPEG);
		break;
	case RES_800x600:
		camera_configureResolution(OV2640_800x600_JPEG);
		break;
	case RES_1024x768:
		camera_configureResolution(OV2640_1024x768_JPEG);
		break;
	case RES_1280x960:
		camera_configureResolution(OV2640_1280x960_JPEG);
	default:
		camera_configureResolution(OV2640_320x240_JPEG);
		break;
	}
}

void camera_CaptureContinuous()
{
	uint8_t pBuff[MAX_PICTURE_BUFF];
	camera_StopDCMI();
	camera_Configuration(OV2640_QVGA);
	HAL_Delay(500);
	HAL_DCMI_Start_DMA(_pdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)pBuff, MAX_PICTURE_BUFF/4);
	HAL_UART_Transmit_DMA(_phuart, pBuff, MAX_PICTURE_BUFF/4);
	//HAL_DCMI_Suspend(_pdcmi);
}


void camera_CaptureSnapshot()
{
	uint8_t pBuff[MAX_PICTURE_BUFF];
	uint8_t headerFinded = 0;
	uint8_t idx = 0;

	memset(pBuff, 0, MAX_PICTURE_BUFF);
	camera_StopDCMI();
	camera_ResolutionConfiguration(RES_320x240);
	HAL_Delay(500);
	HAL_DCMI_Start_DMA(_pdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)pBuff, MAX_PICTURE_BUFF/4);
	HAL_Delay(2500);
	//HAL_DCMI_Suspend(_pdcmi);
		while(1)
		{
			if(headerFinded == 0 && pBuff[idx] == 0xFF && pBuff[idx+1] == 0xD8){
				headerFinded = 1;
			}
			if(headerFinded == 1 && pBuff[idx] == 0xFF && pBuff[idx+1] == 0xD9){
				idx += 2;
				headerFinded = 0;
				break;
			}
			if(idx >= MAX_PICTURE_BUFF){
				break;
			}
			//HAL_UART_Transmit(_phuart, &idx,1, 100);
			idx++;
		}
	HAL_UART_Transmit_DMA(_phuart, pBuff, idx);
}
uint8_t camera_ReadID(void)
{
	SCCB_Write(0xff, 0x01);
	uint8_t pid;
	SCCB_Read(0x0a, &pid);

	return pid;
}

void camera_StopDCMI(void)
{
	HAL_DCMI_Stop(_pdcmi);
	HAL_Delay(10);
}

void camera_setI2C(I2C_HandleTypeDef *hi2c)
{
	_phi2c = hi2c;
}

void camera_setDCMI(DCMI_HandleTypeDef *dcmi)
{
	_pdcmi = dcmi;
}

void camera_setUART(UART_HandleTypeDef *huart)
{
	_phuart = huart;
}
