#include "stm32f4xx_hal.h"                  // Device header
#include "MyI2C.h"
#include "tim.h"

#define VL53L0X_ADDRESS		0x52		//VL53L0X的I2C从机地址

int16_t VL53L0X_distance_origin;
int16_t VL53L0X_CalibrateDistance(int16_t rawDistance);
int16_t VL53L0X_distance;
uint8_t VL53L0X_state = 0;
uint8_t VL53L0X_Flag_state = 0;
uint16_t VL53L0X_get_data_Count;
/**
  * 函    数：VL53L0X写寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考VL53L0X手册的寄存器描述
  * 参    数：Data 要写入寄存器的数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void VL53L0X_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(VL53L0X_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(Data);				//发送要写入寄存器的数据
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_Stop();						//I2C终止
}

/**
  * 函    数：VL53L0X读寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考VL53L0X手册的寄存器描述
  * 返 回 值：读取寄存器的数据，范围：0x00~0xFF
  */
uint8_t VL53L0X_ReadReg(uint8_t RegAddress)
{
	uint8_t data;
	
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(VL53L0X_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	
	MyI2C_Start();						//I2C重复起始
	MyI2C_SendByte(VL53L0X_ADDRESS | 0x01);	//发送从机地址，读写位为1，表示即将读取
	MyI2C_ReceiveAck();					//接收应答
	data = MyI2C_ReceiveByte();			//接收指定寄存器的数据
	MyI2C_SendAck(1);	

	MyI2C_Stop();						//I2C终止
	return data;
}

void VL53L0X_ReadRegs(uint8_t RegAddress,uint8_t *DataArray,uint8_t count)
{
	uint8_t i;
	
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(VL53L0X_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	
	MyI2C_Start();						//I2C重复起始
	MyI2C_SendByte(VL53L0X_ADDRESS | 0x01);	//发送从机地址，读写位为1，表示即将读取
	MyI2C_ReceiveAck();					//接收应答
	for(i = 0;i<count ;i++)
	{
		DataArray[i] = MyI2C_ReceiveByte();			//接收指定寄存器的数据
		if(i<count-1)
		{
		MyI2C_SendAck(0);	
		}
	else{MyI2C_SendAck(1);}		
	}
	MyI2C_Stop();						//I2C终止
	
}
void VL53L0X_StartMeasurement(void){
	VL53L0X_WriteReg(0x00, 0x01);
}
/**
  * 函    数：VL53L0X初始化
  * 参    数：无
  * 返 回 值：无
  */
static uint8_t stop_variable = 0;


// ... existing code ...
void VL53L0X_Init(void)
{
	MyI2C_Init();

	HAL_Delay(100);
	HAL_Delay(100);
	VL53L0X_WriteReg(0x88, 0x00);
	VL53L0X_WriteReg(0x80, 0x01);
	VL53L0X_WriteReg(0xFF, 0x01);
	VL53L0X_WriteReg(0x00, 0x00);

	stop_variable = VL53L0X_ReadReg(0x91);

	VL53L0X_WriteReg(0x00, 0x01);
	VL53L0X_WriteReg(0xFF, 0x00);
	VL53L0X_WriteReg(0x80, 0x00);
	// VL53L0X_WriteReg(0x88, 0x00);
	// VL53L0X_WriteReg(0x80, 0x01);
	// VL53L0X_WriteReg(0xFF, 0x01);
	// VL53L0X_WriteReg(0x00, 0x00);
	//
	// stop_variable = VL53L0X_ReadReg(0x91);
	//
	// VL53L0X_WriteReg(0x00, 0x01);
	// VL53L0X_WriteReg(0xFF, 0x00);
	// VL53L0X_WriteReg(0x80, 0x00);
	//
	// VL53L0X_WriteReg(0xFF, 0x01);
	// VL53L0X_WriteReg(0x00, 0x00);
	// VL53L0X_WriteReg(0xFF, 0x00);
	// VL53L0X_WriteReg(0x09, 0x00);
	// VL53L0X_WriteReg(0x10, 0x00);
	// VL53L0X_WriteReg(0x11, 0x00);
	//
	// VL53L0X_WriteReg(0x24, 0x01);
	// VL53L0X_WriteReg(0x25, 0xFF);
	// VL53L0X_WriteReg(0x75, 0x00);
	//
	// VL53L0X_WriteReg(0xFF, 0x01);
	// VL53L0X_WriteReg(0x4E, 0x2C);
	// VL53L0X_WriteReg(0x48, 0x00);
	// VL53L0X_WriteReg(0x30, 0x20);
	//
	// VL53L0X_WriteReg(0xFF, 0x00);
	// VL53L0X_WriteReg(0x30, 0x09);
	// VL53L0X_WriteReg(0x54, 0x00);
	// VL53L0X_WriteReg(0x31, 0x04);
	// VL53L0X_WriteReg(0x32, 0x03);
	// VL53L0X_WriteReg(0x40, 0x83);
	// VL53L0X_WriteReg(0x46, 0x25);
	// VL53L0X_WriteReg(0x60, 0x00);
	// VL53L0X_WriteReg(0x27, 0x00);
	// VL53L0X_WriteReg(0x50, 0x06);
	// VL53L0X_WriteReg(0x51, 0x00);
	// VL53L0X_WriteReg(0x52, 0x96);
	// VL53L0X_WriteReg(0x56, 0x08);
	// VL53L0X_WriteReg(0x57, 0x30);
	// VL53L0X_WriteReg(0x61, 0x00);
	// VL53L0X_WriteReg(0x62, 0x00);
	// VL53L0X_WriteReg(0x64, 0x00);
	// VL53L0X_WriteReg(0x65, 0x00);
	// VL53L0X_WriteReg(0x66, 0xA0);
	//
	// VL53L0X_WriteReg(0xFF, 0x01);
	// VL53L0X_WriteReg(0x22, 0x32);
	// VL53L0X_WriteReg(0x47, 0x14);
	// VL53L0X_WriteReg(0x49, 0xFF);
	// VL53L0X_WriteReg(0x4A, 0x00);
	//
	// VL53L0X_WriteReg(0xFF, 0x00);
	// VL53L0X_WriteReg(0x7A, 0x0A);
	// VL53L0X_WriteReg(0x7B, 0x00);
	// VL53L0X_WriteReg(0x78, 0x21);
	//
	// VL53L0X_WriteReg(0xFF, 0x01);
	// VL53L0X_WriteReg(0x23, 0x34);
	// VL53L0X_WriteReg(0x42, 0x00);
	// VL53L0X_WriteReg(0x44, 0xFF);
	// VL53L0X_WriteReg(0x45, 0x26);
	// VL53L0X_WriteReg(0x46, 0x05);
	// VL53L0X_WriteReg(0x0E, 0x06);
	// VL53L0X_WriteReg(0x20, 0x1A);
	// VL53L0X_WriteReg(0x43, 0x40);
	//
	// VL53L0X_WriteReg(0xFF, 0x00);
	// VL53L0X_WriteReg(0x34, 0x03);
	// VL53L0X_WriteReg(0x35, 0x44);
	//
	// VL53L0X_WriteReg(0xFF, 0x01);
	// VL53L0X_WriteReg(0x31, 0x04);
	// VL53L0X_WriteReg(0x4B, 0x09);
	// VL53L0X_WriteReg(0x4C, 0x05);
	// VL53L0X_WriteReg(0x4D, 0x04);
	//
	// VL53L0X_WriteReg(0xFF, 0x00);
	// VL53L0X_WriteReg(0x44, 0x00);
	// VL53L0X_WriteReg(0x45, 0x20);
	// VL53L0X_WriteReg(0x47, 0x08);
	// VL53L0X_WriteReg(0x48, 0x28);
	// VL53L0X_WriteReg(0x67, 0x00);
	// VL53L0X_WriteReg(0x70, 0x04);
	// VL53L0X_WriteReg(0x71, 0x01);
	// VL53L0X_WriteReg(0x72, 0xFE);
	// VL53L0X_WriteReg(0x76, 0x00);
	// VL53L0X_WriteReg(0x77, 0x00);
	//
	// VL53L0X_WriteReg(0xFF, 0x01);
	// VL53L0X_WriteReg(0x0D, 0x01);
	//
	// VL53L0X_WriteReg(0xFF, 0x00);
	// VL53L0X_WriteReg(0x80, 0x01);
	// VL53L0X_WriteReg(0x01, 0xF8);
	//
	// VL53L0X_WriteReg(0xFF, 0x01);
	// VL53L0X_WriteReg(0x8E, 0x01);
	// VL53L0X_WriteReg(0x00, 0x01);
	// VL53L0X_WriteReg(0xFF, 0x00);
	// VL53L0X_WriteReg(0x80, 0x00);
	//
	// VL53L0X_WriteReg(0x00, 0x01);
	//
	// VL53L0X_WriteReg(0x80, 0x01);
	// VL53L0X_WriteReg(0xFF, 0x01);
	// VL53L0X_WriteReg(0x00, 0x00);
	// VL53L0X_WriteReg(0x91, stop_variable);
	// VL53L0X_WriteReg(0x00, 0x01);
	// VL53L0X_WriteReg(0xFF, 0x00);
	// VL53L0X_WriteReg(0x80, 0x00);
	//
	// HAL_Delay(50);
	//
	// VL53L0X_WriteReg(0xFF, 0x01);
	// VL53L0X_WriteReg(0x00, 0x00);
	// VL53L0X_WriteReg(0x01, 0x00);
	// VL53L0X_WriteReg(0x02, 0x00);
	// VL53L0X_WriteReg(0x03, 0x0A);
	// VL53L0X_WriteReg(0x04, 0x00);
	// VL53L0X_WriteReg(0x05, 0x00);
	// VL53L0X_WriteReg(0x06, 0x00);
	// VL53L0X_WriteReg(0x07, 0x00);
	// VL53L0X_WriteReg(0x08, 0x00);
	// VL53L0X_WriteReg(0x09, 0x00);
	// VL53L0X_WriteReg(0x0A, 0x00);
	// VL53L0X_WriteReg(0x0B, 0x00);
	// VL53L0X_WriteReg(0x0C, 0x00);
	// VL53L0X_WriteReg(0x0D, 0x01);
	// VL53L0X_WriteReg(0x0E, 0x01);
	// VL53L0X_WriteReg(0x0F, 0x00);
	// VL53L0X_WriteReg(0x10, 0x01);
	// VL53L0X_WriteReg(0xFF, 0x00);
	//
	// VL53L0X_WriteReg(0x80, 0x01);
	// VL53L0X_WriteReg(0xFF, 0x01);
	// VL53L0X_WriteReg(0x00, 0x00);
	// VL53L0X_WriteReg(0x91, stop_variable);
	// VL53L0X_WriteReg(0x00, 0x01);
	// VL53L0X_WriteReg(0xFF, 0x00);
	// VL53L0X_WriteReg(0x80, 0x00);



	HAL_TIM_Base_Start_IT(&htim2);
	VL53L0X_StartMeasurement();


}






/**
  * 函    数：VL53L0X获取ID号
  * 参    数：无
  * 返 回 值：VL53L0X的ID号
  */
uint8_t VL53L0X_GetID(void)
{
	return VL53L0X_ReadReg(0xC0);		//返回WHO_AM_I寄存器的值
}

uint8_t VL53L0X_WaitForDataReady(void) {
	if (VL53L0X_ReadReg(0x14) & 0x01)
	{
		return 0;
	}
	return 1;
}



// uint16_t VL53L0X_ReadDistance(void)
// {
// 	uint8_t data[2];
// 	uint16_t distance;
//
// 	VL53L0X_StartMeasurement();
//
// 	if (VL53L0X_WaitForDataReady() ==0)
// 	{
// 		return 0xFFFF;
// 	}
//
// 	VL53L0X_ReadRegs(0x1E, data, 2);
// 	distance = ((uint16_t)data[0] << 8) | data[1];
//
// 	VL53L0X_WriteReg(0x0B, 0x01);
//
// 	return distance;
// }

void VL53L0X_refresh_distance(void){
	uint8_t data[2];
	static uint8_t error_count = 0;
	if (VL53L0X_WaitForDataReady() ==0) {
		VL53L0X_get_data_Count++;
		VL53L0X_ReadRegs(0x1E, data, 2);
		VL53L0X_distance_origin = ((int16_t)data[0] << 8) | data[1];
		if (VL53L0X_distance == 8191) {
			VL53L0X_state = 1;
		}
		else {
			// VL53L0X_distance = VL53L0X_distance_origin;
			VL53L0X_distance = VL53L0X_CalibrateDistance(VL53L0X_distance_origin);
		}
		VL53L0X_Flag_state = 1;
		VL53L0X_WriteReg(0x0B, 0x01);
		VL53L0X_StartMeasurement();
	}
	else {
		error_count++;
		if (error_count >= 50) { // 连续 10 次超时
			VL53L0X_Flag_state = 1;
			VL53L0X_WriteReg(0x0B, 0x01);
			VL53L0X_StartMeasurement();
			error_count = 0;
		}
	}
}

//数据拟合
typedef struct {
	uint16_t rawValue;
	uint16_t actualValue;
} CalibrationPoint_t;

static const CalibrationPoint_t calibrationTable[] = {
	{56,  30},
	{70,  40},
	{80,  50},
	{90,  60},
	{115, 90},
	{126, 100},
	{142, 110},
	{160, 120},
	{183, 130},
	{190, 140},
	{219, 150},
	{285, 170},
	{294, 180},
	{328, 200},
	{410, 220},
	{490, 240},
	{500, 250},
	{531, 260},
	{470, 280}
};

#define CALIBRATION_POINTS  (sizeof(calibrationTable) / sizeof(calibrationTable[0]))

int16_t VL53L0X_CalibrateDistance(int16_t rawDistance)
{
	if (rawDistance <= calibrationTable[0].rawValue) {
		return calibrationTable[0].actualValue;
	}

	if (rawDistance >= calibrationTable[CALIBRATION_POINTS - 1].rawValue) {
		return calibrationTable[CALIBRATION_POINTS - 1].actualValue;
	}

	for (uint8_t i = 0; i < CALIBRATION_POINTS - 1; i++) {
		if (rawDistance >= calibrationTable[i].rawValue &&
			rawDistance < calibrationTable[i + 1].rawValue) {

			uint16_t rawRange = calibrationTable[i + 1].rawValue - calibrationTable[i].rawValue;
			int16_t actualRange = (int16_t)calibrationTable[i + 1].actualValue - (int16_t)calibrationTable[i].actualValue;
			uint16_t offset = rawDistance - calibrationTable[i].rawValue;

			return calibrationTable[i].actualValue + (int16_t)((offset * actualRange) / rawRange);
			}
	}

	return (int16_t)((int32_t)rawDistance * 270 / 359);
}


