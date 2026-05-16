/*************************************************************************************************
 * @brief   PS2�����Լ����ݴ���
 * @version 2.0
 * @date    2021.12.24
 * @param		none
 * @retval  none
 * @author  ysl
 *************************************************************************************************/

#include "PS2.h"

// 软件SPI引脚定义
#define SPI_SCK_PIN GPIO_PIN_5
#define SPI_SCK_PORT GPIOA
#define SPI_MISO_PIN GPIO_PIN_6
#define SPI_MISO_PORT GPIOA
#define SPI_MOSI_PIN GPIO_PIN_7
#define SPI_MOSI_PORT GPIOA
#define SPI_CS_PIN GPIO_PIN_4
#define SPI_CS_PORT GPIOA

// 函数原型
uint8_t PS2_ReadWriteData(uint8_t cmd);


uint8_t cmd[3] = {0x01,0x42,0x00};  // �����������
uint8_t PS2data[9] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //�洢�ֱ���������
uint16_t XY[4] = {500,500,500,500};  //ҡ��ģ��ֵ
uint8_t i;
uint8_t All_But[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  //���а���״ֵ̬

void delay_us(uint32_t udelay)    //����hal��us���ӳ�
{
  uint32_t startval,tickn,delays,wait;
 
  startval = SysTick->VAL;
  tickn = HAL_GetTick();
  //sysc = 72000;  //SystemCoreClock / (1000U / uwTickFreq);
  delays =udelay * 72; //sysc / 1000 * udelay;
  if(delays > startval)
    {
      while(HAL_GetTick() == tickn)
        {
 
        }
      wait = 72000 + startval - delays;
      while(wait < SysTick->VAL)
        {
 
        }
    }
  else
    {
      wait = startval - delays;
      while(wait < SysTick->VAL && HAL_GetTick() == tickn)
        {
 
        }
    }
}

void PS2_Get(void)    //获取ps2数据
{
	short i = 0;
	
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);  // 拉低CS，开始通信
		
	PS2data[0] = PS2_ReadWriteData(cmd[0]);  // 发送0x01，接收数据
	delay_us(10);
	PS2data[1] = PS2_ReadWriteData(cmd[1]);  // 发送0x42，接收数据
	delay_us(10);
	PS2data[2] = PS2_ReadWriteData(cmd[2]);  // 发送0x00，接收数据
	delay_us(10);
	for(i = 3;i <9;i++)
	{
		PS2data[i] = PS2_ReadWriteData(cmd[2]);  // 发送0x00，接收数据
		delay_us(10);
		
	}

	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);  // 拉高CS，结束通信

}


void GetData(void)  //���ݴ���
{
	
	PS2_Get();   //��ȡԭʼ����
	GetXY();   //��ҡ��ģ��ֵ�Ŵ�洢��������
	All_Button();
	CLear_Date();  //������ݣ��Ա��´�ʹ��

}

void GetXY(void)   //��ҡ��ģ��ֵ����0-1000�仯����������Ҳ��˷Ѿ���
{
	int i;
	for(i = 5;i < 9;i++)
	{
		PS2data[i] =(int) PS2data[i];		
		XY[i-5] = (PS2data[i]* 1000) / 255;   //���ֱ�ҡ�˵�ֵ�ֵ�0-1000֮�䣬���˷�ģ��ֵ����
		if(XY[i-5] <503 && XY[i-5] > 497)  XY[i-5] = 500;   //����
	}
	
}

void CLear_Date(void)
{
	for(i = 0;i<9;i++)
	{
		if(i == 3 || i == 4) PS2data[i] = 0xff;
		else PS2data[i] = 0x00;  //�������
	}
	
}

void All_Button(void)  //��ÿһ��������ֵ��ʵ��ȫ�����޳�ͻ
{
	uint8_t loc = 1;
	uint8_t set = 0;
	uint8_t but = PS2data[3];

  for(loc = 8;loc > 0;loc--)  //λ�����ȡǰ��λ
  {
		loc -= 1;
		All_But[set] = (PS2data[3]&(1<<loc))>>loc;
		loc += 1;
		set++;
  }
	for(loc = 8;loc > 0;loc--)   //λ�����ȡ���λ
  {
		loc -= 1;
		All_But[set] = (PS2data[4]&(1<<loc))>>loc;
		loc += 1;
		set++;
  }
	for(set = 0;set < 16;set++)    //��ΪЭ���ϰ�������Ϊ0��δ����Ϊ1������Ҫ������з�ת
	{
		if(All_But[set] == 1)  All_But[set] = 0;
		else  All_But[set] = 1;			 
	}
	

}

// 软件SPI读写数据 (发送1字节，接收1字节)
uint8_t PS2_ReadWriteData(uint8_t cmd) {
    uint8_t res = 0;
    uint8_t ref;

    // 发送8位数据，接收8位
    for (ref = 0x01; ref > 0x00; ref <<= 1) {
        // 发送一位
        if (ref & cmd) {
            HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_RESET);
        }

        HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_RESET);
        delay_us(16);  // 延时

        // 接收一位
        if (HAL_GPIO_ReadPin(SPI_MISO_PORT, SPI_MISO_PIN)) {
            res |= ref;
        }

        HAL_GPIO_WritePin(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_PIN_SET);
        delay_us(16);
    }

    return res;
}



