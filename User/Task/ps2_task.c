/**
  *********************************************************************
  * @file      ps2_task.c/h
  * @brief     �������Ƕ�ȡ������ps2�ֱ�������ң�����ݣ�
	*            ��ң������ת��Ϊ�������ٶȡ�������ת�ǡ��������ȳ���
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "ps2_task.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "Motor.h"
#include "bsp_can.h"
#include "bsp_dwt.h"

#define DI()      HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)   // 数据输入 (MISO)
#define CMD_H()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)  // 命令高 (MOSI)
#define CMD_L()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET) // 命令低
#define CS_H()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET)   // CS高
#define CS_L()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET) // CS低
#define CLK_H()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET)   // 时钟高
#define CLK_L()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET) // 时钟低

// 软件SPI读写数据 (发送1字节，接收1字节)
static uint8_t PS2_ReadWriteData(uint8_t cmd) {
    uint8_t res = 0;
    uint8_t ref;

    // 发送8位数据，接收8位
    for (ref = 0x01; ref > 0x00; ref <<= 1) {
        // 发送一位
        if (ref & cmd) {
            CMD_H();
        } else {
            CMD_L();
        }

        CLK_L();
        DWT_Delay(0.000016f);  // 延时16us

        // 接收一位
        if (DI()) {
            res |= ref;
        }

        CLK_H();
        DWT_Delay(0.000016f);  // 延时16us
    }

    return res;
}

ps2data_t ps2data;
extern vmc_leg_t left_vmc;
extern void jump_key (chassis_t *chassis,ps2data_t *data);

uint16_t Handkey;	// ����ֵ��ȡ����ʱ�洢��
uint8_t Comd[3]={0x01,0x42,0x00};	//��ʼ�����������
uint8_t Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //���ݴ洢����
uint16_t MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	//����ֵ�밴����


//extern chassis_t chassis_move;
extern INS_t INS;
uint32_t PS2_TIME=10;//ps2�ֱ�����������10ms
uint32_t PS2_TIME_DWT; //dwt��ȡ��ϵͳʱ�������
float ps2_dt;//���ּ�ʱ��
	
void pstwo_task(void)
{		
	 PS2_SetInit();

   while(1)
	 {
		 ps2_dt = DWT_GetDeltaT(&PS2_TIME_DWT);//��ȡϵͳʱ��
		 if(Data[1]!=0x73)
		 {
		  PS2_SetInit();
		 }

	   PS2_data_read(&ps2data);//������
//		 PS2_data_process(&ps2data,&chassis_move,(float)PS2_TIME/1000.0f);//�������ݣ�������������
		 PS2_data_move(&ps2data,&chassis_move,ps2_dt);
	   osDelay(PS2_TIME);
	 }
}

//���ֱ���������
void PS2_Cmd(uint8_t CMD)
{
    Data[1] = PS2_ReadWriteData(CMD);
    DWT_Delay(0.000016f);
}

/**************************************************************************
Function: Read the control of the ps2 handle
Input   : none
Output  : none
�������ܣ���ȡPS2�ֱ��Ŀ�����
��ڲ�������
����  ֵ����
**************************************************************************/	
uint8_t reve_flag=0;

void PS2_data_read(ps2data_t *data)
{
  //��ȡ������ֵ
	data->key=PS2_DataKey(); //��ȡ������ֵ

  //��ȡ���ң��X�᷽���ģ����	
	data->lx=PS2_AnologData(PSS_LX); 

  //��ȡ���ң��Y�᷽���ģ����	
	data->ly=PS2_AnologData(PSS_LY);

  //��ȡ�ұ�ң��X�᷽���ģ����  
	data->rx=PS2_AnologData(PSS_RX);

  //��ȡ�ұ�ң��Y�᷽���ģ����  
	data->ry=PS2_AnologData(PSS_RY);

	if((data->ry<=255&&data->ry>192)||(data->ry<64&&data->ry>=0)) //����ʱ��������
	{
	  data->rx=127;
	}
	if((data->rx<=255&&data->rx>192)||(data->rx<64&&data->rx>=0))
	{
	  data->ry=128;
	}
}

void PS2_data_move(ps2data_t *data,chassis_t *chassis,float dt)
{
	if(data->last_key!=9&&data->key==9){
        ///其实就是监测到按下的是9键，然后执行相应的操作，然后进行一个防抖，假设按一次是连续的10次信号，那么就认这两个不同信号直接切换为一次按键 
		chassis_move.start_flag = 1;// ����׼������������־λ
		chassis_move.turn_set = INS.YawTotalAngle;// ����ʱ������yaw��ʼĿ��ֵ
	}
	
	if(data->last_key!=4&&data->key==4)
	{
		chassis_move.start_flag = 0;// ����δ����
		chassis->recover_flag=0;
		DM_Motor_Command(&FDCAN1_TxFrame, &DM_4310_Motor_leftfront, Motor_Enable);
		DWT_Delay(0.001f); 
		DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor_rightfront, Motor_Enable);
		DWT_Delay(0.001f);  
		DM_Motor_Command(&FDCAN1_TxFrame, &DM_4310_Motor_leftback, Motor_Enable);
		DWT_Delay(0.001f); 
		DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor_rightback, Motor_Enable);
		DWT_Delay(0.001f); 
		DM_Motor_Command(&FDCAN1_TxFrame, &DM_6215_Motor_left, Motor_Enable);
		DWT_Delay(0.001f); 
		DM_Motor_Command(&FDCAN2_TxFrame, &DM_6215_Motor_right, Motor_Enable);
	}
	
	if(data->last_key!=10&&data->key==10)
	{
		chassis_move.start_flag = 0;// ����δ����
		chassis->recover_flag=0;
		DM_Motor_Command(&FDCAN1_TxFrame, &DM_4310_Motor_leftfront, Motor_Disable);
		DWT_Delay(0.001f); 
		DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor_rightfront, Motor_Disable);
		DWT_Delay(0.001f); 
		DM_Motor_Command(&FDCAN1_TxFrame, &DM_4310_Motor_leftback, Motor_Disable);
		DWT_Delay(0.001f); 
		DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor_rightback, Motor_Disable);
		DWT_Delay(0.001f); 
		DM_Motor_Command(&FDCAN1_TxFrame, &DM_6215_Motor_left, Motor_Disable);
		DWT_Delay(0.001f); 
		DM_Motor_Command(&FDCAN2_TxFrame, &DM_6215_Motor_right, Motor_Disable);		
	}
	
	if(chassis_move.start_flag == 1)
	{
		chassis->target_v=((float)(data->ry-128))*(-0.010f);//��ǰ����0
		slope_following(&chassis->target_v,&chassis->v_set,0.0040f);	//	�¶ȸ���
		chassis->x_set = chassis->x_set + chassis->v_set*dt;
		chassis->turn_set=chassis->turn_set+(data->rx-127)*(-0.00050f);//���Ҵ���0
	  			
		//�ȳ��仯
		chassis->leg_set=chassis->leg_set+((float)(data->ly-128))*(-0.000008f);
		mySaturate(&chassis->leg_set,0.085f,0.2f);//�ȳ��޷���0.085m��0.18m֮��
		chassis->roll_target= ((float)(data->lx-127))*(0.0025f);
		slope_following(&chassis->roll_target,&chassis->roll_set,0.0075f);

		jump_key(chassis,data);

		chassis->leg_left_set = chassis->leg_set;
		chassis->leg_right_set = chassis->leg_set;                                                                                                                                                                                                                                                       
		mySaturate(&chassis->leg_left_set,0.085f,0.2f);//�ȳ��޷���0.085m��0.18m֮��
		mySaturate(&chassis->leg_right_set,0.085f,0.2f);//�ȳ��޷���0.085m��0.18m֮��
		
		jump_key(chassis,data);

		if(fabsf(chassis->last_leg_left_set-chassis->leg_left_set)>0.0001f || fabsf(chassis->last_leg_right_set-chassis->leg_right_set)>0.0001f)
		{//ң���������ȳ��ڱ仯
			right_vmc.leg_flag=1;	//Ϊ1��־���ȳ�����������(����������Ӧ����)�����������־���Բ�������ؼ�⣬��Ϊ���ȳ�����������ʱ����ؼ������ж�Ϊ�����
      left_vmc.leg_flag=1;	 			
		}
		else
		{
			right_vmc.leg_flag=0;
			left_vmc.leg_flag=0;
		}
		chassis->last_leg_set=chassis->leg_set;
		chassis->last_leg_left_set=chassis->leg_left_set;
		chassis->last_leg_right_set=chassis->leg_right_set;
	}
}
//extern vmc_leg_t right;			
//extern vmc_leg_t left;	
float acc_test =0.005f;
//void PS2_data_process(ps2data_t *data,chassis_t *chassis,float dt)
//{   
//	if(data->last_key!=4&&data->key==4&&chassis->start_flag==0) 
//	{
//		//�ֱ��ϵ�Start����������
//		chassis->start_flag=1;
//		if(chassis->recover_flag==0
//			&&((chassis->myPithR<((-3.1415926f)/4.0f)&&chassis->myPithR>((-3.1415926f)/2.0f))
//		  ||(chassis->myPithR>(3.1415926f/4.0f)&&chassis->myPithR<(3.1415926f/2.0f))))
//		{
//		  chassis->recover_flag=1;//��Ҫ����
//		}
//	}
//	else if(data->last_key!=4&&data->key==4&&chassis->start_flag==1) 
//	{
//		//�ֱ��ϵ�Start����������
//		chassis->start_flag=0;
//		chassis->recover_flag=0;
//	}
//	
//	data->last_key=data->key;
//  
//	if(chassis->start_flag==1)
//	{//����
//		chassis->target_v=((float)(data->ry-128))*(-0.008f);//��ǰ����0
//		slope_following(&chassis->target_v,&chassis->v_set,0.005f);	//	�¶ȸ���

//		chassis->x_set=chassis->x_set+chassis->v_set*dt;
//		chassis->turn_set=chassis->turn_set+(data->rx-127)*(-0.00025f);//���Ҵ���0
//	  			
//		//�ȳ��仯
//		chassis->leg_set=chassis->leg_set+((float)(data->ly-128))*(-0.000015f);
//		chassis->roll_target= ((float)(data->lx-127))*(0.0025f);

//		slope_following(&chassis->roll_target,&chassis->roll_set,0.0075f);

//		jump_key(chassis,data);

//		chassis->leg_left_set = chassis->leg_set;
//		chassis->leg_right_set = chassis->leg_set;

//		mySaturate(&chassis->leg_left_set,0.065f,0.18f);//�ȳ��޷���0.065m��0.18m֮��
//		mySaturate(&chassis->leg_right_set,0.065f,0.18f);//�ȳ��޷���0.065m��0.18m֮��
//		

//		if(fabsf(chassis->last_leg_left_set-chassis->leg_left_set)>0.0001f || fabsf(chassis->last_leg_right_set-chassis->leg_right_set)>0.0001f)
//		{//ң���������ȳ��ڱ仯
//			right.leg_flag=1;	//Ϊ1��־���ȳ�����������(����������Ӧ����)�����������־���Բ�������ؼ�⣬��Ϊ���ȳ�����������ʱ����ؼ������ж�Ϊ�����
//      left.leg_flag=1;	 			
//		}
//		
//		chassis->last_leg_set=chassis->leg_set;
//		chassis->last_leg_left_set=chassis->leg_left_set;
//		chassis->last_leg_right_set=chassis->leg_right_set;
//	} 
//	else if(chassis->start_flag==0)
//	{//�ر�
//	  chassis->v_set=0.0f;//����
//		chassis->x_set=chassis->x_filter;//����
//	  chassis->turn_set=chassis->total_yaw;//����
//	  chassis->leg_set=0.08f;//ԭʼ�ȳ�
//	}
//	
//		
//}

void jump_key (chassis_t *chassis,ps2data_t *data)
{
	if(data->key == 11)
	{
		if(++chassis->count_key>10)
		{
			if(chassis->jump_flag == 0)
			{
				chassis->jump_flag = 1;
				chassis->jump_leg = chassis->leg_set;
			}
		}
	}
	else
	{
		chassis->count_key = 0;
	}
}



//�ж��Ƿ�Ϊ���ģʽ,0x41=ģ���̵ƣ�0x73=ģ����
//����ֵ��0�����ģʽ
//		  ����������ģʽ
uint8_t PS2_RedLight(void)
{
    CS_L();
    PS2_ReadWriteData(Comd[0]);  // 开始命令
    PS2_ReadWriteData(Comd[1]);  // 请求数据
    CS_H();
    if (Data[1] == 0X73) return 0;
    else return 1;
}
//�õ�PS2�ֱ�����
void PS2_ReadData(void)
{
    CS_L();
    Data[0] = PS2_ReadWriteData(Comd[0]);  // 发送0x01，接收Data[0]
    DWT_Delay(0.000010f);
    Data[1] = PS2_ReadWriteData(Comd[1]);  // 发送0x42，接收Data[1]
    DWT_Delay(0.000010f);
    Data[2] = PS2_ReadWriteData(Comd[2]);  // 发送0x00，接收Data[2]
    DWT_Delay(0.000010f);
    for (short i = 3; i < 9; i++) {
        Data[i] = PS2_ReadWriteData(Comd[2]);  // 发送0x00，接收数据
        DWT_Delay(0.000010f);
    }
    CS_H();
}

//�Զ�������PS2�����ݽ��д���,ֻ������������  
//ֻ��һ����������ʱ����Ϊ0�� δ����Ϊ1
uint8_t PS2_DataKey()
{
	uint8_t index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //����16������  ����Ϊ0�� δ����Ϊ1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //û���κΰ�������
}

//�õ�һ��ҡ�˵�ģ����	 ��Χ0~256
uint8_t PS2_AnologData(uint8_t button)
{
	return Data[button];
}

//������ݻ�����
void PS2_ClearData()
{
	uint8_t a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: �ֱ��𶯺�����
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:�Ҳ�С�𶯵�� 0x00�أ�������
	   motor2:�����𶯵�� 0x40~0xFF �������ֵԽ�� ��Խ��
******************************************************/
void PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
    CS_L();
    DWT_Delay(0.000016f);
    PS2_ReadWriteData(0x01);  // 开始命令
    PS2_ReadWriteData(0x42);  // 请求数据
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(motor1);
    PS2_ReadWriteData(motor2);
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(0X00);
    CS_H();
    DWT_Delay(0.000016f);
}

//short poll
void                                                                                                                                                                                                                              PS2_ShortPoll(void)
{
    CS_L();
    DWT_Delay(0.000016f);
    PS2_ReadWriteData(0x01);
    PS2_ReadWriteData(0x42);
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(0x00);
    PS2_ReadWriteData(0x00);
    CS_H();
    DWT_Delay(0.000016f);
}

//��������
void PS2_EnterConfing(void)
{
    CS_L();
    DWT_Delay(0.000016f);
    PS2_ReadWriteData(0x01);
    PS2_ReadWriteData(0x43);
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(0x01);
    PS2_ReadWriteData(0x00);
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(0X00);
    CS_H();
    DWT_Delay(0.000016f);
}
//����ģʽ����
void PS2_TurnOnAnalogMode(void)
{
    CS_L();
    PS2_ReadWriteData(0x01);
    PS2_ReadWriteData(0x44);
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(0x01); // analog=0x01; digital=0x00  软件模拟模式
    PS2_ReadWriteData(0x03); // 0x03开启震动，进入配置模式
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(0X00);
    CS_H();
    DWT_Delay(0.000016f);
}
//������
void PS2_VibrationMode(void)
{
    CS_L();
    DWT_Delay(0.000016f);
    PS2_ReadWriteData(0x01);
    PS2_ReadWriteData(0x4D);
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(0x00);
    PS2_ReadWriteData(0X01);
    CS_H();
    DWT_Delay(0.000016f);
}
//��ɲ���������
void PS2_ExitConfing(void)
{
    CS_L();
    DWT_Delay(0.000016f);
    PS2_ReadWriteData(0x01);
    PS2_ReadWriteData(0x43);
    PS2_ReadWriteData(0X00);
    PS2_ReadWriteData(0x00);
    PS2_ReadWriteData(0x5A);
    PS2_ReadWriteData(0x5A);
    PS2_ReadWriteData(0x5A);
    PS2_ReadWriteData(0x5A);
    PS2_ReadWriteData(0x5A);
    CS_H();
    DWT_Delay(0.000016f);
}

//�ֱ����ó�ʼ��
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//��������ģʽ
	PS2_TurnOnAnalogMode();	//�����̵ơ�����ģʽ����ѡ���Ƿ񱣴�
	//PS2_VibrationMode();	//������ģʽ
	PS2_ExitConfing();		//��ɲ���������
}




