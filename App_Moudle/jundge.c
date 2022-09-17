#include "jundge.h"
#define SEND_MAX_LEN 120
unpack_data_t unpack_data;
referee_receive_data_t referee_receive_data;
uint8_t My_Queue[256]={0};
QUEUE8_t Queue_Date;
ext_draw_crosshair_t Draw_CH;
ex_draw_char Draw_Char;
ex_sendcustom_t SendCustom;
uint8_t Analysis[256]={0};
static uint8_t ClienTxBuffer[SEND_MAX_LEN]={0};
static uint8_t ClienTxDynamicrBuffer[SEND_MAX_LEN]={0};
static uint8_t Clien_character[60];
/**
 * @Name: referee_data_solve
 * @Description:������ϵͳ���͹����洢�浽��Ӧ��CMD_ID��
 * @Param: *rec ---------- ����3�Ӳ���ϵͳ���յ���δ���������     
           cmd_id -------- ����ϵͳͨ��Э��ID
           data_len ------ ����ϵͳͨ�����������ID�е�����֡����
           *pData -------- ����ϵͳͨ�����������ID�е�����data
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void referee_data_solve(referee_receive_data_t *rec,uint16_t cmd_id,uint16_t data_len,uint8_t *pData)
{
	switch(cmd_id)
	{
		case GAME_STATE_CMD_ID:       memcpy(&rec->jundge_game_state.save[0],&pData[0],data_len);	     break;
		case GAME_ROBOT_BLOOD_CMD_ID:  			break;
		case GAME_AREA_EVENT_CMD_ID:   	 		break;
		case GAME_ROBOT_STATE_CMD_ID: memcpy(&rec->jundge_robot_state.save[0],&pData[0],data_len);      break;
		case GAME_POWER_HEART_CMD_ID: memcpy(&rec->jundge_power_heat_data.save[0],&pData[0],data_len);  break;
		case GAME_ROBOT_POS_CMD_ID:   memcpy(&rec->jundge_robot_pos_data.save[0],&pData[0],data_len);  	  break;
		case GAME_ROBOT_BUFF_CMD_ID:      	break;
		case GAME_ROBOT_SHOOT_CMD_ID: memcpy(&rec->jundge_shoot_data.save[0],&pData[0],data_len);       break;
	}
}
/**
 * @Name: Jundge_Analysis
 * @Description:������ϵͳ���͹��������ݽ��д���
 * @Param: *d ---------- ����3�Ӳ���ϵͳ���յ���δ���������    
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Jundge_Analysis(unpack_data_t *d)
{
	static uint32_t length=0;
	length=QUEUE_PacketOut(&Queue_Date,d->data,QUEUE_PacketLengthGet(&Queue_Date));
	for(int i=0;i<length;i++)
		if(d->data[i]==0xa5)					
				if(verify_crc8(&d->data[i],FRAME_HEADER_LEN))//byte
				{
					d->cmd_id = (uint16_t)(d->data[i+6]<<8|d->data[i+5]);
					d->data_len = (uint16_t)(d->data[i+2]<<8|d->data[i+1]);
					if(verify_crc16(&d->data[i],d->data_len+REF_HAEDER_CMDID_CRC16_SIZE))
						referee_data_solve(&referee_receive_data,d->cmd_id,d->data_len,&d->data[i+REF_HAEDER_CMDID_SIZE]);
				}					
}
/**
 * @Name: Draw_CrossHair
* @Description: �������ݸ�����ϵͳ�������Զ���UI
* @Param: *draw ---------- ���͸�����ϵͳ�����ݰ�
          cmd_id --------- ����ϵͳͨ��Э��ID
          data_id -------- ����ϵͳ�Զ���ͼ����������ID
          tx_id ---------- ������ID
          rx_id ---------- ������ID
 * @Return: void
 * @Author: source
 * @Warning: void
 */

void Draw_Static_CrossHair_1(ext_draw_crosshair_t *draw,uint16_t cmd_id,uint16_t data_id,uint16_t tx_id,uint16_t rx_id)
{	
	static uint8_t Seq_CrossHair=0;
	draw->txFrame_Header.sof = 0xa5;
	draw->txFrame_Header.data_length = sizeof(ext_student_interactive_header_data_t)+7*sizeof(graphic_data_struct_t);
	draw->txFrame_Header.seq = Seq_CrossHair;
	
	memcpy(ClienTxBuffer,&draw->txFrame_Header,sizeof(Frame_Header_t));
	append_crc8(ClienTxBuffer,sizeof(Frame_Header_t));
	Seq_CrossHair++;
	
	draw->CMD_ID = cmd_id;
	draw->Client_Custom_ID.data_cmd_id = data_id;
	draw->Client_Custom_ID.sender_ID = tx_id;
	draw->Client_Custom_ID.receiver_ID = rx_id;
	/*****����׼��*****/	
	
	/*1~5ͼ,������׼*/ /*�������꣨960,540��*/
	//����ͼ��ͼ��ֱ�Ϊ0~2��
	
////////////////	/*׼��*/
	if(Seq_CrossHair%3==0){	
	draw->Graphic_Data[0].graphic_name[0] = '0';
	draw->Graphic_Data[0].graphic_name[1] = '0';
	draw->Graphic_Data[0].graphic_name[2] = '0';
		draw->Graphic_Data[0].operate_tpye = 1;
	draw->Graphic_Data[0].graphic_tpye = 0;
	draw->Graphic_Data[0].layer = 0;
	draw->Graphic_Data[0].color = 2;
	draw->Graphic_Data[0].start_angle = 0;
	draw->Graphic_Data[0].end_angle = 0;
	draw->Graphic_Data[0].width = 1;
	draw->Graphic_Data[0].start_x = 840;
	draw->Graphic_Data[0].start_y = 640;
	draw->Graphic_Data[0].radius = 0;
	draw->Graphic_Data[0].end_x = 1080;
	draw->Graphic_Data[0].end_y = 640;
	
		draw->Graphic_Data[1].operate_tpye = 1;
	draw->Graphic_Data[1].graphic_name[0] = '0';
	draw->Graphic_Data[1].graphic_name[1] = '0';
	draw->Graphic_Data[1].graphic_name[2] = '1';
		draw->Graphic_Data[1].operate_tpye = 1;
	draw->Graphic_Data[1].graphic_tpye = 0;
	draw->Graphic_Data[1].layer = 0;
	draw->Graphic_Data[1].color = 2;
	draw->Graphic_Data[1].start_angle = 0;
	draw->Graphic_Data[1].end_angle = 0;
	draw->Graphic_Data[1].width = 1;
	draw->Graphic_Data[1].start_x = 860;
	draw->Graphic_Data[1].start_y = 410;
	draw->Graphic_Data[1].radius = 0;
	draw->Graphic_Data[1].end_x = 1060;
	draw->Graphic_Data[1].end_y = 410;

		draw->Graphic_Data[2].graphic_name[0] = '0';
	draw->Graphic_Data[2].graphic_name[1] = '0';
	draw->Graphic_Data[2].graphic_name[2] = '2';
		draw->Graphic_Data[2].operate_tpye = 1;
	draw->Graphic_Data[2].graphic_tpye = 0;
	draw->Graphic_Data[2].layer = 0;
	draw->Graphic_Data[2].color = 2;
	draw->Graphic_Data[2].start_angle = 0;
	draw->Graphic_Data[2].end_angle = 0;
	draw->Graphic_Data[2].width = 1;
	draw->Graphic_Data[2].start_x = 880;
	draw->Graphic_Data[2].start_y = 360;
	draw->Graphic_Data[2].radius = 0;
	draw->Graphic_Data[2].end_x = 1040;
	draw->Graphic_Data[2].end_y = 360;
	
			draw->Graphic_Data[3].graphic_name[0] = '0';
	draw->Graphic_Data[3].graphic_name[1] = '0';
	draw->Graphic_Data[3].graphic_name[2] = '3';
		draw->Graphic_Data[3].operate_tpye = 1;
	draw->Graphic_Data[3].graphic_tpye = 0;
	draw->Graphic_Data[3].layer = 0;
	draw->Graphic_Data[3].color = 2;
	draw->Graphic_Data[3].start_angle = 0;
	draw->Graphic_Data[3].end_angle = 0;
	draw->Graphic_Data[3].width = 1;
	draw->Graphic_Data[3].start_x = 900;
	draw->Graphic_Data[3].start_y = 310;
	draw->Graphic_Data[3].radius = 0;
	draw->Graphic_Data[3].end_x = 1020;
	draw->Graphic_Data[3].end_y = 310;
	
			draw->Graphic_Data[4].graphic_name[0] = '0';
	draw->Graphic_Data[4].graphic_name[1] = '0';
	draw->Graphic_Data[4].graphic_name[2] = '4';
		draw->Graphic_Data[4].operate_tpye = 1;
	draw->Graphic_Data[4].graphic_tpye = 0;
	draw->Graphic_Data[4].layer = 0;
	draw->Graphic_Data[4].color = 2;
	draw->Graphic_Data[4].start_angle = 0;
	draw->Graphic_Data[4].end_angle = 0;
	draw->Graphic_Data[4].width = 1;
	draw->Graphic_Data[4].start_x = 920;
	draw->Graphic_Data[4].start_y = 260;
	draw->Graphic_Data[4].radius = 0;
	draw->Graphic_Data[4].end_x = 1000;
	draw->Graphic_Data[4].end_y = 260;
	
						/*����*/
		draw->Graphic_Data[5].graphic_name[0] = '0';
	draw->Graphic_Data[5].graphic_name[1] = '0';
	draw->Graphic_Data[5].graphic_name[2] = '5';
//	if(seq<=5)
//		draw->Graphic_Data[5].operate_tpye = 1;
//	else 
		draw->Graphic_Data[5].operate_tpye = 1;
	draw->Graphic_Data[5].graphic_tpye = 0;
	draw->Graphic_Data[5].layer = 0;
	draw->Graphic_Data[5].color = 1;
	draw->Graphic_Data[5].start_angle = 0;
	draw->Graphic_Data[5].end_angle = 0;
	draw->Graphic_Data[5].width = 1;
	draw->Graphic_Data[5].start_x = 960;
	draw->Graphic_Data[5].start_y = 700;
	draw->Graphic_Data[5].radius = 0;
	draw->Graphic_Data[5].end_x = 960;
	draw->Graphic_Data[5].end_y = 260;
	
						/* Բ */
		draw->Graphic_Data[6].graphic_name[0] = '0';
	draw->Graphic_Data[6].graphic_name[1] = '0';
	draw->Graphic_Data[6].graphic_name[2] = '6';

		draw->Graphic_Data[6].operate_tpye = 1;
	draw->Graphic_Data[6].graphic_tpye = 2;
	draw->Graphic_Data[6].layer = 0;
	draw->Graphic_Data[6].color = 5;
	draw->Graphic_Data[6].start_angle = 0;
	draw->Graphic_Data[6].end_angle = 0;
	draw->Graphic_Data[6].width = 4;
	draw->Graphic_Data[6].start_x = 960;
	draw->Graphic_Data[6].start_y = 540;
	draw->Graphic_Data[6].radius = 30;
	draw->Graphic_Data[6].end_x = 960;
	draw->Graphic_Data[6].end_y = 540;
	}
	
//////////////////	/*������*/
	if(Seq_CrossHair%3==1)
	{
		draw->Graphic_Data[0].graphic_name[0] = '0';
	draw->Graphic_Data[0].graphic_name[1] = '1';
	draw->Graphic_Data[0].graphic_name[2] = '0';
		draw->Graphic_Data[0].operate_tpye = 1;
	draw->Graphic_Data[0].graphic_tpye = 0;
	draw->Graphic_Data[0].layer = 1;
	draw->Graphic_Data[0].color = 2;
	draw->Graphic_Data[0].start_angle = 0;
	draw->Graphic_Data[0].end_angle = 0;
	draw->Graphic_Data[0].width = 5;
	draw->Graphic_Data[0].start_x = 690-100;
	draw->Graphic_Data[0].start_y = 1080-865-100-65+10;
	draw->Graphic_Data[0].radius = 0;
	draw->Graphic_Data[0].end_x = 1230+100;
	draw->Graphic_Data[0].end_y = 1080-865-100-65+10;
	
		draw->Graphic_Data[1].operate_tpye = 1;
	draw->Graphic_Data[1].graphic_name[0] = '0';
	draw->Graphic_Data[1].graphic_name[1] = '1';
	draw->Graphic_Data[1].graphic_name[2] = '1';
		draw->Graphic_Data[1].operate_tpye = 1;
	draw->Graphic_Data[1].graphic_tpye = 0;
	draw->Graphic_Data[1].layer = 1;
	draw->Graphic_Data[1].color = 2;
	draw->Graphic_Data[1].start_angle = 0;
	draw->Graphic_Data[1].end_angle = 45;
	draw->Graphic_Data[1].width = 3;
	draw->Graphic_Data[1].start_x = 630-100;
	draw->Graphic_Data[1].start_y = 10;
	draw->Graphic_Data[1].radius = 0;
	draw->Graphic_Data[1].end_x = 690-100;
	draw->Graphic_Data[1].end_y = 1080-865-100-65+10;

		draw->Graphic_Data[2].graphic_name[0] = '0';
	draw->Graphic_Data[2].graphic_name[1] = '1';
	draw->Graphic_Data[2].graphic_name[2] = '2';
		draw->Graphic_Data[2].operate_tpye = 1;
	draw->Graphic_Data[2].graphic_tpye = 0;
	draw->Graphic_Data[2].layer = 1;
	draw->Graphic_Data[2].color = 2;
	draw->Graphic_Data[2].start_angle = 0;
	draw->Graphic_Data[2].end_angle = 360-45;
	draw->Graphic_Data[2].width = 3;
	draw->Graphic_Data[2].start_x = 1230+100;
	draw->Graphic_Data[2].start_y = 1080-865-100-65+10;
	draw->Graphic_Data[2].radius = 0;
	draw->Graphic_Data[2].end_x = 1280+100;
	draw->Graphic_Data[2].end_y = 10;
	
			draw->Graphic_Data[3].graphic_name[0] = '0';
	draw->Graphic_Data[3].graphic_name[1] = '1';
	draw->Graphic_Data[3].graphic_name[2] = '3';
		draw->Graphic_Data[3].operate_tpye = 1;
	draw->Graphic_Data[3].graphic_tpye = 0;
	draw->Graphic_Data[3].layer = 1;
	draw->Graphic_Data[3].color = 2;
	draw->Graphic_Data[3].start_angle = 0;
	draw->Graphic_Data[3].end_angle = 0;
	draw->Graphic_Data[3].width = 3;
	draw->Graphic_Data[3].start_x = 960+300;
	draw->Graphic_Data[3].start_y = 540-120;
	draw->Graphic_Data[3].radius = 0;
	draw->Graphic_Data[3].end_x = 960+300;
	draw->Graphic_Data[3].end_y = 540+150;
	
			draw->Graphic_Data[4].graphic_name[0] = '0';
	draw->Graphic_Data[4].graphic_name[1] = '1';
	draw->Graphic_Data[4].graphic_name[2] = '4';
		draw->Graphic_Data[4].operate_tpye = 1;
	draw->Graphic_Data[4].graphic_tpye = 0;
	draw->Graphic_Data[4].layer = 1;
	draw->Graphic_Data[4].color = 2;
	draw->Graphic_Data[4].start_angle = 0;
	draw->Graphic_Data[4].end_angle = 0;
	draw->Graphic_Data[4].width = 3;
	draw->Graphic_Data[4].start_x = 960-300;
	draw->Graphic_Data[4].start_y = 540-120;
	draw->Graphic_Data[4].radius = 0;
	draw->Graphic_Data[4].end_x = 960-300;
	draw->Graphic_Data[4].end_y = 540+150;
	
						/*����*/
		draw->Graphic_Data[5].graphic_name[0] = '0';
	draw->Graphic_Data[5].graphic_name[1] = '1';
	draw->Graphic_Data[5].graphic_name[2] = '5';
		draw->Graphic_Data[5].operate_tpye = 1;
	draw->Graphic_Data[5].graphic_tpye = 0;
	draw->Graphic_Data[5].layer = 1;
	draw->Graphic_Data[5].color = 1;
	draw->Graphic_Data[5].start_angle = 0;
	draw->Graphic_Data[5].end_angle = 0;
	draw->Graphic_Data[5].width = 1;
	draw->Graphic_Data[5].start_x = 960;
	draw->Graphic_Data[5].start_y = 700;
	draw->Graphic_Data[5].radius = 0;
	draw->Graphic_Data[5].end_x = 960;
	draw->Graphic_Data[5].end_y = 260;
	
						/* Բ */
		draw->Graphic_Data[6].graphic_name[0] = '0';
	draw->Graphic_Data[6].graphic_name[1] = '1';
	draw->Graphic_Data[6].graphic_name[2] = '6';	
		draw->Graphic_Data[6].operate_tpye = 1;
	draw->Graphic_Data[6].graphic_tpye = 2;
	draw->Graphic_Data[6].layer = 1;
	draw->Graphic_Data[6].color = 5;
	draw->Graphic_Data[6].start_angle = 0;
	draw->Graphic_Data[6].end_angle = 0;
	draw->Graphic_Data[6].width = 4;
	draw->Graphic_Data[6].start_x = 960;
	draw->Graphic_Data[6].start_y = 540;
	draw->Graphic_Data[6].radius = 30;
	draw->Graphic_Data[6].end_x = 960;
	draw->Graphic_Data[6].end_y = 540;
	
	}
	
	
	memcpy(&ClienTxBuffer[5],(uint8_t*)&draw->CMD_ID,(sizeof(ext_student_interactive_header_data_t)+7*sizeof(graphic_data_struct_t)+2*sizeof(uint16_t)));
	append_crc16(ClienTxBuffer,sizeof(ext_draw_crosshair_t));	
	for(int i=0;i<sizeof(ext_draw_crosshair_t);i++)
	{	
		//HAL_UART_Transmit_DMA(&huart6,ClienTxBuffer,sizeof(ClienTxBuffer));
	}
}




int Flage_UI_Mode_Rise=0,Flage_UI_Mode_right1=0,Flage_UI_Mode_right2=0;
void Draw_Dynamicr_CrossHair_1(ext_draw_crosshair_t *draw,uint16_t cmd_id,uint16_t data_id,uint16_t tx_id,uint16_t rx_id)
{	
	static uint8_t Seq_Feature=0;
	draw->txFrame_Header.sof = 0xa5;
	draw->txFrame_Header.data_length = sizeof(ext_student_interactive_header_data_t)+7*sizeof(graphic_data_struct_t);
	draw->txFrame_Header.seq = Seq_Feature;
	
	memcpy(ClienTxDynamicrBuffer,&draw->txFrame_Header,sizeof(Frame_Header_t));
	append_crc8(ClienTxDynamicrBuffer,sizeof(Frame_Header_t));
	Seq_Feature++;	
	draw->CMD_ID = cmd_id;
	draw->Client_Custom_ID.data_cmd_id = data_id;
	draw->Client_Custom_ID.sender_ID = tx_id;
	draw->Client_Custom_ID.receiver_ID = rx_id;
//////////////////////	/*���ܱ�־λ*/

		draw->Graphic_Data[0].graphic_name[0] = '1';
	draw->Graphic_Data[0].graphic_name[1] = '0';
	draw->Graphic_Data[0].graphic_name[2] = '0';
		draw->Graphic_Data[0].operate_tpye = 1;
	draw->Graphic_Data[0].graphic_tpye = 2;
	draw->Graphic_Data[0].layer = 2;
	draw->Graphic_Data[0].color = 2;
	draw->Graphic_Data[0].start_angle = 0;
	draw->Graphic_Data[0].end_angle = 0;
	draw->Graphic_Data[0].width = 2;
	draw->Graphic_Data[0].start_x = 960-480;
	draw->Graphic_Data[0].start_y = 540-270+20;
	draw->Graphic_Data[0].radius = 20;
	draw->Graphic_Data[0].end_x = 0;
	draw->Graphic_Data[0].end_y = 0;
	
	/*			          �䶯ֱ��		          			*/
		draw->Graphic_Data[1].graphic_name[0] = '1';
	draw->Graphic_Data[1].graphic_name[1] = '0';
	draw->Graphic_Data[1].graphic_name[2] = '1';
			if(Seq_Feature%10==0)	//��������ʱ����
		{	draw->Graphic_Data[1].operate_tpye = 1;
			draw->Graphic_Data[1].end_x = 960-480;
			draw->Graphic_Data[1].end_y = 540-270+20+20;
		}
		else 
		{	draw->Graphic_Data[1].operate_tpye = 2;
			draw->Graphic_Data[1].end_x = 960-480-cos(dis_angle/57.3f+3.14/2)*20;
			draw->Graphic_Data[1].end_y = 540-270+sin(dis_angle/57.3f+3.14/2)*20+20;
		}
	draw->Graphic_Data[1].layer = 2;
	draw->Graphic_Data[1].color = 4;
	draw->Graphic_Data[1].start_angle = 0;
	draw->Graphic_Data[1].end_angle = 0;
	draw->Graphic_Data[1].width = 3;
	draw->Graphic_Data[1].start_x = 960-480;
	draw->Graphic_Data[1].start_y = 540-270+20;
	draw->Graphic_Data[1].radius = 0;

		
			/*      �Ҳ������      */
				draw->Graphic_Data[2].graphic_name[0]='1';
			draw->Graphic_Data[2].graphic_name[1]='0';
			draw->Graphic_Data[2].graphic_name[2]='2';
		if(Seq_Feature%10==0)		
		{	draw->Graphic_Data[2].operate_tpye=1;//ͼ�β���
			draw->Graphic_Data[2].color=8;//��ɫ			
		}
		else if(Seq_Feature%10!=0)
		 {
				draw->Graphic_Data[2].operate_tpye=2;
				if(Flage_UI_Mode_right1==1) 
					draw->Graphic_Data[2].color=1;//��ɫ
				else 
					draw->Graphic_Data[2].color=8;//��ɫ
		 }
			draw->Graphic_Data[2].graphic_tpye=1;//ͼ������
			draw->Graphic_Data[2].layer=2;//ͼ����
			draw->Graphic_Data[2].start_angle=0;//��ʼ�Ƕ�
			draw->Graphic_Data[2].end_angle=0;//��ֹ�Ƕ�
			draw->Graphic_Data[2].width=3;//�߿�
			draw->Graphic_Data[2].start_x=1740;//��� x ����
			draw->Graphic_Data[2].start_y=710;//��� y ����(840,720)~(1080,720)
			draw->Graphic_Data[2].radius=30;//�����С���߰뾶
			draw->Graphic_Data[2].end_x=1830;//�յ� x ����
			draw->Graphic_Data[2].end_y=670;//�յ� y ����
			
		
			draw->Graphic_Data[3].graphic_name[0]='1';
			draw->Graphic_Data[3].graphic_name[1]='0';
			draw->Graphic_Data[3].graphic_name[2]='3';
		if(Seq_Feature%10==0) 
		{ 
			draw->Graphic_Data[3].operate_tpye=1;
		  draw->Graphic_Data[3].color=8;//��ɫ
		}
		if(Seq_Feature%10!=0) 
		{ draw->Graphic_Data[3].operate_tpye=2;
			if(Flage_UI_Mode_right2==1)
		  draw->Graphic_Data[3].color=1;//��ɫ
			else
			draw->Graphic_Data[3].color=8;//��ɫ			
		}
		
			draw->Graphic_Data[3].graphic_tpye=1;//ͼ������
			draw->Graphic_Data[3].layer=2;//ͼ����
			draw->Graphic_Data[3].start_angle=0;//��ʼ�Ƕ�
			draw->Graphic_Data[3].end_angle=0;//��ֹ�Ƕ�
			draw->Graphic_Data[3].width=3;//�߿�
			draw->Graphic_Data[3].start_x=1740;//��� x ����
			draw->Graphic_Data[3].start_y=666;//��� y ����(840,720)~(1080,720)
			draw->Graphic_Data[3].radius=30;//�����С���߰뾶
			draw->Graphic_Data[3].end_x=1830;//�յ� x ����
			draw->Graphic_Data[3].end_y=626;//�յ� y ����		
			
	/*						�����������										*/
				draw->Graphic_Data[4].graphic_name[0]='1';
			draw->Graphic_Data[4].graphic_name[1]='0';
			draw->Graphic_Data[4].graphic_name[2]='4';
		if(Seq_Feature%10==0) 
		{ 
			draw->Graphic_Data[4].operate_tpye=1;
		  draw->Graphic_Data[4].color=8;//��ɫ
		}
		else if(Seq_Feature%10!=0) 
		{ draw->Graphic_Data[4].operate_tpye=2;
			if(Flage_UI_Mode_Rise==1)
		  draw->Graphic_Data[4].color=1;//��ɫ		
			else if(Flage_UI_Mode_Rise==10)
			draw->Graphic_Data[4].color=4;//��ɫ
			else	
			draw->Graphic_Data[4].color=8;//��ɫ	
					
		}
			draw->Graphic_Data[4].graphic_tpye=1;//ͼ������
			draw->Graphic_Data[4].layer=2;//ͼ����
			draw->Graphic_Data[4].color=8;//��ɫ
			draw->Graphic_Data[4].start_angle=0;//��ʼ�Ƕ�
			draw->Graphic_Data[4].end_angle=0;//��ֹ�Ƕ�
			draw->Graphic_Data[4].width=3;//�߿�
			draw->Graphic_Data[4].start_x=180-100;//��� x ����
			draw->Graphic_Data[4].start_y=710;//��� y ����(840,720)~(1080,720)
			draw->Graphic_Data[4].radius=30;//�����С���߰뾶
			draw->Graphic_Data[4].end_x=270-100;//�յ� x ����
			draw->Graphic_Data[4].end_y=680;//�յ� y ����	

		
			draw->Graphic_Data[5].graphic_name[0]='1';
			draw->Graphic_Data[5].graphic_name[1]='0';
			draw->Graphic_Data[5].graphic_name[2]='5';
			if(Seq_Feature%10==0)		
		{	draw->Graphic_Data[5].operate_tpye=1;//ͼ�β���
			draw->Graphic_Data[5].color=8;//��ɫ			
		}
		else if(Seq_Feature!=10)
		{ draw->Graphic_Data[5].operate_tpye=2;
			if(Flage_UI_Mode_Rise==2)
		  draw->Graphic_Data[5].color=1;//��ɫ
			else if(Flage_UI_Mode_Rise==10)
			draw->Graphic_Data[5].color=4;//��ɫ
			else draw->Graphic_Data[5].color=8;//��ɫ
		}
			draw->Graphic_Data[5].graphic_tpye=1;//ͼ������
			draw->Graphic_Data[5].layer=2;//ͼ����
			draw->Graphic_Data[5].color=8;//��ɫ
			draw->Graphic_Data[5].start_angle=0;//��ʼ�Ƕ�
			draw->Graphic_Data[5].end_angle=0;//��ֹ�Ƕ�
			draw->Graphic_Data[5].width=3;//�߿�
			draw->Graphic_Data[5].start_x=180-100;//��� x ����
			draw->Graphic_Data[5].start_y=678;//��� y ����(840,720)~(1080,720)
			draw->Graphic_Data[5].radius=30;//�����С���߰뾶
			draw->Graphic_Data[5].end_x=270-100;//�յ� x ����
			draw->Graphic_Data[5].end_y=648;//�յ� y ����

		
	
			draw->Graphic_Data[6].graphic_name[0]='1';
			draw->Graphic_Data[6].graphic_name[1]='0';
			draw->Graphic_Data[6].graphic_name[2]='6';		
			if(Seq_Feature%10==0)		
		{	draw->Graphic_Data[6].operate_tpye=1;//ͼ�β���
			draw->Graphic_Data[6].color=8;//��ɫ			
		}
		else if(Seq_Feature%10!=0)
		{	draw->Graphic_Data[6].operate_tpye=2;
			if(Flage_UI_Mode_Rise==3)
		  draw->Graphic_Data[6].color=1;//��ɫ
			else if(Flage_UI_Mode_Rise==10)
			draw->Graphic_Data[6].color=4;//��ɫ
			else draw->Graphic_Data[6].color=8;//��ɫ
		}
			draw->Graphic_Data[6].graphic_tpye=1;//ͼ������
			draw->Graphic_Data[6].layer=2;//ͼ����
			draw->Graphic_Data[6].color=8;//��ɫ
			draw->Graphic_Data[6].start_angle=0;//��ʼ�Ƕ�
			draw->Graphic_Data[6].end_angle=0;//��ֹ�Ƕ�
			draw->Graphic_Data[6].width=3;//�߿�
			draw->Graphic_Data[6].start_x=180-100;//��� x ����
			draw->Graphic_Data[6].start_y=646;//��� y ����(840,720)~(1080,720)
			draw->Graphic_Data[6].radius=30;//�����С���߰뾶
			draw->Graphic_Data[6].end_x=270-100;//�յ� x ����
			draw->Graphic_Data[6].end_y=616;//�յ� y ����
		
	
	memcpy(&ClienTxDynamicrBuffer[5],(uint8_t*)&draw->CMD_ID,(sizeof(ext_student_interactive_header_data_t)+7*sizeof(graphic_data_struct_t)+2*sizeof(uint16_t)));
	append_crc16(ClienTxDynamicrBuffer,sizeof(ext_draw_crosshair_t));	
	for(int i=0;i<sizeof(ext_draw_crosshair_t);i++)
	{	
			//HAL_UART_Transmit_DMA(&huart6,ClienTxDynamicrBuffer,sizeof(ClienTxDynamicrBuffer));
	}
}



int Char_sizeof=0;
char Dr_Char_1[]={" Norm \n Rota \n Die"};
char Dr_Char_2[]={"Magazi \nFriction"};

void Draw_Static_Char(ex_draw_char *draw,uint16_t cmd_id,uint16_t data_id,uint16_t tx_id,uint16_t rx_id)
{	 
	static uint8_t seq_char=0;
	draw->txFrame_Header.sof = 0xa5;
	draw->txFrame_Header.data_length =sizeof(ext_student_interactive_header_data_t)+45;;
	draw->txFrame_Header.seq = seq_char;
	memcpy(Clien_character,&draw->txFrame_Header,sizeof(Frame_Header_t));
	append_crc8(Clien_character,sizeof(Frame_Header_t));
	seq_char++;
	
	draw->CMD_ID = cmd_id;
	draw->Client_Custom_ID.data_cmd_id = data_id;
	draw->Client_Custom_ID.sender_ID = tx_id;
	draw->Client_Custom_ID.receiver_ID = rx_id;
	for(int i=0;i<29;i++)
	draw->data[i]=' ';
	 //��ĸ
	if(seq_char%2==1)
	{
		draw->grapic_data_struct.graphic_name[1] = 'a';
	draw->grapic_data_struct.graphic_name[1] = '0';
	draw->grapic_data_struct.graphic_name[2] = '0';
		draw->grapic_data_struct.operate_tpye = 1;
	draw->grapic_data_struct.graphic_tpye = 7;
	draw->grapic_data_struct.layer = 5;
	draw->grapic_data_struct.color = 3;
	draw->grapic_data_struct.start_angle = 18;//�����С
	draw->grapic_data_struct.end_angle = 30;//�ֳ�
	draw->grapic_data_struct.width = 2;//�߿�
	draw->grapic_data_struct.start_x = 60+15;//��� x ����
	draw->grapic_data_struct.start_y = 700-5;;//��� y ����
	draw->grapic_data_struct.radius = 0;//�����С���߰뾶
	draw->grapic_data_struct.end_x = 0;
	draw->grapic_data_struct.end_y = 0;
	for(int i=0;i<sizeof(Dr_Char_1);i++)
	draw->data[i]=Dr_Char_1[i];
	}
	
	
	else if(seq_char%2!=1)
	{
		draw->grapic_data_struct.graphic_name[1] = 'a';
	draw->grapic_data_struct.graphic_name[1] = '0';
	draw->grapic_data_struct.graphic_name[2] = '1';
		draw->grapic_data_struct.operate_tpye = 1;
	draw->grapic_data_struct.graphic_tpye = 7;
	draw->grapic_data_struct.layer = 5;
	draw->grapic_data_struct.color = 3;
	draw->grapic_data_struct.start_angle = 18;//�����С
	draw->grapic_data_struct.end_angle = 30;//�ֳ�
	draw->grapic_data_struct.width = 2;
	draw->grapic_data_struct.start_x = 1770-25-5;
	draw->grapic_data_struct.start_y = 698-10;
	draw->grapic_data_struct.radius = 0;
	draw->grapic_data_struct.end_x = 0;
	draw->grapic_data_struct.end_y = 0;
		
			for(int i=0;i<sizeof(Dr_Char_2);i++)
	draw->data[i]=Dr_Char_2[i];
	}
	
	memcpy(&Clien_character[5],(uint8_t*)&draw->CMD_ID,(sizeof(ext_student_interactive_header_data_t)+45+2*sizeof(uint16_t)));
	append_crc16(Clien_character,sizeof(ex_draw_char));	
	for(int i=0;i<sizeof(ext_draw_crosshair_t);i++)
	{
		//HAL_UART_Transmit_DMA(&huart6,Clien_character,sizeof(Clien_character));
	}
}

char Dr_Char_3[]={" \nyaw\npit\n"};
static uint8_t Clien_character[60];
void Draw_Static_Char_1(ex_draw_char *draw,uint16_t cmd_id,uint16_t data_id,uint16_t tx_id,uint16_t rx_id)
{	 
	Char_sizeof=sizeof(Dr_Char_3);
	static uint8_t seq_char=0;
	draw->txFrame_Header.sof = 0xa5;
	draw->txFrame_Header.data_length =sizeof(ext_student_interactive_header_data_t)+45;;
	draw->txFrame_Header.seq = seq_char;
	memcpy(Clien_character,&draw->txFrame_Header,sizeof(Frame_Header_t));
	append_crc8(Clien_character,sizeof(Frame_Header_t));
	seq_char++;
	
	draw->CMD_ID = cmd_id;
	draw->Client_Custom_ID.data_cmd_id = data_id;
	draw->Client_Custom_ID.sender_ID = tx_id;
	draw->Client_Custom_ID.receiver_ID = rx_id;
	for(int i=0;i<29;i++)
	draw->data[i]=0;
	 //��ĸ
	if(seq_char%2==1)
	{draw->grapic_data_struct.graphic_name[1] = 'm';
	draw->grapic_data_struct.graphic_name[1] = 'm';
	draw->grapic_data_struct.graphic_name[2] = 'm';
		draw->grapic_data_struct.operate_tpye = 1;
	draw->grapic_data_struct.graphic_tpye = 7;
	draw->grapic_data_struct.layer = 1;
	draw->grapic_data_struct.color = 3;
	draw->grapic_data_struct.start_angle = 20;//�����С
	draw->grapic_data_struct.end_angle = 30;//�ֳ�
	draw->grapic_data_struct.width = 2;
	draw->grapic_data_struct.start_x = 480-200;
	draw->grapic_data_struct.start_y = 540-100;
	draw->grapic_data_struct.radius = 0;
	draw->grapic_data_struct.end_x = 0;
	draw->grapic_data_struct.end_y = 0;
	for(int i=0;i<sizeof(Dr_Char_3);i++)
	draw->data[i]=Dr_Char_3[i];
	}
	
	//����
	else if(seq_char%2!=1)
	{
		draw->grapic_data_struct.graphic_name[1] = 'm';
	draw->grapic_data_struct.graphic_name[1] = 'a';
	draw->grapic_data_struct.graphic_name[2] = 'a';
		draw->grapic_data_struct.operate_tpye = 1;
	draw->grapic_data_struct.graphic_tpye = 7;
	draw->grapic_data_struct.layer = 6;
	draw->grapic_data_struct.color = 2;
	draw->grapic_data_struct.start_angle = 20;//�����С
	draw->grapic_data_struct.end_angle = 30;//�ֳ�
	draw->grapic_data_struct.width = 2;
	draw->grapic_data_struct.start_x = 480-200;
	draw->grapic_data_struct.start_y = 540-100;
	draw->grapic_data_struct.radius = 0;
	draw->grapic_data_struct.end_x = 0;
	draw->grapic_data_struct.end_y = 0;
	}
	
	memcpy(&Clien_character[5],(uint8_t*)&draw->CMD_ID,(sizeof(ext_student_interactive_header_data_t)+45+2*sizeof(uint16_t)));
	append_crc16(Clien_character,sizeof(ex_draw_char));	
	for(int i=0;i<sizeof(ext_draw_crosshair_t);i++)
	{
		//HAL_UART_Transmit_DMA(&huart6,Clien_character,sizeof(Clien_character));
	}
}
char Dr_Char_4[30]={0};
static uint8_t Clien_character[60];
void Draw_Dynamicr_Char_2(ex_draw_char *draw,uint16_t cmd_id,uint16_t data_id,uint16_t tx_id,uint16_t rx_id)
{	 
	Char_sizeof=sizeof(Dr_Char_4);
	static uint8_t seq_char=0;
	draw->txFrame_Header.sof = 0xa5;
	draw->txFrame_Header.data_length =sizeof(ext_student_interactive_header_data_t)+45;;
	draw->txFrame_Header.seq = seq_char;
	memcpy(Clien_character,&draw->txFrame_Header,sizeof(Frame_Header_t));
	append_crc8(Clien_character,sizeof(Frame_Header_t));
	seq_char++;
	
	draw->CMD_ID = cmd_id;
	draw->Client_Custom_ID.data_cmd_id = data_id;
	draw->Client_Custom_ID.sender_ID = tx_id;
	draw->Client_Custom_ID.receiver_ID = rx_id;
	
		for(int i=0;i<29;i++)
	draw->data[i]=0;
	//����
		draw->grapic_data_struct.graphic_name[1] = 'm';
	draw->grapic_data_struct.graphic_name[1] = 'a';
	draw->grapic_data_struct.graphic_name[2] = 'a';
		draw->grapic_data_struct.operate_tpye = 2;
	draw->grapic_data_struct.graphic_tpye = 7;
	draw->grapic_data_struct.layer = 6;
	draw->grapic_data_struct.color = 3;
	draw->grapic_data_struct.start_angle = 20;//�����С
	draw->grapic_data_struct.end_angle = 30;//�ֳ�
	draw->grapic_data_struct.width = 3;
	draw->grapic_data_struct.start_x = 480-200;
	draw->grapic_data_struct.start_y = 540-100;
	draw->grapic_data_struct.radius = 0;
	draw->grapic_data_struct.end_x = 0;
	draw->grapic_data_struct.end_y = 0;
	

		draw->data[0]='\n';
		draw->data[1]=' ';
		draw->data[2]=' ';
		draw->data[3]=' ';
		if(imu.pit>0) draw->data[4]=' ';
		else draw->data[4]='-';
	draw->data[5]=(int)fabs(imu.pit)/100+'0';
	draw->data[6]=(int)fabs(imu.pit)/10%10+'0';
	draw->data[7]=(int)fabs(imu.pit)%10+'0';
	draw->data[8]='.';
	draw->data[9]=(int)fabs(imu.pit*10)%10+'0';	
	draw->data[10]=' ';
	
	draw->data[11]='\n';
	draw->data[12]=' ';
	draw->data[13]=' ';
	draw->data[14]=' ';
	if(imu.yaw>0) draw->data[15]=' ';
		else draw->data[15]='-';
	draw->data[16]=(int)fabs(imu.yaw)/100+'0';
	draw->data[17]=(int)fabs(imu.yaw)/10%10+'0';
	draw->data[18]=(int)fabs(imu.yaw)%10+'0';
	draw->data[19]='.';
	draw->data[20]=(int)fabs(imu.yaw*10)%10+'0';
	
	memcpy(&Clien_character[5],(uint8_t*)&draw->CMD_ID,(sizeof(ext_student_interactive_header_data_t)+45+2*sizeof(uint16_t)));
	append_crc16(Clien_character,sizeof(ex_draw_char));	
	for(int i=0;i<sizeof(ext_draw_crosshair_t);i++)
	{
		//HAL_UART_Transmit_DMA(&huart6,Clien_character,sizeof(Clien_character));
	}
}



/**
 * @Name: Get_Referee_Chassis_Power_Limit
* @Description: ��ȡ�����˵ȼ��Ի�ù�������ֵ
* @Param: void
 * @Return: power_limit��������ֵ
 * @Author: source
 * @Warning: void
 */
float Get_Referee_Chassis_Power_Limit()
{
	float power_limit;
	power_limit = referee_receive_data.jundge_robot_state.data.chassis_power_limit;
	return power_limit;
}

/**
 * @Name: Get_Referee_Chassis_Power
* @Description: ��ȡ����ʵʱ�����Լ�����������Ϣ
* @Param: *chassis_power ---------- ���̹���ʵʱ��Ϣ
          *chassis_power_buffer --- ���̻�������ʵʱ��Ϣ
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Get_Referee_Chassis_Power(float *chassis_power,float *chassis_power_buffer)
{
	*chassis_power = referee_receive_data.jundge_power_heat_data.date.chassis_power;
	*chassis_power_buffer = referee_receive_data.jundge_power_heat_data.date.chassis_power_buffer;
}
/**
 * @Name: Get_Referee_Shoot_Power_Information
* @Description: ��ȡǹ��ʵʱ��Ϣ
* @Param: *heat -------------- ǹ��ʵʱ����
          *cooling_rate ------ ǹ��ÿ����ȴֵ
          *cooling_limit ----- ǹ����������
          *shot_rate --------- ��������
          *shot_freq --------- ������Ƶ
          *shot_type --------- ����ĵ�������
 * @Return: void
 * @Author: source
 * @Warning: void
 */
//void Get_Referee_Shoot_Power_Information(uint16_t *heat,uint16_t *cooling_rate,uint16_t *cooling_limit,float *shot_rate,float *shot_freq,uint8_t *shot_type)
//{
//	*heat = referee_receive_data.jundge_power_heat_data.date.shooter_id1_17mm_cooling_heat;
//	*cooling_rate = referee_receive_data.jundge_robot_state.data.shooter_id1_17mm_cooling_rate;
//	*cooling_limit = referee_receive_data.jundge_robot_state.data.shooter_id1_17mm_cooling_limit;
//	*shot_rate = referee_receive_data.jundge_shoot_data.date.bullet_speed;
//	*shot_freq = referee_receive_data.jundge_shoot_data.date.bullet_freq;
//	*shot_type = referee_receive_data.jundge_shoot_data.date.bullet_type;
//}	


/**
 * @Name: Get_Referee_Shoot_Power_Information
* @Description: ��ȡǹ��ʵʱ��Ϣ
* @Param: *rt_heat ----------------- ǹ��ʵʱ����
          *cooling_rate ------------ ǹ��ÿ����ȴֵ
          *heat_limit -------------- ǹ����������
          *rt_bullet_speed --------- ��������
          *bullet_freq ------------- ������Ƶ
          *bullet_speed_limit ------ ������������
 * @Return: void
 * @Author: source
 * @Warning: void
 */	
void Get_Referee_Shoot_Power_Information(uint16_t *rt_heat, uint16_t *cooling_rate, uint16_t *heat_limit, float *rt_bullet_speed, float *bullet_freq, uint16_t *bullet_speed_limit)
{
	//���ٿ��Ʋ���
	*rt_heat = referee_receive_data.jundge_power_heat_data.date.shooter_id1_17mm_cooling_heat;//yes
	*cooling_rate = referee_receive_data.jundge_robot_state.data.shooter_id1_17mm_cooling_rate;//yes
	*heat_limit = referee_receive_data.jundge_robot_state.data.shooter_id1_17mm_cooling_limit;//yes
	*rt_bullet_speed= referee_receive_data.jundge_shoot_data.date.bullet_speed;
	*bullet_freq = referee_receive_data.jundge_shoot_data.date.bullet_freq;
	*bullet_speed_limit = referee_receive_data.jundge_robot_state.data.shooter_id1_17mm_speed_limit;//yes
}


