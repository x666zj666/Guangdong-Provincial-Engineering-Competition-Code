/***********************************************
¹«Ë¾£ºÂÖÈ¤¿Æ¼¼£¨¶«Ý¸£©ÓÐÏÞ¹«Ë¾
Æ·ÅÆ£ºWHEELTEC
¹ÙÍø£ºwheeltec.net
ÌÔ±¦µêÆÌ£ºshop114407458.taobao.com 
ËÙÂôÍ¨: https://minibalance.aliexpress.com/store/4455017
°æ±¾£ºV1.0
ÐÞ¸ÄÊ±¼ä£º2022-9-26

Company: WHEELTEC Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update£ºº2022-6-20

All rights reserved
***********************************************/
#include "system.h"

#define ARM_DEBUG 0   //0/1£º»úÐµ±Ûµ÷ÊÔ  (0Õý½â/1Äæ½â)

#define ADD_CYCLE_TIME 10  //Â·ÏßÀÛ¼ÓÖÜÆÚ£¬µ¥Î»ms
#define ADD_CYCLE_TIME_TO_M ADD_CYCLE_TIME*0.001  //Â·ÏßÀÛ¼ÓÖÜÆÚ£¬µ¥Î»s

//Task priority    //ÈÎÎñÓÅÏÈ¼¶
#define START_TASK_PRIO	1

//Task stack size //ÈÎÎñ¶ÑÕ»´óÐ¡	
#define START_STK_SIZE 	256  

//Task Ð¡³µÔË¶¯
//float car_x,car_y,car_z;
char Task_flag = 0;  //µØÍ¼ÈÎÎñ±êÖ¾Î»
double sum_line_x=0,sum_line_y=0;
double last_sumline;

u8 run_flag=0;
u8 Arm_flag=0;
u8 photo_flag=0;
float Target_Roll=0,Target_Pitch=0,Target_Yaw=0;
float Error_Yaw,Last_Error_Yaw;
float True_Roll,True_Pitch,True_Yaw;
float True_accel_x,True_accel_y,True_accel_z;
float vx,vy,vz;
float FBaccel,RLaccel;
float StrePath[50]={0.62,-0.555,-0.66,0.885,-0.81,0.89,0.82};//Â·¶ÎÊý×é
char Recali_flag=0;//Ð£×¼±äÁ¿
char arm_flash_flag=0;
char ArmSuc_flag=0;//ÒÆ¶¯³É¹¦±äÁ¿
float Pose[5]={0,0,0,0,0};
//

u8 Task_QRCode[3]={0,0,0};  //¶þÎ¬Âë±£´æ
u16 Task_Line[2]={0,0};    //Ñ²ÏßÊýÖµ±£´æ
u16 Task_Ball[3]={0,0,0};  //Ð¡ÇòÊýÖµ±£´æ
u16 Task_Target[2]={0,0};  //°Ð×ÓÊýÖµ±£´æ
u16 Task_People[3]={0,0,0};  //ÈËÖÊÊýÖµ±£´æ
u16 Task_Drum[2]={0,0};    //ÅÅ±¬Í°ÊýÖµ±£´æ
//
//Pose6D Arm_Pose = {100.0f, 100.0f, 300.0f, 0.0f, 0.0f, 0.0f}; //»úÐµ±ÛÎ»×Ë
//float pose[6];
//Task handle     //ÈÎÎñ¾ä±ú
TaskHandle_t StartTask_Handler;
TaskHandle_t FLASHTask_Handler;
TaskHandle_t LINETask_Handler;
TaskHandle_t SERVOTask_Handler;
TaskHandle_t PHOTOTask_Handler;
TaskHandle_t MPU6050Task_Handler;
TaskHandle_t ArmTask_Handler;
TaskHandle_t DEBUGTask_Handler;
//Task function   //ÈÎÎñº¯Êý
void start_task(void *pvParameters);
void flash_task(void *pvParameters);
void line_task(void *pvParameters);
void photo_task(void *pvParameters);
void MPU6050_task(void *pvParameters);
void arm_task(void *pvParameters);
void debug_task(void *pvParameters);
//Main function //Ö÷º¯Êý
int main(void)
{

  systemInit(); //Hardware initialization //Ó²¼þ³õÊ¼»¯
  
	//Create the start task //´´½¨¿ªÊ¼ÈÎÎñ
	xTaskCreate((TaskFunction_t )start_task,            //Task function   //ÈÎÎñº¯Êý
							(const char*    )"start_task",          //Task name       //ÈÎÎñÃû³Æ
							(uint16_t       )START_STK_SIZE,        //Task stack size //ÈÎÎñ¶ÑÕ»´óÐ¡
							(void*          )NULL,                  //Arguments passed to the task function //´«µÝ¸øÈÎÎñº¯ÊýµÄ²ÎÊý
							(UBaseType_t    )START_TASK_PRIO,       //Task priority   //ÈÎÎñÓÅÏÈ¼¶
							(TaskHandle_t*  )&StartTask_Handler);   //Task handle     //ÈÎÎñ¾ä±ú    					
	vTaskStartScheduler();  //Enables task scheduling //¿ªÆôÈÎÎñµ÷¶È	
}

//Start task task function //¿ªÊ¼ÈÎÎñÈÎÎñº¯Êý
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //Enter the critical area //½øÈëÁÙ½çÇø

    //Create the task //´´½¨ÈÎÎñ
//	  xTaskCreate(pstwo_task,  "pstwo_task",  PS2_STK_SIZE,  NULL, PS2_TASK_PRIO,  NULL);
	  xTaskCreate(Balance_task,  "Balance_task",  BALANCE_STK_SIZE,  NULL, BALANCE_TASK_PRIO,  NULL);	//Vehicle motion control task //Ð¡³µÔË¶¯¿ØÖÆÈÎÎñ
    xTaskCreate(show_task,     "show_task",     SHOW_STK_SIZE,     NULL, SHOW_TASK_PRIO,     NULL); //The OLED display displays tasks //OLEDÏÔÊ¾ÆÁÏÔÊ¾ÈÎÎñ
    xTaskCreate(led_task,      "led_task",      LED_STK_SIZE,      NULL, LED_TASK_PRIO,      NULL);	//LED light flashing task //LEDµÆÉÁË¸ÈÎÎñ
	  xTaskCreate(adc_task,       "ADC_task",     ADC_STK_SIZE,     NULL, ADC_TASK_PRIO,     NULL);//ADC acquisition task 	//ADC²É¼¯ÈÎÎñ
    
    xTaskCreate(flash_task,       "flash_task",     128,     NULL, 5,     FLASHTask_Handler);
    xTaskCreate(line_task,       "line_task",     512,     NULL, 5,     LINETask_Handler);
    xTaskCreate(photo_task,       "photo_task",     258,     NULL, 4,     PHOTOTask_Handler);
    xTaskCreate(MPU6050_task,       "MPU6050_task",    512 ,     NULL, 5,     MPU6050Task_Handler);
    xTaskCreate(arm_task,   "arm_task", 258,     NULL, 5,     ArmTask_Handler);
    xTaskCreate(debug_task,   "debug_task", 128,     NULL, 3,     DEBUGTask_Handler);
  

    vTaskDelete(StartTask_Handler); //Delete the start task //É¾³ý¿ªÊ¼ÈÎÎñ
    taskEXIT_CRITICAL();            //Exit the critical section//ÍË³öÁÙ½çÇø
}

/*·½Î»Í¼
    x
   ¡ü   
    |
    |
¡ª¡ªo¡ª¡ª¡úy
    |
    |
*/

/*Á¬½Ó·½Ê½
ÂÖ×Ó  | °å×Ó½Ó¿Ú
  LF  |   A
  LB  |   B
  RB  |   C
  RF  |   A
*/

void MPU6050_task(void *pvParameters)  //ÍÓÂÝÒÇ¶ÁÈ¡Êý¾ÝÈÎÎñ
{
  static float comp_roll=0,comp_pitch=0,comp_yaw=0;  //²¹³¥Öµ
  static float Temp_Yaw=0;
  static float comp_ax,comp_ay,comp_az;
  static u32 task_count=0;
  static u32 buzzer_count;
  static u8 flag=0,flag_accel=1;
  static u8 buzzer_flag=0;
  Vector3f filtered_accel = {0};
  
  u32 lastWakeTime = getSysTickCnt();
  
  //µÈ´ýMPU6050ÎÈ¶¨

  while(1)
  {
    Read_DMP();

    if(Yaw>=90.0 && Yaw<=180.0)   //yawÐ¡ÓÚ-180»áÌø±äµ½+180
    {
      Temp_Yaw=Yaw-180;
      True_Yaw=comp_yaw+Temp_Yaw-180;
    }
    else 
    {
      Temp_Yaw=Yaw;
      True_Yaw=comp_yaw+Temp_Yaw;
    }
    True_Roll=comp_roll+Roll;
    True_Pitch=comp_pitch+Pitch;
    
    if(run_flag==1)
    {
      Last_Error_Yaw = Error_Yaw;  //¼ÇÂ¼²îÖµÓÃÓÚPIDËã·¨
      Error_Yaw=Target_Yaw-True_Yaw;  //¼ÇÂ¼²îÖµÓÃÓÚPIDËã·¨
    }
    else 
    {
      Error_Yaw=Last_Error_Yaw=0;
    }
    
    task_count++;
    
    if(task_count%100==0 && flag!=3)  //3s¼ì²âÊÇ·ñ»¹ÓÐÆ¯ÒÆ
    {
      if(flag==0)  //¼ÓÔØ²¹³¥Öµ
      {
        comp_roll=-Roll;
        comp_yaw=-Yaw;
        flag=1;
        buzzer_flag=1;
        buzzer_count=task_count;
      }
      else if(flag==1)  //ÅÐ¶ÏÊÇ·ñ»¹´æÔÚÆ¯ÒÆ
      {
        if((True_Yaw<=-0.3||True_Yaw>=0.3)||!(Pitch>=-0.3&&Pitch<=0.3))  //f²»Îª0ËµÃ÷»¹´æÔÚ  //·¶Î§ºÃ »¹ÊÇ 0ºÃ  £¿
        {
          flag=0;   
          buzzer_flag=2;
          buzzer_count=task_count;
        }
        else      //ÒÑ¾­²»´æÔÚÆ¯ÒÆ
        {
          flag=3;
          buzzer_flag=3;
          buzzer_count=task_count;
          run_flag=1;  //¿ªÆôÑ²Ïß
        }
      }
    }
    
    if(task_count>=700&&flag!=3)   //²¹³¥Ê±¼ä¹ý³¤£¬Ç¿ÖÆÔÝÍ£¼ÓÔØ²¹³¥
    {
      comp_roll=-Roll;
      comp_pitch=-Pitch;
      comp_yaw=-Yaw;
      flag=3;
      buzzer_flag=3;
      buzzer_count=task_count;
      run_flag=1;  //
    }
    
    if(buzzer_flag==3)
    {
      if(task_count<buzzer_count+4)Buzzer=1;
      else if(task_count>=buzzer_count+4 && task_count<buzzer_count+6)Buzzer=0;
      else if(task_count>buzzer_count+6 && task_count<buzzer_count+10)Buzzer=1;
      else if(task_count>=buzzer_count+10)Buzzer=0;
    }
    else if(buzzer_flag==2)
    {
      if(task_count<buzzer_count+10)Buzzer=1;
      else Buzzer=0;
    }
    else if(buzzer_flag==1)
    {
      if(task_count<buzzer_count+10)Buzzer=1;
      else Buzzer=0;
    }
    
    vTaskDelayUntil(&lastWakeTime,30);
  }
}

//run_flag:
//0:Ñ²Ïß½Ó¹Ü
//1:ÍÓÂÝÒÇ½Ó¹Ü
//2:Ëø¶¨½Ç¶È
void flash_task(void *pvParameters)  //µç»úË¢ÐÂand½Ç¶ÈË¢ÐÂand¶æ»úË¢ÐÂÈÎÎñ
{
  u32 lastWakeTime = getSysTickCnt();
  static int motor_count=0;
  while(1)
  {
    if(run_flag==1)Move_Z=Error_Yaw*0.035+(Error_Yaw-Last_Error_Yaw)/F2T(RATE_50_HZ)*0.38;     //PDËã·¨   Ë¢ÐÂÐ¡³µ½ÇËÙ¶È£¨0.035   0.4£©
    else if(run_flag==2)Move_Z=0;  //Ëø¶¨µç»ú

    Drive_Motor(Move_X, Move_Y, Move_Z); //Ë¢ÐÂÐ¡³µËÙ¶È
    if(arm_flash_flag)Update_Servo();   //Ë¢ÐÂ¶æ»úÎ»ÖÃ
    
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ)); //20ms
  }
}

//ºóÁ½ÂÖ4cm£¬×óÁ½ÂÖ3cm
//or°å×ÓÇ°4cm×ó3cm
//or°å×ÓÇ°4.4cm×ó3.3cm
//or¿´ÉãÏñÍ·µÄÏß
//¡üÐ¡³µ·ÅÖÃ
void line_task(void *pvParameters)  //Â·Ïß¹æ»®
{
  u32 lastWakeTime = getSysTickCnt();
  static int status=0;
  static int last_status=0;
  static u8 buzzer_flag=0;
  static u16 buzzer_count=0;
  static float Tar_x=0,Tar_y=0;
  static u8 peo_num=0;
  Target_Yaw=0;

//  vTaskDelay(100);
//  JIAZI_RELEASE;
//  RobotArm_SetAllTargetAngles(222,99,229,106,89,-1);    //×¥×¡ÈËÖÊ¶¯×÷
//  while(1)
//  {
//    
//    
//    vTaskDelay(100);
//  }
//  
  while(run_flag==0)   //µÈ´ýÍÓÂÝÒÇÐ£×¼½áÊø
  {
    vTaskDelay(1);
  }
  vTaskDelay(1000);
  while(1)
  {
    if(Task_flag==0)
    {
      switch(status)
      {
        case 0:  //Ð¡³µ´ÓÆðµã³ö·¢µ½´ïÉ¨Ãè¶þÎ¬ÂëÇø
        {
          Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
          Move_X=0.15;Move_Y=0;
          if(sum_line_x>=0.630)//m
          {
            Clear_SumLine(SUM_LINE_X);
            Move_X=Move_Y=0;
            status=1;
            Task_flag=1; Arm_flag=1; //ÈÎÎñ±äÁ¿ÖÃ1£¬Í£Ö¹Ñ²Ïß  //ºóÃæÌí¼ÓÐ£×¼£¡£¡
          }
          break;
        }
        
        case 1:  //É¨ÃèÍê¶þÎ¬Âë×óÆ½ÒÆµ½ÕÏ°­ÎïÇ°
        {
          Get_SumLine_Y(GetCar_Vy(),ADD_CYCLE_TIME_TO_M);
          if(sum_line_y > -0.575){Move_X=0;Move_Y = -0.14;} //×óÆ½ÒÆ
          else 
          {
            static int count=0;
            count++;
            if(count<10){Move_X=0;Move_Y=0;}
            else if(count>=10){Move_X=0.07;Move_Y=-0.07;}
          }
          if(sum_line_y<=-0.675)  //Ð±ÒÆ
          {
            Clear_SumLine(SUM_LINE_Y);
            Move_X=Move_Y=0;
            status=2;
          }
          
          break;
        }
        case 2:   //¾­¹ýÕÏ°­Îï
        {
          static int count=0;
          count++;
          if(count<10){Move_X=0;Move_Y=0;}
          if(count>=10 && count<110)
          {
            Move_X=0.085;
            Move_Y=0;
          }
          else if(count>=110)
          {
            if(sum_line_x>0.60)
            {
              Move_X=0.05;
              Move_Y=0;
            }
            else
            {
              Move_X=0.15;
              Move_Y=0;
            }
            
          }
          Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
          
          if(sum_line_x>0.65 && True_Pitch<=1.5)  //¿ÉÄÜ»¹ÔÚÆÂÉÏ
          {
              Clear_SumLine(SUM_LINE_X);
              Move_X=Move_Y=0;
              status=3; //Ð£×¼£¬ÍùÇ°×ß
              
          }
          break;
        }
        case 3:  //Âý×ß
        {
          static int count=0;
          static u8 flag=4;
          count++;
          if(count<50)break;
          
          if(flag==0)  //ºóÍËµ½ÕÏ°­ÎïÇ°¿¨×¡
          {
            static int count=0;
            run_flag=0;
            Move_X=-0.03;
            if(GetCar_Vx()>=-0.02 && count>=30)
            {
                flag=3;
                Move_X=0;
            }
          }
            else if(flag==3)  //ÍÓÂÝÒÇÐ£×¼
            {
              
              Move_X=0;
              Task_Line[0]=0;
              flag=4;
            }
            else if(flag==4)
            {
              static u16 count=0;
              if(count<=10000)count++;
              if(count==1)RobotArm_Control(0.008,0.12,0.22,4,13);//ÏòÓÒ¿´µÄ  //1.72s
              if(count==200)photo_flag=2;
              if(count<=250)break;
              run_flag=1;
              
              Move_X=0.10;
              Move_Y=0;
              Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
              if(Task_Line[0]>=138)   //155  && count>=410 && Task_Line[0]<=155
              {
                Clear_SumLine(SUM_LINE_X);
                Move_X=Move_Y=0;
                status=4;
              }
              else if(sum_line_x>0.230)
              {
                Clear_SumLine(SUM_LINE_X);
                Move_X=Move_Y=0;
                status=4;
              }
            }
          
          break;
        }
        case 4:                                                     //¾­¹ýÕÏ°­Îïºó×ó×ß
        {
          static u8 flag=0;  //Ï¸·ÖÈÎÎñ
          Get_SumLine_Y(GetCar_Vy(),ADD_CYCLE_TIME_TO_M);
          
          if(flag==0)
          {
            static int count=0;
            static float Err_Line=0;
            static float LastErr_Line=0;
            count++;
//            if(count==1)RobotArm_Control(0.008,0.12,0.22,4,13);//ÏòÓÒ¿´µÄ  //1.72s
            if(count==1)RobotArm_SetAllTargetAngles(222,222,270,197,90,90);  //Ïò×ó¿´µÄ
            if(count==190)photo_flag=2;  //´ò¿ªÑ²Ïß
            if(count==191)run_flag=0;
            if(count<=300)break;  //ÑÓÊ±
            
            
            
            if(Task_Line[0]!=0)
            {
              LastErr_Line = Err_Line;
              Err_Line=160.0-Task_Line[0];
            }
            Move_X=0;Move_Y=-0.15;
            Move_Z=Err_Line*0.01*0.05+(Err_Line-LastErr_Line)/0.01*1.0;
            
            if(sum_line_y<=-0.15)
            {
              flag=1;
              photo_flag=0;  //¹Ø±ÕÑ²Ïß
              Task_Line[0]=Task_Line[1]=0;//Çå¿ÕÑ²ÏßÖµ
              run_flag=2;  //Çå¿ÕZÖá
            }
          }
          else if(flag==1)
          {
            static int count=0;
            count++;
            if(count==1)RobotArm_Control(0.16,0,0.17,1,16);//ÏòÇ°¿´µÄ  //1.69s
            if(count==200)photo_flag=2;  //´ò¿ªÑ²Ïß
            if(count<=250)break;
            
            run_flag=1;
            
            if(Task_Line[0]>=100)Move_Y=-0.10;
            else Move_Y=-0.15;
            
            if(Task_Line[0]>=140 && Task_Line[0]<=160 && sum_line_y<=-0.55)   ///150 && sum_line_y<=-0.45 && Task_Line[0]<=155
            {
              static u8 i_count=0;
              if(i_count++>=5)
              {
                Clear_SumLine(SUM_LINE_Y);
                Move_X=Move_Y=0;
                status=5;
              }
              
            }
            else if(sum_line_y<=-0.865)
            {
              Clear_SumLine(SUM_LINE_Y);
              Move_X=Move_Y=0;
              status=5;
            }
          }
          
          break;
        }
        
        
        case 5:   //×ó×ªºóÇ°×ß
        {
          static u8 flag=0;
          
          Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
          if(flag==0)
          {
            static u16 count=0;
            static float Err_Line=0;
            static float LastErr_Line=0;
            count++;
            if(count==1)RobotArm_Control(0.16,0,0.17,1,16);//ÏòÇ°¿´µÄ
            if(count<=100)break;  //ÑÓÊ±
            run_flag=0; //¹Ø±Õ½Ç¶ÈËø¶¨
            photo_flag=2;
            
            Move_X=0.12;
            
            if(Task_Line[0]!=0)
            {
              LastErr_Line = Err_Line;
              if(sum_line_x<=0.40)Err_Line=160.0-Task_Line[0];    //155
              else if(sum_line_x>0.40 && sum_line_x<=0.65)Err_Line=161.0-Task_Line[0];    //161
            }
            if(sum_line_x<0.65)Move_Z=Err_Line*0.01*0.8+(Err_Line-LastErr_Line)/0.01*2.1;
            else {Move_Z=0;}
            
            if(sum_line_x>=0.70)
            {
              flag=1;
              Move_X=0.08;
              Move_Y=0;
              Move_Z=0;
            }
          }
          else if(flag==1)
          {
            static u16 count=0;
            count++;
            if(count==1)RobotArm_Control(0.008,0.12,0.22,4,13);//ÏòÓÒ¿´µÄ  //1.97s
            if(count==200)photo_flag=2;
            if(count==200)Task_Line[0]=0;  //ÒªÑÓ³Ù½«ÉÏÒ»´ÎÑ²Ïß¼ÇÂ¼É¾µô
            if(count<=250)break;
//            run_flag=1; //½Ç¶ÈËø¶¨
          
            Move_X=0.10;
            if(Task_Line[0]>=140 && Task_Line[0]<=155 )  // && Task_Line[0]<=153 && sum_line_x>=0.85
            {
              static u8 i_count=0;
              if(i_count++>=5)
              {
                Move_X=Move_Y=0;
              Clear_SumLine(SUM_LINE_X);
//              status=6;
              status=88;
              photo_flag=0;
              }
            }
            
          }
          
          if(sum_line_x>=0.96)  //0.74 //0.96  //·Å½øÈ¥£¿£¿£¿£¿
          {
              Move_X=Move_Y=0;
              Clear_SumLine(SUM_LINE_X);
//              status=6;
              status=88;
              photo_flag=0;
          }
            
          break;
        }
        
        case 88:  //×ªÍäÇ°·ÀÖ¹³ö½çÂß¼­
        {
          static u8 flag=0;
          if(flag==0)
          {
            Move_X=-0.05;
            Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
            if(sum_line_x<=-0.012)
            {
              Move_X=0;
              Clear_SumLine(SUM_LINE_X);
              flag=1;
            }
          }
          else if(flag==1)
          {
            static u8 count=0;
            if(count++>=50)flag=2;
          }
          else if(flag==2)
          {
            Move_Y=0.05;
            Get_SumLine_Y(GetCar_Vy(),ADD_CYCLE_TIME_TO_M);
            if(sum_line_y>=0.012)
            {
              Move_Y=0;
              Clear_SumLine(SUM_LINE_Y);
              status=6;
            }
          }
          
          
          break;
        }
        
        case 6:                                      //µÚÒ»´Î×ªÍä
        {
          static int count6=0;
          static char flag=1;
          count6++;
          run_flag=1;  //´ò¿ª½Ç¶ÈËø¶¨
          
          if(flag){Target_Yaw-=90.0;flag=0;} //ÓÒ×ª90¶È
          if(fabs(Error_Yaw)<=1.5 && count6>=100)
          {
            Move_X=0;Move_Y=0;
//            status=102;
//            status=7;
            status=89;
          }
          break;
        }
        
        case 89:  //×ªÍäºóµÄÐ£Õý
        {
          static u16 count=0;
          static u8 flag=0;
          count++;
          if(count==1)RobotArm_Control(0.16,0,0.17,1,16);//ÏòÇ°¿´µÄ
          if(count==200)photo_flag=2;
          if(count<=250)break;
          if(Task_Line[0]<142)Move_Y=-0.03;
          else if(Task_Line[0]>=168)Move_Y=0.03;
          else 
          {
            status=7;
            Move_Y=0;
          }
          break;
        }
        
        case 7:   //Ö±ÐÐ£¬ÍùÅÅ±¬ÇøÓò×ß
        {
          static u16 count7=0;
          static float Err_Line=0;
          static float LastErr_Line=0;
          count7++;
          if(count7==1)RobotArm_Control(0.16,0,0.17,1,16);//ÏòÇ°¿´µÄ
          if(count7==200)photo_flag=2;
          if(count7<=200)break;
          
          run_flag=0;
          Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
          if(Task_Line[0]!=0)  //158
          {
            LastErr_Line = Err_Line;
            Err_Line=155.0-Task_Line[0];
          }
          Move_X=0.10;
          Move_Z=Err_Line*0.01*0.8+(Err_Line-LastErr_Line)/0.01*2.1;
          if(sum_line_x>=0.40)
          {
            Clear_SumLine(SUM_LINE_X);
            Move_X=Move_Y=0;
            Move_Z=0;
            run_flag=2;//¹Ø±ÕÍÓÂÝÒÇµ÷¿Ø
            status=8;
            
            if(fabs(Target_Yaw-True_Yaw)>1.0) //×ßÍêºóÍÓÂÝÒÇÎó²î¹ý´ó£¬Ñ¡ÔñÏàÐÅÉãÏñÍ·Ñ²Ïß½á¹û£¿£¿£¿£¿£¿
            {
              Target_Yaw=True_Yaw;  
              Last_Error_Yaw=Error_Yaw=0;
            }
//            run_flag=1;
          }
          break;
        }
        case 8:   //¿´ÏòÐ¡Çò
        {
          static u16 count8=0;
          static float err_x=0,err_y=0,lserr_x=0,lserr_y=0;
          static float x=0,y=0,xx=0,yy=0;
          static u8 flag=0;
          static char boll_num=0;  //ÒªÕÒµÄÐ¡ÇòÔÚÄÄÀïµÄ¼ÆÊýÆ÷
          
          if(count8<=10000)count8++;
          if(count8==1)
          {
            SetServo_AccelSpeed(&Servo1,0.12,6.0);
            SetServo_AccelSpeed(&Servo2,0.03,3.0);
            RobotArm_SetAllTargetAngles(222,99,219,168,85,-1);//ÏòÐ¡Çò¿´µÄ
          }
          if(count8==50)Move_Z=0;
          if(count8==240)photo_flag=3;
          
          if(count8==245)run_flag=1;
          if(count8<=250)break;

          
          
          if(flag==0)  //×ßÈ¥µÚÒ»¸öÐ¡ÇòµÄÎ»ÖÃ  //¿ÉÄÜ¿ÉÒÔ²»×ß£¿Ö±½ÓÅÐ¶Ï£¿
          {
            if(Task_Ball[1]>=60 && Task_Ball[1]<=250)
            {
              static u8 add_num=0;
              if(Task_Ball[1]<=155)Move_X=-0.04;
              else if(Task_Ball[1]>=165)Move_X=0.04;
              else add_num++;
              if(add_num>=5)
              {
                flag=66;
                Move_X=Move_Y=0;
              }
            }
            else Move_X=0.04;
          }
          
          else if(flag==66)  //ÏÈÅÐ¶ÏÊÇ²»ÊÇÒªÕÒµÄÐ¡Çò
          {
            if(Task_Ball[0]!=Task_QRCode[0])  //²»ÊÇÒªÕÒµÄ
            {
              boll_num+=1;
              //
              if(boll_num>=3)
              {
                boll_num=2;
                flag=2;
              }
              //
              flag=67;
            }
            else if(Task_Ball[0]==Task_QRCode[0])//ÊÇÒªÕÒµÄ
            {
//              flag=2;
              flag=68;
            }
          }
          
          else if(flag==67)  //È¥ÍùÏÂÒ»¸öÎ»ÖÃÕÒÐ¡Çò
          {
            photo_flag=0;
            Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
            Move_X=0.08;
            if(sum_line_x>0.135)
            {
                Clear_SumLine(SUM_LINE_X);
                Move_X=Move_Y=0;
                photo_flag=3;
                vTaskDelay(1000);
//                flag=1;
              flag=66;
            }
          }
          
          else if(flag==68)  //×¥Ð¡Çò¾«È·µ÷Õû
          {
            static u8 ff=1;
            static float xxx=0,yyy=0;
            static u8 ax=0,ay=0;
            photo_flag=3;
            run_flag=2;  //Ëø¶¨µç»ú
            if(ff<=2)RobotArm_SetAllTargetAngles(222+xxx,99,219,168+yyy,85,-1);//ÏòÐ¡Çò¿´µÄ(222,99,219,168,85)
            if(ff==1)
            {
              if(Task_Ball[1]<158)xxx+=0.02;
              else if(Task_Ball[1]>162)xxx-=0.02;
              else ax++;
              if(ax>=10)ff=2;
              
            }
            else if(ff==2)
            {
              if(Task_Ball[2]<110)yyy-=0.02;
              else if(Task_Ball[2]>114)yyy+=0.02;
              else ay++;
              if(ay>=10)ff=3;
            }
            else if(ff==3)
            {
              SetServo_AccelSpeed(&Servo2,0.04,5);
              JIAZI_RELEASE;
              RobotArm_SetAllTargetAngles(222+xxx,50,162,173+yyy,88,-1);
              vTaskDelay(2000);
              BOLL_CATCH;
              vTaskDelay(1000);
              
              SetServo_AccelSpeed(&Servo2,0.06,6);
              RobotArm_Control(0,-0.19,0.15,2,30);//×¥Ð¡ÇòÆðÀ´
              vTaskDelay(1500);
              RobotArm_Control(0,0.19,0.15,2,30);//×ª¹ýÈ¥Ð¡Çò
              vTaskDelay(2500);
              flag=4;
            }
          }
          
          else if(flag==4)  //Ð¡Çò×¥ÍêÒª»ØÈ¥ÁË
          {
            run_flag=1;
            Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
            if(boll_num==0)  //³µÔÚµÚÒ»¸öÎ»ÖÃ
            {
              Move_X=0.08;
              if(sum_line_x>0.145)
              {
                  Clear_SumLine(SUM_LINE_X);
                  Move_X=Move_Y=0;
//                  flag=5;
                status=33;
              }
            }
            else if(boll_num==1)   //³µÔÚÖÐ¼äµÄÎ»ÖÃ
            {
              Clear_SumLine(SUM_LINE_X);
              Move_X=Move_Y=0;
//              flag=5;
              status=33;
            }
            else if(boll_num==2)  //³µÔÚºóÃæµÄÎ»ÖÃ
            {
              Move_X=-0.08;
            
              if(sum_line_x<-0.145)
              {
                  Clear_SumLine(SUM_LINE_X);
                  Move_X=Move_Y=0;
//                  flag=5;
                status=33;
              }
            }
          }
          
          break;
        }
        case 33://·ÅÐ¡ÇòÂß¼­£¨154£¬133£©ÕÒÅÅ±¬Í°
        {
          
          static u16 count=0;
          static u8 add_flagx=0,add_flagy=0;
          static u8 flag=0;
          static float x=0,y=0;
          if(count<=10000)count++;
          if(count==1)run_flag=2;  //Ëø¶¨µç»ú
          if(count==10)RobotArm_SetAllTargetAngles(51,190,248,196,88,-1);
//          if(count==60)Move_Z=0;
          if(count==100)photo_flag=6;
          if(count<=200)break;
          
          if(flag<=1)RobotArm_SetAllTargetAngles(51+x,190,248,196+y,88,-1);
          if(flag==0)//
          {
            if(Task_Drum[0]==0)x-=0.1;
            else if(Task_Drum[0]<153)x+=0.02;
            else if(Task_Drum[0]>155)x-=0.02;
            else add_flagx++;
            if(add_flagx>=10)flag=1;
          }
          if(flag==1)
          {
            if(Task_Drum[1]<158)y-=0.02;
            else if(Task_Drum[1]>160)y+=0.02;
            else add_flagy++;
            if(add_flagy>=10)flag=2;
          }
          if(flag==2)
          {
            RobotArm_SetAllTargetAngles(51+x,81,180,182+y,89,-1);  //·ÅÐ¡ÇòµÄ
            vTaskDelay(2000);
            JIAZI_RELEASE;
            vTaskDelay(1000);
            RobotArm_SetAllTargetAngles(45,222,270,197,90,90);  //ÉÏÀ´
            vTaskDelay(800);
            status=9;
          }
          
          break;
        }
        case 9:   //×ß³¤Ö±Ïß
        {
          static u16 count=0;
          static float Err_Line=0;
          static float LastErr_Line=0;
          count++;
          if(count==1)RobotArm_Control(0.16,0,0.17,1,16);//ÏòÇ°¿´µÄ
          if(count==100)photo_flag=2;  //´ò¿ªÑ²Ïß
          if(count<=200)break;
          
          run_flag=0;  //¹Ø±Õ½Ç¶Èµ÷¿Ø
          
          if(Task_Line[0]!=0)
          {
            LastErr_Line = Err_Line;
            Err_Line=153.0-Task_Line[0];
          }
          
          Move_X=0.2;
          Move_Z=Err_Line*0.01*0.8+(Err_Line-LastErr_Line)/0.01*2.1;
          Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
          
          if(sum_line_x>=1.5)
          {
            Clear_SumLine(SUM_LINE_X);
            Move_X=Move_Y=0;
            status=10;
            Task_Line[0]=Task_Line[1]=0;
            if(fabs(Target_Yaw-True_Yaw)>0.5) //×ßÍêºóÍÓÂÝÒÇÎó²î¹ý´ó£¬Ñ¡ÔñÏàÐÅÉãÏñÍ·Ñ²Ïß½á¹û£¿£¿£¿£¿£¿
            {
              Target_Yaw=True_Yaw;  
              Last_Error_Yaw=Error_Yaw=0;
            }
            run_flag=1;  //ÖØÐÂ´ò¿ªÍÓÂÝÒÇµ÷¿Ø???
          }
          break;
        }
        case 10:   //×ßµ½°Ð×ÓÇ°Ãæ
        {
          Move_X=0.1;
          Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
          if(sum_line_x>=0.20)
          {
            Clear_SumLine(SUM_LINE_X);
            Move_X=Move_Y=0;
            status=11;
          }
          
          break;
        }
//        case 11:   //Ãé×¼°Ð×Ó  ´ÖÂÔÃé×¼  //»òÐí¿ÉÒÔ×ßµ½°Ð×ÓÕýÇ°Ãæ²¢ÇÒ¼ÇÂ¼Â·¾¶
//        {
//          static u8 flag=0;
//          static u16 count=0;
//          static u8 add_flag=0;
//          if(count<=10000)count++;
//          if(count==10)RobotArm_SetAllTargetAngles(222+Tar_x,222,270,177,90,90);
//          if(count==20)SetServo_AccelSpeed(&Servo1,0.1,8);
//          if(count>=20)photo_flag=4;  //´ò¿ª°Ð×Ó¼ì²â
//          if(count<=200)break;
//          
//          RobotArm_SetAllTargetAngles(222+Tar_x,222,270,177,90,90);
//          
//          
//          if(Task_Target[0]<150)Tar_x+=0.1;
//          else if(Task_Target[0]>160)Tar_x-=0.1;
//          else add_flag++;
//          if(Task_Target[0]>150 && Task_Target[0]<160)add_flag++;
//          if(add_flag>=5)
//          {
//            status = 55;
//          }
//            
//          break;
//        }
        case 11:   //Ãé×¼°Ð×Ó  ´ÖÂÔÃé×¼  //ÐÞ¸Ä£ºÖ±½ÓÉì³¤ÁË  //¼ÓÁËºÃ¶àÅÐ¶Ï
        {
          static u8 flag=0;
          static u16 count=0;
          static u8 add_flag=0;
          if(count<=10000)count++;
          if(count==10)SetServo_AccelSpeed(&Servo1,0.1,8);
          if(count==20)RobotArm_SetAllTargetAngles(218+Tar_x,50,123,150,88,90);//´óÅÚ
          if(count>=20)photo_flag=4;  //´ò¿ª°Ð×Ó¼ì²â
          if(count<=400)break;
          
          if(count==5000)status=12;  //³¬Ê±¼ì²â 50s
          
          RobotArm_SetAllTargetAngles(218+Tar_x,50,123,150,88,90);
          
          
          if(Task_Target[0]<150 && Task_Target[0]!=0)Tar_x+=0.08;
          else if(Task_Target[0]>160)Tar_x-=0.08;
          else if(Task_Target[0]==0)Tar_x+=0.08;
//          else add_flag++;
          
          if(Task_Target[0]>150 && Task_Target[0]<160)add_flag++;
          
          if(222+Tar_x>=270)  //µ½¼«ÏÞ»¹Ã»ÕÒ³öÀ´
          {
            add_flag=0;
            Tar_x=-50;
          }
          if(add_flag>=5)
          {
            status = 55;
          }
            
          break;
        }
        
//        case 11:   //Ãé×¼°Ð×Ó  ´ÖÂÔÃé×¼  //ÐÞ¸Ä£ºÖ»Éì³¤ÁË£¡£¡£¡ 
//        {
//          static u8 flag=0;
//          static u16 count=0;
//          static u8 add_flag=0;
//          if(count<=10000)count++;
//          if(count==10)RobotArm_SetAllTargetAngles(222+Tar_x,50,125,150,88,90);//´óÅÚ
//          if(count==20)SetServo_AccelSpeed(&Servo1,0.1,8);
//          if(count>=20)photo_flag=4;  //´ò¿ª°Ð×Ó¼ì²â
//          if(count<=200)break;
//          
//          RobotArm_SetAllTargetAngles(222+Tar_x,50,125,150,88,90);//´óÅÚ
//          
//          if(Task_Target[0]<150)Tar_x+=0.08;
//          else if(Task_Target[0]>160)Tar_x-=0.08;
////          else add_flag++;
//          if(Task_Target[0]>150 && Task_Target[0]<160)add_flag++;
//          if(add_flag>=5)
//          {
//            status = 55;
//          }
//            
//          break;
//        }
        
        case 55:  //¾«Ï¸Ãé×¼  (155£¬130)Îª¼¤¹âºÍ°Ð×ÓÖØµþµÄÎ»ÖÃ
        {
          static u8 flag=0;
          static u16 count=0;
          static u8 add_flagx=0,add_flagy=0;
          if(count<=10000)count++;
          if(count==1)RobotArm_SetAllTargetAngles(222+Tar_x,50,125,150,88,90);//´óÅÚ(222£¬50£¬125,150,88,90)
          if(count==250)run_flag=2; //Ëø¶¨µç»ú
          if(count<=300)break;
          
          RobotArm_SetAllTargetAngles(222+Tar_x,50,125,150+Tar_y,88,90);
          if(flag==0)
          {
            if(Task_Target[0]>156)Tar_x-=0.02;
            else if(Task_Target[0]<154)Tar_x+=0.02;
            else add_flagx++;
            
            if(add_flagx>=10)flag=1;
          }
          else if(flag==1)
          {
            if(Task_Target[1]<129)Tar_y-=0.02;
            else if(Task_Target[1]>131)Tar_y+=0.02;
            else add_flagy++;
            
            if(add_flagy>=10)
            {
              SetServo_AccelSpeed(&Servo2,0.05,4.0);
              vTaskDelay(1000);
              Laser=1;
              vTaskDelay(1000);
              Laser=0;
              vTaskDelay(500);
              RobotArm_SetAllTargetAngles(222,210,240,197,90,90);
              vTaskDelay(1500);
              RobotArm_Set_TargetAngle(1,135);
              vTaskDelay(1500);
              status = 12;
            }
          }
          break;
        }
        
        case 12:   //×¼±¸×ªÍä
        {
          static u8 flag=0;
          static u16 count12=0;
          if(count12<=10000)count12++;
          if(count12==1)RobotArm_SetAllTargetAngles(45,222,270,197,90,90);
          if(count12==10)run_flag=1;  //Î»ÖÃËø¶¨
          if(count12==200)photo_flag=2;  //´ò¿ªÑ²Ïß¼ì²â
          if(count12<=300)break;
          
          Move_X=0.10;
          Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
          
          if(Task_Line[0]>=140 && Task_Line[0]<=150 && sum_line_x>=0.3)  // && sum_line_x>=0.2
          {
            static u8 i_count=0;
            if(i_count++>=2)
            {
              Clear_SumLine(SUM_LINE_X);
              Move_X=Move_Y=0;
//              status=13;
              status=77;
            }
            
          }
          else if(sum_line_x>=0.55)
          {
            Clear_SumLine(SUM_LINE_X);
            Move_X=Move_Y=0;
            if(fabs(Target_Yaw-True_Yaw)>=1.0)   //Îó²î¹ý´ó
            {
              Target_Yaw=True_Yaw;
              Error_Yaw=0;
            }
            status=77;
//            status=13;
          }
          
          
          break;
        }
        
        case 77:  //×ªÍäÇ°·ÀÖ¹³ö½çÂß¼­
        {
          static u8 flag=0;
          if(flag==0)
          {
            Move_X=-0.05;
            Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
            if(sum_line_x<=-0.010)
            {
              Move_X=0;
              Clear_SumLine(SUM_LINE_X);
              flag=1;
            }
          }
          else if(flag==1)
          {
            static u8 count=0;
            if(count++>=50)flag=2;
          }
          else if(flag==2)
          {
            Move_Y=0.05;
            Get_SumLine_Y(GetCar_Vy(),ADD_CYCLE_TIME_TO_M);
            if(sum_line_y>=0.010)
            {
              Move_Y=0;
              Clear_SumLine(SUM_LINE_Y);
              status=13;
            }
          }
          break;
        }
        
        case 13:   //×ªÍä
        {
          static int count13=0;
          static char flag=1;
          count13++;
          run_flag=1;  //´ò¿ª½Ç¶ÈËø¶¨
          
          if(flag)//ÓÒ×ª90¶È
          {
            Target_Yaw-=90.0;
            flag=0;
          } 
          if(fabs(Error_Yaw)<=1.0 && count13>=50)
          {
//            status=90;
            status=91;
          }
          break;
        }
        case 91:  //»ØÆ«
        {
          Move_Y=-0.05;
          Get_SumLine_Y(GetCar_Vy(),ADD_CYCLE_TIME_TO_M);
          if(sum_line_y<=-0.010)
          {
            Move_Y=0;
            Clear_SumLine(SUM_LINE_Y);
            status=90;
          }
        }
        case 90:  //×ªÍäºóµÄÐ£Õý
        {
          static u16 count=0;
          static u8 flag=0;
          count++;
          if(count==1)RobotArm_Control(0.16,0,0.17,1,16);//ÏòÇ°¿´µÄ
//          if(count==2)run_flag=1;  //½Ç¶Èµ÷¿Ø
          if(count==2)run_flag=0;  //½Ç¶Èµ÷¿Ø
          if(count==200)photo_flag=2;
          if(count<=250)break;
//          if(Task_Line[0]<140)Move_Y=-0.05;
//          else if(Task_Line[0]>=165)Move_Y=0.05;
          if(Task_Line[0]<145)Move_Z=0.2;
          else if(Task_Line[0]>=165)Move_Z=-0.2;
          else 
          {
            status=14;
            Move_X=Move_Y=0;
            run_flag=2;  //Ïû³ýµ÷¿ØºóMove_ZµÄÓ°Ïì
          }
          break;
        }
        case 14:   //×ßÈ¥ÈËÖÊÇøÓò
        {
          static u16 count=0;
          static float Err_Line=0;
          static float LastErr_Line=0;
          count++;
          if(count==1)RobotArm_Control(0.16,0,0.17,1,16);//ÏòÇ°¿´µÄ
          if(count==100)photo_flag=2;  //´ò¿ªÑ²Ïß
          if(count<=200)break;
          
          run_flag=0;  //¹Ø±Õ½Ç¶Èµ÷¿Ø
          
          if(Task_Line[0]!=0)
          {
            LastErr_Line = Err_Line;
            Err_Line=153.0-Task_Line[0];
          }
          
          if(sum_line_x<=0.40)Move_X=0.15;
          else Move_X=0.12;
          Move_Z=Err_Line*0.01*0.8+(Err_Line-LastErr_Line)/0.01*2.1;
          Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
          
          if(sum_line_x>=0.70)
          {
            Clear_SumLine(SUM_LINE_X);
            Move_X=Move_Y=0;
            status=15;

            if(fabs(Target_Yaw-True_Yaw)>0.5) //×ßÍêºóÍÓÂÝÒÇÎó²î¹ý´ó£¬Ñ¡ÔñÏàÐÅÉãÏñÍ·Ñ²Ïß½á¹û£¿£¿£¿£¿£¿
            {
              Target_Yaw=True_Yaw;  
              Last_Error_Yaw=Error_Yaw=0;
            }
            run_flag=1;  //ÖØÐÂ´ò¿ªÍÓÂÝÒÇµ÷¿Ø???
          }
          break;
        }
        case 15:   //ÕÒÈËÖÊ
        {
          static u16 count15=0;

          static u8 flag=99;
          static float x=0,y=0;
          if(count15<=10000)count15++;
          if(count15==10)RobotArm_SetAllTargetAngles(222,182,270,172,87,-1);  //ÕÒÈËÖÐ»úÐµ±Û×´Ì¬
          if(count15==150)photo_flag=5;
          if(count15<=250)break;
          
          if(flag==99) //×ß¹ýÈ¥ÕÒÈËÖÊ
          {
            if(Task_People[1]>=60 && Task_People[1]<=250)
            {
              static u8 add_num=0;
              if(Task_People[1]<=155)Move_X=-0.05;
              else if(Task_People[1]>=165)Move_X=0.05;
              else add_num++;
              if(add_num>=5)
              {
                flag=0;
                Move_X=Move_Y=0;
              }
            }
            else Move_X=0.05;
          }
          if(flag==0)  //ÅÐ¶ÏÈËÖÊ
          {
            if(Task_People[0]!=Task_QRCode[2] && Task_People[0]!=0)  //²»ÊÇÒªÕÒµÄ
            {
              peo_num+=1;
              if(peo_num>2)flag=2;  //»¹ÊÇÕÒ²»µ½£¡£¡£¡Ç¿ÖÆÌøÈëflag2;
              flag=1;
            }
            else if(Task_People[0]==Task_QRCode[2] && Task_People[0]!=0)flag=2;  //ÊÇÒªÕÒµÄ
          }
          else if(flag==1)  //ÍùÇ°×ßÒ»Ð¡²½
          {
            Move_X=0.08;
            Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
            if(sum_line_x>=0.13)
            {
              Clear_SumLine(SUM_LINE_X);
              Move_X=Move_Y=0;
              flag=0;
            }
          }
          else if(flag==2)  //»úÐµ±ÛÐ£×¼¶¯×÷
          {
            
            static u8 add_flag = 0;
            if(Task_People[1]<156)x+=0.02;
            else if(Task_People[1]>162)x-=0.02;
            else add_flag++;
            RobotArm_SetAllTargetAngles(222+x,182,270,172,87,-1);
            run_flag=2;  //Ëø¶¨µç»ú
            
            if(add_flag>=10)flag=3;  //Ð£×¼Íê±Ï£¬×¼±¸×¥È¡
          }
          else if(flag==3)  //»úÐµ±Û×¥È¡¶¯×÷
          {
            SetServo_AccelSpeed(&Servo2,0.03,3.0);
            JIAZI_RELEASE;
            vTaskDelay(550);
//            RobotArm_SetAllTargetAngles(222+x,99,229,106,89,-1);    //×¥×¡ÈËÖÊ¶¯×÷
            RobotArm_SetAllTargetAngles(222+x,95,222,106,89,-1);    //×¥×¡ÈËÖÊ¶¯×÷
            vTaskDelay(2000);
            HOSTAGE_CATCH;
            vTaskDelay(1000);
            SetServo_AccelSpeed(&Servo2,0.04,5.0);
            RobotArm_SetAllTargetAngles(222+x,95,200,106,89,-1);
            vTaskDelay(1000);
            RobotArm_SetAllTargetAngles(222+x,222,270,106,90,-1);
            vTaskDelay(1000);
            RobotArm_SetAllTargetAngles(135,222,270,180,90,-1);
            vTaskDelay(1000);
            status=16;
          }
          break;
        }
        case 16:   //×ß»ØÈ¥ÖÕµã
        {
          static float Err_Line=0;
          static float LastErr_Line=0;
          static u16 count=0;
          if(count<=10000)count++;
          if(count==1)RobotArm_Control(0.16,0,0.17,1,16);//ÏòÇ°¿´µÄ
          if(count==180)photo_flag=2;
          if(count==250)run_flag=0;
          if(count<=250)break;
          
          if(Task_Line[0]!=0) // && Task_Line[0]>=70 && Task_Line[0]<=190
          {
            run_flag=0;
            LastErr_Line = Err_Line;
            Err_Line=155.0-Task_Line[0];
          }
//          else 
//          {
//            run_flag=1;
//          }
          
          Move_X=0.15;
          if(sum_line_x<=1.20)Move_Z=Err_Line*0.01*0.8+(Err_Line-LastErr_Line)/0.01*2.1;
          else Move_Z=0;
          
          
          Get_SumLine_X(GetCar_Vx(),ADD_CYCLE_TIME_TO_M);
          if(sum_line_x>1.67 - peo_num*0.13)
          {
              Clear_SumLine(SUM_LINE_X);
              Move_X=Move_Y=0;
              status=17;
          }
          break;
        }
      }      
    }
    
    if(buzzer_flag==1)
    {
      buzzer_count++;
      Buzzer=1;
      if(buzzer_count>=300)
      {
        buzzer_count=0;
        buzzer_flag=0;
        Buzzer=0;
      }
    }
    vTaskDelay(ADD_CYCLE_TIME);
  }
}
/*Äæ½âËãÎ»ÖÃ

      ³µÍ·
       +x

×ó-y           +yÓÒ

       -x
      ³µÎ²
*/
void arm_task(void *pvParameters)  //»úÐµ±ÛÈÎÎñ
{
  static uint32_t task_counter = 0;
  //µÈ´ýÏµÍ³ÎÈ¶¨
  RobotArm_SetAllTargetAngles(135,222,270,197,90,90);

  arm_flash_flag=1;
  vTaskDelay(1000);
  arm_task_enable=1;
  
  while(1)
  {

    if(arm_task_enable) 
    {
      if(Arm_flag==1)  //É¨Ãè¶þÎ¬ÂëµÄ»úÐµ±Û×´Ì¬
      {
        RobotArm_Control(0.14,0,0.20,2,-2);
        vTaskDelay(1000);
        photo_flag=1;
        while(photo_flag==1){vTaskDelay(10);}
        RobotArm_SetAllTargetAngles(135,222,270,200,90,90);
        vTaskDelay(1500);
        Task_flag=0;
        Arm_flag=0;
      }

      if(Arm_flag==2)  //Ô½ÕÏºó¿´×ó±ßÐ£×¼µÄ»úÐµ±Û×´Ì¬
      {
        static u8 flag=0;
        
        if(flag)
        {
          RobotArm_Control(0.008,0.12,0.22,4,13);//ÏòÓÒ¿´µÄ
//          RobotArm_SetAllTargetAngles(45,222,270,197,90,90);  //ÏòÓÒ±ß¿´µÄ
          flag=0;
        }
        vTaskDelay(1000);
        ArmSuc_flag=1;
      }
      if(Arm_flag==3)  //Ô½ÕÏºóÏò×ó×ßºó¿´Ç°ÃæÐ£×¼µÄ»úÐµ±Û×´Ì¬
      {
        static u8 flag=1;
        if(flag)
        {
          RobotArm_Control(0.12,0,0.24,1,22);//Ç°ÃæÐ£×¼µÄ
          flag=0;
        }
        vTaskDelay(1000);
        ArmSuc_flag=1;
      }
      if(Arm_flag==4)//ÏòÇ°¿´µØÏÂÑ²ÏßµÄ»úÐµ±Û×´Ì¬
      {
        static u8 flag=1;
        if(flag)
        {
          RobotArm_Control(0.16,0,0.17,1,16);//ÏòÇ°¿´µÄ
          flag=0;
        }
        vTaskDelay(1000);
        ArmSuc_flag=1;
      }
      if(Arm_flag==5)//¿´ÏòÕýÇ°·½Ð¡ÇòµÄ»úÐµ±Û×´Ì¬
      {
        static u8 flag=1;
        if(flag)
        {
          RobotArm_Control(0.02,-0.23,0.01,2,64);//ÏòÐ¡Çò¿´µÄ
          flag=0;
        }
        vTaskDelay(1000);
        ArmSuc_flag=1;
      }
      
    }
    
    
     vTaskDelay(10); // 10msÑÓÊ±
  }
}

void photo_task(void *pvParameters)  //½ÓÊÕ´¦Àí
{
  static int i_count=0;

  while(1)
  {
    i_count++;
    if(photo_flag==0)
    {
      if(i_count%1000==0)printf("X\n");//ÏòÉãÏñÍ··¢³öÖ¸Áî
    }
    else if(photo_flag==1)  //¶þÎ¬Âë
    {
      if(!Read_End_flag)
      {
        if(i_count%500==0)printf("Q\n");//ÏòÉãÏñÍ··¢³öÖ¸Áî
      }

      if(Read_End_flag==1)//½ÓÊÕÍêÉãÏñÍ·ÐÅÏ¢
      {
        Show_flag=1;
        if(Receive_Buf[0]=='Q' &&Receive_Buf[4]=='\n')
        {
          Task_QRCode[0]=Receive_Buf[1]-'0';
          Task_QRCode[1]=Receive_Buf[2]-'0';
          Task_QRCode[2]=Receive_Buf[3]-'0';
          photo_flag=0;
        }
        Read_End_flag=0;
        
      }
    }
    
    else if(photo_flag==2)  //Ñ²Ïß
    {
      static u8 flag=0;
      if(Read_End_flag !=1)
      {
        if(i_count%200==0)printf("L\n");//ÏòÉãÏñÍ··¢³öÖ¸Áî
      }
      if(Read_End_flag==1)//½ÓÊÕÍêÉãÏñÍ·ÐÅÏ¢
      {
        Show_flag=1;
        if(Receive_Buf[0]=='L')
        {
          u8 *str = Receive_Buf;
          int num1, num2;
          // Ê¹ÓÃsscanfÌáÈ¡Êý×Ö
          sscanf(str, "L%dA%dB", &num1, &num2);
          Task_Line[0]=num1;
          Task_Line[1]=num2;
        }
        Read_End_flag=0;
      }
    }
    else if(photo_flag==3)  //Ð¡Çò
    { 
      static u8 flag=0;
      if(Read_End_flag !=1)
      {
        if(i_count%200==0)printf("B\n");//ÏòÉãÏñÍ··¢³öÖ¸Áî
      }
      if(Read_End_flag==1)//½ÓÊÕÍêÉãÏñÍ·ÐÅÏ¢
      {
        Show_flag=1;
        if(Receive_Buf[0]=='B')
        {
          u8 *str = Receive_Buf;
          int num1, num2, num3;
          // Ê¹ÓÃsscanfÌáÈ¡Êý×Ö
          sscanf(str, "B%dX%dY%d\n", &num1, &num2, &num3);
          Task_Ball[0]=num1;
          Task_Ball[1]=num2;
          Task_Ball[2]=num3;
        }
        Read_End_flag=0;
      }
    }
    else if(photo_flag==4)   //°Ð×Ó
    {
      static u8 flag=0;
      if(Read_End_flag !=1)
      {
        if(i_count%200==0)printf("T\n");//ÏòÉãÏñÍ··¢³öÖ¸Áî
      }
      if(Read_End_flag==1)//½ÓÊÕÍêÉãÏñÍ·ÐÅÏ¢
      {
        Show_flag=1;
        if(Receive_Buf[0]=='T')
        {
          u8 *str = Receive_Buf;
          int num1, num2;
          // Ê¹ÓÃsscanfÌáÈ¡Êý×Ö
          sscanf(str, "T%dX%dY\n", &num1, &num2);
          Task_Target[0]=num1;
          Task_Target[1]=num2;
        }
        Read_End_flag=0;
      }
    }
    else if(photo_flag==5)  //ÈËÖÊ
    {
      static u8 flag=0;
      if(Read_End_flag !=1)
      {
        if(i_count%200==0)printf("P\n");//ÏòÉãÏñÍ··¢³öÖ¸Áî
      }
      if(Read_End_flag==1)//½ÓÊÕÍêÉãÏñÍ·ÐÅÏ¢
      {
        Show_flag=1;
        if(Receive_Buf[0]=='P')
        {
          u8 *str = Receive_Buf;
          int num1, num2, num3;
          // Ê¹ÓÃsscanfÌáÈ¡Êý×Ö
          sscanf(str, "P%dX%dY%d\n", &num1, &num2, &num3);
          Task_People[0]=num1;
          Task_People[1]=num2;
          Task_People[2]=num3;
        }
        Read_End_flag=0;
      }
    }
    
    else if(photo_flag==6)  //Í°
    {
      static u8 flag=0;
      if(Read_End_flag !=1)
      {
        if(i_count%200==0)printf("D\n");//ÏòÉãÏñÍ··¢³öÖ¸Áî
      }
      if(Read_End_flag==1)//½ÓÊÕÍêÉãÏñÍ·ÐÅÏ¢
      {
        Show_flag=1;
        if(Receive_Buf[0]=='D')
        {
          u8 *str = Receive_Buf;
          int num1, num2;
          // Ê¹ÓÃsscanfÌáÈ¡Êý×Ö
          sscanf(str, "D%dX%dY\n", &num1, &num2);
          Task_Drum[0]=num1;
          Task_Drum[1]=num2;
        }
        Read_End_flag=0;
      }
    }
    
    vTaskDelay(1);
  }
}

void debug_task(void *pvParameters)
{
  static u8 arm_num=0,buzzer_flag=0,add_pt=1;//ÐÞ¸ÄµÄ¼«ÐÔ
  static u8 pose_num=0;
  static u32 task_count=0,buzzer_count=0;
  static int run_flag=0;
  static u8 LongP_Num=0;
  while(1)
  {
    #if ARM_DEBUG == 0
    if(Long_Press_Pro(1500)==1)  //³¤°´
    {
      LongP_Num++;
      buzzer_flag=1;
      arm_num++;
      arm_num%=6;
      
      if(LongP_Num>=2)
      {
        Task_flag=2;
      }
    }
    else if(click_N_Double(250)==1)//µ¥»÷
    {
      ServoController Temp_Servo;
      buzzer_flag=1;
      switch(arm_num)
        {
          case 0:Temp_Servo=Servo1;break;
          case 1:Temp_Servo=Servo2;break;
          case 2:Temp_Servo=Servo3;break;
          case 3:Temp_Servo=Servo4;break;
          case 4:Temp_Servo=Servo5;break;
          case 5:Temp_Servo=Servo6;break;
        }
      if(add_pt==0)RobotArm_Set_TargetAngle(arm_num+1,Temp_Servo.target_angle-1);
      else if(add_pt==1)RobotArm_Set_TargetAngle(arm_num+1,Temp_Servo.target_angle+1);
    }
    else if(click_N_Double_MPU6050(250)==2)//Ë«»÷
    {
      buzzer_flag=1;
      add_pt++;
      add_pt%=2;
    }
    #endif
    
    #if ARM_DEBUG == 1
    if(Long_Press_Pro(1500)==1)  //³¤°´
    {
      buzzer_flag=1;
      pose_num++;
      pose_num%=5;
    }
    else if(click_N_Double(250)==1)//µ¥»÷
    {
      buzzer_flag=1;
      if(add_pt)
      {
        if(pose_num<=2)Pose[pose_num]+=0.01;
        else Pose[pose_num]+=1;
      }
      else 
      {
        if(pose_num<=2)Pose[pose_num]-=0.01;
        else Pose[pose_num]-=1;
      }
    }
    else if(click_N_Double_MPU6050(250)==2)//Ë«»÷
    {
      buzzer_flag=1;
      add_pt++;
      add_pt%=2;
    }
    #endif
    
    if(buzzer_flag==1)
    {
      buzzer_count++;
      Buzzer=1;
      if(buzzer_count>=100)
      {
        buzzer_count=0;
        buzzer_flag=0;
        Buzzer=0;
      }
    }
    
    vTaskDelay(1);
  }
}