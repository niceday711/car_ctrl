// #include <inttypes.h>
// #include "car_drive.h"
// #include "ir_track.h"
// #include "motor_drive.h"
// #include "oled_i2c.h"
// #include "usb_uart.h"
// #include "usart.h"
// #include "angle_kalman_filter.h"
// #include "adc.h"
// #include "esp32.h"
// #include "user_flash.h"
// #include "uart_dma.h"

// extern __IO uint8_t  g_music_enable  ;

// car_config_t g_CarConfig =
// {
// 	.KP = 30,
// 	.KI = 0.0 ,
// 	.KD = 2.8,   //3.2,
// 	.car_speed_set = 400 ,
// 	.car_speed_max = 700 ,
// 	.car_speed_min = 300 ,	
// 	.adc_compare_gate = 500 ,	
// 	.adc_interval     = 2 ,
// 	.car_ctrl_interval= 2 ,
// 	.kalman_enable    = 1 ,
// 	.kalman_confidence = { 0.2 , 0.2 } ,
// 	.bt_name = "CAR2023_77777" ,
// 	.bt_pwd  = "8888"	
// };

// car_ctrl_t 	g_CarCtrl;



// uint8_t g_cmd_buf[CMD_BUF_LEN] = {0};

// void UserConfigCallback(uint8_t *buf, void *ptr);
// void UserManualCtrlCallback(uint8_t *buf, void *ptr);
// void UserCtrlCmdCallback(uint8_t *buf, void *ptr);

// car_ctrl_cmd_t g_CarCtrlCmd[] = {
// 	{	USER_CMD_START			,	&UserCtrlCmdCallback, 			0 , 0 },
// 	{	USER_CMD_BT_ON			,	&UserCtrlCmdCallback, 			0 , 0 },
// 	{	USER_CMD_BT_OFF			,	&UserCtrlCmdCallback, 			0 , 0 },
// 	{	USER_CMD_MC_F				,	&UserManualCtrlCallback, 		0 , 0 },
// 	{	USER_CMD_MC_B				,	&UserManualCtrlCallback, 		0 , 0 },
// 	{	USER_CMD_MC_L				,	&UserManualCtrlCallback, 		0 , 0 },
// 	{	USER_CMD_MC_R				,	&UserManualCtrlCallback, 		0 , 0 },
// 	{	USER_CMD_MC_S				,	&UserCtrlCmdCallback, 		  0 , 0 },
// 	{	USER_CMD_SPEED			,	&UserConfigCallback, 		&g_CarConfig.car_speed_set, CONV_U16   },
// 	{	USER_CMD_KP					,	&UserConfigCallback, 		&g_CarConfig.KP, 						CONV_FLOAT },
// 	{	USER_CMD_KI					,	&UserConfigCallback, 		&g_CarConfig.KI, 						CONV_FLOAT },
// 	{	USER_CMD_KD					,	&UserConfigCallback, 		&g_CarConfig.KD, 						CONV_FLOAT },
// 	{	USER_CMD_KLM_EN			, &UserConfigCallback, 	  &g_CarConfig.kalman_enable, CONV_U8    },
// 	{	USER_CMD_KLM_CFD0		, &UserConfigCallback, 	  &g_CarConfig.kalman_confidence[0], CONV_FLOAT    },
// 	{	USER_CMD_KLM_CFD1		, &UserConfigCallback, 	  &g_CarConfig.kalman_confidence[1], CONV_FLOAT    },
// 	{	USER_CMD_BT_NAME		,	&UserConfigCallback, 		&g_CarConfig.bt_name[8], 		CONV_NUM_CHAR},
// 	{	USER_CMD_BT_PWD			,	&UserConfigCallback, 		&g_CarConfig.bt_pwd, 				CONV_NUM_CHAR}, 
// 	{	USER_CMD_ADC_TIME		, &UserConfigCallback, 	  &g_CarConfig.adc_interval			, CONV_U8    },
// 	{	USER_CMD_CTRL_TIME	, &UserConfigCallback, 	  &g_CarConfig.car_ctrl_interval, CONV_U8    },
// 	{	USER_CMD_SPEED_MAX	,	&UserConfigCallback, 		&g_CarConfig.car_speed_max, CONV_U16   },
// 	{	USER_CMD_SPEED_MIN	,	&UserConfigCallback, 		&g_CarConfig.car_speed_min, CONV_U16   },
// 	{	USER_CMD_SHOW_CFG		,	&UserCtrlCmdCallback, 		  0 , 0 },	
// 	{	USER_CMD_FLASH_INIT	,	&UserCtrlCmdCallback, 		  0 , 0 },		
// 	{	USER_CMD_MUSIC_ON	  ,	&UserCtrlCmdCallback, 		  0 , 0 },		
// 	{	USER_CMD_MUSIC_OFF	,	&UserCtrlCmdCallback, 		  0 , 0 },		
//	
// };

// void CarCtrlInit( car_config_t *p_car_cfg )
// {
// 	memset( &g_CarCtrl , 0 , sizeof(g_CarCtrl) );
// 	g_CarCtrl.car_ctrl_freq = p_car_cfg->car_ctrl_interval / p_car_cfg->adc_interval ;
// 	g_CarCtrl.left_speed = p_car_cfg->car_speed_set ;
// 	g_CarCtrl.right_speed = p_car_cfg->car_speed_set ;	
// 	g_CarCtrl.track_start = 0;
// 	g_CarCtrl.car_mode = CAR_FIND_START;


// }

// void CarMotoCtrl(int16_t left_speed, int16_t right_speed)
// {
// 	MotoLeftFrontCtrl(left_speed);
// 	MotoLeftBackCtrl(left_speed);
// 	MotoRightFrontCtrl(right_speed);
// 	MotoRightBackCtrl(right_speed);
// }

// void ManualCarCtrl(void)
// {
// 	if(g_CarCtrl.car_mode == CAR_MANU_CTRL)
// 	{
// 		CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
// 	}
// }

// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
// {
// 	static uint32_t t = 0 ;
// 	static uint32_t straight = 0 ;
// 	static uint32_t no_line = 0 ;
//	
// 	int    last_err_diff ;
//	
// 	ADC_NormalCal();
// 	//倒车代码

// 	if ( (g_TrackStatus.full_white == 1 ) && ( g_CarCtrl.car_mode == CAR_TRACKING ) )
// 	{
// 		if ( no_line > 350 )
// 		{			
// 			StopAllMoto();
// 			CarMotoCtrl(-500,-500);
// 			return ;
// 		}
// 		no_line++;
// 		//bCarMotoCtrl(50,50);	//修改了
// 	}
// 	else
// 	{
// 		no_line = 0 ;
// 	}
//	
// 	last_err_diff = g_TrackStatus.track_error[g_TrackStatus.adc_value] - g_CarCtrl.last_error ;
// 	if( g_CarConfig.kalman_enable )
// 	{
// 		KalmanFilter( g_TrackStatus.track_error[g_TrackStatus.adc_value] , last_err_diff , g_CarConfig.adc_interval );
// 	}
// 	else
// 	{
// 		g_CarCtrl.total_err_diff += last_err_diff ;
// 		g_CarCtrl.total_err  += g_TrackStatus.track_error[g_TrackStatus.adc_value] ;
// 	}
// 	g_CarCtrl.last_error = g_TrackStatus.track_error[g_TrackStatus.adc_value];
//	
//	
// 	t++;
// 	if( ( t == g_CarCtrl.car_ctrl_freq ) && ( g_SystemMode == SYSTEM_TRACK ) )
// 	{
// 		t = 0;
// 		if ( g_CarCtrl.last_error == 0) 
// 		{
// 			if ( straight > 450 )       // accelerate to fastest speed
// 			{
// 				g_CarCtrl.car_speed = g_CarConfig.car_speed_max   ;
// 			}
// 			else if ( straight > 350 )  // accelerate to faster speed
// 			{
// 				g_CarCtrl.car_speed = g_CarConfig.car_speed_max -100  ;
// 				straight++ ;
// 			}
// 			else if ( straight > 250 )  // accelerate to faster speed
// 			{
// 				g_CarCtrl.car_speed = g_CarConfig.car_speed_max -200  ;
// 				straight++ ;
// 			}
// 			else if ( straight > 150 )  // accelerate to faster speed
// 			{
// 				g_CarCtrl.car_speed = g_CarConfig.car_speed_set  ;
// 				straight++ ;
// 			}
// 			else
// 			{
// 				straight++ ;
// 			}
// 		}
// 		else {
//			if( straight > 200)
//			{
//				if((g_CarCtrl.car_speed > 500))
//					g_CarCtrl.car_speed = -1200 ;
//				else if((g_CarCtrl.car_speed > 400))
//					g_CarCtrl.car_speed = -700 ;
//				else
//					g_CarCtrl.car_speed = -300 ;
//				
//			}
//			else if( straight > 100 ) {
//					if((g_CarCtrl.car_speed > 500))
//						g_CarCtrl.car_speed = -800 ;
//				else if((g_CarCtrl.car_speed > 400))
//					g_CarCtrl.car_speed = -400 ;
//				else
//					g_CarCtrl.car_speed = -200 ;  // brake car 
//				
//			}
//			else{
//				g_CarCtrl.car_speed = -200 ;
//			}
//				
//	 //			else if ( straight > 50 ) {
//	 //			g_CarCtrl.car_speed = -50;  // brake car 
// //			}
//		}
// 			else
// 			{
// 				g_CarCtrl.car_speed = g_CarConfig.car_speed_min ;  // curve speed 
// 			}
// 			straight = 0 ;
// 		
//		
// 		g_CarCtrl.track_err_diff = (float)g_CarCtrl.total_err_diff / (float)g_CarConfig.car_ctrl_interval ;
// 		g_CarCtrl.track_err = (float)g_CarCtrl.total_err / (float)g_CarCtrl.car_ctrl_freq ;
// 		CarTrackCtrl();
// 		g_CarCtrl.total_err_diff = 0.0 ;
// 		g_CarCtrl.total_err = 0.0 ;		
//		}
// 	else if(t > g_CarCtrl.car_ctrl_freq )
// 	{
// 		t = 0;
// 	}
// }

// void CarTrackCtrl(void)
// {
// 	switch(g_CarCtrl.car_mode)
// 	{			
// 		case CAR_FIND_START:
// 			if(g_TrackStatus.full_black && g_CarCtrl.track_start == 0)
// 			{
// 				g_CarCtrl.track_start = 1;
// 			}
// 			else
// 			{
// 				if(g_TrackStatus.full_black == 0 && g_CarCtrl.track_start == 1)
// 				{
// 					g_CarCtrl.track_start = 0;
// 					g_CarCtrl.car_mode = CAR_TRACKING;
// 				}
// 			}
// 		break;
//			
// 		case CAR_TRACKING:

// 			if(g_TrackStatus.full_black)
// 			{
// 				HAL_TIM_Base_Stop_IT(&htim6);
// 				StopAllMoto();
// 				CarMotoCtrl( 0 , 0 );
// 				HAL_ADC_Stop_DMA( &hadc1 );
// 				IR_Track_Power_Off();
// 				HAL_TIM_Base_Start_IT(&htim7);
// 				g_CarCtrl.car_mode = CAR_IDLE;
// 			}
// 			else
// 			{
// 				if( g_CarConfig.kalman_enable )
// 				{
// 						CarPIDSpeedCtrl( g_AngleKalman.last_angle , g_AngleKalman.last_diff);
// 				}
// 				else
// 				{
// 						CarPIDSpeedCtrl( g_CarCtrl.track_err, g_CarCtrl.track_err_diff );
// 				}
// 			}
// 		break;
//		
// 		default:
// 			break;
//		
// 	}
// }

// void CarPIDSpeedCtrl(float error, float error_diff)
// {
// 	float pid;	
// 	int  	left_speed;
// 	int 	right_speed;

// 	pid = ( g_CarConfig.KP * error +  g_CarConfig.KD * error_diff ) / 2;
//	
// 	left_speed =  g_CarCtrl.car_speed + pid;	//*g_CarCtrl.car_speed/200;
// 	right_speed = g_CarCtrl.car_speed - pid;	//*g_CarCtrl.car_speed/200;
// 	StopAllMoto();
// 	CarMotoCtrl(left_speed,right_speed);




// }

// /////////////////////////////////////////////////////////////////////////////////////////
// //
// // user contrl command paser and execute
// //
// /////////////////////////////////////////////////////////////////////////////////////////

// car_ctrl_cmd_t *CheckUserCmd(uint8_t *buf)
// {
// 	car_ctrl_cmd_t *ptr = NULL;
// 	int i = 0 ;
// 	for ( i = 0 ; i < sizeof(g_CarCtrlCmd)/sizeof(car_ctrl_cmd_t) ; i++ )
// 	{
// 		if(strncmp((const uint8_t *)g_CarCtrlCmd[i].cmd, (const char *)buf, strlen((const char *)g_CarCtrlCmd[i].cmd)) == 0)
// 		{
// 			return &g_CarCtrlCmd[i];
// 		}
// 	}
// 	return ptr;
// }

// void UserConfigCallback(uint8_t *buf, void *ptr)
// {
// 	uint8_t symbol = 0;
// 	uint8_t len;
// 	int int_temp0 = 0;
// 	float float_result = 0.0;
// 	uint8_t point = 0;
// 	float weight = 1.0;
// 	car_ctrl_cmd_t *cmd_ptr = (car_ctrl_cmd_t *)ptr;
//	
// 	len = strlen((const char *)buf);
// 	switch(cmd_ptr->type)
// 	{
// 		case CONV_INT:
// 		case CONV_U8:
// 		case CONV_U16:
// 		{			
// 			for(int i = 0; i < len; i++)
// 			{
// 				if(buf[i] == '-')
// 				{
// 					if(i != 0)
// 					{
// 						return;
// 					}
// 					symbol = 1;
// 				}
// 				else if( ( buf[i] >= '0' ) && ( buf[i] <= '9' ) )
// 				{
// 					int_temp0 = int_temp0 * 10 + ( buf[i] - '0' );
// 				}
// 				else
// 				{
// 					return;
// 				}
// 			}
// 			switch(cmd_ptr->type)
// 			{
// 				case CONV_INT:
// 					*(int *)cmd_ptr->param = (symbol == 0) ? int_temp0 : (-1 * int_temp0);
// 					break;
//				
// 				case CONV_U8:
// 					if(symbol == 1 || int_temp0 > 255)
// 					{
// 						return;
// 					}
// 					*(uint8_t *)cmd_ptr->param = int_temp0;
// 					break;
//				
// 				case CONV_U16:
// 					if(symbol == 1 || int_temp0 > 65535)
// 					{
// 						return;
// 					}
// 					*(uint16_t *)cmd_ptr->param = int_temp0;
// 					break;
//					
// 				default:
// 					return;
// 					break;
// 			}			
// 		}
// 		break;
//		
// 		case CONV_FLOAT:
// 		{
// 			for(int i = 0; i < len; i++)
// 			{
// 				if(buf[i] == '-')
// 				{
// 					if(i != 0)
// 					{
// 						return;
// 					}
// 					symbol = 1;
// 				}
// 				else if(buf[i] == '.')
// 				{
// 					if(point != 0)
// 					{
// 						return;
// 					}
// 					point = 1;
// 					float_result = (float)int_temp0 ;
// 					int_temp0 = 0 ;
// 					weight  = 1.0 ;					
// 				}
// 				else if( ( buf[i] >= '0' ) && ( buf[i] <= '9' ) )
// 				{
// 						int_temp0 = int_temp0 * 10 + ( buf[i] - '0' );
// 					  weight *= 10 ;
// 				}
// 				else
// 				{
// 						return;
// 				}
// 			}
// 			float_result = (point == 0) ? (float)int_temp0 : float_result + (float)int_temp0 / weight;
// 			*(float *)cmd_ptr->param = (symbol == 0) ? float_result : (-1.0 * float_result);
// 		}
// 		break;
//		
// 		case CONV_NUM_CHAR:
// 			if(len > 5)
// 			{
// 				return;
// 			}
// 			for(int i = 0; i < len; i++)
// 			{
// 				if( ( buf[i] < '0' ) || ( buf[i] > '9' ) )
// 				{
// 					return;
// 				}
// 			}
// 			memcpy((uint8_t *)cmd_ptr->param, buf, len);
// 		break;
//		
// 		default:
// 			return;
// 			break;
//			
//			
// 	}
// 	UserFlashDataWrite( &g_CarConfig);
// 	CarCtrlInit( &g_CarConfig );
// 	AngleKalmanInit(&g_CarConfig);
// }

// void UserManualCtrlCallback(uint8_t *buf, void *ptr)
// {
// 	int int_temp0 = 0;
// 	int manu_speed = 0;
// 	uint8_t len;
// 	car_ctrl_cmd_t *cmd_ptr = (car_ctrl_cmd_t *)ptr;
// 	len = strlen( (const char *)buf );
//	
// 	for(int i = 0; i < len; i++)
// 	{
// 		if(buf[i] == '-')
// 		{
// 			return;
// 		}
// 		else if( ( buf[i] >= '0' ) && ( buf[i] <= '9' ) )
// 		{
// 			int_temp0 = int_temp0 * 10 + ( buf[i] - '0' );
// 		}
// 		else
// 		{
// 			return;
// 		}
// 	}
//	
// 	if(int_temp0 > 100)
// 	{
// 		return;
// 	}
//	
// 	manu_speed = ( (float)int_temp0 / 100 ) * ( g_CarConfig.car_speed_max - g_CarConfig.car_speed_min ) ;

// 	if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_F ) == 0)
// 	{
// 			g_CarCtrl.car_mode = CAR_MANU_CTRL;
// 			g_CarCtrl.left_speed =  manu_speed ;
// 			g_CarCtrl.right_speed = manu_speed ;
// 			StopAllMoto();
// 			CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
// 	}
// 	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_B ) == 0)
// 	{
// 			g_CarCtrl.car_mode = CAR_MANU_CTRL;
// 			g_CarCtrl.left_speed  = -1 * manu_speed ;
// 			g_CarCtrl.right_speed = -1 * manu_speed ;
// 			StopAllMoto();
// 			CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
// 	}
// 	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_L ) == 0)
// 	{
// 			g_CarCtrl.car_mode = CAR_MANU_CTRL;
// 			if(g_CarCtrl.right_speed >= g_CarConfig.car_speed_min )
// 			{
// 				g_CarCtrl.left_speed = g_CarCtrl.right_speed - manu_speed;
// 			}
// 			else if(g_CarCtrl.right_speed <= -1*g_CarConfig.car_speed_min)
// 			{
// 				g_CarCtrl.left_speed = g_CarCtrl.right_speed + manu_speed;
// 			}
// 			else
// 			{
// 				if(g_CarCtrl.right_speed > 0)
// 				{
// 					g_CarCtrl.right_speed = g_CarConfig.car_speed_min;
// 					g_CarCtrl.left_speed = g_CarCtrl.right_speed - manu_speed;
// 				}
// 				else
// 				{
// 					g_CarCtrl.right_speed = -1* g_CarConfig.car_speed_min ;
// 					g_CarCtrl.left_speed = g_CarCtrl.right_speed + manu_speed;
// 				}
// 			}
// 			StopAllMoto();
// 			CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
// 	}
// 	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_R ) == 0)
// 	{
// 			g_CarCtrl.car_mode = CAR_MANU_CTRL;
// 			if(g_CarCtrl.left_speed >= g_CarConfig.car_speed_min)
// 			{
// 				g_CarCtrl.right_speed = g_CarCtrl.left_speed - manu_speed;
// 			}
// 			else if(g_CarCtrl.left_speed <= -1*g_CarConfig.car_speed_min)
// 			{
// 				g_CarCtrl.right_speed = g_CarCtrl.left_speed + manu_speed;
// 			}
// 			else
// 			{
// 				if(g_CarCtrl.left_speed > 0)
// 				{
// 					g_CarCtrl.left_speed = g_CarConfig.car_speed_min;
// 					g_CarCtrl.right_speed = g_CarCtrl.left_speed - manu_speed;
// 				}
// 				else
// 				{
// 					g_CarCtrl.left_speed = -1* g_CarConfig.car_speed_min;
// 					g_CarCtrl.right_speed = g_CarCtrl.left_speed + manu_speed;
// 				}
// 			}
// 			StopAllMoto();
// 			CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
// 	}
// }

// void UserCtrlCmdCallback(uint8_t *buf, void *ptr)
// {
// 	car_ctrl_cmd_t *cmd_ptr = (car_ctrl_cmd_t *)ptr;
// 	if ( strcmp( cmd_ptr->cmd , USER_CMD_START ) == 0)
// 	{
// 			//HAL_TIM_Base_Stop_IT(&htim7);
// 			IR_Track_Init();
// 			CarCtrlInit( &g_CarConfig );
// 			HAL_TIM_Base_Start_IT( &htim6 );
// 			CarMotoCtrl(g_CarConfig.car_speed_set, g_CarConfig.car_speed_set);
// 	}
// 	else if ( strcmp( cmd_ptr->cmd , USER_CMD_BT_ON ) == 0)
// 	{
// 			HAL_UART_Transmit(&huart1, (uint8_t *)BT_ENABLE, sizeof(BT_ENABLE), 10);
// 	}
// 	else if ( strcmp( cmd_ptr->cmd , USER_CMD_BT_OFF ) == 0)
// 	{
// 			HAL_UART_Transmit(&huart1, (uint8_t *)BT_DISABLE, sizeof(BT_DISABLE), 10);
// 	}
// 	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_S ) == 0)
// 	{
// 			g_CarCtrl.car_mode = CAR_MANU_CTRL;
// 			StopAllMoto();
// 	}
// 	else if ( strcmp( cmd_ptr->cmd , USER_CMD_SHOW_CFG ) == 0)
// 	{
// 		printf("BT Name:%s , Password : %s\n" , g_CarConfig.bt_name , g_CarConfig.bt_pwd );
// 		printf("Speed:%d , Kp: %.2f , Kd : %.2f\n" , g_CarConfig.car_speed_set , g_CarConfig.KP , g_CarConfig.KD );
// 		printf("Kalman enable : %d \n" , g_CarConfig.kalman_enable );
// 		printf("ADC interval: %d ms \n" , g_CarConfig.adc_interval );
// 		printf("Car contrl interval: %d ms \n" , g_CarConfig.car_ctrl_interval );
// 		printf("ADC threshhold: %d \n" , g_CarConfig.adc_compare_gate );
// 	}
// 	else if ( strcmp( cmd_ptr->cmd , USER_CMD_FLASH_INIT ) == 0)
// 	{
// 		EreaseFlashData( FLASH_BANK1_END -  FLASH_PAGE_SIZE + 1, 1 );
// 		__set_FAULTMASK(1);
// 		NVIC_SystemReset();
// 	}	
// 	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MUSIC_ON ) == 0)
// 	{
// 		g_music_enable = 1;
// 	}
// 	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MUSIC_OFF ) == 0)
// 	{
// 		g_music_enable = 0;
// 	}
// }

// uint8_t UserCmdParse(uint8_t * buf, uint32_t len)
// {
// 	uint8_t field_num = 0 ;
// 	uint8_t cmd_buf[16];
// 	uint8_t cmd_index = 0;
// 	car_ctrl_cmd_t *cmd_ptr = NULL;
//	
// 	for(int i = 0; i < len; i++)
// 	{
// 		if(buf[i] == '|' || i == len - 1)
// 		{
// 			if(cmd_index == 0 && i < len - 1)
// 			{
// 				return 0;
// 			}
// 			else
// 			{
// 				if(buf[i] == '|')
// 				{
// 					cmd_buf[cmd_index] = 0;
// 				}
// 				else
// 				{
// 					cmd_buf[cmd_index++] = buf[i];
// 					cmd_buf[cmd_index] = 0;
// 				}
// 				if(field_num == 0)
// 				{
// 					cmd_ptr = CheckUserCmd(cmd_buf);
// 					if(cmd_ptr == NULL)
// 					{
// 						return 0;
// 					}
// 					if(cmd_ptr->param == NULL )
// 					{
// 						cmd_ptr->func_callback(cmd_buf, cmd_ptr);
// 						return 1;
// 					}
// 					field_num++;
// 				}
// 				else if(field_num == 1)
// 				{
// 					if(cmd_ptr == NULL)
// 					{
// 						return 0;
// 					}
// 					if(cmd_ptr->func_callback)
// 					{
// 						cmd_ptr->func_callback(cmd_buf, cmd_ptr);
// 						return 1;
// 					}
// 				}
// 				cmd_index = 0;				
// 			}
// 		}
// 		else
// 		{
// 			if(cmd_index < 15)
// 			{
// 				cmd_buf[cmd_index++] = buf[i];
// 			}
// 			else
// 			{
// 				return 0;
// 			}
// 		}
// 	}	
// 	return 0;
// }

// uint32_t UartCtrl_RxDataCallback( uint8_t * buf , uint32_t len )
// {
// 	static uint8_t tag_flag = 0;
// 	static uint8_t index;
// 	for(int i = 0; i < len; i++)
// 	{ 
// 		if( buf[i] == '@' )
// 		{
// 			tag_flag = 1;
// 			index = 0;
// 		}
// 		else if( buf[i] == '&' )
// 		{
// 			if(tag_flag == 1 && index > 0)
// 			{
// 				g_cmd_buf[index] = 0;
// 				UserCmdParse(g_cmd_buf, index);
// 				return 1;
// 			}
// 		}
// 		else
// 		{
// 			if(tag_flag == 1)
// 			{
// 				if(index < 63)
// 				{
// 					g_cmd_buf[index++] = buf[i];
// 				}
// 			}
// 		}
// 	}
// 	return 1;
// }


////#include <inttypes.h>
////#include "car_drive.h"
////#include "ir_track.h"
////#include "motor_drive.h"
////#include "oled_i2c.h"
////#include "usb_uart.h"
////#include "usart.h"
////#include "angle_kalman_filter.h"
////#include "adc.h"
////#include "esp32.h"
////#include "user_flash.h"
////#include "uart_dma.h"

////extern __IO uint8_t  g_music_enable  ;

////car_config_t g_CarConfig =
////{
////	.KP = 18,
////	.KI = 0.0 ,
////	.KD = 5.8 ,
////	.car_speed_set = 300 ,
////	.car_speed_max = 800 ,
////	.car_speed_min = 200 ,	
////	.adc_compare_gate = 2000 ,	
////	.adc_interval     = 2 ,
////	.car_ctrl_interval= 2 ,
////	.kalman_enable    = 1 ,
////	.kalman_confidence = { 0.2 , 0.2 } ,
////	.bt_name = "CAR2023_20325" ,
////	.bt_pwd  = "1234"	
////};

////car_ctrl_t 	g_CarCtrl;



////uint8_t g_cmd_buf[CMD_BUF_LEN] = {0};

////void UserConfigCallback(uint8_t *buf, void *ptr);
////void UserManualCtrlCallback(uint8_t *buf, void *ptr);
////void UserCtrlCmdCallback(uint8_t *buf, void *ptr);

////car_ctrl_cmd_t g_CarCtrlCmd[] = {
////	{	USER_CMD_START			,	&UserCtrlCmdCallback, 			0 , 0 },
////	{	USER_CMD_BT_ON			,	&UserCtrlCmdCallback, 			0 , 0 },
////	{	USER_CMD_BT_OFF			,	&UserCtrlCmdCallback, 			0 , 0 },
////	{	USER_CMD_MC_F				,	&UserManualCtrlCallback, 		0 , 0 },
////	{	USER_CMD_MC_B				,	&UserManualCtrlCallback, 		0 , 0 },
////	{	USER_CMD_MC_L				,	&UserManualCtrlCallback, 		0 , 0 },
////	{	USER_CMD_MC_R				,	&UserManualCtrlCallback, 		0 , 0 },
////	{	USER_CMD_MC_S				,	&UserCtrlCmdCallback, 		  0 , 0 },
////	{	USER_CMD_SPEED			,	&UserConfigCallback, 		&g_CarConfig.car_speed_set, CONV_U16   },
////	{	USER_CMD_KP					,	&UserConfigCallback, 		&g_CarConfig.KP, 						CONV_FLOAT },
////	{	USER_CMD_KI					,	&UserConfigCallback, 		&g_CarConfig.KI, 						CONV_FLOAT },
////	{	USER_CMD_KD					,	&UserConfigCallback, 		&g_CarConfig.KD, 						CONV_FLOAT },
////	{	USER_CMD_KLM_EN			, &UserConfigCallback, 	  &g_CarConfig.kalman_enable, CONV_U8    },
////	{	USER_CMD_KLM_CFD0		, &UserConfigCallback, 	  &g_CarConfig.kalman_confidence[0], CONV_FLOAT    },
////	{	USER_CMD_KLM_CFD1		, &UserConfigCallback, 	  &g_CarConfig.kalman_confidence[1], CONV_FLOAT    },
////	{	USER_CMD_BT_NAME		,	&UserConfigCallback, 		&g_CarConfig.bt_name[8], 		CONV_NUM_CHAR},
////	{	USER_CMD_BT_PWD			,	&UserConfigCallback, 		&g_CarConfig.bt_pwd, 				CONV_NUM_CHAR}, 
////	{	USER_CMD_ADC_TIME		, &UserConfigCallback, 	  &g_CarConfig.adc_interval			, CONV_U8    },
////	{	USER_CMD_CTRL_TIME	, &UserConfigCallback, 	  &g_CarConfig.car_ctrl_interval, CONV_U8    },
////	{	USER_CMD_SPEED_MAX	,	&UserConfigCallback, 		&g_CarConfig.car_speed_max, CONV_U16   },
////	{	USER_CMD_SPEED_MIN	,	&UserConfigCallback, 		&g_CarConfig.car_speed_min, CONV_U16   },
////	{	USER_CMD_SHOW_CFG		,	&UserCtrlCmdCallback, 		  0 , 0 },	
////	{	USER_CMD_FLASH_INIT	,	&UserCtrlCmdCallback, 		  0 , 0 },		
////	{	USER_CMD_MUSIC_ON	  ,	&UserCtrlCmdCallback, 		  0 , 0 },		
////	{	USER_CMD_MUSIC_OFF	,	&UserCtrlCmdCallback, 		  0 , 0 },		
////	
////};

////void CarCtrlInit( car_config_t *p_car_cfg )
////{
////	memset( &g_CarCtrl , 0 , sizeof(g_CarCtrl) );
////	g_CarCtrl.car_ctrl_freq = p_car_cfg->car_ctrl_interval / p_car_cfg->adc_interval ;
////	g_CarCtrl.left_speed = p_car_cfg->car_speed_set ;
////	g_CarCtrl.right_speed = p_car_cfg->car_speed_set ;	
////	g_CarCtrl.track_start = 0;
////	g_CarCtrl.car_mode = CAR_FIND_START;


////}

////void CarMotoCtrl(int16_t left_speed, int16_t right_speed)
////{
////	MotoLeftFrontCtrl(left_speed);
////	MotoLeftBackCtrl(left_speed);
////	MotoRightFrontCtrl(right_speed);
////	MotoRightBackCtrl(right_speed);
////}

////void ManualCarCtrl(void)
////{
////	if(g_CarCtrl.car_mode == CAR_MANU_CTRL)
////	{
////		CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
////	}
////}

////void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
////{
////	static uint32_t t = 0 ;
////	static uint32_t repeat_stra = 0; // 连续中间位置计数，越大说明直道越长
////	static uint32_t repeat_curv = 0; // 连续非中间位置计数，越大说明弯道越长
////	static uint32_t repeat_white = 0; // 连续白线位置计数，越大说明丢线越长
////	static uint32_t repeat_black = 0; // 连续黑线位置计数，越大说明没丢线越长
////	int    last_err_diff ;
////	
////	ADC_NormalCal();
////	last_err_diff = g_TrackStatus.track_error[g_TrackStatus.adc_value] - g_CarCtrl.last_error ;
////	if( g_CarConfig.kalman_enable )
////	{
////		KalmanFilter( g_TrackStatus.track_error[g_TrackStatus.adc_value] , last_err_diff , g_CarConfig.adc_interval );
////	}
////	else
////	{
////		g_CarCtrl.total_err_diff += last_err_diff ;
////		g_CarCtrl.total_err  += g_TrackStatus.track_error[g_TrackStatus.adc_value] ;
////	}
////	g_CarCtrl.last_error = g_TrackStatus.track_error[g_TrackStatus.adc_value];
////	
////	t++;
////	if( ( t == g_CarCtrl.car_ctrl_freq ) && ( g_SystemMode == SYSTEM_TRACK ) )
////	{
////		t = 0;
////		/*直道加速算法*/
////		if( g_CarCtrl.last_error == 0)
////		{
////			if(repeat_stra > 50) // 如果连续10个点都是直道，则判断为真直道，加速或全速
////			{
////				if( repeat_stra > 300 )
////				{
////					g_CarCtrl.car_speed = g_CarConfig.car_speed_max; // accelerate to fastest speed 连续300个直道点加点速
////				}
////				else if( repeat_stra > 100 )
////				{
////					g_CarCtrl.car_speed = g_CarConfig.car_speed_set; // accelerate to faster speed 连续200个直道点加点速
////				}
////				repeat_curv = 0; // 真直道，弯道标志位清零
////			}
////			repeat_stra++; // 直道标志位++
////		} // 直道加速/全速策略
////		else
////		{
////			if(repeat_curv > 3) // 如果连续2个点都是弯道，则判断为真弯道，减速或刹车
////			{
////				if( repeat_stra > 100 )
////				{
////					g_CarCtrl.car_speed = -2000; // brake car 高速进弯要急刹车
////				}
////				else if( repeat_stra > 30 )
////				{
////					g_CarCtrl.car_speed = -1000; // brake car 高速进弯要急刹车
////				}
////				else
////				{
////					g_CarCtrl.car_speed = g_CarConfig.car_speed_min; // curve speed 弯道循迹用低速
////				}
////				repeat_stra = 0; // 真弯道，直道标志位清零
////			}
////			repeat_curv++; // 弯道标志位++
////		} // 弯道减速/刹车策略
////		/*直道加速算法*/
////		/*丢线倒车算法*/
////		if( g_TrackStatus.adc_value == 0 )
////		{
////			if( repeat_white > 200 )
////			{
////				g_CarCtrl.car_speed = -720;
////				repeat_black = 0;
////			}
////			repeat_white ++;
////		}
////		else if( repeat_white != 0 )
////		{
////			if( repeat_black > 8 )
////			{
////				g_CarCtrl.car_speed = 2000;
////				repeat_white = 0;
////			}
////			repeat_black ++;
////		}
////		/*丢线开环倒车算法*/
////		g_CarCtrl.track_err_diff = (float)g_CarCtrl.total_err_diff / (float)g_CarConfig.car_ctrl_interval ;
////		g_CarCtrl.track_err = (float)g_CarCtrl.total_err / (float)g_CarCtrl.car_ctrl_freq ;
////		CarTrackCtrl();
////		g_CarCtrl.total_err_diff = 0.0 ;
////		g_CarCtrl.total_err = 0.0 ;		
////	}
////	else if(t > g_CarCtrl.car_ctrl_freq )
////	{
////		t = 0;
////	}
////}

////void CarTrackCtrl(void)
////{
////	switch(g_CarCtrl.car_mode)
////	{			
////		case CAR_FIND_START:
////			if(g_TrackStatus.full_black && g_CarCtrl.track_start == 0)
////			{
////				g_CarCtrl.track_start = 1;
////			}
////			else
////			{
////				if(g_TrackStatus.full_black == 0 && g_CarCtrl.track_start == 1)
////				{
////					g_CarCtrl.track_start = 0;
////					g_CarCtrl.car_mode = CAR_TRACKING;
////				}
////			}
////		break;
////			
////		case CAR_TRACKING:

////			if(g_TrackStatus.full_black)
////			{
////				HAL_TIM_Base_Stop_IT(&htim6);
////				StopAllMoto();
////				CarMotoCtrl( 0 , 0 );
////				HAL_ADC_Stop_DMA( &hadc1 );
////				IR_Track_Power_Off();
////				HAL_TIM_Base_Start_IT(&htim7);
////				g_CarCtrl.car_mode = CAR_IDLE;
////			}
////			else
////			{
////				if( g_CarConfig.kalman_enable )
////				{
////						CarPIDSpeedCtrl( g_AngleKalman.last_angle , g_AngleKalman.last_diff);
////				}
////				else
////				{
////						CarPIDSpeedCtrl( g_CarCtrl.track_err, g_CarCtrl.track_err_diff );
////				}
////			}
////		break;
////		
////		default:
////			break;
////		
////	}
////}

////void CarPIDSpeedCtrl(float angle, float diff)
////{
////	float pid;	
////	int  	left_speed;
////	int 	right_speed;

////	pid = ( g_CarConfig.KP * angle + g_CarConfig.KD * diff ) / 2;
////	
////	left_speed =  g_CarCtrl.car_speed + pid;
////	right_speed = g_CarCtrl.car_speed - pid;
////	StopAllMoto();
////	CarMotoCtrl(left_speed,right_speed);
////}

/////////////////////////////////////////////////////////////////////////////////////////////
//////
////// user contrl command paser and execute
//////
/////////////////////////////////////////////////////////////////////////////////////////////

////car_ctrl_cmd_t *CheckUserCmd(uint8_t *buf)
////{
////	car_ctrl_cmd_t *ptr = NULL;
////	int i = 0 ;
////	for ( i = 0 ; i < sizeof(g_CarCtrlCmd)/sizeof(car_ctrl_cmd_t) ; i++ )
////	{
////		if(strncmp((const uint8_t *)g_CarCtrlCmd[i].cmd, (const char *)buf, strlen((const char *)buf)) == 0)
////		{
////			return &g_CarCtrlCmd[i];
////		}
////	}
////	return ptr;
////}

////void UserConfigCallback(uint8_t *buf, void *ptr)
////{
////	uint8_t symbol = 0;
////	uint8_t len;
////	int int_temp0 = 0;
////	float float_result = 0.0;
////	uint8_t point = 0;
////	float weight = 1.0;
////	car_ctrl_cmd_t *cmd_ptr = (car_ctrl_cmd_t *)ptr;
////	
////	len = strlen((const char *)buf);
////	switch(cmd_ptr->type)
////	{
////		case CONV_INT:
////		case CONV_U8:
////		case CONV_U16:
////		{			
////			for(int i = 0; i < len; i++)
////			{
////				if(buf[i] == '-')
////				{
////					if(i != 0)
////					{
////						return;
////					}
////					symbol = 1;
////				}
////				else if( ( buf[i] >= '0' ) && ( buf[i] <= '9' ) )
////				{
////					int_temp0 = int_temp0 * 10 + ( buf[i] - '0' );
////				}
////				else
////				{
////					return;
////				}
////			}
////			switch(cmd_ptr->type)
////			{
////				case CONV_INT:
////					*(int *)cmd_ptr->param = (symbol == 0) ? int_temp0 : (-1 * int_temp0);
////					break;
////				
////				case CONV_U8:
////					if(symbol == 1 || int_temp0 > 255)
////					{
////						return;
////					}
////					*(uint8_t *)cmd_ptr->param = int_temp0;
////					break;
////				
////				case CONV_U16:
////					if(symbol == 1 || int_temp0 > 65535)
////					{
////						return;
////					}
////					*(uint16_t *)cmd_ptr->param = int_temp0;
////					break;
////					
////				default:
////					return;
////					break;
////			}			
////		}
////		break;
////		
////		case CONV_FLOAT:
////		{
////			for(int i = 0; i < len; i++)
////			{
////				if(buf[i] == '-')
////				{
////					if(i != 0)
////					{
////						return;
////					}
////					symbol = 1;
////				}
////				else if(buf[i] == '.')
////				{
////					if(point != 0)
////					{
////						return;
////					}
////					point = 1;
////					float_result = (float)int_temp0 ;
////					int_temp0 = 0 ;
////					weight  = 1.0 ;					
////				}
////				else if( ( buf[i] >= '0' ) && ( buf[i] <= '9' ) )
////				{
////						int_temp0 = int_temp0 * 10 + ( buf[i] - '0' );
////					  weight *= 10 ;
////				}
////				else
////				{
////						return;
////				}
////			}
////			float_result = (point == 0) ? (float)int_temp0 : float_result + (float)int_temp0 / weight;
////			*(float *)cmd_ptr->param = (symbol == 0) ? float_result : (-1.0 * float_result);
////		}
////		break;
////		
////		case CONV_NUM_CHAR:
////			if(len > 5)
////			{
////				return;
////			}
////			for(int i = 0; i < len; i++)
////			{
////				if( ( buf[i] < '0' ) || ( buf[i] > '9' ) )
////				{
////					return;
////				}
////			}
////			memcpy((uint8_t *)cmd_ptr->param, buf, len);
////		break;
////		
////		default:
////			return;
////			break;
////			
////			
////	}
////	UserFlashDataWrite( &g_CarConfig);
////	CarCtrlInit( &g_CarConfig );
////	AngleKalmanInit(&g_CarConfig);
////}

////void UserManualCtrlCallback(uint8_t *buf, void *ptr)
////{
////	int int_temp0 = 0;
////	int manu_speed = 0;
////	uint8_t len;
////	car_ctrl_cmd_t *cmd_ptr = (car_ctrl_cmd_t *)ptr;
////	len = strlen( (const char *)buf );
////	
////	for(int i = 0; i < len; i++)
////	{
////		if(buf[i] == '-')
////		{
////			return;
////		}
////		else if( ( buf[i] >= '0' ) && ( buf[i] <= '9' ) )
////		{
////			int_temp0 = int_temp0 * 10 + ( buf[i] - '0' );
////		}
////		else
////		{
////			return;
////		}
////	}
////	
////	if(int_temp0 > 100)
////	{
////		return;
////	}
////	
////	manu_speed = ( (float)int_temp0 / 100 ) * ( g_CarConfig.car_speed_max - g_CarConfig.car_speed_min ) ;

////	if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_F ) == 0)
////	{
////			g_CarCtrl.car_mode = CAR_MANU_CTRL;
////			g_CarCtrl.left_speed =  manu_speed ;
////			g_CarCtrl.right_speed = manu_speed ;
////			StopAllMoto();
////			CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
////	}
////	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_B ) == 0)
////	{
////			g_CarCtrl.car_mode = CAR_MANU_CTRL;
////			g_CarCtrl.left_speed  = -1 * manu_speed ;
////			g_CarCtrl.right_speed = -1 * manu_speed ;
////			StopAllMoto();
////			CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
////	}
////	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_L ) == 0)
////	{
////			g_CarCtrl.car_mode = CAR_MANU_CTRL;
////			if(g_CarCtrl.right_speed >= g_CarConfig.car_speed_min )
////			{
////				g_CarCtrl.left_speed = g_CarCtrl.right_speed - manu_speed;
////			}
////			else if(g_CarCtrl.right_speed <= -1*g_CarConfig.car_speed_min)
////			{
////				g_CarCtrl.left_speed = g_CarCtrl.right_speed + manu_speed;
////			}
////			else
////			{
////				if(g_CarCtrl.right_speed > 0)
////				{
////					g_CarCtrl.right_speed = g_CarConfig.car_speed_min;
////					g_CarCtrl.left_speed = g_CarCtrl.right_speed - manu_speed;
////				}
////				else
////				{
////					g_CarCtrl.right_speed = -1* g_CarConfig.car_speed_min ;
////					g_CarCtrl.left_speed = g_CarCtrl.right_speed + manu_speed;
////				}
////			}
////			StopAllMoto();
////			CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
////	}
////	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_R ) == 0)
////	{
////			g_CarCtrl.car_mode = CAR_MANU_CTRL;
////			if(g_CarCtrl.left_speed >= g_CarConfig.car_speed_min)
////			{
////				g_CarCtrl.right_speed = g_CarCtrl.left_speed - manu_speed;
////			}
////			else if(g_CarCtrl.left_speed <= -1*g_CarConfig.car_speed_min)
////			{
////				g_CarCtrl.right_speed = g_CarCtrl.left_speed + manu_speed;
////			}
////			else
////			{
////				if(g_CarCtrl.left_speed > 0)
////				{
////					g_CarCtrl.left_speed = g_CarConfig.car_speed_min;
////					g_CarCtrl.right_speed = g_CarCtrl.left_speed - manu_speed;
////				}
////				else
////				{
////					g_CarCtrl.left_speed = -1* g_CarConfig.car_speed_min;
////					g_CarCtrl.right_speed = g_CarCtrl.left_speed + manu_speed;
////				}
////			}
////			StopAllMoto();
////			CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
////	}
////}

////void UserCtrlCmdCallback(uint8_t *buf, void *ptr)
////{
////	car_ctrl_cmd_t *cmd_ptr = (car_ctrl_cmd_t *)ptr;
////	if ( strcmp( cmd_ptr->cmd , USER_CMD_START ) == 0)
////	{
////			//HAL_TIM_Base_Stop_IT(&htim7);
////			IR_Track_Init();
////			CarCtrlInit( &g_CarConfig );
////			HAL_TIM_Base_Start_IT( &htim6 );
////			CarMotoCtrl(g_CarConfig.car_speed_set, g_CarConfig.car_speed_set);
////	}
////	else if ( strcmp( cmd_ptr->cmd , USER_CMD_BT_ON ) == 0)
////	{
////			HAL_UART_Transmit(&huart1, (uint8_t *)BT_ENABLE, sizeof(BT_ENABLE), 10);
////	}
////	else if ( strcmp( cmd_ptr->cmd , USER_CMD_BT_OFF ) == 0)
////	{
////			HAL_UART_Transmit(&huart1, (uint8_t *)BT_DISABLE, sizeof(BT_DISABLE), 10);
////	}
////	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_S ) == 0)
////	{
////			g_CarCtrl.car_mode = CAR_MANU_CTRL;
////			StopAllMoto();
////	}
////	else if ( strcmp( cmd_ptr->cmd , USER_CMD_SHOW_CFG ) == 0)
////	{
////		printf("BT Name:%s , Password : %s\n" , g_CarConfig.bt_name , g_CarConfig.bt_pwd );
////		printf("Speed:%d , Kp: %.2f , Kd : %.2f\n" , g_CarConfig.car_speed_set , g_CarConfig.KP , g_CarConfig.KD );
////		printf("Kalman enable : %d \n" , g_CarConfig.kalman_enable );
////		printf("ADC interval: %d ms \n" , g_CarConfig.adc_interval );
////		printf("Car contrl interval: %d ms \n" , g_CarConfig.car_ctrl_interval );
////		printf("ADC threshhold: %d \n" , g_CarConfig.adc_compare_gate );
////	}
////	else if ( strcmp( cmd_ptr->cmd , USER_CMD_FLASH_INIT ) == 0)
////	{
////		EreaseFlashData( FLASH_BANK1_END -  FLASH_PAGE_SIZE + 1, 1 );
////		__set_FAULTMASK(1);
////		NVIC_SystemReset();
////	}	
////	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MUSIC_ON ) == 0)
////	{
////		g_music_enable = 1;
////	}
////	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MUSIC_OFF ) == 0)
////	{
////		g_music_enable = 0;
////	}
////}

////uint8_t UserCmdParse(uint8_t * buf, uint32_t len)
////{
////	uint8_t field_num = 0 ;
////	uint8_t cmd_buf[16];
////	uint8_t cmd_index = 0;
////	car_ctrl_cmd_t *cmd_ptr = NULL;
////	
////	for(int i = 0; i < len; i++)
////	{
////		if(buf[i] == '|' || i == len - 1)
////		{
////			if(cmd_index == 0 && i < len - 1)
////			{
////				return 0;
////			}
////			else
////			{
////				if(buf[i] == '|')
////				{
////					cmd_buf[cmd_index] = 0;
////				}
////				else
////				{
////					cmd_buf[cmd_index++] = buf[i];
////					cmd_buf[cmd_index] = 0;
////				}
////				if(field_num == 0)
////				{
////					cmd_ptr = CheckUserCmd(cmd_buf);
////					if(cmd_ptr == NULL)
////					{
////						return 0;
////					}
////					if(cmd_ptr->param == NULL )
////					{
////						cmd_ptr->func_callback(cmd_buf, cmd_ptr);
////						return 1;
////					}
////					field_num++;
////				}
////				else if(field_num == 1)
////				{
////					if(cmd_ptr == NULL)
////					{
////						return 0;
////					}
////					if(cmd_ptr->func_callback)
////					{
////						cmd_ptr->func_callback(cmd_buf, cmd_ptr);
////						return 1;
////					}
////				}
////				cmd_index = 0;				
////			}
////		}
////		else
////		{
////			if(cmd_index < 15)
////			{
////				cmd_buf[cmd_index++] = buf[i];
////			}
////			else
////			{
////				return 0;
////			}
////		}
////	}	
////	return 0;
////}

////uint32_t UartCtrl_RxDataCallback( uint8_t * buf , uint32_t len )
////{
////	static uint8_t tag_flag = 0;
////	static uint8_t index;
////	for(int i = 0; i < len; i++)
////	{
////		if( buf[i] == '@' )
////		{
////			tag_flag = 1;
////			index = 0;
////		}
////		else if( buf[i] == '&' )
////		{
////			if(tag_flag == 1 && index > 0)
////			{
////				g_cmd_buf[index] = 0;
////				UserCmdParse(g_cmd_buf, index);
////				return 1;
////			}
////		}
////		else
////		{
////			if(tag_flag == 1)
////			{
////				if(index < 63)
////				{
////					g_cmd_buf[index++] = buf[i];
////				}
////			}
////		}
////	}
////	return 1;
////}

#include <inttypes.h>
#include "car_drive.h"
#include "ir_track.h"
#include "motor_drive.h"
#include "oled_i2c.h"
#include "usb_uart.h"
#include "usart.h"
#include "angle_kalman_filter.h"
#include "adc.h"
#include "esp32.h"
#include "user_flash.h"
#include "uart_dma.h"

#include "math.h"

int nb=0;
int flagb=0;
int blackflag=0;
int blacktime=0;
int jizhuan = 0;


extern __IO uint8_t  g_music_enable  ;

car_config_t g_CarConfig =
{
	.KP = 30,
	.KI = 0.0 ,
	.KD = 8,	//6.98,
	.car_speed_set = 300 ,		//YUAN  300
	.car_speed_max = 600 ,
	.car_speed_min = 300 ,	
	.adc_compare_gate = 2000 ,	
	.adc_interval     = 2 ,
	.car_ctrl_interval= 2 ,
	.kalman_enable    = 1 ,
	.kalman_confidence = { 0.2 , 0.2 } ,
	.bt_name = "CAR2023_77777" ,
	.bt_pwd  = "8888"	
};

car_ctrl_t 	g_CarCtrl;



uint8_t g_cmd_buf[CMD_BUF_LEN] = {0};

void UserConfigCallback(uint8_t *buf, void *ptr);
void UserManualCtrlCallback(uint8_t *buf, void *ptr);
void UserCtrlCmdCallback(uint8_t *buf, void *ptr);

car_ctrl_cmd_t g_CarCtrlCmd[] = {
	{	USER_CMD_START			,	&UserCtrlCmdCallback, 			0 , 0 },
	{	USER_CMD_BT_ON			,	&UserCtrlCmdCallback, 			0 , 0 },
	{	USER_CMD_BT_OFF			,	&UserCtrlCmdCallback, 			0 , 0 },
	{	USER_CMD_MC_F				,	&UserManualCtrlCallback, 		0 , 0 },
	{	USER_CMD_MC_B				,	&UserManualCtrlCallback, 		0 , 0 },
	{	USER_CMD_MC_L				,	&UserManualCtrlCallback, 		0 , 0 },
	{	USER_CMD_MC_R				,	&UserManualCtrlCallback, 		0 , 0 },
	{	USER_CMD_MC_S				,	&UserCtrlCmdCallback, 		  0 , 0 },
	{	USER_CMD_SPEED			,	&UserConfigCallback, 		&g_CarConfig.car_speed_set, CONV_U16   },
	{	USER_CMD_KP					,	&UserConfigCallback, 		&g_CarConfig.KP, 						CONV_FLOAT },
	{	USER_CMD_KI					,	&UserConfigCallback, 		&g_CarConfig.KI, 						CONV_FLOAT },
	{	USER_CMD_KD					,	&UserConfigCallback, 		&g_CarConfig.KD, 						CONV_FLOAT },
	{	USER_CMD_KLM_EN			, &UserConfigCallback, 	  &g_CarConfig.kalman_enable, CONV_U8    },
	{	USER_CMD_KLM_CFD0		, &UserConfigCallback, 	  &g_CarConfig.kalman_confidence[0], CONV_FLOAT    },
	{	USER_CMD_KLM_CFD1		, &UserConfigCallback, 	  &g_CarConfig.kalman_confidence[1], CONV_FLOAT    },
	{	USER_CMD_BT_NAME		,	&UserConfigCallback, 		&g_CarConfig.bt_name[8], 		CONV_NUM_CHAR},
	{	USER_CMD_BT_PWD			,	&UserConfigCallback, 		&g_CarConfig.bt_pwd, 				CONV_NUM_CHAR}, 
	{	USER_CMD_ADC_TIME		, &UserConfigCallback, 	  &g_CarConfig.adc_interval			, CONV_U8    },
	{	USER_CMD_CTRL_TIME	, &UserConfigCallback, 	  &g_CarConfig.car_ctrl_interval, CONV_U8    },
	{	USER_CMD_SPEED_MAX	,	&UserConfigCallback, 		&g_CarConfig.car_speed_max, CONV_U16   },
	{	USER_CMD_SPEED_MIN	,	&UserConfigCallback, 		&g_CarConfig.car_speed_min, CONV_U16   },
	{	USER_CMD_SHOW_CFG		,	&UserCtrlCmdCallback, 		  0 , 0 },	
	{	USER_CMD_FLASH_INIT	,	&UserCtrlCmdCallback, 		  0 , 0 },		
	{	USER_CMD_MUSIC_ON	  ,	&UserCtrlCmdCallback, 		  0 , 0 },		
	{	USER_CMD_MUSIC_OFF	,	&UserCtrlCmdCallback, 		  0 , 0 },		
	
	
};

void CarCtrlInit( car_config_t *p_car_cfg )
{
	memset( &g_CarCtrl , 0 , sizeof(g_CarCtrl) );
	g_CarCtrl.car_ctrl_freq = p_car_cfg->car_ctrl_interval / p_car_cfg->adc_interval ;
	g_CarCtrl.left_speed = p_car_cfg->car_speed_set ;
	g_CarCtrl.right_speed = p_car_cfg->car_speed_set ;	
	
  
	
	
	g_CarCtrl.track_start = 0;
	g_CarCtrl.car_mode = CAR_FIND_START;


}

void CarMotoCtrl(int16_t left_speed, int16_t right_speed)
{
	MotoLeftFrontCtrl(left_speed);
	MotoLeftBackCtrl(left_speed);
	MotoRightFrontCtrl(right_speed);
	MotoRightBackCtrl(right_speed);
}

void ManualCarCtrl(void)           //  遥控车没吊用
{
	if(g_CarCtrl.car_mode == CAR_MANU_CTRL)
	{
		CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	static uint32_t t = 0 ;
	
	//////////////////////////////////////
	static uint32_t repeat = 0 ;	
	//////////////////////////////////////
	
	int    last_err_diff ;
	
	ADC_NormalCal();
	
/////////////////////////////////////////////////////////////////////////////////	
	/*
	
	if(fabs(g_TrackStatus.track_error[g_TrackStatus.adc_value])>30)
	{
	  g_CarCtrl.shishi_speed=g_CarConfig.car_speed_set-600;		
	}
	else if(fabs(g_TrackStatus.track_error[g_TrackStatus.adc_value])>20)
	{
	  g_CarCtrl.shishi_speed=g_CarConfig.car_speed_set-600;		
	}	
	else if(fabs(g_TrackStatus.track_error[g_TrackStatus.adc_value])>10)
	{
	  g_CarCtrl.shishi_speed=g_CarConfig.car_speed_set-600;		
	}	
	else g_CarCtrl.shishi_speed=g_CarConfig.car_speed_set;
	
	*/
	
////////////////////////////////////////////////////////////////////////////////	
	
	last_err_diff = g_TrackStatus.track_error[g_TrackStatus.adc_value] - g_CarCtrl.last_error ;
	if( g_CarConfig.kalman_enable )
	{
		KalmanFilter( g_TrackStatus.track_error[g_TrackStatus.adc_value] , last_err_diff , g_CarConfig.adc_interval );
	}
	else
	{
		g_CarCtrl.total_err_diff += last_err_diff ;
		g_CarCtrl.total_err  += g_TrackStatus.track_error[g_TrackStatus.adc_value] ;
	}
	g_CarCtrl.last_error = g_TrackStatus.track_error[g_TrackStatus.adc_value];
	
	t++;
	if( ( t == g_CarCtrl.car_ctrl_freq ) && ( g_SystemMode == SYSTEM_TRACK ) )
	{
		t = 0;
		
///////////////////////////////////////////////////////		
		
		if(g_TrackStatus.adc_value==0)            //  全白
		{			  
			  if(nb==30)                                          // num_bai  全白倒车
				{
					g_CarCtrl.shishi_speed=-500;nb++;
				
					 	//CarMotoCtrl(-350,-650);
				
					
				
				}
        else if(nb==666)	{g_CarCtrl.shishi_speed=400;}		  // 倒车过量往前开补救	
				else nb++;
				
		} 
		else 
		{	
			nb=0;
			if(g_CarCtrl.last_error==0)         //  直线  分段速度
			{
								  
					if(repeat>220)
					{
							g_CarCtrl.shishi_speed=400;	
					}
					else if(repeat>200)
					{
							g_CarCtrl.shishi_speed=570;			
						
					}
					else if(repeat>120)
					{
							g_CarCtrl.shishi_speed=550;	
							repeat++;
					}
					else if(repeat>50)
					{
							g_CarCtrl.shishi_speed=520;	
							repeat++;
					}				
					else
					{
							repeat++;
					}
          			
/*					
					float frepeat=repeat;
					
					if(repeat>50)
					{
					     g_CarCtrl.shishi_speed=-0.02917*frepeat*frepeat+5.75*frepeat+416.67;   
					     //拟合直线情况变速 repeat为记录的直线长度
					}
*/
          

			}
			else                      //  弯到    急刹缓刹车
				{
				
					if(repeat>100)
					{
							g_CarCtrl.shishi_speed=-1900;				
							jizhuan = 1;
 					}									
					else if(repeat>40)
					{
							g_CarCtrl.shishi_speed=-1900;	
							jizhuan = 1;
					}	
					else if(repeat>30)
					{
							g_CarCtrl.shishi_speed=-1800;	
							jizhuan = 1;
					}				
					else if(repeat>20)
					{
							g_CarCtrl.shishi_speed=-1700;	
							jizhuan = 1;
					}			
					 
					else
					{
							g_CarCtrl.shishi_speed=300;	//zuixiao zhanwan
							jizhuan = 1;
					}
					repeat=0;
			
			  }
		}
		
///////////////////////////////////////////////////////		
				
		g_CarCtrl.track_err_diff = (float)g_CarCtrl.total_err_diff / (float)g_CarConfig.car_ctrl_interval ;
		g_CarCtrl.track_err = (float)g_CarCtrl.total_err / (float)g_CarCtrl.car_ctrl_freq ;
		CarTrackCtrl();
		g_CarCtrl.total_err_diff = 0.0 ;
		g_CarCtrl.total_err = 0.0 ;		
	}
	else if(t > g_CarCtrl.car_ctrl_freq )
	{
		t = 0;
	}
}

void CarTrackCtrl(void)
{
	switch(g_CarCtrl.car_mode)
	{			
		case CAR_FIND_START:
			if(g_CarCtrl.track_start == 0)     // 第一次全黑开始寻迹
			{
				g_CarCtrl.track_start = 1;
				blacktime=10;                                     // =2*八字数+1
			}
			else
			{
				if(g_TrackStatus.full_black == 0 && g_CarCtrl.track_start == 1)
				{
					g_CarCtrl.track_start = 0;
					g_CarCtrl.car_mode = CAR_TRACKING;
				}
			}
		break;
			
		case CAR_TRACKING:
			if(g_TrackStatus.full_black==0&&blackflag==1)
			{
				blackflag=0;
			}
      
			else if(g_TrackStatus.full_black&&blacktime!=0)      // 设置blackflag确保只记录一次blacktime
			{
				if(blackflag==0)
				{
					blackflag=1;
					blacktime--;
				}
			}
			else if(g_TrackStatus.full_black && blacktime==0)    //  全黑并且八字弯记完停车
			{
				HAL_TIM_Base_Stop_IT(&htim6);
				StopAllMoto();
				CarMotoCtrl( 0 , 0 );
				HAL_ADC_Stop_DMA( &hadc1 );
				IR_Track_Power_Off();
				HAL_TIM_Base_Start_IT(&htim7);
				g_CarCtrl.car_mode = CAR_IDLE;
			}
			
/*			
			if(g_TrackStatus.full_black==1)
			{
				 if(blacktime==3)
			   {
						HAL_TIM_Base_Stop_IT(&htim6);
						StopAllMoto();
						CarMotoCtrl( 0 , 0 );
						HAL_ADC_Stop_DMA( &hadc1 );
						IR_Track_Power_Off();
						HAL_TIM_Base_Start_IT(&htim7);
						g_CarCtrl.car_mode = CAR_IDLE;			       
			   }
			   else 
			   {
			      blacktime++;
			      g_CarCtrl.car_mode = CAR_FIND_START;
		     }
			}
			
			
*/			
			
			
			else                                             // 正常迅即
			{
				if( g_CarConfig.kalman_enable )
				{
						CarPIDSpeedCtrl( g_AngleKalman.last_angle , g_AngleKalman.last_diff);
				}
				else
				{
						CarPIDSpeedCtrl( g_CarCtrl.track_err, g_CarCtrl.track_err_diff );
				}
			}
		break;
		
		default:
			break;
		
	}
}

void CarPIDSpeedCtrl(float angle, float diff)
{
	float pid;	
	int  	left_speed;
	int 	right_speed;

	
	
	
	
	pid = ( g_CarConfig.KP * angle + g_CarConfig.KD * diff ) / 2;
//	g_CarCtrl.shishi_speed=600;
	if(jizhuan == 1)
	{
		left_speed =  g_CarCtrl.shishi_speed + pid*1.8;
		right_speed = g_CarCtrl.shishi_speed - pid*1.8;
		
		jizhuan = 0;
	}
	else{
		
	left_speed =  g_CarCtrl.shishi_speed + pid;
	right_speed = g_CarCtrl.shishi_speed - pid;
	}
//	if(nb==666)  {right_speed+=100;}
	
	StopAllMoto();
	CarMotoCtrl(left_speed,right_speed);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// user contrl command paser and execute
//
/////////////////////////////////////////////////////////////////////////////////////////

car_ctrl_cmd_t *CheckUserCmd(uint8_t *buf)
{
	car_ctrl_cmd_t *ptr = NULL;
	int i = 0 ;
	for ( i = 0 ; i < sizeof(g_CarCtrlCmd)/sizeof(car_ctrl_cmd_t) ; i++ )
	{
		if(strncmp((const uint8_t *)g_CarCtrlCmd[i].cmd, (const char *)buf, strlen((const char *)buf)) == 0)
		{
			return &g_CarCtrlCmd[i];
		}
	}
	return ptr;
}

void UserConfigCallback(uint8_t *buf, void *ptr)
{
	uint8_t symbol = 0;
	uint8_t len;
	int int_temp0 = 0;
	
	float float_result = 0.0;
	uint8_t point = 0;
	float weight = 1.0;
	car_ctrl_cmd_t *cmd_ptr = (car_ctrl_cmd_t *)ptr;
	
	len = strlen((const char *)buf);
	switch(cmd_ptr->type)
	{
		case CONV_INT:
		case CONV_U8:
		case CONV_U16:
		{			
			for(int i = 0; i < len; i++)
			{
				if(buf[i] == '-')
				{
					if(i != 0)
					{
						return;
					}
					symbol = 1;
				}
				else if( ( buf[i] >= '0' ) && ( buf[i] <= '9' ) )
				{
					int_temp0 = int_temp0 * 10 + ( buf[i] - '0' );
				}
				else
				{
					return;
				}
			}
			switch(cmd_ptr->type)
			{
				case CONV_INT:
					*(int *)cmd_ptr->param = (symbol == 0) ? int_temp0 : (-1 * int_temp0);
					break;
				
				case CONV_U8:
					if(symbol == 1 || int_temp0 > 255)
					{
						return;
					}
					*(uint8_t *)cmd_ptr->param = int_temp0;
					break;
				
				case CONV_U16:
					if(symbol == 1 || int_temp0 > 65535)
					{
						return;
					}
					*(uint16_t *)cmd_ptr->param = int_temp0;
					break;
					
				default:
					return;
					break;
			}			
		}
		break;
		
		case CONV_FLOAT:
		{
			for(int i = 0; i < len; i++)
			{
				if(buf[i] == '-')
				{
					if(i != 0)
					{
						return;
					}
					symbol = 1;
				}
				else if(buf[i] == '.')
				{
					if(point != 0)
					{
						return;
					}
					point = 1;
					float_result = (float)int_temp0 ;
					int_temp0 = 0 ;
					weight  = 1.0 ;					
				}
				else if( ( buf[i] >= '0' ) && ( buf[i] <= '9' ) )
				{
						int_temp0 = int_temp0 * 10 + ( buf[i] - '0' );
					  weight *= 10 ;
				}
				else
				{
						return;
				}
			}
			float_result = (point == 0) ? (float)int_temp0 : float_result + (float)int_temp0 / weight;
			*(float *)cmd_ptr->param = (symbol == 0) ? float_result : (-1.0 * float_result);
		}
		break;
		
		case CONV_NUM_CHAR:
			if(len > 5)
			{
				return;
			}
			for(int i = 0; i < len; i++)
			{
				if( ( buf[i] < '0' ) || ( buf[i] > '9' ) )
				{
					return;
				}
			}
			memcpy((uint8_t *)cmd_ptr->param, buf, len);
		break;
		
		default:
			return;
			break;
			
			
	}
	UserFlashDataWrite( &g_CarConfig);
	
	CarCtrlInit( &g_CarConfig );
	AngleKalmanInit(&g_CarConfig);
}

void UserManualCtrlCallback(uint8_t *buf, void *ptr)
{
	int int_temp0 = 0;
	int manu_speed = 0;
	uint8_t len;
	car_ctrl_cmd_t *cmd_ptr = (car_ctrl_cmd_t *)ptr;
	len = strlen( (const char *)buf );
	
	for(int i = 0; i < len; i++)
	{
		if(buf[i] == '-')
		{
			return;
		}
		else if( ( buf[i] >= '0' ) && ( buf[i] <= '9' ) )
		{
			int_temp0 = int_temp0 * 10 + ( buf[i] - '0' );
		}
		else
		{
			return;
		}
	}
	
	if(int_temp0 > 100)
	{
		return;
	}
	
	manu_speed = ( (float)int_temp0 / 100 ) * ( g_CarConfig.car_speed_max - g_CarConfig.car_speed_min ) ;

	if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_F ) == 0)
	{
			g_CarCtrl.car_mode = CAR_MANU_CTRL;
			g_CarCtrl.left_speed =  manu_speed ;
			g_CarCtrl.right_speed = manu_speed ;
			StopAllMoto();

		CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
	}
	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_B ) == 0)
	{
			g_CarCtrl.car_mode = CAR_MANU_CTRL;
			g_CarCtrl.left_speed  = -1 * manu_speed ;
			g_CarCtrl.right_speed = -1 * manu_speed ;
			StopAllMoto();
			CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
	}
	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_L ) == 0)
	{
			g_CarCtrl.car_mode = CAR_MANU_CTRL;
			if(g_CarCtrl.right_speed >= g_CarConfig.car_speed_min )
			{
				g_CarCtrl.left_speed = g_CarCtrl.right_speed - manu_speed;
			}
			else if(g_CarCtrl.right_speed <= -1*g_CarConfig.car_speed_min)
			{
				g_CarCtrl.left_speed = g_CarCtrl.right_speed + manu_speed;
			}
			else
			{
				if(g_CarCtrl.right_speed > 0)
				{
					g_CarCtrl.right_speed = g_CarConfig.car_speed_min;
					g_CarCtrl.left_speed = g_CarCtrl.right_speed - manu_speed;
				}
				else
				{
					g_CarCtrl.right_speed = -1* g_CarConfig.car_speed_min ;
					g_CarCtrl.left_speed = g_CarCtrl.right_speed + manu_speed;
				}
			}
			StopAllMoto();
			CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
	}
	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_R ) == 0)
	{
			g_CarCtrl.car_mode = CAR_MANU_CTRL;
			if(g_CarCtrl.left_speed >= g_CarConfig.car_speed_min)
			{
				g_CarCtrl.right_speed = g_CarCtrl.left_speed - manu_speed;
			}
			else if(g_CarCtrl.left_speed <= -1*g_CarConfig.car_speed_min)
			{
				g_CarCtrl.right_speed = g_CarCtrl.left_speed + manu_speed;
			}
			else
			{
				if(g_CarCtrl.left_speed > 0)
				{
					g_CarCtrl.left_speed = g_CarConfig.car_speed_min;
					g_CarCtrl.right_speed = g_CarCtrl.left_speed - manu_speed;
				}
				else
				{
					g_CarCtrl.left_speed = -1* g_CarConfig.car_speed_min;
					g_CarCtrl.right_speed = g_CarCtrl.left_speed + manu_speed;
				}
			}
			StopAllMoto();
			CarMotoCtrl(g_CarCtrl.left_speed, g_CarCtrl.right_speed);
	}
}

void UserCtrlCmdCallback(uint8_t *buf, void *ptr)
{
	car_ctrl_cmd_t *cmd_ptr = (car_ctrl_cmd_t *)ptr;
	if ( strcmp( cmd_ptr->cmd , USER_CMD_START ) == 0)
	{
			//HAL_TIM_Base_Stop_IT(&htim7);
			IR_Track_Init();
			CarCtrlInit( &g_CarConfig );
			HAL_TIM_Base_Start_IT( &htim6 );
			CarMotoCtrl(g_CarConfig.car_speed_set, g_CarConfig.car_speed_set);
	}
	else if ( strcmp( cmd_ptr->cmd , USER_CMD_BT_ON ) == 0)
	{
			HAL_UART_Transmit(&huart1, (uint8_t *)BT_ENABLE, sizeof(BT_ENABLE), 10);
	}
	else if ( strcmp( cmd_ptr->cmd , USER_CMD_BT_OFF ) == 0)
	{
			HAL_UART_Transmit(&huart1, (uint8_t *)BT_DISABLE, sizeof(BT_DISABLE), 10);
	}
	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MC_S ) == 0)
	{
			g_CarCtrl.car_mode = CAR_MANU_CTRL;
			StopAllMoto();
	}
	else if ( strcmp( cmd_ptr->cmd , USER_CMD_SHOW_CFG ) == 0)
	{
		printf("BT Name:%s , Password : %s\n" , g_CarConfig.bt_name , g_CarConfig.bt_pwd );
		printf("Speed:%d , Kp: %.2f , Kd : %.2f\n" , g_CarConfig.car_speed_set , g_CarConfig.KP , g_CarConfig.KD );
		printf("Kalman enable : %d \n" , g_CarConfig.kalman_enable );
		printf("ADC interval: %d ms \n" , g_CarConfig.adc_interval );
		printf("Car contrl interval: %d ms \n" , g_CarConfig.car_ctrl_interval );
		printf("ADC threshhold: %d \n" , g_CarConfig.adc_compare_gate );
	}
	else if ( strcmp( cmd_ptr->cmd , USER_CMD_FLASH_INIT ) == 0)
	{
		EreaseFlashData( FLASH_BANK1_END -  FLASH_PAGE_SIZE + 1, 1 );
		__set_FAULTMASK(1);
		NVIC_SystemReset();
	}	
	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MUSIC_ON ) == 0)
	{
		g_music_enable = 1;
	}
	else if ( strcmp( cmd_ptr->cmd , USER_CMD_MUSIC_OFF ) == 0)
	{
		g_music_enable = 0;
	}
}

uint8_t UserCmdParse(uint8_t * buf, uint32_t len)
{
	uint8_t field_num = 0 ;
	uint8_t cmd_buf[16];
	uint8_t cmd_index = 0;
	car_ctrl_cmd_t *cmd_ptr = NULL;
	
	for(int i = 0; i < len; i++)
	{
		if(buf[i] == '|' || i == len - 1)
		{
			if(cmd_index == 0 && i < len - 1)
			{
				return 0;
			}
			else
			{
				if(buf[i] == '|')
				{
					cmd_buf[cmd_index] = 0;
				}
				else
				{
					cmd_buf[cmd_index++] = buf[i];
					cmd_buf[cmd_index] = 0;
				}
				if(field_num == 0)
				{
					cmd_ptr = CheckUserCmd(cmd_buf);
					if(cmd_ptr == NULL)
					{
						return 0;
					}
					if(cmd_ptr->param == NULL )
					{
						cmd_ptr->func_callback(cmd_buf, cmd_ptr);
						return 1;
					}
					field_num++;
				}
				else if(field_num == 1)
				{
					if(cmd_ptr == NULL)
					{
						return 0;
					}
					if(cmd_ptr->func_callback)
					{
						cmd_ptr->func_callback(cmd_buf, cmd_ptr);
						return 1;
					}
				}
				cmd_index = 0;				
			}
		}
		else
		{
			if(cmd_index < 15)
			{
				cmd_buf[cmd_index++] = buf[i];
			}
			else
			{
				return 0;
			}
		}
	}	
	return 0;
}

uint32_t UartCtrl_RxDataCallback( uint8_t * buf , uint32_t len )
{
	static uint8_t tag_flag = 0;
	static uint8_t index;
	for(int i = 0; i < len; i++)
	{
		if( buf[i] == '@' )
		{
			tag_flag = 1;
			index = 0;
		}
		else if( buf[i] == '&' )
		{
			if(tag_flag == 1 && index > 0)
			{
				g_cmd_buf[index] = 0;
				UserCmdParse(g_cmd_buf, index);
				return 1;
			}
		}
		else
		{
			if(tag_flag == 1)
			{
				if(index < 63)
				{
					g_cmd_buf[index++] = buf[i];
				}
			}
		}
	}
	return 1;
}





