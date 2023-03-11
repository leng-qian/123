#include "bsp.h"
#include "modbus_slave.h"
#include "modbus_reg_addr.h"
#include "SEGGER_RTT.h"
#include "24Cxx.h"



const uint8_t Num_code[2][12] = {{Num0(a_),Num1(a_),Num2(a_),Num3(a_),Num4(a_),Num5(a_),Num6(a_),Num7(a_),Num8(a_),Num9(a_),NumE(a_),Numblk},\
    {Num0(b_),Num1(b_),Num2(b_),Num3(b_),Num4(b_),Num5(b_),Num6(b_),Num7(b_),Num8(b_),Num9(b_),NumE(b_),Numblk}
};
const uint8_t LCD_SEG_Num[32]=  { 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
                                  0, 0, 0, 0, 1, 1, 1, 1, 0, 0,
                                  0, 0, 0, 1, 1, 1, 1, 0, 0, 0,
                                  0, 0
                                };
uint8_t LCD_Buf[33]= {0};



extern _LCD_icon LCD_icon;
extern sensor_Data_t sensor_Data;
uint8_t ucKeyCode;		/* 按键代码 */
_KEY_State KEY_State=_usKEY_State_NONE;
_KEY_SET_State KEY_SET_State=_usKEY_Set_upper;
_Relay_State Relay_State=_usRelay_OFF;




void KEY_Work (void)
{
    /* 按键滤波和检测由后台systick中断服务程序实现，我们只需要调用bsp_GetKey读取键值即可。 */
    ucKeyCode = bsp_GetKey();	/* 读取键值, 无键按下时返回 KEY_NONE = 0 */
    if (ucKeyCode != KEY_NONE)
    {
        switch (ucKeyCode)
        {
        case KEY_UP_SET:
            SEGGER_RTT_printf(0,"KEY_UP_SET键按下 KEY_State= %d\r\n",KEY_State);		  /* 设置键 */
            BEEP_KeyTone();
            if (KEY_State==_usKEY_State_Set)
            {
                KEY_SET_State++;
                if (KEY_SET_State>_usKEY_Set_Electric)
                {
                    KEY_SET_State=1;
                }
            }
            else if (KEY_State==_usKEY_State_SuperSet)
            {
				if (KEY_SET_State==_usKEY_Set_State_SuperSet)
					{
						if (sensor_Data.Set>=103)
						{
							sensor_Data.Set=101;
							KEY_SET_State=_usKEY_Set_Parameter;
							break;
						}
						sensor_Data.Set++;
					}
				else
					{
                KEY_SET_State++;
				}
            }
            if (KEY_State==_usKEY_State_NONE)
            {
	            KEY_State=_usKEY_State_Set;
				KEY_SET_State=1;
            }
            break;
        case KEY_LONG_DOWN_SET:
            SEGGER_RTT_printf(0,"KEY_LONG_DOWN_JS键按下 KEY_State= %d\r\n",KEY_State);         /* 长按设置键 */
            BEEP_KeyTone();
			if (LCD_icon.AUTO)
			{
				LCD_icon.AUTO=0;
				LCD_icon.Manual=1;
				Relay_OFF();
				ee_SendOneByte(254,LCD_icon.AUTO);
				bsp_StopTimer(_usTime_Auto);
			}
			else
			{
				LCD_icon.AUTO=1;
				LCD_icon.Manual=0;
				Relay_OFF();
				ee_SendOneByte(254,LCD_icon.AUTO);
				bsp_StartAutoTimer(_usTime_Auto,(uint32_t)sensor_Data.Interval_time[sensor_Data.P_Device_Addr]*60000);
			}
			
            break;

        case KEY_AUTO_SET:
            SEGGER_RTT_printf(0,"KEY_AUTO_SET键按下 KEY_State= %d\r\n",KEY_State);         /* 自动连发设置键 6S*/
            BEEP_KeyTone();
            if (KEY_State!=_usKEY_State_Lock)
            {
				if (KEY_State==_usKEY_State_SuperSet)
					{
						KEY_State=_usKEY_State_debug;
						LCD_icon.AUTO=0;
						LCD_icon.Manual=1;
						Relay_OFF();
						ee_SendOneByte(254,LCD_icon.AUTO);
						bsp_StopTimer(_usTime_Auto);
					}
				else
					{
						KEY_State=_usKEY_State_SuperSet;
						KEY_SET_State=_usKEY_Set_Parameter;
						LCD_icon.AUTO=0;
						LCD_icon.Manual=1;
						Relay_OFF();
						ee_SendOneByte(254,LCD_icon.AUTO);
						bsp_StopTimer(_usTime_Auto);
					}

            }
            break;

        case KEY_UP_CF:
            SEGGER_RTT_printf(0,"KEY_UP_CF键按下 KEY_State= %d\r\n",KEY_State);         /* 晨风键 */
            BEEP_KeyTone();
            if (KEY_State!=_usKEY_State_Lock)
            {
                if (KEY_State==_usKEY_State_Set||KEY_State==_usKEY_State_SuperSet  )
                {
                    KEY_State = _usKEY_State_NONE;
                    KEY_SET_State=_usKEY_Set_upper;                  ///////////////////////需要保存参数
                    sensor_Data.Set=101;
					if (sensor_Data.P_Mb_Mode)
						{
						sensor_Data.P_Device_Addr=1;
						}
					
                    Send_IIC_Parameter();
                }else
					if (KEY_State==_usKEY_State_debug)
						{
						KEY_State = _usKEY_State_NONE;
						KEY_SET_State=_usKEY_Set_upper; 				 ///////////////////////需要保存参数
						sensor_Data.Opening_time[sensor_Data.P_Device_Addr]=0;
						Opening_time_Update(1);
						}
                else
                {

                    if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]==0)
                    {
                        Relay_UP_ON();
                        bsp_StartTimer(_usTime_Relay_OFF,sensor_Data.Start_Time[sensor_Data.P_Device_Addr]*60000/sensor_Data.Parameter);
                    }
                }
            }
            break;
        case KEY_AUTO_CF:
            SEGGER_RTT_printf(0,"KEY_LONG_UP_CF KEY_State= %d\r\n",KEY_State);         /* 自动连发晨风键 8S*/
            BEEP_KeyTone();
//            Relay_DOWN_ON();
//            bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)sensor_Data.All_time[sensor_Data.P_Device_Addr]*60000/sensor_Data.Parameter);
            break;
        case KEY_UP_DOWN:
            SEGGER_RTT_printf(0,"KEY_UP_DOWN键按下 KEY_State= %d\r\n",KEY_State);         /* 下行键 */
            BEEP_KeyTone();
            if (KEY_State!=_usKEY_State_Lock)
            {
                if (KEY_State==_usKEY_State_NONE &&LCD_icon.Manual==1 )
                {
                    if (Relay_State==_usRelay_OFF)
                    {
                        if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]!=0)
                        {
                            Relay_DOWN_ON();
                            bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)sensor_Data.Running_time[sensor_Data.P_Device_Addr]*60000/sensor_Data.Parameter);
                            if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]<=sensor_Data.Running_time[sensor_Data.P_Device_Addr]*100/sensor_Data.All_time[sensor_Data.P_Device_Addr])
                            {
                                bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)sensor_Data.Opening_time[sensor_Data.P_Device_Addr]*sensor_Data.All_time[sensor_Data.P_Device_Addr]*600/sensor_Data.Parameter);
                            }
                            else
                            {
                            }
                        }

                    }
                    else if(Relay_State==_usRelay_UP_ON)
                    {
                        Relay_OFF();
                        HAL_Delay(200);
                        Opening_time_Update(1);
                        if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]!=0)
                        {
                            Relay_DOWN_ON();
                            bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)sensor_Data.Running_time[sensor_Data.P_Device_Addr]*60000/sensor_Data.Parameter);
                            if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]<=sensor_Data.Running_time[sensor_Data.P_Device_Addr]*100/sensor_Data.All_time[sensor_Data.P_Device_Addr])
                            {
                                bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)sensor_Data.Opening_time[sensor_Data.P_Device_Addr]*sensor_Data.All_time[sensor_Data.P_Device_Addr]*600/sensor_Data.Parameter);
                            }
                            else
                            {
                            }
                        }

                    }
                    else
                    {
                        Relay_OFF();
                        Opening_time_Update(1);
                    }

                }
                if (KEY_State==_usKEY_State_Set)
                {
                    switch (KEY_SET_State)
                    {
                    case _usKEY_Set_upper:           /* 上限 */
                        if (sensor_Data.Upper_limit[sensor_Data.P_Device_Addr]<=(sensor_Data.Lower_limit[sensor_Data.P_Device_Addr]+1))
                        {
						   if (sensor_Data.Lower_limit[sensor_Data.P_Device_Addr]<=4)
							   {
							   break;
							   }
                            sensor_Data.Lower_limit[sensor_Data.P_Device_Addr]--;
							if (sensor_Data.Lower_limit[sensor_Data.P_Device_Addr]<sensor_Data.Start[sensor_Data.P_Device_Addr])
								{
								sensor_Data.Start[sensor_Data.P_Device_Addr]--;
								if (sensor_Data.Start[sensor_Data.P_Device_Addr]<=sensor_Data.Last[sensor_Data.P_Device_Addr])
									{
									sensor_Data.Last[sensor_Data.P_Device_Addr]--;
									}
								}
                        }
                        sensor_Data.Upper_limit[sensor_Data.P_Device_Addr]--;
                        break;

                    case _usKEY_Set_Lower:           /* 下限 */
						if (sensor_Data.Lower_limit[sensor_Data.P_Device_Addr]<=sensor_Data.Last[sensor_Data.P_Device_Addr]+1)
							{
							if (sensor_Data.Last[sensor_Data.P_Device_Addr]<=3)
								{
								break;
								}
							sensor_Data.Last[sensor_Data.P_Device_Addr]--;
							}
                        sensor_Data.Lower_limit[sensor_Data.P_Device_Addr]--;
                        break;

                    case _usKEY_Set_Start:           /* 开风 */
						if (sensor_Data.Start[sensor_Data.P_Device_Addr]<=sensor_Data.Last[sensor_Data.P_Device_Addr]+1)
							{
							if (sensor_Data.Last[sensor_Data.P_Device_Addr]<=3)
								{
								break;
								}
							sensor_Data.Last[sensor_Data.P_Device_Addr]--;
							}
                        sensor_Data.Start[sensor_Data.P_Device_Addr]--;
                        break;

                    case _usKEY_Set_Last:            /* 关风 */
						if (sensor_Data.Last[sensor_Data.P_Device_Addr]>3)
							{
							sensor_Data.Last[sensor_Data.P_Device_Addr]--;
							}
                        break;
                    case _usKEY_Set_Start_Time:      /* 首次 */
                        if (sensor_Data.Start_Time[sensor_Data.P_Device_Addr]==0)
                        {
                            sensor_Data.Start_Time[sensor_Data.P_Device_Addr]=99;
                        }
                        sensor_Data.Start_Time[sensor_Data.P_Device_Addr]--;
                        break;

                    case _usKEY_Set_Running_time:    /* 单次 */
                        if (sensor_Data.Running_time[sensor_Data.P_Device_Addr]==0)
                        {
                            sensor_Data.Running_time[sensor_Data.P_Device_Addr]=99;
                        }
                        sensor_Data.Running_time[sensor_Data.P_Device_Addr]--;
                        break;
                    case _usKEY_Set_Interval_time:   /* 分段 */
                        if (sensor_Data.Interval_time[sensor_Data.P_Device_Addr]==0)
                        {
                            sensor_Data.Interval_time[sensor_Data.P_Device_Addr]=99;
                        }
                        sensor_Data.Interval_time[sensor_Data.P_Device_Addr]--;
                        break;

                    case _usKEY_Set_ADDR:            /* 站号 */
                        if (sensor_Data.P_Device_Addr==0)
                        {
                            sensor_Data.P_Device_Addr=16;
                        }
                        sensor_Data.P_Device_Addr--;
                        break;
                    case _usKEY_Set_Electric:        /* 电流 */
                        if (sensor_Data.Electric_current==0)
                        {
                            sensor_Data.Electric_current=99;
                        }
                        sensor_Data.Electric_current--;
                        break;
                    }
                }
                else if (KEY_State==_usKEY_State_SuperSet)
                {
                    switch (KEY_SET_State)
                    {
                    case _usKEY_Set_Parameter:		 /* 参数 */
                        if (sensor_Data.Parameter==0)
                        {
                            sensor_Data.Parameter=99;
                        }
                        sensor_Data.Parameter--;
                        break;

                    case _usKEY_Set_All_time:		/* 总行程 */
                        if (sensor_Data.All_time[sensor_Data.P_Device_Addr]==0)
                        {
                            sensor_Data.All_time[sensor_Data.P_Device_Addr]=999;
                        }
                        sensor_Data.All_time[sensor_Data.P_Device_Addr]--;
                        break;
					case _usKEY_Set_State_SuperSet:		/* 工厂配置 */
							switch (sensor_Data.Set)
							{
							case 101:	/* 工厂配置 */
								if (sensor_Data.Electric_current_flag)
									{
									sensor_Data.Electric_current_flag=0;
									break;
									}
									sensor_Data.Electric_current_flag=1;
								break;
							case 102:	/* 工厂配置 */
								if (sensor_Data.Limit_flag)
									{
									sensor_Data.Limit_flag=0;
									break;
									}
									sensor_Data.Limit_flag=1;
								break;
							case 103:	/* 工厂配置 */
								if (sensor_Data.P_Mb_Mode)
									{
									sensor_Data.P_Mb_Mode=0;
									break;
									}
									sensor_Data.P_Mb_Mode=1;
								break;
							default:
								LCD_WirteBuf(31,sensor_Data.fault,1,1);
								break;
							}
						
						break;
                    }

                }
				else if(KEY_State==_usKEY_State_debug)
					{
					if (Relay_State==_usRelay_OFF)
						{
						Relay_DOWN_ON();
						bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)sensor_Data.Running_time[sensor_Data.P_Device_Addr]*60000/sensor_Data.Parameter);
						}
					else
						{
						Relay_OFF();
						bsp_StopTimer(_usTime_Relay_OFF);
						}
					}
            }

            break;
        case KEY_UP_UP:
            SEGGER_RTT_printf(0,"KEY_UP_UP键按下 KEY_State= %d\r\n",KEY_State);		  /* 上行键 */
            BEEP_KeyTone();
            if (KEY_State!=_usKEY_State_Lock)
            {
                if (KEY_State==_usKEY_State_NONE &&LCD_icon.Manual==1 )
                {
                    if (Relay_State==_usRelay_OFF)
                    {
                        if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]==0)
                        {
                            Relay_UP_ON();
                            bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)sensor_Data.Start_Time[sensor_Data.P_Device_Addr]*60000/sensor_Data.Parameter);
//                            bsp_StartAutoTimer(_usTime_Auto,(uint32_t)sensor_Data.Interval_time[sensor_Data.P_Device_Addr]*60000);
                        }
                        else if(sensor_Data.Opening_time[sensor_Data.P_Device_Addr]!=100)
                        {
                            Relay_UP_ON();
                            bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)sensor_Data.Running_time[sensor_Data.P_Device_Addr]*60000/sensor_Data.Parameter);
                            if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]>100)
                            {
                                bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)(sensor_Data.Opening_time[sensor_Data.P_Device_Addr]-100)*sensor_Data.All_time[sensor_Data.P_Device_Addr]*600/sensor_Data.Parameter);
                            }
                        }
                    }
                    else if(Relay_State==_usRelay_DOWN_ON)
                    {
                        Relay_OFF();
                        HAL_Delay(200);
                        Opening_time_Update(1);
                        if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]==0)
                        {
                            Relay_UP_ON();
                            bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)sensor_Data.Start_Time[sensor_Data.P_Device_Addr]*60000/sensor_Data.Parameter);
                        }
                        else if(sensor_Data.Opening_time[sensor_Data.P_Device_Addr]!=100)
                        {
                            Relay_UP_ON();
                            bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)sensor_Data.Running_time[sensor_Data.P_Device_Addr]*60000/sensor_Data.Parameter);
                            if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]>100)
                            {
                                bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)(sensor_Data.Opening_time[sensor_Data.P_Device_Addr]-100)*sensor_Data.All_time[sensor_Data.P_Device_Addr]*600/sensor_Data.Parameter);
                            }
                        }
                    }
                    else
                    {
                        Relay_OFF();
                        Opening_time_Update(1);
                    }

                }
                if (KEY_State==_usKEY_State_Set)
                {
                    switch (KEY_SET_State)
                    {
                    case _usKEY_Set_upper:           /* 上限 */
                        sensor_Data.Upper_limit[sensor_Data.P_Device_Addr]++;
                        if (sensor_Data.Upper_limit[sensor_Data.P_Device_Addr]>50)
                        {
                            sensor_Data.Upper_limit[sensor_Data.P_Device_Addr]=sensor_Data.Lower_limit[sensor_Data.P_Device_Addr]+1;
                        }
                        break;

                    case _usKEY_Set_Lower:           /* 下限 */
                        sensor_Data.Lower_limit[sensor_Data.P_Device_Addr]++;
                        if (sensor_Data.Lower_limit[sensor_Data.P_Device_Addr]>=sensor_Data.Upper_limit[sensor_Data.P_Device_Addr])
                        {
                            sensor_Data.Lower_limit[sensor_Data.P_Device_Addr]=sensor_Data.Last[sensor_Data.P_Device_Addr]+1;
                        }
                        break;

                    case _usKEY_Set_Start:           /* 开风 */
                        sensor_Data.Start[sensor_Data.P_Device_Addr]++;
                        if (sensor_Data.Start[sensor_Data.P_Device_Addr]>=sensor_Data.Upper_limit[sensor_Data.P_Device_Addr])
                        {
                            sensor_Data.Start[sensor_Data.P_Device_Addr]=sensor_Data.Last[sensor_Data.P_Device_Addr]+1;
                        }
                        break;

                    case _usKEY_Set_Last:            /* 关风 */
                        sensor_Data.Last[sensor_Data.P_Device_Addr]++;
                        if (sensor_Data.Last[sensor_Data.P_Device_Addr]>=sensor_Data.Start[sensor_Data.P_Device_Addr])
                        {
                            sensor_Data.Last[sensor_Data.P_Device_Addr]=3;
                        }
                        break;
                    case _usKEY_Set_Start_Time:      /* 首次 */
                        sensor_Data.Start_Time[sensor_Data.P_Device_Addr]++;
                        if (sensor_Data.Start_Time[sensor_Data.P_Device_Addr]>99)
                        {
                            sensor_Data.Start_Time[sensor_Data.P_Device_Addr]=1;
                        }
                        break;

                    case _usKEY_Set_Running_time:    /* 单次 */
                        sensor_Data.Running_time[sensor_Data.P_Device_Addr]++;
                        if (sensor_Data.Running_time[sensor_Data.P_Device_Addr]>99)
                        {
                            sensor_Data.Running_time[sensor_Data.P_Device_Addr]=1;
                        }
                        break;
                    case _usKEY_Set_Interval_time:   /* 分段 */
                        sensor_Data.Interval_time[sensor_Data.P_Device_Addr]++;
                        if (sensor_Data.Interval_time[sensor_Data.P_Device_Addr]>99)
                        {
                            sensor_Data.Interval_time[sensor_Data.P_Device_Addr]=1;
                        }
                        break;

                    case _usKEY_Set_ADDR:            /* 站号 */
                        sensor_Data.P_Device_Addr++;
                        if (sensor_Data.P_Device_Addr>16)
                        {
                            sensor_Data.P_Device_Addr=1;
                        }
                        break;

                    case _usKEY_Set_Electric:        /* 电流 */
                        sensor_Data.Electric_current++;
                        if (sensor_Data.Electric_current>99)
                        {
                            sensor_Data.Electric_current=0;
                        }
                        break;
                    }
                }
                else if (KEY_State==_usKEY_State_SuperSet)
                {
                    switch (KEY_SET_State)
                    {
                    case _usKEY_Set_Parameter:		 /* 参数 */
                        if (sensor_Data.Parameter>99)
                        {
                            sensor_Data.Parameter=1;
                        }
                        sensor_Data.Parameter++;
                        break;

                    case _usKEY_Set_All_time:		/* 总行程 */
                        if (sensor_Data.All_time[sensor_Data.P_Device_Addr]>999)
                        {
                            sensor_Data.All_time[sensor_Data.P_Device_Addr]=1;
                        }
                        sensor_Data.All_time[sensor_Data.P_Device_Addr]++;
                        break;
						case _usKEY_Set_State_SuperSet: 	/* 工厂配置 */
								switch (sensor_Data.Set)
								{
								case 101:	/* 工厂配置 */
									if (sensor_Data.Electric_current_flag)
										{
										sensor_Data.Electric_current_flag=0;
										break;
										}
										sensor_Data.Electric_current_flag=1;
									break;
								case 102:	/* 工厂配置 */
									if (sensor_Data.Limit_flag)
										{
										sensor_Data.Limit_flag=0;
										break;
										}
										sensor_Data.Limit_flag=1;
									break;
								case 103:	/* 工厂配置 */
									if (sensor_Data.P_Mb_Mode)
										{
										sensor_Data.P_Mb_Mode=0;
										break;
										}
										sensor_Data.P_Mb_Mode=1;
									break;
								default:
									LCD_WirteBuf(31,sensor_Data.fault,1,1);
									break;
								}
							
							break;
                    }

                }
            }
            break;

        default:
            SEGGER_RTT_printf(0,"其他按键 ucKeyCode = %d  KEY_State= %d\r\n",ucKeyCode,KEY_State);		  /* 设置键 */
            break;
        }

    }
}



void AUTO_Mode_work(void)
{
    if((bsp_CheckTimer(_usTime_Auto) || sensor_Data.Opening_time[sensor_Data.P_Device_Addr]==0 || sensor_Data.Temperature[sensor_Data.P_Device_Addr]<sensor_Data.Last[sensor_Data.P_Device_Addr] ) && Relay_State==_usRelay_OFF)	 //  100MS	定时
    {
        if (LCD_icon.AUTO)
        {

            if (sensor_Data.Start[sensor_Data.P_Device_Addr]<=sensor_Data.Temperature[sensor_Data.P_Device_Addr] && sensor_Data.Opening_time[sensor_Data.P_Device_Addr]==0)
            {
                SEGGER_RTT_SetTerminal(3);
                SEGGER_RTT_printf(0,"Temperature[sensor_Data.P_Device_Addr] = x%02d   Start[sensor_Data.P_Device_Addr] = %02d  Opening_time[sensor_Data.P_Device_Addr] = %02d\r\n",sensor_Data.Temperature[sensor_Data.P_Device_Addr],sensor_Data.Start[sensor_Data.P_Device_Addr],sensor_Data.Opening_time[sensor_Data.P_Device_Addr]);
//				Opening_time_Update(1);
                Relay_UP_ON();
                bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)sensor_Data.Start_Time[sensor_Data.P_Device_Addr]*60000/sensor_Data.Parameter);
            }
            else if(sensor_Data.Upper_limit[sensor_Data.P_Device_Addr]<sensor_Data.Temperature[sensor_Data.P_Device_Addr])
            {
                if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]!=100)
                {
                    Relay_UP_ON();
                    bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)sensor_Data.Running_time[sensor_Data.P_Device_Addr]*60000/sensor_Data.Parameter);
                    if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]+sensor_Data.Running_time[sensor_Data.P_Device_Addr]*100/sensor_Data.All_time[sensor_Data.P_Device_Addr]>100)
                    {
                        bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)(sensor_Data.Opening_time[sensor_Data.P_Device_Addr]-100)*sensor_Data.All_time[sensor_Data.P_Device_Addr]*600/sensor_Data.Parameter);
                    }
                }
            }

            else if(sensor_Data.Temperature[sensor_Data.P_Device_Addr]<sensor_Data.Lower_limit[sensor_Data.P_Device_Addr])
            {

                if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]!=0)
                {
                    SEGGER_RTT_SetTerminal(3);
                    SEGGER_RTT_printf(0,"Temperature[sensor_Data.P_Device_Addr] = x%02d   Start[sensor_Data.P_Device_Addr] = %02d  Opening_time[sensor_Data.P_Device_Addr] = %02d\r\n",sensor_Data.Temperature[sensor_Data.P_Device_Addr],sensor_Data.Start[sensor_Data.P_Device_Addr],sensor_Data.Opening_time[sensor_Data.P_Device_Addr]);
                    if (sensor_Data.Temperature[sensor_Data.P_Device_Addr]<sensor_Data.Last[sensor_Data.P_Device_Addr])
                    {
                        Relay_DOWN_ON();
                        bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)sensor_Data.Opening_time[sensor_Data.P_Device_Addr]*sensor_Data.All_time[sensor_Data.P_Device_Addr]*600/sensor_Data.Parameter);
                    }
                    else
                    {
                      uint16_t  _usStart_Time=sensor_Data.Start_Time[sensor_Data.P_Device_Addr]*100/sensor_Data.All_time[sensor_Data.P_Device_Addr];
                        bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)sensor_Data.Running_time[sensor_Data.P_Device_Addr]*60000/sensor_Data.Parameter);
                        if ((sensor_Data.Opening_time[sensor_Data.P_Device_Addr]-_usStart_Time)<=sensor_Data.Running_time[sensor_Data.P_Device_Addr]*100/sensor_Data.All_time[sensor_Data.P_Device_Addr])
                        {
                           if(sensor_Data.Opening_time[sensor_Data.P_Device_Addr]>_usStart_Time)
                           	{
                            bsp_StartTimer(_usTime_Relay_OFF,(uint32_t)(sensor_Data.Opening_time[sensor_Data.P_Device_Addr]-sensor_Data.Start_Time[sensor_Data.P_Device_Addr]*100/sensor_Data.All_time[sensor_Data.P_Device_Addr])*sensor_Data.All_time[sensor_Data.P_Device_Addr]*600/sensor_Data.Parameter);

						   }
						   else
						   	{
						   	bsp_StopTimer(_usTime_Relay_OFF);
							Relay_OFF();
							return;
						   	}
                        }
                        Relay_DOWN_ON();
                    }
                }
            }

        }
    }

    if(bsp_CheckTimer(_usTime_Relay_OFF))    //  100MS  定时
    {
        Relay_OFF();
    }

}



void Opening_time_Update (uint8_t Update)
{
    static uint16_t Opening_time_cache=0;
    uint16_t Run_time=0,_Opening_time=0;
	uint32_t Timer_Time=0;

    if (Update)
    {
        Opening_time_cache=sensor_Data.Opening_time[sensor_Data.P_Device_Addr];
        ee_SendLenByte(40,&sensor_Data.Opening_time[sensor_Data.P_Device_Addr],1);
    }
    if (Relay_State==_usRelay_UP_ON)
    {
        if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]<100)
        {
        Timer_Time=bsp_GetTimer_RunTime(_usTime_Relay_OFF);
		Run_time=Timer_Time*sensor_Data.Parameter/(uint32_t)600/sensor_Data.All_time[sensor_Data.P_Device_Addr];
		_Opening_time=Opening_time_cache+Run_time+1;
		if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]!=_Opening_time&&Timer_Time)
			{
			SEGGER_RTT_SetTerminal(3);
			SEGGER_RTT_printf(0,"_usRelay_DOWN_ON	Opening_time_cache = x%02d	Opening_time[sensor_Data.P_Device_Addr] = %02d   _Opening_time=%02d\r\n",Opening_time_cache,sensor_Data.Opening_time[sensor_Data.P_Device_Addr],_Opening_time);
			sensor_Data.Opening_time[sensor_Data.P_Device_Addr]=_Opening_time;
            ee_SendLenByte(40,&sensor_Data.Opening_time[sensor_Data.P_Device_Addr],1);
			}
        }
		else if(sensor_Data.Opening_time[sensor_Data.P_Device_Addr]!=100)
			{
				sensor_Data.Opening_time[sensor_Data.P_Device_Addr]=100;
				ee_SendLenByte(40,&sensor_Data.Opening_time[sensor_Data.P_Device_Addr],1);
			}

    }
    else if(Relay_State==_usRelay_DOWN_ON)
    {
		if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr])
			{
			Timer_Time=bsp_GetTimer_RunTime(_usTime_Relay_OFF);
			Run_time=Timer_Time*sensor_Data.Parameter/(uint32_t)600/sensor_Data.All_time[sensor_Data.P_Device_Addr];
	        if (Opening_time_cache>Run_time&&Timer_Time)
	        {
		        _Opening_time=Opening_time_cache-Run_time-1;
				if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]!=_Opening_time)
					{
					SEGGER_RTT_SetTerminal(3);
					SEGGER_RTT_printf(0,"_usRelay_DOWN_ON   Opening_time_cache = x%02d  Opening_time[sensor_Data.P_Device_Addr] = %02d   _Opening_time=%02d\r\n",Opening_time_cache,sensor_Data.Opening_time[sensor_Data.P_Device_Addr],_Opening_time);
					sensor_Data.Opening_time[sensor_Data.P_Device_Addr]=_Opening_time;
					ee_SendLenByte(40,&sensor_Data.Opening_time[sensor_Data.P_Device_Addr],1);
					}
	        }
		}
          else 
				{
				sensor_Data.Opening_time[sensor_Data.P_Device_Addr]=0;
				ee_SendLenByte(40,&sensor_Data.Opening_time[sensor_Data.P_Device_Addr],1);
				}

    }
    else
    {
        if (sensor_Data.Opening_time[sensor_Data.P_Device_Addr]!=Opening_time_cache)
        {
			SEGGER_RTT_SetTerminal(3);
			SEGGER_RTT_printf(0,"Opening_time_cache = x%02d  Opening_time[sensor_Data.P_Device_Addr] = %02d\r\n",Opening_time_cache,sensor_Data.Opening_time[sensor_Data.P_Device_Addr]);
            Opening_time_cache=sensor_Data.Opening_time[sensor_Data.P_Device_Addr];
            ee_SendLenByte(40,&sensor_Data.Opening_time[sensor_Data.P_Device_Addr],1);
        }
    }

}




/*
*********************************************************************************************************
*	函 数 名: LCD_WirteBuf
*	功能说明: 写LCD缓存数组
*	形    参:  	_LCD_addr   : 缓存地址。
*				_LCD_wr_Num : 缓存数据可  HEX&ansi用_usLen来区分
*				_usLen      : 数据的字个数           HEX时+3
*				_us_icon    : 是否显示相应的图标              0 不显示      1 显示
*	返 回 值: 无
*********************************************************************************************************
*/


void LCD_WirteBuf(uint8_t _LCD_addr,uint16_t _LCD_wr_Num,uint8_t _usLen,uint8_t _us_icon )
{
    uint8_t WR_addr=_LCD_addr;
    if (_LCD_addr>31)
    {
        return ;
    }

    if (LCD_SEG_Num[WR_addr])
    {
        switch (_usLen)
        {
        case 1:
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num%12];
            LCD_Buf[WR_addr]|=(_us_icon<<7);
            break;
        case 2:
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num/10%12];
            LCD_Buf[WR_addr--]|=(_us_icon<<7);
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num%10];
            break;
        case 3:
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num/100%10];
            LCD_Buf[WR_addr--]|=(_us_icon<<7);
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num/10%10];
            WR_addr--;
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num%10];
            break;

///////////////////////////////////////////////////////////////////////////////////////////  HEX

        case 4:
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num%16];
            LCD_Buf[WR_addr]|=(_us_icon<<7);
            break;
        case 5:
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num/16%16];
            LCD_Buf[WR_addr--]|=(_us_icon<<7);
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num%16];
            break;
        case 6:
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num/256%16];
            LCD_Buf[WR_addr--]|=(_us_icon<<7);
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num/16%16];
            WR_addr--;
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num%16];
            break;
        }
    }
    else
    {
        switch (_usLen)
        {
        case 1:
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num%12];
            LCD_Buf[WR_addr]|=_us_icon;
            break;
        case 2:
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num/10%12];
            LCD_Buf[WR_addr++]|=_us_icon;
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num%10];
            if (WR_addr==28)
            {
                LCD_Buf[WR_addr]|=_us_icon;
            }
            break;
        case 3:
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num/100%10];
            LCD_Buf[WR_addr++]|=_us_icon;
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num/10%10];
            WR_addr++;
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num%10];
            break;

///////////////////////////////////////////////////////////////////////////////////////////  HEX

        case 4:
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num%16];
            LCD_Buf[WR_addr]|=_us_icon;
            break;
        case 5:
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num/16%16];
            LCD_Buf[WR_addr++]|=_us_icon;
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num%16];
            if (WR_addr==28)
            {
                LCD_Buf[WR_addr]|=_us_icon;
            }
            break;
        case 6:
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num/256%16];
            LCD_Buf[WR_addr++]|=_us_icon;
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num/16%16];
            WR_addr++;
            LCD_Buf[WR_addr]=Num_code[LCD_SEG_Num[WR_addr]][_LCD_wr_Num%16];
            break;
        }

    }

}






void TM_1621_flush(void)
{
    static uint8_t Disp_Set_count=0;

    LCD_WirteBuf(0,sensor_Data.Temperature[sensor_Data.P_Device_Addr],2,1);
    LCD_WirteBuf(2,sensor_Data.Opening_time[sensor_Data.P_Device_Addr],3,1);
    LCD_WirteBuf(9,sensor_Data.Upper_limit[sensor_Data.P_Device_Addr],2,1);
    LCD_WirteBuf(7,sensor_Data.Start_Time[sensor_Data.P_Device_Addr],2,1);
    LCD_WirteBuf(10,sensor_Data.Lower_limit[sensor_Data.P_Device_Addr],2,1);
    LCD_WirteBuf(12,sensor_Data.Running_time[sensor_Data.P_Device_Addr],2,1);
    LCD_WirteBuf(17,sensor_Data.Start[sensor_Data.P_Device_Addr],2,1);
    LCD_WirteBuf(15,sensor_Data.Interval_time[sensor_Data.P_Device_Addr],2,1);
    LCD_WirteBuf(18,sensor_Data.Last[sensor_Data.P_Device_Addr],2,1);
    LCD_WirteBuf(20,sensor_Data.All_time[sensor_Data.P_Device_Addr],3,1);
    LCD_WirteBuf(26,sensor_Data.P_Device_Addr,2,1);
    LCD_WirteBuf(24,sensor_Data.Parameter,2,1);
    LCD_WirteBuf(27,sensor_Data.Electric_current,2,1);
    LCD_WirteBuf(29,sensor_Data.Set,2,1);
	if (KEY_State==_usKEY_State_SuperSet)
	{
		switch (KEY_SET_State)
		{
		case _usKEY_Set_State_SuperSet: 	/* 工厂配置 */
			switch (sensor_Data.Set)
			{
			case 101: 	/* 工厂配置 */
				LCD_WirteBuf(31,sensor_Data.Electric_current_flag,1,0);
				break;
			case 102: 	/* 工厂配置 */
				LCD_WirteBuf(31,sensor_Data.Limit_flag,1,0);
				break;
			case 103: 	/* 工厂配置 */
				LCD_WirteBuf(31,sensor_Data.P_Mb_Mode,1,0);
				break;
			default:
				LCD_WirteBuf(31,sensor_Data.fault,1,1);
				break;
			}
			break;
		default:
			LCD_WirteBuf(31,sensor_Data.fault,1,1);
			break;
		}
	
	}
    else
    {
        LCD_WirteBuf(31,sensor_Data.fault,1,1);
    }

    Disp_Set_count++;
    if (Disp_Set_count>2)
    {
        Disp_Set_count=0;
    }
    if (Disp_Set_count==1)
    {
        if (KEY_State==_usKEY_State_Set)
        {
            switch (KEY_SET_State)
            {
            case _usKEY_Set_upper:			 /* 上限 */
                LCD_WirteBuf(9,0XBB,2+3,1);
                break;

            case _usKEY_Set_Lower:			 /* 下限 */
                LCD_WirteBuf(10,0XBB,2+3,1);
                break;

            case _usKEY_Set_Start:			 /* 开风 */
                LCD_WirteBuf(17,0XBB,2+3,1);
                break;

            case _usKEY_Set_Last:			 /* 关风 */
                LCD_WirteBuf(18,0XBB,2+3,1);
                break;
            case _usKEY_Set_Start_Time: 	 /* 首次 */
                LCD_WirteBuf(7,0XBB,2+3,1);
                break;

            case _usKEY_Set_Running_time:	 /* 单次 */
                LCD_WirteBuf(12,0XBB,2+3,1);
                break;
            case _usKEY_Set_Interval_time:	 /* 分段 */
                LCD_WirteBuf(15,0XBB,2+3,1);
                break;

            case _usKEY_Set_ADDR:			 /* 站号 */
                LCD_WirteBuf(26,0XBB,2+3,1);
                break;
            case _usKEY_Set_Parameter:		 /* 参数 */
                LCD_WirteBuf(24,0XBB,2+3,1);
                break;

            case _usKEY_Set_Electric:		 /* 电流 */
                LCD_WirteBuf(27,0XBB,2+3,1);
                break;
            }
        }
        else if (KEY_State==_usKEY_State_SuperSet)
        {
            switch (KEY_SET_State)
            {
            case _usKEY_Set_Parameter:		 /* 参数 */
                LCD_WirteBuf(24,0XBB,2+3,1);
                break;

            case _usKEY_Set_All_time:	    /* 总行程 */
                LCD_WirteBuf(20,0xbbb,3+3,1);
                break;
			case _usKEY_Set_State_SuperSet:		/* 工厂配置 */
				LCD_WirteBuf(29,0XBB,2+3,1);
				LCD_WirteBuf(31,0XB,1+3,0);
				break;
            }

        }
		else if(KEY_State==_usKEY_State_debug)
			{
				LCD_WirteBuf(2,0xbbb,3+3,1);
			}
    }


    LCD_Buf[19]|=LCD_icon.Mode;
    LCD_Buf[30]|=LCD_icon.s19;
    LCD_Buf[25]|=(LCD_icon.AUTO<<7);
    LCD_Buf[21]|=LCD_icon.Manual;
    WriteAllData(0,LCD_Buf,32);

}


void modbus_Update (void)
{
	if (LCD_icon.AUTO)
	{
		sensor_Data.State_feedback[sensor_Data.P_Device_Addr]|=0x01;
	}
	else
	{
		sensor_Data.State_feedback[sensor_Data.P_Device_Addr]&=0xfe;
	}
    if (Relay_State==_usRelay_UP_ON)
    {
		sensor_Data.State_feedback[sensor_Data.P_Device_Addr]|=0x80;
    }
    else if(Relay_State==_usRelay_DOWN_ON)
    	{
			sensor_Data.State_feedback[sensor_Data.P_Device_Addr]&=0x7f;
    	}

}





