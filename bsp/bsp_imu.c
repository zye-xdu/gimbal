#include "bsp_imu.h"
#include "usart.h"

extern DMA_HandleTypeDef hdma_usart2_rx;
//extern uint32_t remote_disconnect_ms;


//remote control data 
//遥控器接收dma缓存区
uint8_t imu_rcv_buf[IMU_BUFF_SIZE];
//遥控器控制变量
imu_t imu;



void imu_init(void);
//void get_imu_value(gimbal_data_t *data);

void imu_rate_process(uint8_t i);
void imu_angle_process(uint8_t i);

void imu_init(void)
{
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart2, imu_rcv_buf, IMU_BUFF_SIZE);
}


void get_imu_value(gimbal_data_t *data)
{
    data->p_yaw     =   imu.angle.yaw;
    data->p_pitch   =   -imu.angle.pitch;
    data->v_yaw     =   imu.rate.yaw;
    data->v_pitch   =   -imu.rate.pitch;
}



/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
	/* USER CODE BEGIN USART2_IRQn 0 */

	/* USER CODE END USART2_IRQn 0 */
	//HAL_UART_IRQHandler(&huart2);
	/* USER CODE BEGIN USART2_IRQn 1 */
    
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);	
        
		HAL_UART_AbortReceive(&huart2);//关闭dma
              
        uint16_t RX_Length = IMU_BUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);//计算数据长度
        
        if ((RX_Length % IMU_PACK_LENGTH) == 0)
        {
            for (uint8_t i = 0; i < (RX_Length / IMU_PACK_LENGTH); i++)
            {
                if (imu_rcv_buf[i*IMU_PACK_LENGTH] == 0x55)
                {
                    if (imu_rcv_buf[i*IMU_PACK_LENGTH + 1] == 0x52)		//angle_speed
                    {
                        imu_rate_process(i);
                    }
                    else if (imu_rcv_buf[i*IMU_PACK_LENGTH + 1] == 0x53)		//angle
                    {
                        imu_angle_process(i);
                    }
                }
            }
        }

	}
    //重启DMA
	HAL_UART_Receive_DMA(&huart2, imu_rcv_buf, IMU_BUFF_SIZE);			//陀螺仪接收数据huart2
  /* USER CODE END USART2_IRQn 1 */
}


void imu_rate_process(uint8_t i)
{
    imu.rate.row    =   (fp32) ((int16_t) ((imu_rcv_buf[i*IMU_PACK_LENGTH + 3]<<8)|imu_rcv_buf[i*IMU_PACK_LENGTH + 2]) ) /32768*2000;
    imu.rate.pitch  =   (fp32) ((int16_t) ((imu_rcv_buf[i*IMU_PACK_LENGTH + 5]<<8)|imu_rcv_buf[i*IMU_PACK_LENGTH + 4]) ) /32768*2000;
    imu.rate.yaw    =   (fp32) ((int16_t) ((imu_rcv_buf[i*IMU_PACK_LENGTH + 7]<<8)|imu_rcv_buf[i*IMU_PACK_LENGTH + 6]) ) /32768*2000;
}

void imu_angle_process(uint8_t i)
{
    imu.angle.row   =   (fp32) ((int16_t) ((imu_rcv_buf[i*IMU_PACK_LENGTH + 3]<<8)|imu_rcv_buf[i*IMU_PACK_LENGTH + 2]) ) /32768*180;
    imu.angle.pitch =   (fp32) ((int16_t) ((imu_rcv_buf[i*IMU_PACK_LENGTH + 5]<<8)|imu_rcv_buf[i*IMU_PACK_LENGTH + 4]) ) /32768*180;
    imu.angle.yaw   =   (fp32) ((int16_t) ((imu_rcv_buf[i*IMU_PACK_LENGTH + 7]<<8)|imu_rcv_buf[i*IMU_PACK_LENGTH + 6]) ) /32768*180;
}
