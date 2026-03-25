#include "bluetooth.h"

//*******************发送缓存数组**********************//
char data_buffer[256] = {0};

/* @brief 蓝牙初始化 */
void BlueTooth_Init(void)
{
    if(ble6a20_init())    // 判断是否通过初始化
    {
        while(1)                                                                
        {
            //gpio_toggle_level(LED1);                                            
            system_delay_ms(100);                                               
        }
    }
    system_delay_ms(1000); 
    ble6a20_send_string(".\r\n"); 
    ble6a20_send_string("SEEKFREE ble6a20 text:\r\n");        // 初始化正常

}

/* @brief 发送文档数据
   状态、x坐标、y坐标、角度、速度、编码器1、编码器2、编码器3、编码器4
    */	
void send_txt_data(int State, int32_t Pos_X, int32_t Pos_Y, 
               int32_t Angle, int32_t Speed, 
               int16_t Encoder0, int16_t Encoder1, int16_t Encoder2, int16_t Encoder3)
{
    memset(data_buffer, 0, sizeof(data_buffer)); 
    snprintf(data_buffer, sizeof(data_buffer), "%d, (%d,%d), %d, %d, %d, %d, %d, %d\r\n",
        State, 
        Pos_X, 
        Pos_Y, 
        Angle,
        Speed,
        Encoder0,
        Encoder1,
        Encoder2,
        Encoder3
        );
    ble6a20_send_string(data_buffer);
}

/* @brief 发送各个轮子数据
   编码器1、速度1、位置1
   编码器2、速度2、位置2
   编码器3、速度3、位置3
   编码器4、速度4、位置4
    */	
void send_wheel_data(int16_t Encoder0, int32_t speed0, int32_t position0, 
                     int16_t Encoder1, int32_t speed1, int32_t position1, 
                     int16_t Encoder2, int32_t speed2, int32_t position2, 
                     int16_t Encoder3, int32_t speed3, int32_t position3)
{
    memset(data_buffer, 0, sizeof(data_buffer)); 
    snprintf(data_buffer, sizeof(data_buffer),
        "Encoder0: %d\r\n"
        "position: %d, speed: %d\r\n"				
        "Encoder1: %d\r\n"
        "position: %d, speed: %d\r\n"	
        "Encoder2: %d\r\n"
        "position: %d, speed: %d\r\n"	
        "Encoder3: %d\r\n"
        "position: %d, speed: %d\r\n",	
        Encoder0, position0, speed0, 
        Encoder1, position1, speed1,
        Encoder2, position2, speed2,
        Encoder3, position3, speed3
        );
    ble6a20_send_string(data_buffer);
}


void send_imu_data(void)
{
    memset(data_buffer, 0, sizeof(data_buffer));			
    snprintf(data_buffer, sizeof(data_buffer),                                   
        "acc data: x=%5d, y=%5d, z=%5d\r\n"
        "gyro data:  x=%5d, y=%5d, z=%5d\r\n"				
        "mag data: x=%5d, y=%5d, z=%5d\r\n",				
        imu963ra_acc_x, imu963ra_acc_y, imu963ra_acc_z,
        imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z,				
        imu963ra_mag_x, imu963ra_mag_y, imu963ra_mag_z); 			
    ble6a20_send_string(data_buffer);
}


void send_data(int32_t a, int32_t b, int32_t c, int32_t d){
    memset(data_buffer, 0, sizeof(data_buffer));
    snprintf(data_buffer, sizeof(data_buffer),
        "%d,   %d   ,%d,   %d\r\n",
         a, b, c, d);
    ble6a20_send_string(data_buffer);
}

/*
 * Print one full OpenART map frame over BLE in human-readable text form.
 *
 * Output format:
 * - one summary line with counters and player position
 * - twelve map rows in row-major order
 *
 * The summary line helps correlate a printed map with the compact four-number
 * telemetry that is still sent in the main loop.
 */
void send_openart_map_debug(const char map[OPENART_MAP_HEIGHT][OPENART_MAP_WIDTH],
                            int32_t player_x,
                            int32_t player_y,
                            int32_t valid_count,
                            int32_t invalid_count,
                            int32_t parse_error_count)
{
    char line_buffer[64];

    memset(line_buffer, 0, sizeof(line_buffer));
    snprintf(line_buffer, sizeof(line_buffer),
             "OA map cnt=%ld inv=%ld err=%ld p=(%ld,%ld)\r\n",
             (long)valid_count,
             (long)invalid_count,
             (long)parse_error_count,
             (long)player_x,
             (long)player_y);
    ble6a20_send_string(line_buffer);

    for(uint32_t row = 0; row < OPENART_MAP_HEIGHT; row++)
    {
        /*
         * Each row holds 16 ASCII cells. Copy exactly 16 bytes, then append
         * CRLF so the host terminal renders the map as a clean 12-line grid.
         */
        memset(line_buffer, 0, sizeof(line_buffer));
        memcpy(line_buffer, map[row], OPENART_MAP_WIDTH);
        line_buffer[OPENART_MAP_WIDTH] = '\r';
        line_buffer[OPENART_MAP_WIDTH + 1] = '\n';
        line_buffer[OPENART_MAP_WIDTH + 2] = '\0';
        ble6a20_send_string(line_buffer);
    }
}

