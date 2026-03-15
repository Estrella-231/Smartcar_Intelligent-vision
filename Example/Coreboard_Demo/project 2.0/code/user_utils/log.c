#include "log.h"

void send2txt(void)
{
    static uint32 last_send = 0;

    

    last_send = g_vehicle_ctrl.count_time;
}


