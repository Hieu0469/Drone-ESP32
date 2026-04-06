#ifndef REMOTE_INPUT_H
#define REMOTE_INPUT_H

#include "main.h"
#include <stdint.h>

/* Trạng thái lệnh điều khiển lấy từ app Bluetooth */
typedef struct
{
    uint8_t valid;           // Đã từng nhận lệnh hợp lệ hay chưa
    uint8_t armed;           // 1 = arm, 0 = disarm
    uint16_t throttle;       // Ga nền
    int16_t roll_cmd;        // Lệnh roll
    int16_t pitch_cmd;       // Lệnh pitch
    int16_t yaw_cmd;         // Lệnh yaw
    uint32_t last_update_ms; // Thời điểm nhận lệnh gần nhất
} RemoteCommand_t;

void RemoteInput_Init(void);
void RemoteInput_ParseByte(uint8_t byte);
RemoteCommand_t RemoteInput_Get(void);

#endif
