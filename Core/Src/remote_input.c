#include "remote_input.h"

/* Giới hạn an toàn */
#define REMOTE_THROTTLE_MIN         1000
#define REMOTE_THROTTLE_MAX         1800
#define REMOTE_AXIS_MIN             (-100)
#define REMOTE_AXIS_MAX             (100)

/* Bước thay đổi cho app nút bấm */
#define REMOTE_STEP_THR             25
#define REMOTE_STEP_ROLL_PITCH      30
#define REMOTE_STEP_YAW             30

static RemoteCommand_t g_remote_cmd;
static uint8_t g_pid_line_mode = 0;   // 1 = đang đi qua chuỗi PID kiểu 1P..., 2I..., 3D...

/* Ép giá trị về đúng miền an toàn */
static void RemoteInput_Clamp(RemoteCommand_t *cmd)
{
    if (cmd->throttle < REMOTE_THROTTLE_MIN) cmd->throttle = REMOTE_THROTTLE_MIN;
    if (cmd->throttle > REMOTE_THROTTLE_MAX) cmd->throttle = REMOTE_THROTTLE_MAX;

    if (cmd->roll_cmd < REMOTE_AXIS_MIN)  cmd->roll_cmd = REMOTE_AXIS_MIN;
    if (cmd->roll_cmd > REMOTE_AXIS_MAX)  cmd->roll_cmd = REMOTE_AXIS_MAX;

    if (cmd->pitch_cmd < REMOTE_AXIS_MIN) cmd->pitch_cmd = REMOTE_AXIS_MIN;
    if (cmd->pitch_cmd > REMOTE_AXIS_MAX) cmd->pitch_cmd = REMOTE_AXIS_MAX;

    if (cmd->yaw_cmd < REMOTE_AXIS_MIN)   cmd->yaw_cmd = REMOTE_AXIS_MIN;
    if (cmd->yaw_cmd > REMOTE_AXIS_MAX)   cmd->yaw_cmd = REMOTE_AXIS_MAX;
}

/* Đánh dấu vừa có lệnh mới */
static void RemoteInput_Touch(void)
{
    g_remote_cmd.valid = 1;
    g_remote_cmd.last_update_ms = HAL_GetTick();
    RemoteInput_Clamp(&g_remote_cmd);
}

/* Trả các trục điều khiển về 0, nhưng giữ nguyên arm và throttle */
static void RemoteInput_NeutralAxes(void)
{
    g_remote_cmd.roll_cmd = 0;
    g_remote_cmd.pitch_cmd = 0;
    g_remote_cmd.yaw_cmd = 0;
    RemoteInput_Touch();
}

/* Disarm và đưa hệ về trạng thái an toàn */
static void RemoteInput_DisarmAndReset(void)
{
    g_remote_cmd.valid = 1;
    g_remote_cmd.armed = 0;
    g_remote_cmd.throttle = REMOTE_THROTTLE_MIN;
    g_remote_cmd.roll_cmd = 0;
    g_remote_cmd.pitch_cmd = 0;
    g_remote_cmd.yaw_cmd = 0;
    g_remote_cmd.last_update_ms = HAL_GetTick();
}

void RemoteInput_Init(void)
{
    g_remote_cmd.valid = 0;
    g_remote_cmd.armed = 0;
    g_remote_cmd.throttle = REMOTE_THROTTLE_MIN;
    g_remote_cmd.roll_cmd = 0;
    g_remote_cmd.pitch_cmd = 0;
    g_remote_cmd.yaw_cmd = 0;
    g_remote_cmd.last_update_ms = 0;
    g_pid_line_mode = 0;
}

void RemoteInput_ParseByte(uint8_t byte)
{
    /* main.c đang tự xử lý PID bằng chuỗi 1P..., 2I..., 3D...
       nên remote chỉ né phần đó, không can thiệp */
    if (g_pid_line_mode)
    {
        if (byte == '\r' || byte == '\n' || byte == ';')
        {
            g_pid_line_mode = 0;
        }
        return;
    }

    /* Gặp đầu chuỗi PID thì nhường hoàn toàn cho parser trong main.c */
    if (byte == '1' || byte == '2' || byte == '3')
    {
        g_pid_line_mode = 1;
        return;
    }

    /* Bỏ qua ký tự kết thúc dòng */
    if (byte == '\r' || byte == '\n' || byte == ';')
    {
        return;
    }

    switch (byte)
    {
        case 'A':   /* Start Button: Arm */
            g_remote_cmd.armed = 1;
            g_remote_cmd.roll_cmd = 0;
            g_remote_cmd.pitch_cmd = 0;
            g_remote_cmd.yaw_cmd = 0;
            if (g_remote_cmd.throttle < REMOTE_THROTTLE_MIN)
            {
                g_remote_cmd.throttle = REMOTE_THROTTLE_MIN;
            }
            RemoteInput_Touch();
            break;

        case 'P':   /* Pause Button: Disarm */
            RemoteInput_DisarmAndReset();
            break;

        case 'T':   /* Triangle: tăng ga */
            g_remote_cmd.throttle += REMOTE_STEP_THR;
            RemoteInput_Touch();
            break;

        case 'X':   /* Cross: giảm ga */
            if (g_remote_cmd.throttle > REMOTE_THROTTLE_MIN)
            {
                g_remote_cmd.throttle -= REMOTE_STEP_THR;
            }
            RemoteInput_Touch();
            break;

        case 'F':   /* Up: tiến tới */
            g_remote_cmd.pitch_cmd = REMOTE_STEP_ROLL_PITCH;
            g_remote_cmd.roll_cmd = 0;
            g_remote_cmd.yaw_cmd = 0;
            RemoteInput_Touch();
            break;

        case 'B':   /* Down: lùi */
            g_remote_cmd.pitch_cmd = -REMOTE_STEP_ROLL_PITCH;
            g_remote_cmd.roll_cmd = 0;
            g_remote_cmd.yaw_cmd = 0;
            RemoteInput_Touch();
            break;

        case 'L':   /* Left: nghiêng trái */
            g_remote_cmd.roll_cmd = -REMOTE_STEP_ROLL_PITCH;
            g_remote_cmd.pitch_cmd = 0;
            g_remote_cmd.yaw_cmd = 0;
            RemoteInput_Touch();
            break;

        case 'R':   /* Right: nghiêng phải */
            g_remote_cmd.roll_cmd = REMOTE_STEP_ROLL_PITCH;
            g_remote_cmd.pitch_cmd = 0;
            g_remote_cmd.yaw_cmd = 0;
            RemoteInput_Touch();
            break;

        case 'C':   /* Circle: quay phải */
            g_remote_cmd.yaw_cmd = REMOTE_STEP_YAW;
            g_remote_cmd.roll_cmd = 0;
            g_remote_cmd.pitch_cmd = 0;
            RemoteInput_Touch();
            break;

        case 'S':   /* Square: quay trái */
            g_remote_cmd.yaw_cmd = -REMOTE_STEP_YAW;
            g_remote_cmd.roll_cmd = 0;
            g_remote_cmd.pitch_cmd = 0;
            RemoteInput_Touch();
            break;

        case '0':   /* Release của app: dừng lệnh hướng, giữ arm và throttle */
            RemoteInput_NeutralAxes();
            break;

        default:    /* Ký tự lạ thì bỏ qua */
            break;
    }
}

RemoteCommand_t RemoteInput_Get(void)
{
    return g_remote_cmd;
}
