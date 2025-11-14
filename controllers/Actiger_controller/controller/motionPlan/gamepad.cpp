#include "gamepad.hpp"
#include <iostream>

using namespace std;
using namespace quadruped;


Gamepad::Gamepad(void)
{
    memset(&map, 0, sizeof(xbox_map_t));
    xbox_fd = xbox_open("/dev/input/js0");
    if (xbox_fd < 0)
    {
        cout << "gamepad init error!" << endl;
    }
}


Gamepad::~Gamepad(void) {}


int Gamepad::xbox_open(const char *file_name)
{
    int xbox_fd;
    xbox_fd = open(file_name, O_RDWR | O_NOCTTY | O_NDELAY);

    if (xbox_fd < 0)
    {
        perror("open");
        return -1;
    }

    return xbox_fd;
}


int Gamepad::xbox_map_read(int xbox_fd, xbox_map_t *p_map)
{
    int len, type, number, value;
    struct js_event js;

    len = read(xbox_fd, &js, sizeof(struct js_event));

    if (len < 0)
    {
        perror("read");
        return -1;
    }

    type = js.type;
    number = js.number;
    value = js.value;
    p_map->time = js.time;

    if (type == JS_EVENT_BUTTON)
    {
        switch (number)
        {
            case XBOX_BUTTON_A:
                p_map->a = value;
                break;

            case XBOX_BUTTON_B:
                p_map->b = value;
                break;

            case XBOX_BUTTON_X:
                p_map->x = value;
                break;

            case XBOX_BUTTON_Y:
                p_map->y = value;
                break;

            case XBOX_BUTTON_UP:
                p_map->up = value;
                break;

            case XBOX_BUTTON_DOWN:
                p_map->down = value;
                break;

            case XBOX_BUTTON_LEFT:
                p_map->left = value;
                break;

            case XBOX_BUTTON_RIGHT:
                p_map->right = value;
                break;

            case XBOX_BUTTON_L:
                p_map->l = value;
                break;

            case XBOX_BUTTON_R:
                p_map->r = value;
                break;

            case XBOX_BUTTON_LZ:
                p_map->lz = value;
                break;

            case XBOX_BUTTON_RZ:
                p_map->rz = value;
                break;

            case XBOX_BUTTON_HOME:
                p_map->home = value;
                break;

            case XBOX_BUTTON_LO:
                p_map->lo = value;
                break;

            case XBOX_BUTTON_RO:
                p_map->ro = value;
                break;

            case XBOX_BUTTON_PLUS:
                p_map->plus = value;
                break;

            case XBOX_BUTTON_MINUS:
                p_map->minus = value;
                break;

            default:
                break;
        }
    }
    else if (type == JS_EVENT_AXIS)
    {
        switch (number)
        {
            case XBOX_AXIS_LX:
                p_map->lx = -value;
                break;

            case XBOX_AXIS_LY:
                p_map->ly = -value;
                break;

            case XBOX_AXIS_RX:
                p_map->rx = -value;
                break;

            case XBOX_AXIS_RY:
                p_map->ry = -value;
                break;

            default:
                break;
        }
    }
    else
    {

    }

    return len;
}


void Gamepad::xbox_close(int xbox_fd)
{
    close(xbox_fd);
    return;
}


void Gamepad::GetKey(uint8_t *p_gaitNow)
{
    int len;
    len = xbox_map_read(xbox_fd, &map);

    if (map.lo == 1 && map.ro ==1)
    {
        map.lo = 0;
        map.ro = 0;
        handle.stepOver = 1;
        handle.start = 1;
        handle.unlock = 0;
        handle.s_roll = 0.0;
        handle.s_pitch = 0.0;
        handle.s_yaw = 0.0;
        handle.y_speed = 0.0;
        handle.x_speed = 0.0;
        handle.yaw_omega = 0.0;

        handle.z_height = INIT_HEIGHT;
    }

    if (map.l == 1 && map.r == 1 && handle.standReady == 1)
        handle.unlock = 1;

    else if (map.lz == 1 && map.rz == 1 && *p_gaitNow == STAND && handle.unlock == 0)
        *p_gaitNow = LYING;

    else if (map.l == 1 && map.r == 1 && *p_gaitNow == LYING && handle.unlock == 0)
        *p_gaitNow = STAND;


    if (handle.unlock == 1)
    {
        if(map.l == 1 && map.r == 1)
        {
            *p_gaitNow = STOP;
        }
        else
        {
            *p_gaitNow = TROT;
        }    
    }
        

    if (map.minus == 1)
        handle.terrain = FLAT;
    if (map.b == 1)
        handle.terrain = SLOPE;
    if (map.a == 1)
        handle.terrain = STAGE;
    if (map.x == 1)
        handle.terrain = SILL;
    if (map.y == 1)
        handle.terrain = CALANDRIA;  
    if (map.plus == 1)
        handle.terrain = STAIRS;
   
    if (*p_gaitNow == STAND)
    {
        handle.s_roll = map.rx / XBOX_AXIS_VAL_MAX * MAX_CMD_STANCE_ROLL;
        handle.s_pitch = map.ry / XBOX_AXIS_VAL_MAX * MAX_CMD_STANCE_PITCH;
        handle.s_yaw = map.lx / XBOX_AXIS_VAL_MAX * MAX_CMD_STANCE_YAW;
        handle.y_speed = 0.0;
        handle.x_speed = 0.0;
        handle.yaw_omega = 0.0;
        handle.z_height += map.ly / XBOX_AXIS_VAL_MAX * 0.001;

        if (handle.z_height >= MAX_CMD_HEIGHT_Z)
            handle.z_height = MAX_CMD_HEIGHT_Z;
        else if (handle.z_height <= MIN_CMD_HEIGHT_Z)
            handle.z_height = MIN_CMD_HEIGHT_Z;
    }

    else if (*p_gaitNow == LYING)
    {
        handle.s_roll = 0.0;
        handle.s_pitch = 0.0;
        //handle.s_yaw = 0.0;
        handle.y_speed = 0.0;
        handle.x_speed = 0.0;
        handle.yaw_omega = 0.0;
        handle.z_height = INIT_HEIGHT;
    }

    else if (*p_gaitNow == STOP)
    {
        handle.s_roll = 0.0;
        handle.x_speed = 0.0;
        handle.y_speed = 0.0;
        handle.yaw_omega = 0.0;
    }

    else
    {
        handle.s_roll = 0.0;
        handle.s_pitch = 0.0;
        //handle.s_yaw = 0.0;
        handle.y_speed = map.rx / XBOX_AXIS_VAL_MAX * MAX_CMD_VEL_Y;
        handle.x_speed = map.ry / XBOX_AXIS_VAL_MAX * MAX_CMD_VEL_X;
        handle.yaw_omega = map.lx / XBOX_AXIS_VAL_MAX * MAX_CMD_OMEGA_YAW;
        handle.z_height += map.ly / XBOX_AXIS_VAL_MAX * 0.001;

        if (handle.z_height >= MAX_CMD_HEIGHT_Z)
            handle.z_height = MAX_CMD_HEIGHT_Z;
        else if (handle.z_height <= MIN_CMD_HEIGHT_Z)
            handle.z_height = MIN_CMD_HEIGHT_Z;
    }

    fflush(stdout);
}