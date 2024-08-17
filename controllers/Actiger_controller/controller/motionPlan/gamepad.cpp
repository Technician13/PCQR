/**
 * @file gamepad.hpp
 * @brief Actiger遥控器类Gamepad
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-19
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "gamepad.hpp"
#include <iostream>

using namespace std;
using namespace quadruped;

/**
 * @brief Gamepad类的构造函数，完成硬件的读取
 */
Gamepad::Gamepad(void)
{
    memset(&map, 0, sizeof(xbox_map_t));
    xbox_fd = xbox_open("/dev/input/js0");
    if (xbox_fd < 0)
    {
        cout << "gamepad init error!" << endl;
    }
}

/**
 * @brief Gamepad类的析构函数
 */
Gamepad::~Gamepad(void) {}

/**
 * @brief 打开遥控器
 * @param  file_name  设备文件      
 * @return int 
 */
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

/**
 * @brief 读取遥控器按键状态并完成键值映射
 * @param  xbox_fd  设备文件        
 * @param  p_map  键值映射结构体          
 * @return int 
 */
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
        // std::cout << "-----------------JS_EVENT_BUTTON----------------" << std::endl;
        // std::cout << number << std::endl;
        // std::cout << "------------------------------------------------" << std::endl;
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
        // std::cout << "------------------JS_EVENT_AXIS-----------------" << std::endl;
        // std::cout << number << std::endl;
        // std::cout << "------------------------------------------------" << std::endl;
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

/**
 * @brief 关闭遥控器
 * @param  xbox_fd  设备文件        
 */
void Gamepad::xbox_close(int xbox_fd)
{
    close(xbox_fd);
    return;
}

/**
 * @brief 根据遥控器映射的键值完成状态标志位的转换
 * @param  p_gaitNow  Actiger当前的步态      
 */
void Gamepad::GetKey(uint8_t *p_gaitNow)
{
    int len;
    len = xbox_map_read(xbox_fd, &map);

    /* 如果手柄的home建被按下 */
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

        /* 设定站立高度为0.5m */
        handle.z_height = INIT_HEIGHT;
    }

    /* 解锁 */
    if (map.l == 1 && map.r == 1 && handle.standReady == 1)
        handle.unlock = 1;

    /* 从静止站立转换到趴下 */
    else if (map.lz == 1 && map.rz == 1 && *p_gaitNow == STAND && handle.unlock == 0)
        *p_gaitNow = LYING;

    /* 从趴下转换到静止站立 */
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
        

   /* 地形相关 */
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
   
    // if (map.b == 1)
    //     handle.terrain = SEESAW;
    // if (map.y == 1)
    //     handle.terrain = HURDLES;
    // if (map.plus == 1)
    //     handle.terrain = BALL_POOL;
    // if (map.minus == 1)
    //     handle.terrain = BRIDGE;

    /* 如果当前状态为静止站立 */
    if (*p_gaitNow == STAND)
    {
        /* 调整机体静态状态
         * 右摇杆横向 ： 机体横滚变化（-30deg ~ 30deg）
         * 右摇杆纵向 ： 机体俯仰变化（-30deg ~ 30deg）
         * 左摇杆横向 ： 机体偏航变化（-30deg ~ 30deg） 
         * 左摇杆纵向 ： 机体高度变化*/
        handle.s_roll = map.rx / XBOX_AXIS_VAL_MAX * MAX_CMD_STANCE_ROLL;
        handle.s_pitch = map.ry / XBOX_AXIS_VAL_MAX * MAX_CMD_STANCE_PITCH;
        handle.s_yaw = map.lx / XBOX_AXIS_VAL_MAX * MAX_CMD_STANCE_YAW;
        handle.y_speed = 0.0;
        handle.x_speed = 0.0;
        handle.yaw_omega = 0.0;
        handle.z_height += map.ly / XBOX_AXIS_VAL_MAX * 0.001;

        /* 机体高度限幅 */
        if (handle.z_height >= MAX_CMD_HEIGHT_Z)
            handle.z_height = MAX_CMD_HEIGHT_Z;
        else if (handle.z_height <= MIN_CMD_HEIGHT_Z)
            handle.z_height = MIN_CMD_HEIGHT_Z;
    }

    /* 如果当前状态为趴下 */
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

    /* 其他情况，即有步态执行的情况 */
    else
    {
        handle.s_roll = 0.0;
        handle.s_pitch = 0.0;
        //handle.s_yaw = 0.0;
        handle.y_speed = map.rx / XBOX_AXIS_VAL_MAX * MAX_CMD_VEL_Y;
        handle.x_speed = map.ry / XBOX_AXIS_VAL_MAX * MAX_CMD_VEL_X;
        handle.yaw_omega = map.lx / XBOX_AXIS_VAL_MAX * MAX_CMD_OMEGA_YAW;
        handle.z_height += map.ly / XBOX_AXIS_VAL_MAX * 0.001;

        /*机体高度限幅*/
        if (handle.z_height >= MAX_CMD_HEIGHT_Z)
            handle.z_height = MAX_CMD_HEIGHT_Z;
        else if (handle.z_height <= MIN_CMD_HEIGHT_Z)
            handle.z_height = MIN_CMD_HEIGHT_Z;
    }

    fflush(stdout);
}