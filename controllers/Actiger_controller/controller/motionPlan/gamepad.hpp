/**
 * @file gamepad.hpp
 * @brief Actiger遥控器类Gamepad头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-19
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef GAMEPAD_HPP
#define GAMEPAD_HPP

#include <iostream>
#include <unistd.h>  
#include <string.h>  
#include <sys/types.h>  
#include <sys/stat.h>  
#include <sys/time.h>
#include <fcntl.h>  
#include <errno.h>  
#include <linux/input.h>  
#include <linux/joystick.h>  
#include "parameters.hpp"

namespace quadruped
{
    /**
     * @brief Actiger遥控器类
     */
    class Gamepad
    {
    private:
        int xbox_fd ;        
        fd_set rfds;
        struct timeval tv;

    protected:

    public:   
        explicit Gamepad(void) ;
        virtual ~Gamepad(void);

        int xbox_open(const char *file_name);
        int xbox_map_read(int xbox_fd, xbox_map_t *p_map) ;
        void xbox_close(int xbox_fd);
        void GetKey(uint8_t* p_gaitNow);

        /** @brief 遥控器键值映射结构体成员 */
        xbox_map_t map;
    };
}

#endif // !GAMEPAD_HPP