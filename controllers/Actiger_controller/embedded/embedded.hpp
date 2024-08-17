#include "parameters.hpp"

#ifdef ROBOT

    #ifndef __EMBEDDED_HPP
    #define __EMBEDDED_HPP

    #include "mcucomm.hpp"
    #include "imu.hpp"
    #include "cppStruct.h"
    #include <thread>
    #include <sched.h>
    #include <pthread.h> 
    #include <sys/time.h>
    #include <sys/sem.h>
    #include <semaphore.h>
    #include <iostream>

    extern "C" {

    #include <errno.h>
    #include <unistd.h>
    #include "timer.h"
        //进程通讯头文件
    //#include "process_comm.h"

    }
    
    void Init(void);
    void Embedded(void);
    void ConfigTask(void);
    int8_t ThreadBindCPU(void);
    imu_t GetRobotEuler(void);
    robotLegMsg_t GetRobotJointMsg(void);
    void SetRobotJointTorque(robotLegTorque_t legTorque);
    bool GetTimeFlag(void);
    bool GetTimeFlagMpc(void);
    bool GetTimeFlagMotionPlan(void);
    #endif

#endif