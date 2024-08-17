 #include "embedded.hpp"
 
#ifdef ROBOT

    
    int imuSerial = 0;
    extern pthread_mutex_t mtx;
	extern uint8_t timeFlag ;
    extern uint8_t mpcTimeFlag;
    extern uint8_t motionPlanTimeFlag;
    Imu* imu = new Imu();

    Comm* comm = new Comm();

    void Init(void)
    {
        /*初始化*/
        std::cout << "init .." << std::endl;

        ThreadBindCPU();

        // imu->SerialInit(imuSerial, (char*)"/dev/ttyUSB0", B115200);

        // //开启子线程用于串口收数
        // std::thread serialThread(ReceiveData, imuSerial, imu->imuTalkOk, imu->imuReadyFlag, &imu->imuData);
        // //while(1);
        // serialThread.detach();

        // std::cout << "waiting for spi ..." << std::endl;
        // //等待MCU初始化
        // // comm->WaitMcuPrepare();
        
        //初始化定时器
        TimeInit();

        std::cout << "init done!!!" << std::endl;

    }
    void Embedded(void)
    {
        //与mcu通信
			comm->Communicate2Mcu();
    }

    imu_t GetRobotEuler(void)
    {
        imu_t return_imu={0};
        pthread_mutex_lock(&mtx); //上锁
        return_imu = imu->imuData;
        pthread_mutex_unlock(&mtx); //解锁

        return return_imu;
    }

    robotLegMsg_t GetRobotJointMsg(void)
    {
        robotLegMsg_t return_legMsg={0};

        return_legMsg.lf=comm->slaveMsg.lf;
        return_legMsg.lr=comm->slaveMsg.lr;
        return_legMsg.rf=comm->slaveMsg.rf;
        return_legMsg.rr=comm->slaveMsg.rr;

        return return_legMsg;
    }

    void SetRobotJointTorque(robotLegTorque_t p_legTorque)
    {
        comm->hostMsg.lf_torque=p_legTorque.lf_torque;
        comm->hostMsg.lr_torque=p_legTorque.lr_torque;
        comm->hostMsg.rf_torque=p_legTorque.rf_torque;
        comm->hostMsg.rr_torque=p_legTorque.rr_torque;

    }

	//获取5ms周期标志位
	bool GetTimeFlag(void)
	{
		if(timeFlag == 1)
		{
            
			timeFlag=0;
			return 1;
		}
		else
			return 0;
		
	}

    
	//获取5ms周期标志位
	bool GetTimeFlag(void)
	{
		if(timeFlag == 1)
		{
            timeCnt_mpc++;
			timeFlag=0;
			return 1;
		}
		else
			return 0;
		
	}

    bool GetTimeFlagMpc(void)
    {
        if(mpcTimeFlag == 1)
        {
            mpcTimeFlag = 0;
            return 1;
        }
        
        return 0;

    }

    bool GetTimeFlagMotionPlan(void)
    {
        if(motionPlanTimeFlag == 1)
        {
            motionPlanTimeFlag = 0;
            return 1;
        }
        
        return 0;

    }

    int8_t ThreadBindCPU(void)
    {
        int cpus = 0;
        cpu_set_t mask;

        cpus = sysconf(_SC_NPROCESSORS_CONF);
        std::cout << "cpus: " << cpus << std::endl;

        CPU_ZERO(&mask);    /* 初始化set集，将set置为空*/
        CPU_SET(0, &mask);  /* 依次将0、1、2、3号cpu加入到集合，前提是你的机器是多核处理器*/

        /*设置cpu 亲和性（affinity）*/
        if (sched_setaffinity(0, sizeof(mask), &mask) == -1) {
            std::cout << "Set CPU affinity failue, ERROR: "<< strerror(errno) << std::endl;
            return -1;
        }
        usleep(1000); /* 让当前的设置有足够时间生效*/

        return 0;

    }

#endif