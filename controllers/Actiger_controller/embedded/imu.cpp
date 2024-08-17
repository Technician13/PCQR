#include "imu.hpp"
#ifdef ROBOT

    pthread_mutex_t mtx;

    /*ms级延时*/
    void Delay_ms(int time) 
    { 
        struct timespec start_spec;
        struct timespec current_spec;
        int msec_time = 0;
        clock_gettime(CLOCK_REALTIME,&start_spec);
        //CLOCK计数单位是us
        while(msec_time < time)
        {
            clock_gettime(CLOCK_REALTIME,&current_spec);
            msec_time =
            (current_spec.tv_sec - start_spec.tv_sec) * 1000
            + (current_spec.tv_nsec - start_spec.tv_nsec) / 1000000;
        }; 
    } 

    Imu::Imu(void)
    {

    }

    Imu::~Imu(void)
    {

    }

    void Imu::SerialInit(int& fd, char* port, int baudrate)
    {
        std::cout << "init " << port << " ...\n";
        fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
        std::cout << fd << std::endl;
        if (fd == -1)
        {
            perror("open_port: Unable to open\n");
            return ;
        }
        tcgetattr(fd, &termAttr);
        //设置波特率
        cfsetispeed(&termAttr, baudrate);
        cfsetospeed(&termAttr, baudrate);
        //设置无奇偶校验 8位
        termAttr.c_cflag &= ~PARENB;
        termAttr.c_cflag &= ~CSTOPB;
        termAttr.c_cflag &= ~CSIZE;
        termAttr.c_cflag |= CS8;

        termAttr.c_cflag |= (CLOCAL | CREAD);
        termAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        termAttr.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | ISTRIP);
        termAttr.c_oflag &= ~OPOST;
        termAttr.c_cc[VMIN] = 0;
        termAttr.c_cc[VTIME] = 1;
        tcflush(fd, TCIOFLUSH);
        tcsetattr(fd, TCSANOW, &termAttr);
        std::cout << port << ": serial init ok....\n";

    }

    void ReceiveData(int fd, uint8_t imuTalkOk, uint8_t imuReadyFlag, imu_t *imuData)
    {
        union
        {
            uint8_t data8[GET_DATA_LEN];
            float dataf[GET_DATA_LEN / 4];
        }rData;
        //struct pollfd fdset[2];
        //int ret;
        //fdset[0].fd = fd;
        //int rc, timeOut = 1;
        //fdset[0].events = POLLIN;
        tcflush(fd, TCIOFLUSH);
        int count = 0;
        static uint8_t dataCnt = 0;
        int nread;
        char ch;

        while (1)
        {
            
            while ((nread = read(fd, &ch, 1)) > 0)
            {

                switch (count)
                {
                case 0:
                    if (ch == 0x0d || ch == 'O')
                    {
                        count++;
                    }
                    else
                        count = 0;
                    break;

                case 1:
                    if (ch == 0x0a)
                    {
                        dataCnt = 0;
                        count++;
                    }
                    else if (ch == 'K')
                    {
                        imuTalkOk = 1;
                        count = 0;
                    }
                    else
                        count = 0;
                    break;

                case 2:
                    rData.data8[dataCnt] = ch;
                    dataCnt++;
                    if (dataCnt >= GET_DATA_LEN)
                    {
                        dataCnt = 0;
                        count++;
                    }
                    break;

                case 3:
                    if (ch == 0x0a)
                        count++;
                    else
                        count = 0;
                    break;

                case 4:
                    if (ch == 0x0d)
                    {
                        
                        //一帧数据接受完成
                        if ((!imuReadyFlag) && (fabs(rData.dataf[0]) > 0.0001))
                        {
                            imuReadyFlag = 1;
                        }
                        if (imuReadyFlag)
                        {
                            
                            pthread_mutex_lock(&mtx); //上锁
                        //std::cout<<"88888888888"<<std::endl;
                            (*imuData).roll=-rData.dataf[0]*PI/180.0;
                            (*imuData).pitch = rData.dataf[1]*PI/180.0;
                            (*imuData).yaw = rData.dataf[2]*PI/180.0;
                            (*imuData).omega_roll = -rData.dataf[3]*PI/180.0;
                            (*imuData).omega_pitch =rData.dataf[4]*PI/180.0;
                            (*imuData).omega_yaw = rData.dataf[5]*PI/180.0;


                            pthread_mutex_unlock(&mtx); //解锁
                        }
                    }
                    count = 0;
                    break;
                default:
                    count = 0;
                    break;
                }
            }
        }
    }


    /*测试与定位系统通信是否正常
    给定位系统发送 AT 返回OK
    @param fd 发送时使用的串口号
    */
    void Imu::TalkToImu(int fd)
    {
        uint8_t tdata[4];
        tdata[0] = 'A';
        tdata[1] = 'T';
        tdata[2] = '\r';
        tdata[3] = '\n';

        imuTalkOk = 0;
        while (!imuTalkOk)
        {
            Delay_ms(2);
            write(fd, tdata, 4);
        }
        std::cout << "imu connect ok" << std::endl;
    }
    /*等待定位系统初始化*/
    void Imu::WaitImuReady(int fd)
    {
        TalkToImu(fd);
        while (!imuReadyFlag) {}
    }

    void Imu::printImuInfo(imu_t imuData_input)
    {
        // std::cout<<"-------------------------------------------------------------------------"<<std::endl;
        
        // pthread_mutex_lock(&mtx); //上锁
        // std::cout<<" --imuData.pitch : "<<imuData_input.pitch<<std::endl;
        // std::cout<<" --imuData.roll : "<<imuData_input.roll<<std::endl;
        // std::cout<<" --imuData.yaw : "<<imuData_input.yaw<<std::endl;

        // std::cout<<" --imuData.omega_pitch : "<<imuData_input.omega_pitch<<std::endl;
        // std::cout<<" --imuData.omega_roll : "<<imuData_input.omega_roll<<std::endl;
        // std::cout<<" --imuData.omega_yaw : "<<imuData_input.omega_yaw<<std::endl;
        // pthread_mutex_unlock(&mtx); //解锁

        
    }


#endif


