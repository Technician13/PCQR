#include "mcucomm.hpp"
#ifdef ROBOT


    Comm::Comm(void)
    {
        int retval = 0;
        retval = init_device();
        if( retval == -1 )
        {	
            printf("Init device error\n");
        }
    }



    Comm::~Comm(void)
    {

    }

    /*
    * ********************************************************************
    * init_device( void )
    *
    * Function : open device, Get Driver Version, Get Chip ID
    * ********************************************************************
    */
    int Comm::init_device( void )
    {
        // int retval = 0;      
        // dev_fd = CH34xOpenDevice( 3 );
        // if( dev_fd <= 0 )
        // {
        //     err("CH34xOpenDevice Open Error");
        //     return -1;		
        // }	
        // dbg( "Open sucessful" );
        // //Get Driver Version
        // char *drv_version = (char *)malloc( sizeof( char ) * 30 );       
        // retval = CH34x_GetDrvVer( drv_version );
        // if( retval == false )
        // {
        //     err( "CH34x_GetDrvVer Error" );
        //     return -1;
        // }       
        // printf( "drv_version : %s\n", drv_version );
        // free( drv_version );
        // //Get Chip Version
        // PULONG VendorId = (PULONG)malloc( sizeof(unsigned long));
        // retval = CH34x_GetVendorId( VendorId );
        // if( retval == false )
        // {
        //     err( "CH34x_GetVendorId Error" );
        //     return -1;
        // }	
        // printf( "VendorId : 0x%04x\n", (int)*VendorId );
        // free( VendorId );
        // return true;

        int ret;
        VSI_INIT_CONFIG SPI_Config;
        VSI_BOARD_INFO BoardInfo;
        ret = VSI_ScanDevice(1);
        if (ret <= 0)
        {
            std::cout<<"No device connect!"<<std::endl;
            return ret;
        }
        // Open device
        ret = VSI_OpenDevice(VSI_USBSPI, 0, 0);
        if (ret != ERR_SUCCESS)
        {
            std::cout<<"Open device error!"<<std::endl;
            return ret;
        }
        
        // Initialize device(Master Mode, Hardware SPI, Half-Duplex)
        SPI_Config.ControlMode = 1;
        SPI_Config.MasterMode = 1;
        SPI_Config.ClockSpeed = 36000000;
        SPI_Config.CPHA = 0;
        SPI_Config.CPOL = 0;
        SPI_Config.LSBFirst = 0;
        SPI_Config.TranBits = 8;
	    SPI_Config.SelPolarity = 0;
        ret = VSI_InitSPI(VSI_USBSPI, 0, &SPI_Config);
        if (ret != ERR_SUCCESS)
        {
            std::cout<<"Initialize device error!"<<std::endl;
            return ret;
        }
    }

    void Comm::SPISendAndRcvData(uint8_t* p_tx,uint8_t* p_rx,uint32_t p_lenth)
    {
        // ULONG iChipselect = 0x80
        // PUCHAR buf = (PUCHAR)malloc(sizeof(UCHAR)*p_lenth);
        // for(int i=0;i<p_lenth;i++)
        //     buf[i]=p_tx[i];
        // if( CH34xStreamSPI4( iChipselect, p_lenth, buf ) == false )
        //     std::cout<<"SPI FAULSE !!!" <<std::endl;
        // else
        // {
        //     for(int i=0;i<p_lenth;i++)
        //         p_rx[i]=buf[i];
        // }
        // free(buf);


        int ret;
        uint8_t write_buffer[10240];
        uint8_t read_buffer[10240];
        for (int i = 0; i < p_lenth; i++)
        {
            write_buffer[i] = p_tx[i];
        }
        ret = VSI_WriteReadBytes(VSI_USBSPI, 0, 0, write_buffer, p_lenth, read_buffer, p_lenth);
        if (ret != ERR_SUCCESS)
        {
            std::cout<<"WriteRead data error!!!"<<std::endl;
        }
        else
        {
            for (int i = 0; i < p_lenth; i++)
            {
                p_rx[i]=read_buffer[i];
            }
        }
    }


    static uint8_t mcuTalkOk = 0;
    void Comm::WaitMcuPrepare(void)
    {
        tx[0] = 'A';
        tx[1] = 'T';
        tx[MCU_SPI_LENGTH-3] = '\r';
        tx[MCU_SPI_LENGTH-2] = '\n';
        tx[MCU_SPI_LENGTH-1] = 0;
        // std::cout<<"8888888888888888888888"<<std::endl;
        while(!mcuTalkOk)  
        {
            //要延时5ms
            // std::cout<<"before "<<rx[0]<<rx[1]<<rx[MCU_SPI_LENGTH - 3]<<rx[MCU_SPI_LENGTH - 2];
            usleep(4000);
            // std::cout<<"after "<<rx[0]<<rx[1]<<rx[MCU_SPI_LENGTH - 3]<<rx[MCU_SPI_LENGTH - 2];
            //向mcu发送AT并读回mcu的消息
            SPISendAndRcvData(tx,rx,MCU_SPI_LENGTH);

            for(int i=0;i<MCU_SPI_LENGTH;i++)
            {

                std::cout<<(int)rx[i]<< " ";


            }
            std::cout<<std::endl;
            //如果mcu回复OK 通信成功
            if(rx[0] == 'O' && rx[1] == 'K' && rx[MCU_SPI_LENGTH - 3] == '\r' && rx[MCU_SPI_LENGTH - 2] == '\n')
            {
                mcuTalkOk = 1;
                for(int i = 0; i < (MCU_SPI_LENGTH - 5); i ++)
                {
                    rxTransData[i] = rx[i+2];
                }
                ReadSpiData(rxTransData);
            }
            //将处理完的接收数组发出并清空
            for(uint8_t i = 0; i < MCU_SPI_LENGTH; i++)
            {
                rx[i] = 0;
            }
        }
    }

    void Comm::Communicate2Mcu(void)
    {
        clock_t time_start,time_end;
        uint8_t spiShiftFlag = 0;
        tx[0] = 'H';
        tx[1] = 'M';
        tx[MCU_SPI_LENGTH - 3] = '\r';
        tx[MCU_SPI_LENGTH - 2] = '\n';
        tx[MCU_SPI_LENGTH - 1] = 0;


        WriteSpiData(txTransData);

        for(int i = 0; i < (MCU_SPI_LENGTH - 5); i ++)
        {
            tx[i+2] = txTransData[i];
        }
        
        time_start=clock();
        //与mcu通信
        SPISendAndRcvData(tx,rx,MCU_SPI_LENGTH);
        time_end=clock();
        std::cout<< "-----------------------TIME--------------------"<<std::endl;
            std::cout<<time_end-time_start<<std::endl;
          for(int i=0;i<MCU_SPI_LENGTH;i++)
            {

                std::cout<<(int)rx[i]<< " ";


            }
            std::cout<<std::endl;
        if(rx[0] == 'S' && rx[1] == 'M' && rx[MCU_SPI_LENGTH-3] == '\r' && rx[MCU_SPI_LENGTH-2] == '\n')
        {
            //gRobot.mcuHeart = 0;
            spiShiftFlag  = 0;
            for(int i = 0; i < (MCU_SPI_LENGTH - 5); i ++)
            {
                rxTransData[i] = rx[i+2];
            }
            ReadSpiData(rxTransData);
        }
        else if(rx[1] == 'S' && rx[2] == 'M' && rx[MCU_SPI_LENGTH-2] == '\r' && rx[MCU_SPI_LENGTH-1] == '\n')
        {

            spiShiftFlag  = 1;
            for(int i = 0; i < (MCU_SPI_LENGTH - 5); i ++)
            {
                rxTransData[i] = rx[i+3];
            }
            ReadSpiData(rxTransData);
        }


        for(uint8_t i = 0; i < MCU_SPI_LENGTH; i++)
        {
            rx[i] = 0;
        }
    }

    void Comm::WriteSpiData(uint8_t *data)
    {
        union trans_t
        {
            float dataf[12];
            uint8_t data8[48];
        }trans;
        
        trans.dataf[0] = hostMsg.lf_torque.shoulder_torque;
        trans.dataf[1] = hostMsg.lf_torque.hip_torque;
        trans.dataf[2] = hostMsg.lf_torque.knee_torque;

        trans.dataf[3] = hostMsg.lr_torque.shoulder_torque;
        trans.dataf[4] = hostMsg.lr_torque.hip_torque;
        trans.dataf[5] = hostMsg.lr_torque.knee_torque;

        trans.dataf[6] = hostMsg.rf_torque.shoulder_torque;
        trans.dataf[7] = hostMsg.rf_torque.hip_torque;
        trans.dataf[8] = hostMsg.rf_torque.knee_torque;

        trans.dataf[9] = hostMsg.rr_torque.shoulder_torque;
        trans.dataf[10] = hostMsg.rr_torque.hip_torque;
        trans.dataf[11] = hostMsg.rr_torque.knee_torque;
        //*********************************************************************************
        
        for (int j = 0; j < 48; j++)
        {
            data[j] = trans.data8[j];
        }
    }


    void Comm::ReadSpiData(uint8_t* data)
    {
        union trans_t
        {
            float dataf[6];
            uint8_t data8[24];
        }trans;

        legMsg_t leg[4] = {0};

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 24; j++)
            {
                trans.data8[j] = data[j+i*25];
            }
            leg[i].shoulder_pos = trans.dataf[0];
            leg[i].shoulder_vel = trans.dataf[1];

            leg[i].hip_pos = trans.dataf[2];
            leg[i].hip_vel = trans.dataf[3];

            leg[i].knee_pos = trans.dataf[4];
            leg[i].knee_vel = trans.dataf[5];

            leg[i].contact = data[(i+1) * 25 - 1];

            //TEST*****************************************************************************
            // std::cout<<" --leg[ "<<i<<" ].shoulder_pos ： "<<trans.dataf[0]<<std::endl;
            // std::cout<<" --leg[ "<<i<<" ].shoulder_vel ： "<<trans.dataf[1]<<std::endl;

            // std::cout<<" --leg[ "<<i<<" ].hip_pos ： "<<trans.dataf[2]<<std::endl;
            // std::cout<<" --leg[ "<<i<<" ].hip_vel ： "<<trans.dataf[3]<<std::endl;

            // std::cout<<" --leg[ "<<i<<" ].knee_pos ： "<<trans.dataf[4]<<std::endl;
            // std::cout<<" --leg[ "<<i<<" ].knee_vel ： "<<trans.dataf[5]<<std::endl;

            // std::cout<<" --leg[ "<<i<<" ].contact ： "<<(int)leg[i].contact<<std::endl;
            //*********************************************************************************

        }
        // std::cout<<"-------------------------------------------------------------------------"<<std::endl;


        slaveMsg.lf = leg[0];
        slaveMsg.lr = leg[1];
        slaveMsg.rf = leg[2];
        slaveMsg.rr = leg[3];

    }

#endif