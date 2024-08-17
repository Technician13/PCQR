#include "parameters.hpp"

#ifdef ROBOT

	#ifndef _IMU_H
	#define _IMU_H
	#include <atomic>
	#include <ctime> 
	#include <stdio.h>
	#include <string.h>
	#include <sys/types.h>
	#include <errno.h>
	#include <sys/stat.h>
	#include <fcntl.h>
	#include <unistd.h>
	#include <termios.h>
	#include <stdlib.h>
	#include <signal.h>
	#include <string.h>
	#include <stdarg.h>
	#include <iostream>
	#include <cmath>
	#include "cppStruct.h"	

	extern "C" 
	{
		#include <stdint.h>
		#include "timer.h"
		
	}
	
	#define GET_DATA_LEN (24)

	void ReceiveData(int fd, uint8_t imuTalkOk, uint8_t imuReadyFlag, imu_t* imuData);
	class Imu
	{
	public:
		uint8_t imuTalkOk = 0;
		uint8_t imuReadyFlag = 0;
		imu_t imuData = {0};
		explicit Imu();
		virtual ~Imu();
		void SerialInit(int& fd, char* port, int baudrate);
		
		void TalkToImu(int fd);
		void WaitImuReady(int fd);

		//TEST*****************************************************************************
		void printImuInfo(imu_t imuData_input);
		//*********************************************************************************

	protected:


	private:
		struct termios termAttr;
		struct sigaction saio;
		int ifSerial = 0;

	};

	#endif

#endif