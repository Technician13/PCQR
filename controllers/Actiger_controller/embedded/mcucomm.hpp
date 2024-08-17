#include "parameters.hpp"
#ifdef ROBOT

	#ifndef _MCUCOMM_H
	#define _MCUCOMM_H

	#include <iostream>
	#include "cppStruct.h"

	extern "C" 
	{

		#include <stdlib.h>
		#include <stdint.h>
		#include <unistd.h>
		#include <stdarg.h>
		#include "timer.h"
		#include "ControlSPI.h"
	}

	#define MCU_SPI_LENGTH (105)
	#define MCU_SPI_UINT8_CNT (4)
	#define MCU_SPI_FLOAT_CNT (24)



	#define dbg( format, arg...)	printf( format "\n", ##arg );
	#define err( format, arg... )	\
		printf( "error %d: " format "\n", __LINE__, ##arg )

	typedef union
	{
		uint8_t data8[4];
		int32_t data32;
		float dataf;
	}transData_t; 


	/******************С���Ժ�����֮��*************************/
	typedef struct
	{
		legMsg_t lf;
		legMsg_t lr;
		legMsg_t rf;
		legMsg_t rr;

	}slaveMsg_t;

	typedef struct
	{	
		legTorque_t lf_torque;
		legTorque_t lr_torque;
		legTorque_t rf_torque;
		legTorque_t rr_torque;
		uint8_t blank[52];

	}hostMsg_t;

	extern int dev_fd;

	class Comm
	{
	public:
		slaveMsg_t slaveMsg = { 0 };
		hostMsg_t hostMsg = { 0 };

		explicit Comm(void);
		virtual ~Comm(void);
		void WaitMcuPrepare(void);
		void Communicate2Mcu(void);
		void WriteSpiData(uint8_t* data);
		void ReadSpiData(uint8_t* data);
		void SPISendAndRcvData(uint8_t* p_tx,uint8_t* p_rx,uint32_t p_lenth);
	protected:


	private:
		//�� mcu master������Ϣ  
		uint8_t tx[MCU_SPI_LENGTH] = { 0 };
		//���� mcu master ����Ϣ
		uint8_t rx[MCU_SPI_LENGTH] = { 0 };

		//ȥ����ֹ������Ϣ
		uint8_t txTransData[MCU_SPI_LENGTH - 5] = { 0 };
		uint8_t rxTransData[MCU_SPI_LENGTH - 5] = { 0 };

		int init_device( void );


	};


	#endif // _MCUCOMM_H
	
#endif