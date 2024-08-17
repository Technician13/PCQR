
#ifndef __TIME_H
#define __TIME_H

#include <stdint.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include "config.h"

#ifdef ROBOT
    //间隔时间，单位是us
    #define INTERVAL 5000 

    void set_timer();
    void signal_handler(int m);
    void TimeInit(void);

#endif




#endif 

