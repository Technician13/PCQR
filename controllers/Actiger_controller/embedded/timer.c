#include "timer.h"
#ifdef ROBOT

    #define MPC_TIME_PERIOD  3
    uint8_t timeFlag = 0;
    uint8_t mpcTimeFlag = 0;
    uint8_t motionPlanTimeFlag = 0;
    uint8_t mpc_cnt=MPC_PERIOD;

    static struct itimerval oldtv;

    void set_timer()
    {
        struct itimerval itv;
        itv.it_interval.tv_sec = 0;
        itv.it_interval.tv_usec = INTERVAL;
        itv.it_value.tv_sec = 0;
        itv.it_value.tv_usec = INTERVAL;
        setitimer(ITIMER_REAL, &itv, &oldtv);
    }


    void signal_handler(int m)
    {

        timeFlag = 1;
        motionPlanTimeFlag = 1;
        mpc_cnt--;
        if(mpc_cnt == 0)
        {  
            mpc_cnt = MPC_TIME_PERIOD;
            mpcTimeFlag = 1;
        }
            
    }


    void TimeInit(void)
    {
        signal(SIGALRM, signal_handler);
        set_timer();

    }

#endif

