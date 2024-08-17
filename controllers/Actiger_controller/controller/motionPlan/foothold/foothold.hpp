#ifndef FOOTHOLD_HPP
#define FOOTHOLD_HPP

#include "fH_nlp.hpp"
#include "fH_updataInfo.hpp"
#include "IpIpoptApplication.hpp"

namespace quadruped
{
    
    class Foothold
    {
        public:

            explicit Foothold(Parameters* p_para,Terrain* p_terrain_now);
            virtual ~Foothold(void);

            void GetFootHold(void);

            FH_updataInfo* updataInfo;

            SmartPtr<TNLP> fH_nlp; 

        private:
            
            SmartPtr<IpoptApplication> app;
            ApplicationReturnStatus status;




    };


}
















#endif

