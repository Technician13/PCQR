#ifndef FLOATINGBASE_MODEL_HPP
#define FLOATINGBASE_MODEL_HPP

#include <rbdl/rbdl.h>
#include "common_convert_calcu.hpp"
#include "DynModel.hpp"
#include "KinModel.hpp"
namespace quadruped
{
    class FloatingBaseModel{
    public:
        FloatingBaseModel();
        ~FloatingBaseModel(void);

        bool getMassInertia(dynacore::Matrix & A) const ;
        bool getInverseMassInertia(dynacore::Matrix & Ainv) const; 
        bool getGravity(dynacore::Vector & grav) const;
        bool getCoriolis(dynacore::Vector & coriolis) const;
        void getCentroidJacobian(dynacore::Matrix & Jcent) const;
        void getCentroidInertia(dynacore::Matrix & Icent) const;
        void getCoMPosition(dynacore::Vect3 & com_pos) const;
        void getCoMVelocity(dynacore::Vect3 & com_vel) const;
        void getPos(int link_id, dynacore::Vect3 & pos) const;
        void getOri(int link_id, dynacore::Vect3 & rpy) const;
        void getLinearVel(int link_id, dynacore::Vect3 & lin_vel) const;
        void getAngularVel(int link_id, dynacore::Vect3 & ang_vel) const;
        void getCentroidVelocity(dynacore::Vector & centroid_vel) const;
        void getCoMJacobian(dynacore::Matrix & J) const;
        void getFullJacobian(int link_id, dynacore::Matrix & J) const;
        void getFullJDotQdot(int link_id, dynacore::Vector & JDotQdot) const;
        void getWorld2BodyMatrix(dynacore::Mat3 & _World2Body);

        void UpdateSystem(const dynacore::Vector & q, const dynacore::Vector & qdot);
        void PrintLinkList();
        unsigned int FindLinkId(const char* link_name);
        RigidBodyDynamics::Model* model_;
    protected:
        DynModel* dyn_model_;
        KinModel* kin_model_;

        
    };
}



#endif
