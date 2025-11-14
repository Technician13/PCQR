#include "parameters.hpp"

#ifndef ROBOT

    #ifndef WEBOTS_API_H
    #define WEBOTS_API_H

    #define WEBOTS_TIME_STEP (5)
    #define MPC_TIME_PERIOD  (3)

    #include <vector>
    #include "cppTypes.h"
    #include "cppStruct.h" 
    #include "commonFunction.hpp"
    #include <string>
    #include </usr/local/webots/include/controller/cpp/webots/Accelerometer.hpp> 
    #include </usr/local/webots/include/controller/cpp/webots/InertialUnit.hpp>
    #include </usr/local/webots/include/controller/cpp/webots/PositionSensor.hpp>
    #include </usr/local/webots/include/controller/cpp/webots/Supervisor.hpp>
    #include </usr/local/webots/include/controller/cpp/webots/TouchSensor.hpp>
    #include </usr/local/webots/include/controller/cpp/webots/Node.hpp>
    #include </usr/local/webots/include/controller/cpp/webots/Motor.hpp>
    #include </usr/local/webots/include/controller/cpp/webots/Keyboard.hpp>
    #include </usr/local/webots/include/controller/cpp/webots/Field.hpp>
    #include </usr/local/webots/include/controller/cpp/webots/Camera.hpp>
    #include </usr/local/webots/include/controller/cpp/webots/Accelerometer.hpp>

    extern webots::Node *existingBodyTraj;
    extern webots::Node *bodyTrajLineSetNode;
    extern webots::Node *bodyTrajCoordinatesNode;
    extern webots::Field *bodyTrajPointField;
    extern webots::Field *bodyTrajCoordIndexField;
    extern int bodyTrajIndex;
    extern bool bodyTrajFirstStep;

    extern webots::Node *existingLFTraj;
    extern webots::Node *lfTrajLineSetNode;
    extern webots::Node *lfTrajCoordinatesNode;
    extern webots::Field *lfTrajPointField;
    extern webots::Field *lfTrajCoordIndexField;
    extern int lfTrajIndex;
    extern bool lfTrajFirstStep;

    extern webots::Node *existingLRTraj;
    extern webots::Node *lrTrajLineSetNode;
    extern webots::Node *lrTrajCoordinatesNode;
    extern webots::Field *lrTrajPointField;
    extern webots::Field *lrTrajCoordIndexField;
    extern int lrTrajIndex;
    extern bool lrTrajFirstStep;

    extern webots::Node *existingRFTraj;
    extern webots::Node *rfTrajLineSetNode;
    extern webots::Node *rfTrajCoordinatesNode;
    extern webots::Field *rfTrajPointField;
    extern webots::Field *rfTrajCoordIndexField;
    extern int rfTrajIndex;
    extern bool rfTrajFirstStep;

    extern webots::Node *existingRRTraj;
    extern webots::Node *rrTrajLineSetNode;
    extern webots::Node *rrTrajCoordinatesNode;
    extern webots::Field *rrTrajPointField;
    extern webots::Field *rrTrajCoordIndexField;
    extern int rrTrajIndex;
    extern bool rrTrajFirstStep;

    void Init(void);
    void WebotsApi(void);
    imu_t GetRobotEuler(void);
    robotLegMsg_t GetRobotJointMsg(void);
    void GetRobotBodyVel(Vec3<double>& p_body_pos,Vec3<double>& p_body_vel);
    void GetRobotBodyAcc(Vec3<double>& p_body_acc);    
    void SetRobotJointTorque(robotLegTorque_t legTorque);
    bool GetTimeFlag(void);
    bool GetTimeFlag(void);
    bool GetTimeFlagMpc(void);
    bool GetTimeFlagMotionPlan(void);

    Vec3<double> GetObstaclePos(std::string p_obstacleName);
    double GetSlopeFirstEdge(void);
    double GetStageFirstEdge(void);
    double GetSillFirstEdge(void);
    double GetCalandriaFirstEdge(void);
    double GetStairsFirstEdge(void);
    double GetSeesawFirstEdge(void);
    double GetBridgeFirstEdge(void);

    void BodyTrajSet(void);
    void LFTrajSet(void);
    void LRTrajSet(void);
    void RFTrajSet(void);
    void RRTrajSet(void);

    #endif // !WEBOTS_API_H

#endif