#include "webotsApi.hpp"
#include <iostream>

#ifndef ROBOT

    using namespace webots;

    uint8_t mpcTimeFlag = 0;
    uint8_t motionPlanTimeFlag = 0;
    uint8_t mpc_cnt = MPC_PERIOD;

    std::string obstacle[7] = {"slope", "stage", "sill", "calandria", "stairs", "seesaw_support", "bridge"};

    Supervisor *supervisor = new Supervisor();
    Node *Root_node = supervisor->getRoot();
    Node *Robot_node = supervisor->getFromDef("quadruped");
    Field *rootChildrenField = Root_node->getField("children");

    Node *existingBodyTraj;
    Node *bodyTrajLineSetNode;
    Node *bodyTrajCoordinatesNode;
    Field *bodyTrajPointField;
    Field *bodyTrajCoordIndexField;
    int bodyTrajIndex;
    bool bodyTrajFirstStep;

    webots::Node *existingLFTraj;
    webots::Node *lfTrajLineSetNode;
    webots::Node *lfTrajCoordinatesNode;
    webots::Field *lfTrajPointField;
    webots::Field *lfTrajCoordIndexField;
    int lfTrajIndex;
    bool lfTrajFirstStep;

    webots::Node *existingLRTraj;
    webots::Node *lrTrajLineSetNode;
    webots::Node *lrTrajCoordinatesNode;
    webots::Field *lrTrajPointField;
    webots::Field *lrTrajCoordIndexField;
    int lrTrajIndex;
    bool lrTrajFirstStep;

    webots::Node *existingRFTraj;
    webots::Node *rfTrajLineSetNode;
    webots::Node *rfTrajCoordinatesNode;
    webots::Field *rfTrajPointField;
    webots::Field *rfTrajCoordIndexField;
    int rfTrajIndex;
    bool rfTrajFirstStep;

    webots::Node *existingRRTraj;
    webots::Node *rrTrajLineSetNode;
    webots::Node *rrTrajCoordinatesNode;
    webots::Field *rrTrajPointField;
    webots::Field *rrTrajCoordIndexField;
    int rrTrajIndex;
    bool rrTrajFirstStep;

    std::string sensor_lf[4] = {"touch_4", "LF_pos_1", "LF_pos_2", "LF_pos_3"};
    std::string sensor_lr[4] = {"touch_3", "LR_pos_1", "LR_pos_2", "LR_pos_3"};
    std::string sensor_rf[4] = {"touch_2", "RF_pos_1", "RF_pos_2", "RF_pos_3"};
    std::string sensor_rr[4] = {"touch_1", "RR_pos_1", "RR_pos_2", "RR_pos_3"};

    std::string mototr_lf[3] = {"LF_motor_1", "LF_motor_2", "LF_motor_3"};
    std::string mototr_lr[3] = {"LR_motor_1", "LR_motor_2", "LR_motor_3"};
    std::string mototr_rf[3] = {"RF_motor_1", "RF_motor_2", "RF_motor_3"};
    std::string mototr_rr[3] = {"RR_motor_1", "RR_motor_2", "RR_motor_3"};

    static InertialUnit* imu_webots = new InertialUnit("imu");
    static Camera* camera_webots = new Camera("camera");
    static Accelerometer* accelerometer_webots = new Accelerometer("acc");

    static PositionSensor *lf_pos_shouder = new PositionSensor(sensor_lf[1]);
    static PositionSensor *lf_pos_hip = new PositionSensor(sensor_lf[2]);
    static PositionSensor *lf_pos_knee = new PositionSensor(sensor_lf[3]);
    static TouchSensor *lf_touch = new TouchSensor(sensor_lf[0]);

    static PositionSensor *lr_pos_shouder = new PositionSensor(sensor_lr[1]);
    static PositionSensor *lr_pos_hip = new PositionSensor(sensor_lr[2]);
    static PositionSensor *lr_pos_knee = new PositionSensor(sensor_lr[3]);
    static TouchSensor *lr_touch = new TouchSensor(sensor_lr[0]);

    static PositionSensor *rf_pos_shouder = new PositionSensor(sensor_rf[1]);
    static PositionSensor *rf_pos_hip = new PositionSensor(sensor_rf[2]);
    static PositionSensor *rf_pos_knee = new PositionSensor(sensor_rf[3]);
    static TouchSensor *rf_touch = new TouchSensor(sensor_rf[0]);

    static PositionSensor *rr_pos_shouder = new PositionSensor(sensor_rr[1]);
    static PositionSensor *rr_pos_hip = new PositionSensor(sensor_rr[2]);
    static PositionSensor *rr_pos_knee = new PositionSensor(sensor_rr[3]);
    static TouchSensor *rr_touch = new TouchSensor(sensor_rr[0]);

    static Motor *lf_motor_shouder = new Motor(mototr_lf[0]);
    static Motor *lf_motor_hip = new Motor(mototr_lf[1]);
    static Motor *lf_motor_knee = new Motor(mototr_lf[2]);

    static Motor *lr_motor_shouder = new Motor(mototr_lr[0]);
    static Motor *lr_motor_hip = new Motor(mototr_lr[1]);
    static Motor *lr_motor_knee = new Motor(mototr_lr[2]);

    static Motor *rf_motor_shouder = new Motor(mototr_rf[0]);
    static Motor *rf_motor_hip = new Motor(mototr_rf[1]);
    static Motor *rf_motor_knee = new Motor(mototr_rf[2]);

    static Motor *rr_motor_shouder = new Motor(mototr_rr[0]);
    static Motor *rr_motor_hip = new Motor(mototr_rr[1]);
    static Motor *rr_motor_knee = new Motor(mototr_rr[2]);

    void Init(void)
    {
        camera_webots->enable(WEBOTS_TIME_STEP);
        imu_webots->enable(WEBOTS_TIME_STEP);
        accelerometer_webots->enable(WEBOTS_TIME_STEP);

        lf_pos_shouder->enable(WEBOTS_TIME_STEP);
        lf_pos_hip->enable(WEBOTS_TIME_STEP);
        lf_pos_knee->enable(WEBOTS_TIME_STEP);
        lf_touch->enable(WEBOTS_TIME_STEP);

        lr_pos_shouder->enable(WEBOTS_TIME_STEP);
        lr_pos_hip->enable(WEBOTS_TIME_STEP);
        lr_pos_knee->enable(WEBOTS_TIME_STEP);
        lr_touch->enable(WEBOTS_TIME_STEP);

        rf_pos_shouder->enable(WEBOTS_TIME_STEP);
        rf_pos_hip->enable(WEBOTS_TIME_STEP);
        rf_pos_knee->enable(WEBOTS_TIME_STEP);
        rf_touch->enable(WEBOTS_TIME_STEP);

        rr_pos_shouder->enable(WEBOTS_TIME_STEP);
        rr_pos_hip->enable(WEBOTS_TIME_STEP);
        rr_pos_knee->enable(WEBOTS_TIME_STEP);
        rr_touch->enable(WEBOTS_TIME_STEP);

        lf_motor_shouder->enableTorqueFeedback(WEBOTS_TIME_STEP);
        lf_motor_hip->enableTorqueFeedback(WEBOTS_TIME_STEP);
        lf_motor_knee->enableTorqueFeedback(WEBOTS_TIME_STEP);

        lr_motor_shouder->enableTorqueFeedback(WEBOTS_TIME_STEP);
        lr_motor_hip->enableTorqueFeedback(WEBOTS_TIME_STEP);
        lr_motor_knee->enableTorqueFeedback(WEBOTS_TIME_STEP);

        rf_motor_shouder->enableTorqueFeedback(WEBOTS_TIME_STEP);
        rf_motor_hip->enableTorqueFeedback(WEBOTS_TIME_STEP);
        rf_motor_knee->enableTorqueFeedback(WEBOTS_TIME_STEP);

        rr_motor_shouder->enableTorqueFeedback(WEBOTS_TIME_STEP);
        rr_motor_hip->enableTorqueFeedback(WEBOTS_TIME_STEP);
        rr_motor_knee->enableTorqueFeedback(WEBOTS_TIME_STEP);
    }


    void WebotsApi(void) {}


    imu_t GetRobotEuler(void)
    {
        imu_t return_imu = {0};
        const double *euler = NULL;
        static double eulerLast[3] = {0};

        euler = imu_webots->getRollPitchYaw();

        return_imu.pitch = euler[0];
        return_imu.roll = euler[1];
        return_imu.yaw = euler[2];

        return_imu.omega_pitch = (euler[0] - eulerLast[0]) / ((double)WEBOTS_TIME_STEP / 1000.0);
        return_imu.omega_roll = (euler[1] - eulerLast[1]) / ((double)WEBOTS_TIME_STEP / 1000.0);
        return_imu.omega_yaw = (euler[2] - eulerLast[2]) / ((double)WEBOTS_TIME_STEP / 1000.0);

        for (int i = 0; i < 3; i++)
        {
            eulerLast[i] = euler[i];
        }

        return return_imu;
    }


    void GetRobotBodyVel(Vec3<double> &p_body_pos, Vec3<double> &p_body_vel)
    {
        static double pos_last[3] = {0};
        const double *pos = Robot_node->getPosition();

        p_body_pos(0) = pos[2];
        p_body_pos(1) = pos[0];
        p_body_pos(2) = pos[1] - 0.02;

        p_body_vel(0) = (pos[2] - pos_last[2]) / 0.005;
        p_body_vel(1) = (pos[0] - pos_last[0]) / 0.005;
        p_body_vel(2) = (pos[1] - pos_last[1]) / 0.005;

        for (int i = 0; i < 3; i++)
            pos_last[i] = pos[i];
    }


    void GetRobotBodyAcc(Vec3<double> &p_body_acc)
    {
        const double *acc = accelerometer_webots->getValues();

        p_body_acc(0) = acc[2];
        p_body_acc(1) = acc[0];
        p_body_acc(2) = acc[1];
    }


    robotLegMsg_t GetRobotJointMsg(void)
    {
        robotLegMsg_t return_legMsg = {0};

        static double lf_pos_last[3] = {0};
        static double lr_pos_last[3] = {0};
        static double rf_pos_last[3] = {0};
        static double rr_pos_last[3] = {0};

        return_legMsg.lf.shoulder_pos = lf_pos_shouder->getValue();
        return_legMsg.lf.hip_pos = lf_pos_hip->getValue();
        return_legMsg.lf.knee_pos = lf_pos_knee->getValue() - PI / 2.0;
        return_legMsg.lf.contact = lf_touch->getValue();

        return_legMsg.lr.shoulder_pos = lr_pos_shouder->getValue();
        return_legMsg.lr.hip_pos = lr_pos_hip->getValue();
        return_legMsg.lr.knee_pos = lr_pos_knee->getValue() - PI / 2.0;
        return_legMsg.lr.contact = lr_touch->getValue();

        return_legMsg.rf.shoulder_pos = rf_pos_shouder->getValue();
        return_legMsg.rf.hip_pos = rf_pos_hip->getValue();
        return_legMsg.rf.knee_pos = rf_pos_knee->getValue() - PI / 2.0;
        return_legMsg.rf.contact = rf_touch->getValue();

        return_legMsg.rr.shoulder_pos = rr_pos_shouder->getValue();
        return_legMsg.rr.hip_pos = rr_pos_hip->getValue();
        return_legMsg.rr.knee_pos = rr_pos_knee->getValue() - PI / 2.0;
        return_legMsg.rr.contact = rr_touch->getValue();

        return_legMsg.lf.shoulder_vel = (return_legMsg.lf.shoulder_pos - lf_pos_last[0]) / ((double)WEBOTS_TIME_STEP / 1000.0);
        return_legMsg.lf.hip_vel = (return_legMsg.lf.hip_pos - lf_pos_last[1]) / ((double)WEBOTS_TIME_STEP / 1000.0);
        return_legMsg.lf.knee_vel = (return_legMsg.lf.knee_pos - lf_pos_last[2]) / ((double)WEBOTS_TIME_STEP / 1000.0);

        return_legMsg.lr.shoulder_vel = (return_legMsg.lr.shoulder_pos - lr_pos_last[0]) / ((double)WEBOTS_TIME_STEP / 1000.0);
        return_legMsg.lr.hip_vel = (return_legMsg.lr.hip_pos - lr_pos_last[1]) / ((double)WEBOTS_TIME_STEP / 1000.0);
        return_legMsg.lr.knee_vel = (return_legMsg.lr.knee_pos - lr_pos_last[2]) / ((double)WEBOTS_TIME_STEP / 1000.0);

        return_legMsg.rf.shoulder_vel = (return_legMsg.rf.shoulder_pos - rf_pos_last[0]) / ((double)WEBOTS_TIME_STEP / 1000.0);
        return_legMsg.rf.hip_vel = (return_legMsg.rf.hip_pos - rf_pos_last[1]) / ((double)WEBOTS_TIME_STEP / 1000.0);
        return_legMsg.rf.knee_vel = (return_legMsg.rf.knee_pos - rf_pos_last[2]) / ((double)WEBOTS_TIME_STEP / 1000.0);

        return_legMsg.rr.shoulder_vel = (return_legMsg.rr.shoulder_pos - rr_pos_last[0]) / ((double)WEBOTS_TIME_STEP / 1000.0);
        return_legMsg.rr.hip_vel = (return_legMsg.rr.hip_pos - rr_pos_last[1]) / ((double)WEBOTS_TIME_STEP / 1000.0);
        return_legMsg.rr.knee_vel = (return_legMsg.rr.knee_pos - rr_pos_last[2]) / ((double)WEBOTS_TIME_STEP / 1000.0);

        lf_pos_last[0] = return_legMsg.lf.shoulder_pos;
        lf_pos_last[1] = return_legMsg.lf.hip_pos;
        lf_pos_last[2] = return_legMsg.lf.knee_pos;

        lr_pos_last[0] = return_legMsg.lr.shoulder_pos;
        lr_pos_last[1] = return_legMsg.lr.hip_pos;
        lr_pos_last[2] = return_legMsg.lr.knee_pos;

        rf_pos_last[0] = return_legMsg.rf.shoulder_pos;
        rf_pos_last[1] = return_legMsg.rf.hip_pos;
        rf_pos_last[2] = return_legMsg.rf.knee_pos;

        rr_pos_last[0] = return_legMsg.rr.shoulder_pos;
        rr_pos_last[1] = return_legMsg.rr.hip_pos;
        rr_pos_last[2] = return_legMsg.rr.knee_pos;

        return return_legMsg;
    }


    void SetRobotJointTorque(robotLegTorque_t legTorque)
    {
        lf_motor_shouder->setTorque(legTorque.lf_torque.shoulder_torque);
        lf_motor_hip->setTorque(legTorque.lf_torque.hip_torque);
        lf_motor_knee->setTorque(legTorque.lf_torque.knee_torque);

        lr_motor_shouder->setTorque(legTorque.lr_torque.shoulder_torque);
        lr_motor_hip->setTorque(legTorque.lr_torque.hip_torque);
        lr_motor_knee->setTorque(legTorque.lr_torque.knee_torque);

        rf_motor_shouder->setTorque(legTorque.rf_torque.shoulder_torque);
        rf_motor_hip->setTorque(legTorque.rf_torque.hip_torque);
        rf_motor_knee->setTorque(legTorque.rf_torque.knee_torque);

        rr_motor_shouder->setTorque(legTorque.rr_torque.shoulder_torque);
        rr_motor_hip->setTorque(legTorque.rr_torque.hip_torque);
        rr_motor_knee->setTorque(legTorque.rr_torque.knee_torque);

    }


    bool GetTimeFlag(void)
    {
        if (supervisor->step(WEBOTS_TIME_STEP) != -1)
        {
            motionPlanTimeFlag = 1;
            mpc_cnt--;
            if (mpc_cnt == 0)
            {
                mpc_cnt = MPC_TIME_PERIOD;
                mpcTimeFlag = 1;
            }
            return 1;
        }
        else
            return 0;
    }


    bool GetTimeFlagMpc(void)
    {
        if (mpcTimeFlag == 1)
        {
            mpcTimeFlag = 0;
            return 1;
        }

        return 0;
    }


    bool GetTimeFlagMotionPlan(void)
    {
        if (motionPlanTimeFlag == 1)
        {
            motionPlanTimeFlag = 0;
            return 1;
        }

        return 0;
    }


    Vec3<double> GetObstaclePos(std::string p_obstacleName)
    {
        Node *Obstacle_node = supervisor->getFromDef(p_obstacleName);
        const double *obstaclePos = Obstacle_node->getPosition();
        Vec3<double> res;
        res.setZero();
        res[0] = obstaclePos[0];
        res[1] = obstaclePos[1];
        res[2] = obstaclePos[2];
        return res;
    }


    double GetSlopeFirstEdge(void)
    {
        Vec3<double> slopePos = GetObstaclePos(obstacle[0]);
        double res;
        res = slopePos[2] - 0.5 * SLOPE_MID_LENGTH - SLOPE_UP_LENGTH * cos(SLOPE_UP_ANGLE);
        return res;
    }


    double GetStageFirstEdge(void)
    {
        Vec3<double> stagePos = GetObstaclePos(obstacle[1]);
        double res;
        res = stagePos[2] - 0.5 * STAGE_LENGTH;
        return res;
    }


    double GetSillFirstEdge(void)
    {
        Vec3<double> sillPos = GetObstaclePos(obstacle[2]);
        double res;
        res = sillPos[2] - 0.5 * SILL_WIDTH;
        return res;
    }


    double GetCalandriaFirstEdge(void)
    {
        Vec3<double> calandriaPos = GetObstaclePos(obstacle[3]);
        double res;
        res = calandriaPos[2];
        return res;
    }


    double GetStairsFirstEdge(void)
    {
        Vec3<double> stairsPos = GetObstaclePos(obstacle[4]);
        double res;
        res = stairsPos[2] - 2.5 * STAIRS_DEPTH;
        return res;
    }


    double GetSeesawFirstEdge(void)
    {
        Vec3<double> seesawPos = GetObstaclePos(obstacle[5]);
        double res;
        res = seesawPos[2] - 0.5 * SEESAW_LENGTH * cos(SEESAW_UP_ANGLE);
        return res;
    }


    double GetBridgeFirstEdge(void)
    {
        Vec3<double> bridgePos = GetObstaclePos(obstacle[6]);
        double res;
        res = bridgePos[2] - 0.5 * BRIDGE_LENGTH;
        return res;
    }


    void BodyTrajSet(void)
    {
        existingBodyTraj = supervisor->getFromDef("bodytraj");

        if (existingBodyTraj)
            existingBodyTraj->remove();

        int i_body;
        string bodyTrajString;

        bodyTrajString = bodyTrajString + "DEF bodytraj Shape {\n"
                                        + "  appearance Appearance {\n"
                                        + "    material Material {\n"
                                        + "      diffuseColor 1 1 0\n"
                                        + "      emissiveColor 1 1 0\n"
                                        + "    }\n"
                                        + "  }\n"
                                        + "  geometry DEF body_traj_line_set IndexedLineSet {\n"
                                        + "    coord Coordinate {\n"
                                        + "      point [\n";

        for (i_body = 0; i_body < MAX_NUM_DISPLAY_DOT; ++i_body)
            bodyTrajString = bodyTrajString + "      0 0 0\n";

        bodyTrajString = bodyTrajString + "      ]\n"
                                        + "    }\n"
                                        + "    coordIndex [\n";

        for (i_body = 0; i_body < MAX_NUM_DISPLAY_DOT; ++i_body)
            bodyTrajString = bodyTrajString + "      0 0 -1\n";

        bodyTrajString = bodyTrajString + "    ]\n"
                                        + "  }\n"
                                        + "}\n";

        rootChildrenField->importMFNodeFromString(-1, bodyTrajString);

        bodyTrajLineSetNode = supervisor->getFromDef("body_traj_line_set");
        bodyTrajCoordinatesNode = bodyTrajLineSetNode->getField("coord")->getSFNode();
        bodyTrajPointField = bodyTrajCoordinatesNode->getField("point");
        bodyTrajCoordIndexField = bodyTrajLineSetNode->getField("coordIndex");

        bodyTrajIndex = 0;
        bodyTrajFirstStep = true;
    }


    void LFTrajSet(void)
    {
        existingLFTraj = supervisor->getFromDef("lftraj");

        if (existingLFTraj)
            existingLFTraj->remove();

        int i_lf;
        string lfTrajString;

        lfTrajString = lfTrajString + "DEF lftraj Shape {\n"
                                    + "  appearance Appearance {\n"
                                    + "    material Material {\n"
                                    + "      diffuseColor 1 0 0\n"
                                    + "      emissiveColor 1 0 0\n"
                                    + "    }\n"
                                    + "  }\n"
                                    + "  geometry DEF lf_traj_line_set IndexedLineSet {\n"
                                    + "    coord Coordinate {\n"
                                    + "      point [\n";

        for (i_lf = 0; i_lf < MAX_NUM_DISPLAY_DOT; ++i_lf)
            lfTrajString = lfTrajString + "      0 0 0\n";

        lfTrajString = lfTrajString + "      ]\n"
                                    + "    }\n"
                                    + "    coordIndex [\n";

        for (i_lf = 0; i_lf < MAX_NUM_DISPLAY_DOT; ++i_lf)
            lfTrajString = lfTrajString + "      0 0 -1\n";

        lfTrajString = lfTrajString + "    ]\n"
                                    + "  }\n"
                                    + "}\n";

        rootChildrenField->importMFNodeFromString(-1, lfTrajString);

        lfTrajLineSetNode = supervisor->getFromDef("lf_traj_line_set");
        lfTrajCoordinatesNode = lfTrajLineSetNode->getField("coord")->getSFNode();
        lfTrajPointField = lfTrajCoordinatesNode->getField("point");
        lfTrajCoordIndexField = lfTrajLineSetNode->getField("coordIndex");

        lfTrajIndex = 0;
        lfTrajFirstStep = true;
    }


    void LRTrajSet(void)
    {
        existingLRTraj = supervisor->getFromDef("lrtraj");

        if (existingLRTraj)
            existingLRTraj->remove();

        int i_lr;
        string lrTrajString;

        lrTrajString = lrTrajString + "DEF lrtraj Shape {\n"
                                    + "  appearance Appearance {\n"
                                    + "    material Material {\n"
                                    + "      diffuseColor 0 1 0\n"
                                    + "      emissiveColor 0 1 0\n"
                                    + "    }\n"
                                    + "  }\n"
                                    + "  geometry DEF lr_traj_line_set IndexedLineSet {\n"
                                    + "    coord Coordinate {\n"
                                    + "      point [\n";

        for (i_lr = 0; i_lr < MAX_NUM_DISPLAY_DOT; ++i_lr)
            lrTrajString = lrTrajString + "      0 0 0\n";

        lrTrajString = lrTrajString + "      ]\n"
                                    + "    }\n"
                                    + "    coordIndex [\n";

        for (i_lr = 0; i_lr < MAX_NUM_DISPLAY_DOT; ++i_lr)
            lrTrajString = lrTrajString + "      0 0 -1\n";

        lrTrajString = lrTrajString + "    ]\n"
                                    + "  }\n"
                                    + "}\n";

        rootChildrenField->importMFNodeFromString(-1, lrTrajString);

        lrTrajLineSetNode = supervisor->getFromDef("lr_traj_line_set");
        lrTrajCoordinatesNode = lrTrajLineSetNode->getField("coord")->getSFNode();
        lrTrajPointField = lrTrajCoordinatesNode->getField("point");
        lrTrajCoordIndexField = lrTrajLineSetNode->getField("coordIndex");

        lrTrajIndex = 0;
        lrTrajFirstStep = true;
    }


    void RFTrajSet(void)
    {
        existingRFTraj = supervisor->getFromDef("rftraj");

        if (existingRFTraj)
            existingRFTraj->remove();

        int i_rf;
        string rfTrajString;

        rfTrajString = rfTrajString + "DEF rftraj Shape {\n"
                                    + "  appearance Appearance {\n"
                                    + "    material Material {\n"
                                    + "      diffuseColor 0 1 0\n"
                                    + "      emissiveColor 0 1 0\n"
                                    + "    }\n"
                                    + "  }\n"
                                    + "  geometry DEF rf_traj_line_set IndexedLineSet {\n"
                                    + "    coord Coordinate {\n"
                                    + "      point [\n";

        for (i_rf = 0; i_rf < MAX_NUM_DISPLAY_DOT; ++i_rf)
            rfTrajString = rfTrajString + "      0 0 0\n";

        rfTrajString = rfTrajString + "      ]\n"
                                    + "    }\n"
                                    + "    coordIndex [\n";

        for (i_rf = 0; i_rf < MAX_NUM_DISPLAY_DOT; ++i_rf)
            rfTrajString = rfTrajString + "      0 0 -1\n";

        rfTrajString = rfTrajString + "    ]\n"
                                    + "  }\n"
                                    + "}\n";

        rootChildrenField->importMFNodeFromString(-1, rfTrajString);

        rfTrajLineSetNode = supervisor->getFromDef("rf_traj_line_set");
        rfTrajCoordinatesNode = rfTrajLineSetNode->getField("coord")->getSFNode();
        rfTrajPointField = rfTrajCoordinatesNode->getField("point");
        rfTrajCoordIndexField = rfTrajLineSetNode->getField("coordIndex");

        rfTrajIndex = 0;
        rfTrajFirstStep = true;
    }


    void RRTrajSet(void)
    {
        existingRRTraj = supervisor->getFromDef("rrtraj");

        if (existingRRTraj)
            existingRRTraj->remove();

        int i_rr;
        string rrTrajString;

        rrTrajString = rrTrajString + "DEF rrtraj Shape {\n"
                                    + "  appearance Appearance {\n"
                                    + "    material Material {\n"
                                    + "      diffuseColor 1 0 0\n"
                                    + "      emissiveColor 1 0 0\n"
                                    + "    }\n"
                                    + "  }\n"
                                    + "  geometry DEF rr_traj_line_set IndexedLineSet {\n"
                                    + "    coord Coordinate {\n"
                                    + "      point [\n";

        for (i_rr = 0; i_rr < MAX_NUM_DISPLAY_DOT; ++i_rr)
            rrTrajString = rrTrajString + "      0 0 0\n";

        rrTrajString = rrTrajString + "      ]\n"
                                    + "    }\n"
                                    + "    coordIndex [\n";

        for (i_rr = 0; i_rr < MAX_NUM_DISPLAY_DOT; ++i_rr)
            rrTrajString = rrTrajString + "      0 0 -1\n";

        rrTrajString = rrTrajString + "    ]\n"
                                    + "  }\n"
                                    + "}\n";

        rootChildrenField->importMFNodeFromString(-1, rrTrajString);

        rrTrajLineSetNode = supervisor->getFromDef("rr_traj_line_set");
        rrTrajCoordinatesNode = rrTrajLineSetNode->getField("coord")->getSFNode();
        rrTrajPointField = rrTrajCoordinatesNode->getField("point");
        rrTrajCoordIndexField = rrTrajLineSetNode->getField("coordIndex");

        rrTrajIndex = 0;
        rrTrajFirstStep = true;
    }
 
#endif