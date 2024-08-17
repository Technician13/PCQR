/**
 * @file webotsApi.cpp
 * @brief webots仿真环境中的相关函数封装
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-18
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "webotsApi.hpp"
#include <iostream>

#ifndef ROBOT

    using namespace webots;

    /** @brief 判断是否运行Actiger模型预测控制器的标志位 */
    uint8_t mpcTimeFlag = 0;
    /** @brief 判断是否运行Actiger规划器的标志位 */
    uint8_t motionPlanTimeFlag = 0;
    /** @brief Actiger模型预测控制器的运行周期 */
    uint8_t mpc_cnt = MPC_PERIOD;

    std::string obstacle[7] = {"slope", "stage", "sill", "calandria", "stairs", "seesaw_support", "bridge"};

    Supervisor *supervisor = new Supervisor();
    Node *Root_node = supervisor->getRoot();
    Node *Robot_node = supervisor->getFromDef("quadruped");
    Field *rootChildrenField = Root_node->getField("children");

    /******************************** 机体轨迹可视化相关变量 ********************************/
    Node *existingBodyTraj;
    Node *bodyTrajLineSetNode;
    Node *bodyTrajCoordinatesNode;
    Field *bodyTrajPointField;
    Field *bodyTrajCoordIndexField;
    int bodyTrajIndex;
    bool bodyTrajFirstStep;

    /****************************** 左前腿足端轨迹可视化相关变量 ******************************/
    webots::Node *existingLFTraj;
    webots::Node *lfTrajLineSetNode;
    webots::Node *lfTrajCoordinatesNode;
    webots::Field *lfTrajPointField;
    webots::Field *lfTrajCoordIndexField;
    int lfTrajIndex;
    bool lfTrajFirstStep;

    /****************************** 左后腿足端轨迹可视化相关变量 ******************************/
    webots::Node *existingLRTraj;
    webots::Node *lrTrajLineSetNode;
    webots::Node *lrTrajCoordinatesNode;
    webots::Field *lrTrajPointField;
    webots::Field *lrTrajCoordIndexField;
    int lrTrajIndex;
    bool lrTrajFirstStep;

    /****************************** 右前腿足端轨迹可视化相关变量 ******************************/
    webots::Node *existingRFTraj;
    webots::Node *rfTrajLineSetNode;
    webots::Node *rfTrajCoordinatesNode;
    webots::Field *rfTrajPointField;
    webots::Field *rfTrajCoordIndexField;
    int rfTrajIndex;
    bool rfTrajFirstStep;

    /****************************** 右后腿足端轨迹可视化相关变量 ******************************/
    webots::Node *existingRRTraj;
    webots::Node *rrTrajLineSetNode;
    webots::Node *rrTrajCoordinatesNode;
    webots::Field *rrTrajPointField;
    webots::Field *rrTrajCoordIndexField;
    int rrTrajIndex;
    bool rrTrajFirstStep;

    /** @brief 左前腿传感器集合：LF足端触地传感器 | LF肩关节角度传感器 | LF髋关节角度传感器 | LF膝关节角度传感器 */
    std::string sensor_lf[4] = {"touch_4", "LF_pos_1", "LF_pos_2", "LF_pos_3"};
    /** @brief 左后腿传感器集合：LR足端触地传感器 | LR肩关节角度传感器 | LR髋关节角度传感器 | LR膝关节角度传感器 */
    std::string sensor_lr[4] = {"touch_3", "LR_pos_1", "LR_pos_2", "LR_pos_3"};
    /** @brief 右前腿传感器集合：RF足端触地传感器 | RF肩关节角度传感器 | RF髋关节角度传感器 | RF膝关节角度传感器 */
    std::string sensor_rf[4] = {"touch_2", "RF_pos_1", "RF_pos_2", "RF_pos_3"};
    /** @brief 右后腿传感器集合：RR足端触地传感器 | RR肩关节角度传感器 | RR髋关节角度传感器 | RR膝关节角度传感器 */
    std::string sensor_rr[4] = {"touch_1", "RR_pos_1", "RR_pos_2", "RR_pos_3"};

    /** @brief 左前腿电机集合：LF肩关节电机 | LF髋关节电机 | LF膝关节电机 */
    std::string mototr_lf[3] = {"LF_motor_1", "LF_motor_2", "LF_motor_3"};
    /** @brief 左后腿电机集合：LR肩关节电机 | LR髋关节电机 | LR膝关节电机 */
    std::string mototr_lr[3] = {"LR_motor_1", "LR_motor_2", "LR_motor_3"};
    /** @brief 右前腿电机集合：RF肩关节电机 | RF髋关节电机 | RF膝关节电机 */
    std::string mototr_rf[3] = {"RF_motor_1", "RF_motor_2", "RF_motor_3"};
    /** @brief 右后腿电机集合：RR肩关节电机 | RR髋关节电机 | RR膝关节电机 */
    std::string mototr_rr[3] = {"RR_motor_1", "RR_motor_2", "RR_motor_3"};

    /** @brief IMU模块 */
    static InertialUnit* imu_webots = new InertialUnit("imu");
    /** @brief 相机模块 */
    static Camera* camera_webots = new Camera("camera");
    /** @brief 加速度计模块 */
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

    /**
     * @brief 初始化函数，使能webots中的传感器和电机
     */
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

    /**
     * @brief 空函数
     */
    void WebotsApi(void) {}

    /**
     * @brief 调用webots接口函数获取机器人角度和角速度
     * @return imu_t 
     */
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

    /**
     * @brief 调用webots接口函数获取机器人位置和速度
     * @param  p_body_pos  存储Actiger在全局坐标系下的位置的向量，按引用传递      
     * @param  p_body_vel  存储Actiger在全局坐标系下的速度的向量，按引用传递
     */
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

    /**
     * @brief 调用webots接口函数获取机器人加速度
     * @param  p_body_acc   存储Actiger在机体坐标系下的加速度，按引用传递    
     */
    void GetRobotBodyAcc(Vec3<double> &p_body_acc)
    {
        const double *acc = accelerometer_webots->getValues();

        p_body_acc(0) = acc[2];
        p_body_acc(1) = acc[0];
        p_body_acc(2) = acc[1];
    }

    /**
     * @brief 获取机器人整体腿部状态信息
     * @return robotLegMsg_t 
     */
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

    /**
     * @brief 输出扭矩
     * @param  legTorque  计算后输出的Actiger扭矩集合      
     */
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

        // std::cout<<"===================================================================================="<<std::endl;
        // std::cout<<"LF_torque : "<<legTorque.lf_torque.shoulder_torque<<"   "<<legTorque.lf_torque.hip_torque<<"   "<<legTorque.lf_torque.knee_torque<<std::endl;
        // std::cout<<"LR_torque : "<<legTorque.lr_torque.shoulder_torque<<"   "<<legTorque.lr_torque.hip_torque<<"   "<<legTorque.lr_torque.knee_torque<<std::endl;
        // std::cout<<"RF_torque : "<<legTorque.rf_torque.shoulder_torque<<"   "<<legTorque.rf_torque.hip_torque<<"   "<<legTorque.rf_torque.knee_torque<<std::endl;
        // std::cout<<"RR_torque : "<<legTorque.rr_torque.shoulder_torque<<"   "<<legTorque.rr_torque.hip_torque<<"   "<<legTorque.rr_torque.knee_torque<<std::endl;
    }

    /**
     * @brief MotionPlan以及MPC标志位计数以及触发
     * @return true 程序正常运行
     * @return false 程序崩溃
     */
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

    /**
     * @brief 获取MPC标志位状态
     * @return true MPC标志位为1且将其置0
     * @return false MPC标志位为0
     */
    bool GetTimeFlagMpc(void)
    {
        if (mpcTimeFlag == 1)
        {
            mpcTimeFlag = 0;
            return 1;
        }

        return 0;
    }

    /**
     * @brief 获取MotionPlan标志位状态
     * @return true MotionPlan标志位为1且将其置0
     * @return false MotionPlan标志位为0
     */
    bool GetTimeFlagMotionPlan(void)
    {
        if (motionPlanTimeFlag == 1)
        {
            motionPlanTimeFlag = 0;
            return 1;
        }

        return 0;
    }

    /**
     * @brief 获取webots仿真环境中的障碍的位置
     * @param  p_obstacleName  webots环境中待获取位置的障碍名称 
     * @return Vec3<double> 
     */
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

    /**
     * @brief 获取webots仿真环境中的坡面障碍与地面的第一根交线在webots中Z坐标轴上的位置
     * @return double 
     */
    double GetSlopeFirstEdge(void)
    {
        Vec3<double> slopePos = GetObstaclePos(obstacle[0]);
        double res;
        res = slopePos[2] - 0.5 * SLOPE_MID_LENGTH - SLOPE_UP_LENGTH * cos(SLOPE_UP_ANGLE);
        return res;
    }

    /**
     * @brief 获取webots仿真环境中的平台障碍与地面的第一根交线在webots中Z坐标轴上的位置
     * @return double 
     */
    double GetStageFirstEdge(void)
    {
        Vec3<double> stagePos = GetObstaclePos(obstacle[1]);
        double res;
        res = stagePos[2] - 0.5 * STAGE_LENGTH;
        return res;
    }

    /**
     * @brief 获取webots仿真环境中的门槛障碍与地面的第一根交线在webots中Z坐标轴上的位置
     * @return double 
     */
    double GetSillFirstEdge(void)
    {
        Vec3<double> sillPos = GetObstaclePos(obstacle[2]);
        double res;
        res = sillPos[2] - 0.5 * SILL_WIDTH;
        return res;
    }

    /**
     * @brief 获取webots仿真环境中的排管障碍中第一根排管轴线在webots中Z坐标轴上的位置
     * @return double 
     */
    double GetCalandriaFirstEdge(void)
    {
        Vec3<double> calandriaPos = GetObstaclePos(obstacle[3]);
        double res;
        res = calandriaPos[2];
        return res;
    }

    /**
     * @brief 获取webots仿真环境中的阶梯障碍与地面的第一根交线在webots中Z坐标轴上的位置
     * @return double 
     */
    double GetStairsFirstEdge(void)
    {
        Vec3<double> stairsPos = GetObstaclePos(obstacle[4]);
        double res;
        res = stairsPos[2] - 2.5 * STAIRS_DEPTH;
        return res;
    }

    /**
     * @brief 获取webots仿真环境中的跷跷板障碍与地面的第一根交线在webots中Z坐标轴上的位置
     * @return double 
     */
    double GetSeesawFirstEdge(void)
    {
        Vec3<double> seesawPos = GetObstaclePos(obstacle[5]);
        double res;
        res = seesawPos[2] - 0.5 * SEESAW_LENGTH * cos(SEESAW_UP_ANGLE);
        return res;
    }

    /**
     * @brief 获取webots仿真环境中的双木桥障碍与地面的第一根交线在webots中Z坐标轴上的位置
     * @return double 
     */
    double GetBridgeFirstEdge(void)
    {
        Vec3<double> bridgePos = GetObstaclePos(obstacle[6]);
        double res;
        res = bridgePos[2] - 0.5 * BRIDGE_LENGTH;
        return res;
    }

    /**
     * @brief 机体轨迹可视化设置
     */
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

    /**
     * @brief 左前腿足端轨迹可视化设置
     */
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

    /**
     * @brief 左后腿足端轨迹可视化设置
     */
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

    /**
     * @brief 右前腿足端轨迹可视化设置
     */
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

    /**
     * @brief 右后腿足端轨迹可视化设置
     */
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