// /**
//  * @file     positionEstimate.cpp
//  * @author   
//  * @version  V1.0.0
//  * @data     18-June-2021
//  * @brief    位置估计初始版本
//  **/

// #include "positionEstimate.hpp"


// using namespace quadruped;
// PositionEstimate::PositionEstimate()
// {
//     position_x_ = 0.0;
//     position_y_ = 0.0;
//     position_z_ = 0.0;
//     cnt = 0;

    
// }

// PositionEstimate::~PositionEstimate()
// {

// }



// void PositionEstimate::Position(RobotModel *_robotmodel)
// {
//     // if(cnt < 2)
//     // {
//     //     cnt++;
//     // }
//     // else if(cnt < 3)
//     // {
//     //     cnt++;
//     //     z_offset = -0.25 * (_robotmodel->legState[0]->endfoot_hip[2] + 
//     //                         _robotmodel->legState[1]->endfoot_hip[2] +
//     //                         _robotmodel->legState[2]->endfoot_hip[2] +
//     //                         _robotmodel->legState[3]->endfoot_hip[2]);
//     // }
//     // else
//     // {
//     //     position_x_ = position_x_ + _robotmodel->bodyVelocity[0] * 0.005;
//     //     position_y_ = position_y_ + _robotmodel->bodyVelocity[1] * 0.005;
            
//     //     if(!(_robotmodel->legState[0]->contact_state || 
//     //          _robotmodel->legState[1]->contact_state || 
//     //          _robotmodel->legState[2]->contact_state || 
//     //          _robotmodel->legState[3]->contact_state))
//     //     {
//     //         //air 
//     //     }
//     //     else
//     //     {
//     //         position_z_ = position_z_ + _robotmodel->bodyVelocity[2] * 0.005;
//     //     }

//     //     bodyPosWorld << position_x_ , position_y_ , position_z_+z_offset;


//     // }    
// }