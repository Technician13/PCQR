/**
 * @file terrain.cpp
 * @brief 地形类
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-25
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "terrain.hpp"

using namespace quadruped;

/**
 * @brief 地形类的构造函数
 */
Terrain::Terrain(void) {}

/**
 * @brief 地形类的析构函数
 */
Terrain::~Terrain(void) {}


double Terrain::GetDerivativeOfHeightWrt(Dim2D p_dim, double p_x, double p_y)
{
    double height = 0;
    if(p_dim == X)
        height = GetHeightDerivWrtX(p_x,p_y);
    else
        height = GetHeightDerivWrtY(p_x,p_y);

    return height;
}

double Terrain::GetSecondDerivativeOfHeightWrt(Dim2D p_dim1, Dim2D p_dim2,double p_x, double p_y)
{
    double height = 0;
    if(p_dim1 == X)
    {
        if(p_dim2 == X)
            height = GetHeightDerivWrtXX(p_x,p_y);
        else
            height = GetHeightDerivWrtXY(p_x,p_y);
    }
    else
    {
        if(p_dim2 == X)
            height = GetHeightDerivWrtYX(p_x,p_y);
        else
            height = GetHeightDerivWrtYY(p_x,p_y);
    }
    
    return height;
    
}

/****************************************************************** 平地类 ******************************************************************/

/**
 * @brief FlatGround类的构造函数，完成地形标识符以及地形分割线的初始化
 */
FlatGround::FlatGround(void)
{
    Terrain::terrain_id = FLAT;
    numSplit = 0;
}

/**
 * @brief 获取当前地形在世界坐标系下的高度
 * @param  p_x  当前点在世界坐标系位置坐标的X值            
 * @param  p_y  当前点在世界坐标系位置坐标的Y值          
 * @return double 
 */
double FlatGround::GetHight(double p_x,double p_y,double p_curBodyPitch)
{
    double height = 0.0;
    return height;
}

/**
 * @brief 获取当前地形在世界坐标系下的法向量
 * @param  p_x  当前点在世界坐标系位置坐标的X值              
 * @param  p_y  当前点在世界坐标系位置坐标的Y值          
 * @return Vec3<double> 
 */
Vec3<double> FlatGround::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;
    normalVector << 0.0 , 0.0 , 1.0;
    return normalVector;
}

/****************************************************************** 坡面类 ******************************************************************/

/**
 * @brief Slope类的构造函数，完成地形标识符以及地形分割线的初始化
 */
Slope::Slope(void)
{
    Terrain::terrain_id = SLOPE;
    numSplit = 0;
}

/**
 * @brief 获取当前地形在世界坐标系下的高度
 * @param  p_x  当前点在世界坐标系位置坐标的X值            
 * @param  p_y  当前点在世界坐标系位置坐标的Y值          
 * @return double 
 */
double Slope::GetHight(double p_x,double p_y,double p_curBodyPitch)
{
    double height;

    #ifndef ROBOT
        if(p_x <= first_edge_D_x)
        {
            height = 0.0;
        }
        else if(p_x <= (first_edge_D_x + SLOPE_UP_LENGTH * cos(SLOPE_UP_ANGLE)))
        {
            height = (p_x - first_edge_D_x) * tan(abs(SLOPE_UP_ANGLE));
        }
        else if(p_x <= (first_edge_D_x + SLOPE_UP_LENGTH * cos(SLOPE_UP_ANGLE) + SLOPE_MID_LENGTH))
        {
            height = SLOPE_MID_HEIGHT;
        }
        else if(p_x <= (first_edge_D_x + SLOPE_UP_LENGTH * cos(SLOPE_UP_ANGLE) + SLOPE_DOWN_LENGTH * cos(SLOPE_DOWN_ANGLE) + SLOPE_MID_LENGTH))
        {
            height = ((first_edge_D_x + SLOPE_UP_LENGTH * cos(SLOPE_UP_ANGLE) + SLOPE_DOWN_LENGTH * cos(SLOPE_DOWN_ANGLE) + SLOPE_MID_LENGTH) - p_x) * tan(abs(SLOPE_DOWN_ANGLE));
        }
        else
        {
            height = 0.0;
        }
        
    #else

    #endif

    return height;
}

/**
 * @brief 获取坡面与地面的交线中点在世界坐标系下的位置
 * @param  p_first_edge_D_x  当前点在世界坐标系位置坐标的X值
 * @param  p_first_edge_D_y  当前点在世界坐标系位置坐标的Y值
 */
void Slope::UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y)
{
    first_edge_D_x = p_first_edge_D_x;
    first_edge_D_y = p_first_edge_D_y;
}

/**
 * @brief 预留接口 | 获取坡面某一点处的法向量
 * @param  p_x              
 * @param  p_y              
 * @return Vec3<double> 
 */
Vec3<double> Slope::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;

    return normalVector;  
}

/****************************************************************** 平台类 ******************************************************************/

/**
 * @brief Stage类的构造函数，完成地形标识符以及地形分割线的初始化
 */
Stage::Stage(void)
{
    Terrain::terrain_id = STAGE;
    numSplit = 2;
    averageDangerArea = STAGE_HEIGHT;
}

/**
 * @brief 获取当前地形在世界坐标系下的高度
 * @param  p_x  当前点在世界坐标系位置坐标的X值            
 * @param  p_y  当前点在世界坐标系位置坐标的Y值          
 * @return double 
 */
double Stage::GetHight(double p_x,double p_y,double p_curBodyPitch)
{
    double height;

    #ifndef ROBOT
        if(p_x <= first_edge_D_x)
        {
            height = 0.0;
        }
        else if(p_x <= (first_edge_D_x + STAGE_LENGTH))
        {
            height = STAGE_HEIGHT;
        }
        else
        {
            height = 0.0;
        }
        
    #else

    #endif

    return height;
}

/**
 * @brief 获取平台与地面的交线中点在世界坐标系下的位置以及分割线和危险区的设置
 * @param  p_first_edge_D_x  当前点在世界坐标系位置坐标的X值
 * @param  p_first_edge_D_y  当前点在世界坐标系位置坐标的Y值
 */
void Stage::UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y)
{
    first_edge_D_x = p_first_edge_D_x;
    first_edge_D_y = p_first_edge_D_y;

    split[0] = first_edge_D_x;
    split[1] = first_edge_D_x + STAGE_LENGTH;

    std::pair<double,double> temp_up;
    std::pair<double,double> temp_down;
    temp_up = make_pair(0.2 * 2.0 * STAGE_HEIGHT , 0.8 * 2.0 * STAGE_HEIGHT);
    temp_down = make_pair(0.8 * 2.0 * STAGE_HEIGHT , 0.2 * 2.0 * STAGE_HEIGHT);

    safeArea[0] = temp_up;
    safeArea[1] = temp_down;

    for(int i = 0 ; i < numSplit ; i++)
    {
        split[i] = split[i] + 0.5 * (safeArea[i].first - safeArea[i].second);
        safeArea[i].first = averageDangerArea;
        safeArea[i].second = LIMIT_INFINITE;
    }
}

/**
 * @brief 预留接口 | 获取平台某一点处的法向量
 * @param  p_x              
 * @param  p_y              
 * @return Vec3<double> 
 */
Vec3<double> Stage::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;
    
    return normalVector;  
}

/****************************************************************** 门槛类 ******************************************************************/

/**
 * @brief Sill类的构造函数，完成地形标识符以及地形分割线的初始化
 */
Sill::Sill(void)
{
    Terrain::terrain_id = SILL;
    numSplit = 1;
    averageDangerArea = 3.0 * SILL_HEIGHT;
}

/**
 * @brief 获取当前地形在世界坐标系下的高度
 * @param  p_x  当前点在世界坐标系位置坐标的X值            
 * @param  p_y  当前点在世界坐标系位置坐标的Y值          
 * @return double 
 */
double Sill::GetHight(double p_x,double p_y,double p_curBodyPitch)
{
    double height;

    #ifndef ROBOT
        height = 0.0;  
    #else
        height = 0.0;
    #endif

    return height;
}

/**
 * @brief 获取门槛轴线中点在世界坐标系下的位置以及分割线和危险区的设置
 * @param  p_first_edge_D_x  当前点在世界坐标系位置坐标的X值
 * @param  p_first_edge_D_y  当前点在世界坐标系位置坐标的Y值
 */
void Sill::UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y)
{
    first_edge_D_x = p_first_edge_D_x;
    first_edge_D_y = p_first_edge_D_y;

    split[0] = first_edge_D_x + 0.5 * SILL_WIDTH;

    for(int i = 0 ; i < numSplit ; i++)
    {
        split[i] = split[i];
        safeArea[i].first = averageDangerArea;
        safeArea[i].second = LIMIT_INFINITE;
    }
}

/**
 * @brief 预留接口 | 获取门槛某一点处的法向量
 * @param  p_x              
 * @param  p_y              
 * @return Vec3<double> 
 */
Vec3<double> Sill::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;
    
    return normalVector;  
}

/****************************************************************** 排管类 ******************************************************************/

/**
 * @brief Calandria类的构造函数，完成地形标识符以及地形分割线的初始化
 */
Calandria::Calandria(void)
{
    Terrain::terrain_id = CALANDRIA;
    numSplit = 5;
    averageDangerArea = 3.0 * CALANDRIA_HEIGHT;
}

/**
 * @brief 获取当前地形在世界坐标系下的高度
 * @param  p_x  当前点在世界坐标系位置坐标的X值            
 * @param  p_y  当前点在世界坐标系位置坐标的Y值          
 * @return double 
 */
double Calandria::GetHight(double p_x,double p_y,double p_curBodyPitch)
{
    double height;

    #ifndef ROBOT
        height = 0.0;  
    #else
        height = 0.0;
    #endif

    return height;
}

/**
 * @brief 获取每根排管轴线中点在世界坐标系下的位置以及分割线和危险区的设置
 * @param  p_first_edge_D_x  当前点在世界坐标系位置坐标的X值
 * @param  p_first_edge_D_y  当前点在世界坐标系位置坐标的Y值
 */
void Calandria::UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y)
{
    first_edge_D_x = p_first_edge_D_x;
    first_edge_D_y = p_first_edge_D_y;

    split[0] = first_edge_D_x;
    split[1] = first_edge_D_x + CALANDRIA_DISTANCE;
    split[2] = first_edge_D_x + 2.0 * CALANDRIA_DISTANCE;
    split[3] = first_edge_D_x + 3.0 * CALANDRIA_DISTANCE;
    split[4] = first_edge_D_x + 4.0 * CALANDRIA_DISTANCE;

    for(int i = 0 ; i < numSplit ; i++)
    {
        split[i] = split[i];
        safeArea[i].first = averageDangerArea;
        safeArea[i].second = LIMIT_INFINITE;
    }
}

/**
 * @brief 预留接口 | 获取排管某一点处的法向量
 * @param  p_x              
 * @param  p_y              
 * @return Vec3<double> 
 */
Vec3<double> Calandria::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;
    
    return normalVector;  
}

/****************************************************************** 阶梯类 ******************************************************************/

/**
 * @brief Stairs类的构造函数，完成地形标识符以及地形分割线的初始化
 */
Stairs::Stairs(void)
{
    Terrain::terrain_id = STAIRS;
    numSplit = 6;
    averageDangerArea = STAIRS_HEIGHT;
}

/**
 * @brief 获取当前地形在世界坐标系下的高度
 * @param  p_x  当前点在世界坐标系位置坐标的X值            
 * @param  p_y  当前点在世界坐标系位置坐标的Y值          
 * @return double 
 */
double Stairs::GetHight(double p_x,double p_y,double p_curBodyPitch)
{
    double height;

    #ifndef ROBOT
        if(p_x <= first_edge_D_x)
        {
            height = 0.0;
        }
        else if(p_x <= (first_edge_D_x + STAIRS_DEPTH))
        {
            height = STAIRS_HEIGHT;
        }
        else if(p_x <= (first_edge_D_x + 2.0 * STAIRS_DEPTH))
        {
            height = 2.0 * STAIRS_HEIGHT;
        }
        else if(p_x <= (first_edge_D_x + 3.0 * STAIRS_DEPTH))
        {
            height = 3.0 * STAIRS_HEIGHT;
        }
        else if(p_x <= (first_edge_D_x + 4.0 * STAIRS_DEPTH))
        {
            height = 2.0 * STAIRS_HEIGHT;
        }
        else if(p_x <= (first_edge_D_x + 5.0 * STAIRS_DEPTH))
        {
            height = STAIRS_HEIGHT;
        }
        else
        {
            height = 0.0;
        }   

    #else

    #endif

    return height;
}

/**
 * @brief 获取阶梯与地面的交线中点在世界坐标系下的位置以及分割线和危险区的设置
 * @param  p_first_edge_D_x  当前点在世界坐标系位置坐标的X值
 * @param  p_first_edge_D_y  当前点在世界坐标系位置坐标的Y值
 */
void Stairs::UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y)
{
    first_edge_D_x = p_first_edge_D_x;
    first_edge_D_y = p_first_edge_D_y;

    split[0] = first_edge_D_x;
    split[1] = first_edge_D_x + STAIRS_DEPTH;
    split[2] = first_edge_D_x + 2.0 * STAIRS_DEPTH;
    split[3] = first_edge_D_x + 3.0 * STAIRS_DEPTH;
    split[4] = first_edge_D_x + 4.0 * STAIRS_DEPTH;
    split[5] = first_edge_D_x + 5.0 * STAIRS_DEPTH;

    std::pair<double,double> temp_up;
    std::pair<double,double> temp_down;
    temp_up = make_pair(0.2 * 2.0 * STAIRS_HEIGHT , 0.8 * 2.0 * STAIRS_HEIGHT);
    temp_down = make_pair(0.8 * 2.0 * STAIRS_HEIGHT , 0.2 * 2.0 * STAIRS_HEIGHT);

    safeArea[0] = temp_up;
    safeArea[1] = temp_up;
    safeArea[2] = temp_up;
    safeArea[3] = temp_down;
    safeArea[4] = temp_down;
    safeArea[5] = temp_down;

    for(int i = 0 ; i < numSplit ; i++)
    {
        split[i] = split[i] + 0.5 * (safeArea[i].first - safeArea[i].second);
        safeArea[i].first = averageDangerArea;
        safeArea[i].second = LIMIT_INFINITE;
    }
}

/**
 * @brief 预留接口 | 获取阶梯某一点处的法向量
 * @param  p_x              
 * @param  p_y              
 * @return Vec3<double> 
 */
Vec3<double> Stairs::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;
    
    return normalVector;  
}

/****************************************************************** 跷跷板类 ******************************************************************/

/**
 * @brief Seesaw类的构造函数，完成地形标识符的初始化
 */
Seesaw::Seesaw(void)
{
    Terrain::terrain_id = SEESAW;
    numSplit = 0;
}

/**
 * @brief 获取当前地形在世界坐标系下的高度
 * @param  p_x  当前点在世界坐标系位置坐标的X值            
 * @param  p_y  当前点在世界坐标系位置坐标的Y值          
 * @return double 
 */
double Seesaw::GetHight(double p_x,double p_y,double p_curBodyPitch)
{
    double height;

    #ifndef ROBOT
        if(p_x <= first_edge_D_x)
        {
            height = 0.0;
        }
        else if(p_x <= first_edge_D_x + 0.25 * SEESAW_LENGTH * cos(SEESAW_UP_ANGLE))
        {
            height = (p_x - first_edge_D_x) * tan(abs(SEESAW_UP_ANGLE));
        }
        else if(p_x <= first_edge_D_x + 0.75 * SEESAW_LENGTH * cos(SEESAW_UP_ANGLE))
        {
            height = 0.3 - (p_x - first_edge_D_x - 0.5 * SEESAW_LENGTH * cos(SEESAW_UP_ANGLE)) * tan(p_curBodyPitch);
        }
        else if(p_x <= first_edge_D_x + SEESAW_LENGTH * cos(SEESAW_UP_ANGLE))
        {
            height = (first_edge_D_x + SEESAW_LENGTH * cos(SEESAW_UP_ANGLE) - p_x) * tan(abs(SEESAW_UP_ANGLE));
        }
        else
        {
            height = 0.0;
        }
        
    #else

    #endif

    return height;
}

/**
 * @brief 获取跷跷板与地面的交线中点在世界坐标系下的位置
 * @param  p_first_edge_D_x  当前点在世界坐标系位置坐标的X值
 * @param  p_first_edge_D_y  当前点在世界坐标系位置坐标的Y值
 */
void Seesaw::UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y)
{
    first_edge_D_x = p_first_edge_D_x;
    first_edge_D_y = p_first_edge_D_y;
}

/**
 * @brief 预留接口 | 获取跷跷板某一点处的法向量
 * @param  p_x              
 * @param  p_y              
 * @return Vec3<double> 
 */
Vec3<double> Seesaw::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;
    
    return normalVector;  
}

/****************************************************************** 双木桥类 ******************************************************************/

/**
 * @brief Bridge类的构造函数，完成地形标识符以及地形分割线的初始化
 */
Bridge::Bridge(void)
{
    Terrain::terrain_id = BRIDGE;
    numSplit = 2;
    averageDangerArea = BRIDGE_HEIGHT;
}

/**
 * @brief 获取当前地形在世界坐标系下的高度
 * @param  p_x  当前点在世界坐标系位置坐标的X值            
 * @param  p_y  当前点在世界坐标系位置坐标的Y值          
 * @return double 
 */
double Bridge::GetHight(double p_x,double p_y,double p_curBodyPitch)
{
    double height;

    #ifndef ROBOT
        if(p_x <= first_edge_D_x)
        {
            height = 0.0;
        }
        else if(p_x <= (first_edge_D_x + BRIDGE_LENGTH))
        {
            height = BRIDGE_HEIGHT;
        }
        else
        {
            height = 0.0;
        }
        
    #else

    #endif

    return height;
}

/**
 * @brief 获取双木桥与地面的交线中点在世界坐标系下的位置以及分割线和危险区的设置
 * @param  p_first_edge_D_x  当前点在世界坐标系位置坐标的X值
 * @param  p_first_edge_D_y  当前点在世界坐标系位置坐标的Y值
 */
void Bridge::UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y)
{
    first_edge_D_x = p_first_edge_D_x;
    first_edge_D_y = p_first_edge_D_y;

    split[0] = first_edge_D_x;
    split[1] = first_edge_D_x + BRIDGE_LENGTH;

    std::pair<double,double> temp_up;
    std::pair<double,double> temp_down;
    temp_up = make_pair(0.2 * 2.0 * BRIDGE_HEIGHT , 0.8 * 2.0 * BRIDGE_HEIGHT);
    temp_down = make_pair(0.8 * 2.0 * BRIDGE_HEIGHT , 0.2 * 2.0 * BRIDGE_HEIGHT);

    safeArea[0] = temp_up;
    safeArea[1] = temp_down;

    for(int i = 0 ; i < numSplit ; i++)
    {
        split[i] = split[i] + 0.5 * (safeArea[i].first - safeArea[i].second);
        safeArea[i].first = averageDangerArea;
        safeArea[i].second = LIMIT_INFINITE;
    }
}

/**
 * @brief 预留接口 | 获取双木桥某一点处的法向量
 * @param  p_x              
 * @param  p_y              
 * @return Vec3<double> 
 */
Vec3<double> Bridge::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;
    
    return normalVector;  
}


















































































































/******************************************************************/
/******************************************************************/
/******************************************************************/
double Hurdles::GetHight(double p_x,double p_y,double p_curBodyPitch)
{
    double height = 0;  
    return height;
}

void Hurdles::UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y)
{
    first_edge_D_x = p_first_edge_D_x;
}

Vec3<double> Hurdles::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;

    normalVector << 0,
                    0,
                    1.0;


    return normalVector;  
}

/******************************************************************/
double Ballpool::GetHight(double p_x,double p_y,double p_curBodyPitch)
{
    double height = 0;
    
    return height;
}

void Ballpool::UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y)
{
    first_edge_D_x = p_first_edge_D_x;
}

Vec3<double> Ballpool::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;

    normalVector << 0,
                    0,
                    1.0;


    return normalVector;  
}

/******************************************************************/