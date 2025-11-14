#include "terrain.hpp"

using namespace quadruped;

Terrain::Terrain(void) {}


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


FlatGround::FlatGround(void)
{
    Terrain::terrain_id = FLAT;
    numSplit = 0;
}


double FlatGround::GetHight(double p_x,double p_y,double p_curBodyPitch)
{
    double height = 0.0;
    return height;
}


Vec3<double> FlatGround::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;
    normalVector << 0.0 , 0.0 , 1.0;
    return normalVector;
}


Slope::Slope(void)
{
    Terrain::terrain_id = SLOPE;
    numSplit = 0;
}


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


void Slope::UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y)
{
    first_edge_D_x = p_first_edge_D_x;
    first_edge_D_y = p_first_edge_D_y;
}


Vec3<double> Slope::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;

    return normalVector;  
}


Stage::Stage(void)
{
    Terrain::terrain_id = STAGE;
    numSplit = 2;
    averageDangerArea = STAGE_HEIGHT;
}


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


Vec3<double> Stage::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;
    
    return normalVector;  
}


Sill::Sill(void)
{
    Terrain::terrain_id = SILL;
    numSplit = 1;
    averageDangerArea = 3.0 * SILL_HEIGHT;
}


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


Vec3<double> Sill::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;
    
    return normalVector;  
}


Calandria::Calandria(void)
{
    Terrain::terrain_id = CALANDRIA;
    numSplit = 5;
    averageDangerArea = 3.0 * CALANDRIA_HEIGHT;
}


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


Vec3<double> Calandria::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;
    
    return normalVector;  
}


Stairs::Stairs(void)
{
    Terrain::terrain_id = STAIRS;
    numSplit = 6;
    averageDangerArea = STAIRS_HEIGHT;
}


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


Vec3<double> Stairs::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;
    
    return normalVector;  
}


Seesaw::Seesaw(void)
{
    Terrain::terrain_id = SEESAW;
    numSplit = 0;
}


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


void Seesaw::UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y)
{
    first_edge_D_x = p_first_edge_D_x;
    first_edge_D_y = p_first_edge_D_y;
}


Vec3<double> Seesaw::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;
    
    return normalVector;  
}


Bridge::Bridge(void)
{
    Terrain::terrain_id = BRIDGE;
    numSplit = 2;
    averageDangerArea = BRIDGE_HEIGHT;
}


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


Vec3<double> Bridge::GetNormalVector(double p_x,double p_y)
{
    Vec3<double> normalVector;
    
    return normalVector;  
}


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

