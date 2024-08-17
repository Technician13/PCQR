/**
 * @file terrain.hpp
 * @brief 地形类头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-25
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef TERRAIN_HPP
#define TERRAIN_HPP

#include "parameters.hpp"
#include "commonFunction.hpp"

namespace quadruped
{
    enum Dim2D {X=0,Y};

    /**
     * @brief 地形类父类
     */
    class Terrain
    {
    private:

    protected:

    public:
        explicit Terrain(void);
        virtual ~Terrain(void);

        double GetDerivativeOfHeightWrt(Dim2D p_dim, double p_x, double p_y);
        double GetSecondDerivativeOfHeightWrt(Dim2D p_dim1, Dim2D p_dim2,double p_x, double p_y);
        virtual double GetHight(double p_x,double p_y, double p_curBodyPitch) {return 0.0;}
        // virtual Vec3<double> GetFootTrajectory(double p_x,double p_y,bool p_is_foothold) {return 0.0;}
        // first derivatives that must be implemented by the user
        virtual double GetHeightDerivWrtX(double x, double y) {return 0.0;}
        virtual double GetHeightDerivWrtY(double x, double y) {return 0.0;}
        // second derivatives with respect to first letter, then second
        virtual double GetHeightDerivWrtXX(double x, double y) {return 0.0;}
        virtual double GetHeightDerivWrtXY(double x, double y) {return 0.0;}
        virtual double GetHeightDerivWrtYX(double x, double y) {return 0.0;}
        virtual double GetHeightDerivWrtYY(double x, double y) {return 0.0;}
        virtual void UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y) {}
        virtual Vec3<double> GetNormalVector(double p_x,double p_y) = 0;

        /** @brief 地形标识符 */
        uint8_t terrain_id = 0;
        /** @brief 障碍与地面的交线中点在机体坐标系X方向上的位置 */
        double first_edge_D_x = 0;
        /** @brief 障碍与地面的交线中点在机体坐标系Y方向上的位置 */
        double first_edge_D_y = 0;
        /** @brief 地形分割线，对应于障碍的棱线 */
        std::vector<double> split = {0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0};
        /** @brief 以地形分割线为中心的安全区 */
        std::vector<std::pair<double,double>> safeArea = {{0.0 , LIMIT_INFINITE} , 
                                                          {0.0 , LIMIT_INFINITE} ,
                                                          {0.0 , LIMIT_INFINITE} ,
                                                          {0.0 , LIMIT_INFINITE} ,
                                                          {0.0 , LIMIT_INFINITE} ,
                                                          {0.0 , LIMIT_INFINITE} };
        /** @brief 地形分割线个数 */
        int numSplit;
        /** @brief 平均危险区 */
        double averageDangerArea;
    };




    /**
     * @brief 平地类子类
     */
    class FlatGround : public Terrain
    {
    private:
        /** @brief 平地的高度 */
        double height_flat = 0.0;
    
    protected:

    public: 
        FlatGround(void);
        virtual double GetHight(double p_x,double p_y,double p_curBodyPitch)  override;
        virtual Vec3<double> GetNormalVector(double p_x,double p_y) override;
    };




    /**
     * @brief 坡面类子类
     */
    class Slope : public Terrain
    {
    private:

    protected:
    
    public: 
        Slope(void);
        virtual double GetHight(double p_x,double p_y,double p_curBodyPitch) override;
        virtual void UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y) override;
        virtual Vec3<double> GetNormalVector(double p_x,double p_y) override;
    };




    /**
     * @brief 平台类子类
     */
    class Stage : public Terrain
    {
    private:

    protected:
    
    public: 
        Stage(void);
        virtual double GetHight(double p_x,double p_y,double p_curBodyPitch) override;
        virtual void UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y) override;
        virtual Vec3<double> GetNormalVector(double p_x,double p_y) override;
    };




    /**
     * @brief 门槛类子类
     */
    class Sill : public Terrain
    {
    private:

    protected:
    
    public: 
        Sill(void);
        virtual double GetHight(double p_x,double p_y,double p_curBodyPitch) override;
        virtual void UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y) override;
        virtual Vec3<double> GetNormalVector(double p_x,double p_y) override;    
    }; 




    /**
     * @brief 排管类子类
     */
    class Calandria : public Terrain
    {
    private:

    protected:

    public: 
        Calandria(void);
        virtual double GetHight(double p_x,double p_y,double p_curBodyPitch)  override;
        virtual void UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y)  override;
        virtual Vec3<double> GetNormalVector(double p_x,double p_y) override;
    };




    /**
     * @brief 阶梯类子类
     */
    class Stairs : public Terrain
    {
    private:

    protected:

    public: 
        Stairs(void);
        virtual double GetHight(double p_x,double p_y,double p_curBodyPitch)  override;
        virtual void UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y)  override;
        virtual Vec3<double> GetNormalVector(double p_x,double p_y) override;
    };




    /**
     * @brief 跷跷板类子类
     */
    class Seesaw : public Terrain
    {
        public: 
            Seesaw(void);
            virtual double GetHight(double p_x,double p_y,double p_curBodyPitch);
            virtual void UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y) override;
            virtual Vec3<double> GetNormalVector(double p_x,double p_y) override;

        private:
            double seesaw_length = 1.2;
            double seesaw_mid_height = 0.3;

            double up_angle_tan = 0.2527;
            double down_angle_tan = -0.2527;

            double height_deadZone = -5.0;
    };




    /**
     * @brief 双木桥类子类
     */
    class Bridge : public Terrain
    {
    private:

    protected:
    
    public: 
        Bridge(void);
        virtual double GetHight(double p_x,double p_y,double p_curBodyPitch) override;
        virtual void UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y) override;
        virtual Vec3<double> GetNormalVector(double p_x,double p_y) override;  
    };


































































    

    
    

    
    

    
    class Hurdles : public Terrain
    {
        public: 
            Hurdles(void){Terrain::terrain_id = HURDLES ;}
            virtual double GetHight(double p_x,double p_y,double p_curBodyPitch) override;
            virtual void UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y) override;
            virtual Vec3<double> GetNormalVector(double p_x,double p_y) override;

        private:
            double height_flat = 0.0;
            double height_hurdles = 0.325; 

            double height_deadZone = -5.0;

    };

    
    class Ballpool : public Terrain
    {
        public: 
            Ballpool(void){Terrain::terrain_id = BALL_POOL ;}
            virtual double GetHight(double p_x,double p_y,double p_curBodyPitch) override;
            virtual void UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y) override;
            virtual Vec3<double> GetNormalVector(double p_x,double p_y) override;

        private:
            double height_flat = 0.0;
            double height_hom = 0.1;

            double height_deadZone = -5.0;

    };

    
    

    
    



}








#endif