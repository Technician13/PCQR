#ifndef TERRAIN_HPP
#define TERRAIN_HPP

#include "parameters.hpp"
#include "commonFunction.hpp"

namespace quadruped
{
    enum Dim2D {X=0,Y};

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
        virtual double GetHeightDerivWrtX(double x, double y) {return 0.0;}
        virtual double GetHeightDerivWrtY(double x, double y) {return 0.0;}
        virtual double GetHeightDerivWrtXX(double x, double y) {return 0.0;}
        virtual double GetHeightDerivWrtXY(double x, double y) {return 0.0;}
        virtual double GetHeightDerivWrtYX(double x, double y) {return 0.0;}
        virtual double GetHeightDerivWrtYY(double x, double y) {return 0.0;}
        virtual void UpdataEdge(double p_first_edge_D_x,double p_first_edge_D_y) {}
        virtual Vec3<double> GetNormalVector(double p_x,double p_y) = 0;

        uint8_t terrain_id = 0;
        double first_edge_D_x = 0;
        double first_edge_D_y = 0;
        std::vector<double> split = {0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0};
        std::vector<std::pair<double,double>> safeArea = {{0.0 , LIMIT_INFINITE} , 
                                                          {0.0 , LIMIT_INFINITE} ,
                                                          {0.0 , LIMIT_INFINITE} ,
                                                          {0.0 , LIMIT_INFINITE} ,
                                                          {0.0 , LIMIT_INFINITE} ,
                                                          {0.0 , LIMIT_INFINITE} };
        int numSplit;
        double averageDangerArea;
    };


    class FlatGround : public Terrain
    {
    private:
        double height_flat = 0.0;
    
    protected:

    public: 
        FlatGround(void);
        virtual double GetHight(double p_x,double p_y,double p_curBodyPitch)  override;
        virtual Vec3<double> GetNormalVector(double p_x,double p_y) override;
    };


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