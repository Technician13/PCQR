/**
 * @file fH_nlp.cpp
 * @brief 落足点规划的相关非线性规划问题
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-27
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "fH_nlp.hpp"
#include "commonFunction.hpp"
#include <iostream>
using namespace quadruped;

/**
 * @description: 非线性规划问题参数设置
 * @param {FH_updataInfo*} p_updataInfo 落足点规划的信息更新对象
 * @return {*}
 */
FH_nlp::FH_nlp(FH_updataInfo* p_updataInfo)
{
    updataInfo = p_updataInfo;
    /* 约束 */
    cst = new FH_constraints(p_updataInfo);
    /* 决策变量数目：12 */
    var_num = FH_VAR_NUM_SINGLE;
    /* 约束数目：41 */
    cst_num = FH_KIN_CST_NUM + FH_OTH_CST_NUM;
    /* 雅可比矩阵中非零元素数目 */
    cst_jac_num = FH_KIN_CST_JAC_NUM + FH_OTH_CST_JAC_NUM;
    /* 海森矩阵中非零元素数目 */
    cst_hes_num = 2;

}

FH_nlp::~FH_nlp(void){}

/**
 * @description: 非线性规划问题参数设置
 * @param {FH_updataInfo*} p_updataInfo
 * @return {*}
 */
bool FH_nlp::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                          Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    /************************Optimization variables*************************
     * bodyPos(3d) | bodyVel(3d) | frontfootPos(3d) | rearfootPos(3d)
     *********************************************************************/
    n = var_num;
    m = cst_num;
    nnz_jac_g = cst_jac_num;
    nnz_h_lag = cst_hes_num;
    index_style = TNLP::C_STYLE;
    return true;
}

/**
 * @description: 设置边界
 * @param {*}
 * @return {*}
 */
bool FH_nlp::get_bounds_info(Index n, Number* x_l, Number* x_u,
                             Index m, Number* g_l, Number* g_u)
{
    assert(n == var_num);
    assert(m == cst_num);

    /* 决策变量边界 */
    double x_lb[FH_VAR_NUM_SINGLE] = { 0 };
    double x_ub[FH_VAR_NUM_SINGLE] = { 0 };
    updataInfo->GetXBound(x_ub, x_lb);
    for (Index i = 0; i < FH_VAR_NUM_SINGLE; i++)
    {
        x_l[i] = x_lb[i];
        x_u[i] = x_ub[i];
    }

    /* 运动学约束边界 */
    double g_k_lb[FH_KIN_CST_NUM] = { 0 };
    double g_k_ub[FH_KIN_CST_NUM] = { 0 };
    cst->GetKinematicsCstBound(g_k_ub, g_k_lb);
    for (int i = 0; i < FH_KIN_CST_NUM; i++)
    {
        g_l[i] = g_k_lb[i];
        g_u[i] = g_k_ub[i];
    }

    /* 其他约束边界 */
    double g_o_lb[FH_OTH_CST_NUM] = { 0 };
    double g_o_ub[FH_OTH_CST_NUM] = { 0 };
    cst->GetOtherCstBound(g_o_ub, g_o_lb);
    for (int i = 0; i < FH_OTH_CST_NUM; i++)
    {
        g_l[FH_KIN_CST_NUM+i] = g_o_lb[i];
        g_u[FH_KIN_CST_NUM+i] = g_o_ub[i];
    }
    return true;
}

/**
 * @description: 配置初始点
 * @param {*}
 * @return {*}
 */
bool FH_nlp::get_starting_point(Index n, bool init_x, Number* x,
                                bool init_z, Number* z_L, Number* z_U,
                                Index m, bool init_lambda, Number* lambda)
{
    double x_init[var_num] = {0}; 
    updataInfo->GetXInital(x_init);
    for(int i = 0 ; i<var_num ; i++)
        x[i]=x_init[i];

    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    return true;
}

/**
 * @description: 目标函数，这里为常数0
 * @param {*}
 * @return {*}
 */
bool FH_nlp::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
    assert(n == var_num);
    obj_value = 0.0;
    return true;
}

/**
 * @description: 目标函数的梯度
 * @param {*}
 * @return {*}
 */
bool FH_nlp::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{

    for (int i = 0; i < var_num; i++)
    {
        grad_f[i] = 0;
    }
    return true;
}

/**
 * @description: 计算约束的值
 * @param {Index} n
 * @param {const Number*} x
 * @param {bool} new_x
 * @param {Index} m
 * @param {Number*} g
 * @return {*}
 */
bool FH_nlp::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{

    assert(n == var_num);
    assert(m == cst_num);
    
    Vec18<double> k_cst = Vec18<double>::Zero();
    Vec23<double> o_cst = Vec23<double>::Zero();

    Vec3<double> bodyPos = Vec3<double>::Zero();
    Vec3<double> bodyVel = Vec3<double>::Zero();
    Vec3<double> foothold_front = Vec3<double>::Zero();
    Vec3<double> foothold_rear = Vec3<double>::Zero();

    for(int i=0;i<3;i++)
    {
        bodyPos(i) = x[i];
        bodyVel(i) = x[i+3];
        foothold_front(i)=x[i+6];
        foothold_rear(i)=x[i+9];
    }

    cst->GetKinematicsCst(&k_cst,bodyPos,bodyVel,foothold_front,foothold_rear);
    cst->GetOtherCst(&o_cst,bodyPos,bodyVel,foothold_front,foothold_rear);

    for (int k_cnt = 0; k_cnt < FH_KIN_CST_NUM; k_cnt++)
        g[k_cnt] = k_cst(k_cnt);

    for (int o_cnt = 0; o_cnt < FH_OTH_CST_NUM; o_cnt++)
        g[FH_KIN_CST_NUM + o_cnt] = o_cst(o_cnt);

    return true;
}


/**
 * @description: 约束的雅可比矩阵
 * @param {*}
 * @return {*}
 */
bool FH_nlp::eval_jac_g(Index n, const Number* x, bool new_x,
                        Index m, Index nele_jac, Index* iRow, Index* jCol,
                        Number* values)
{
    // std::cout<< "eval_jac_g"<<std::endl;
    static DVec<int> row(cst_jac_num, 1);
    static DVec<int> col(cst_jac_num, 1);


    if (values == NULL) 
    {
        row.setZero();
        col.setZero();

        DMat<double> jac_pos(cst_num,var_num);
        jac_pos.setZero();

        Mat18x13<double> kin_jac_pos = Mat18x13<double>::Zero();
        Mat23x12<double> oth_jac_pos = Mat23x12<double>::Zero();

        cst->GetKinematicsCstJacobianPos(&kin_jac_pos);
        cst->GetOtherCstJacobianPos(&oth_jac_pos);

        jac_pos.block(0, 0 , FH_KIN_CST_NUM, FH_VAR_NUM_SINGLE) = 
        kin_jac_pos.block(0,0,FH_KIN_CST_NUM, FH_VAR_NUM_SINGLE);

        jac_pos.block(FH_KIN_CST_NUM,0 ,FH_OTH_CST_NUM, FH_VAR_NUM_SINGLE) = 
        oth_jac_pos.block(0,0,FH_OTH_CST_NUM, FH_VAR_NUM_SINGLE);

        int cnt = 0;
        
        for (int i = 0; i < cst_num; i++)
        {
            for (int j = 0; j < var_num; j++)
            {
                if (jac_pos(i, j) > 0.5)
                {
                    iRow[cnt] = i;
                    jCol[cnt] = j;
                    row(cnt) = i;
                    col(cnt) = j;
                    cnt++;                
                }
            }
            //std::cout<<"???????????????????????????????????????????????????????????????????????????????????????????????????????  "<<i<<std::endl;
        }
        // std::cout<<cnt<<std::endl;
    }
    else
    {
        Vec3<double> bodyPos = Vec3<double>::Zero();
        Vec3<double> bodyVel = Vec3<double>::Zero();
        Vec3<double> foothold_front = Vec3<double>::Zero();
        Vec3<double> foothold_rear = Vec3<double>::Zero();

        for(int i=0;i<3;i++)
        {
                

            bodyPos(i) = x[i];
                    

            bodyVel(i) = x[i+3];
            foothold_front(i)=x[i+6];
            foothold_rear(i)=x[i+9];

        }

        DMat<double> jac_value(cst_num, var_num);
        jac_value.setZero();

        Mat18x13<double> kin_jac_value = Mat18x13<double>::Zero();
        Mat23x12<double> oth_jac_value = Mat23x12<double>::Zero();

        cst->GetKinematicsCstJacobian(&kin_jac_value);
        cst->GetOtherCstJacobian(&oth_jac_value, bodyPos, bodyVel, foothold_front, foothold_rear);

        jac_value.block(0, 0 , FH_KIN_CST_NUM, FH_VAR_NUM_SINGLE) = kin_jac_value.block(0,0,FH_KIN_CST_NUM, FH_VAR_NUM_SINGLE);
        jac_value.block(FH_KIN_CST_NUM,0 ,FH_OTH_CST_NUM, FH_VAR_NUM_SINGLE) = oth_jac_value.block(0,0,FH_OTH_CST_NUM, FH_VAR_NUM_SINGLE);
        
        for (int i = 0; i < cst_jac_num; i++)
        {
            values[i] = jac_value(row(i),col(i));
        }

    }
    return true;
}

/**
 * @description: 目标函数和约束的海森矩阵
 * @param {*}
 * @return {*}
 */
bool FH_nlp::eval_h(Index n, const Number* x, bool new_x,
                    Number obj_factor, Index m, const Number* lambda,
                    bool new_lambda, Index nele_hess, Index* iRow,
                    Index* jCol, Number* values)
{
    static DVec<int> row(cst_hes_num, 1);
    static DVec<int> col(cst_hes_num, 1);

    if (values == NULL) 
    {
        row.setZero();
        col.setZero();

        DMat<double> hes_pos(var_num, var_num);
        hes_pos.setZero();

        Mat12<double> oth_hes_pos = Mat12<double>::Zero();

        cst->GetOtherCstHessianPos(&oth_hes_pos);

        hes_pos.block(0,0,FH_VAR_NUM_SINGLE,FH_VAR_NUM_SINGLE) = 
        hes_pos.block(0,0,FH_VAR_NUM_SINGLE,FH_VAR_NUM_SINGLE) + oth_hes_pos.block(0,0,FH_VAR_NUM_SINGLE,FH_VAR_NUM_SINGLE);

        int cnt = 0;
        for (int i = 0; i < var_num; i++)
        {
            for (int j = 0; j < var_num; j++)
            {
                if (hes_pos(i, j) > 0.5)
                {
                    iRow[cnt] = i;
                    jCol[cnt] = j;
                    row(cnt) = i;
                    col(cnt) = j;
                    cnt++;                
                }
            }
        }
    }
    else 
    {
        DMat<double> hes_value(var_num, var_num);
        hes_value.setZero();

        double lambda_oth[FH_OTH_CST_NUM] = {0};

        Mat12<double> oth_hes_value = Mat12<double>::Zero();

        for(int j = 0; j < FH_OTH_CST_NUM; j++)
        {
            lambda_oth[j] = lambda[FH_KIN_CST_NUM + j];
        }

        cst->GetOtherCstHessian(lambda_oth, &oth_hes_value);

        hes_value.block(0,0,FH_VAR_NUM_SINGLE,FH_VAR_NUM_SINGLE) = 
        hes_value.block(0,0,FH_VAR_NUM_SINGLE,FH_VAR_NUM_SINGLE) + oth_hes_value.block(0,0,FH_VAR_NUM_SINGLE,FH_VAR_NUM_SINGLE);
        
        for (int i = 0; i < cst_hes_num; i++)
        {
            values[i] = hes_value(row(i),col(i));
        }
         
    }

    return true;
}

/**
 * @description: 优化结果
 * @param {*}
 * @return {*}
 */
void FH_nlp::finalize_solution(SolverReturn status,Index n, const Number* x, const Number* z_L, const Number* z_U,
                               Index m, const Number* g, const Number* lambda, Number obj_value,
                               const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq)
{
    updataInfo->return_spatial_desire.block(0,0,3,1) = updataInfo->desEuler;
    updataInfo->return_spatial_desire(3) = x[0];
    updataInfo->return_spatial_desire(4) = x[1];
    updataInfo->return_spatial_desire(5) = x[2];
    
    updataInfo->return_spatial_dot_desire(2) = (updataInfo->desEuler(2) - updataInfo->curEuler(2)) / updataInfo->Ts;
    updataInfo->return_spatial_dot_desire(3) = x[3];
    updataInfo->return_spatial_dot_desire(4) = x[4];
    updataInfo->return_spatial_dot_desire(5) = x[5];
    
    updataInfo->return_desEndfoot_pos_front[0] = x[6];
    updataInfo->return_desEndfoot_pos_front[1] = x[7];
    updataInfo->return_desEndfoot_pos_front[2] = x[8];

    updataInfo->return_desEndfoot_pos_rear[0] = x[9];
    updataInfo->return_desEndfoot_pos_rear[1] = x[10];
    updataInfo->return_desEndfoot_pos_rear[2] = x[11];
}
