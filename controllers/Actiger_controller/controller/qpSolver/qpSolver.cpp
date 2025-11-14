#include "qpSolver.hpp"
#include <iostream>

using namespace std;
using namespace quadruped;


uint8_t NearZero(double val)
{
    return (val < 1e-10 && val > -1e-10);
}
 

QpSolver::QpSolver(void)
{
    cout << "QpSolver Init" << endl;
}


QpSolver::~QpSolver(void) {}


void QpSolver::Free(void)
{
    c_free(h_data);
    c_free(h_indices);
    c_free(h_indptr);
    c_free(a_data);
    c_free(a_indices);
    c_free(a_indptr);
    c_free(q_data);
    c_free(l_data);
    c_free(u_data);
}


void QpSolver::CalculateDataH(DMat<double> H_matrix)
{
    std::vector<c_float> H_data;
    std::vector<c_int> H_indices;
    std::vector<c_int> H_indptr;

    int ind_h = 0;
    for (int i = 0; i < H_matrix.cols(); i++)
    {
        (&H_indptr)->emplace_back(ind_h);
        for (int j = 0; j < H_matrix.rows(); j++)
        {
            if (!NearZero(H_matrix(j, i)))
            {
                (&H_indices)->emplace_back(j);
                (&H_data)->emplace_back(H_matrix( j, i));
                ind_h++;
            }
        }
    }
    (&H_indptr)->emplace_back(ind_h);
    
    h_data = (c_float*)c_malloc(sizeof(c_float) * H_data.size());
    h_indices = (c_int*)c_malloc(sizeof(c_int) * H_indices.size());
    h_indptr = (c_int*)c_malloc(sizeof(c_int) * H_indptr.size());

    size_H = H_data.size();
    for (int i = 0; i < H_data.size(); i++)
    {
        h_data[i] = H_data[i];
    }
    for (int i = 0; i < H_indices.size(); i++)
    {
        h_indices[i] = H_indices[i];
    }
    for (int i = 0; i < H_indptr.size(); i++)
    {
        h_indptr[i] = H_indptr[i];
    }
       
}

void QpSolver::CalculateDataG(DVec<double> G_vector)
{
    q_data = (c_float*)c_malloc(sizeof(c_float) * G_vector.size());

    for (int i = 0; i < G_vector.size(); i++)
    {
        q_data[i] = G_vector(i);
    }
       
}


void QpSolver::CalculateDataA(DMat<double> A_matrix)
{
    std::vector<c_float> A_data;
    std::vector<c_int> A_indices;
    std::vector<c_int> A_indptr;

    int ind_a = 0;
    for (int i = 0; i < A_matrix.cols(); i++)
    {
        (&A_indptr)->emplace_back(ind_a);
        for (int j = 0; j < A_matrix.rows(); j++)
        {
            if (!NearZero(A_matrix(j, i)))
            {
                (&A_indices)->emplace_back(j);
                (&A_data)->emplace_back(A_matrix(j, i));
                ind_a++;
            }
        }
    }
    (&A_indptr)->emplace_back(ind_a);

    a_data = (c_float*)c_malloc(sizeof(c_float) * A_data.size());
    a_indices = (c_int*)c_malloc(sizeof(c_int) * A_indices.size());
    a_indptr = (c_int*)c_malloc(sizeof(c_int) * A_indptr.size());

    size_A = A_data.size();
    for (int i = 0; i < A_data.size(); i++)
    {
        a_data[i] = A_data[i];
    }
    for (int i = 0; i < A_indices.size(); i++)
    {
        a_indices[i] = A_indices[i];
    }
    for (int i = 0; i < A_indptr.size(); i++)
    {
        a_indptr[i] = A_indptr[i];
    }
}


void QpSolver::CalculateDataLandU(DVec<double> L_vector, DVec<double> U_vector)
{
    l_data = (c_float*)c_malloc(sizeof(c_float) * L_vector.size());
    u_data = (c_float*)c_malloc(sizeof(c_float) * U_vector.size());

    for (int i = 0; i < L_vector.size(); i++)
    {
        l_data[i] = L_vector(i);
    }
    for (int i = 0; i < U_vector.size(); i++)
    {
        
        u_data[i] = U_vector(i);
    }   
}


DVec<double> QpSolver::Solve(DMat<double> H_matrix, DVec<double> G_vector,
                           DMat<double> A_matrix, DVec<double> L_vector, DVec<double> U_vector)
{
    DVec<double> return_data;

    OSQPWorkspace* work = nullptr;
    OSQPSettings* settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
    OSQPData* data = (OSQPData*)c_malloc(sizeof(OSQPData));

    size_A = 0;
    size_H = 0;

    CalculateDataH(H_matrix);

    CalculateDataG(G_vector);

    CalculateDataA(A_matrix);

    CalculateDataLandU(L_vector, U_vector);

    return_data.resize(A_matrix.cols(),1);

    c_int p_row = A_matrix.cols();
    c_int a_row = A_matrix.rows();

    if (data) {
        data->n = p_row;
        data->m = a_row;
        data->P = csc_matrix(data->n, data->n, size_H, h_data, h_indices, h_indptr);
        data->q = q_data;
        data->A = csc_matrix(data->m, data->n, size_A, a_data, a_indices, a_indptr);
        data->l = l_data;
        data->u = u_data;
    }
    if (settings != nullptr)
    {
        osqp_set_default_settings(settings);
        settings->polish = true;
        settings->scaled_termination = true;
        settings->verbose = false;
        settings->max_iter = 2000;
        settings->eps_abs = 0.000001;
        settings->eps_rel = 0.000001;
    }
    work = osqp_setup(data, settings);

    osqp_solve(work);
    auto status = work->info->status_val;

    if (status < 0 || (status != 1 && status != 2))
    {
        cout << "failed optimization status:\t" << work->info->status << endl;
    }
    else if (work->solution == nullptr)
    {
        cout << "The solution from OSQP is nullptr" << endl;
    }

    for (size_t i = 0; i < A_matrix.cols(); ++i) {
        return_data(i) = work->solution->x[i];
        
    }

    Free();
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings) c_free(settings);
    c_free(work);
    
    return return_data;
}