// Slam_Test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#pragma once
#include <iostream>
#include "Common.h"
#include "Image.h"
#include "sift.h"
#include "Matrix.h"
#include "Reconstruct.h"

extern void Test_Main();

extern "C"
{
#include "Buddy_System.h"
}

static void Test_2()
{
    typedef float _T;
    _T(*pKsi)[7];
    Measurement<_T>* pMeasurement;
    int i, j, iIter, iResult, iPoint_Count, iMeasurement_Count;
    union { _T Ti[4 * 4]; _T Ti_Inv[4 * 4]; };
    union { _T M_4x4[4 * 4]; _T M_Inv_4x4[4 * 4]; };
    union { _T Tj[4 * 4]; _T Tj_Inv[4 * 4]; };

    _T E_6[6], E_4x4[4 * 4], //E可以是一个矩阵
        fSum_e, fSum_e_Pre = (_T)1e10;
    _T J_Inv[6 * 6], Adj[6 * 6], Temp[6 * 6], Delta_Pose[4 * 4];
    _T* Jt, * J, * H, * JEt, * Sigma_H, * Sigma_JEt, * Delta_X;

    _T(*Camera)[4 * 4];
    iResult = bTemp_Load_Data("D:\\Samp\\YBKJ\\Slam_Test\\Slam_Test\\Sample\\sphere.g2o", &pKsi, &iPoint_Count, &pMeasurement, &iMeasurement_Count);
    if (!iResult)return;
    int iAdjust_Count = 10;

    Jt = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 36 * sizeof(_T));
    J = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 36 * sizeof(_T));
    H = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
    JEt = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
    Sigma_JEt = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
    Delta_X = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
    Sigma_H = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
    Camera = (_T(*)[4 * 4])pMalloc(&oMatrix_Mem, iAdjust_Count * 4 * 4 * sizeof(_T));
    for (i = 0; i < iAdjust_Count; i++)
        TQ_2_Rt(pKsi[i], Camera[i]);

    for (iIter = 0;; iIter++)
    {
        fSum_e = 0;
        memset(Sigma_H, 0, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
        memset(Sigma_JEt, 0, iAdjust_Count * 6 * sizeof(_T));
        for (i = 0; i < iAdjust_Count - 1; i++)
        {//没一个测量都带来一个调整权重
            Measurement<_T> oM = pMeasurement[i];
            //_T *ksi_i = pKsi[oM.m_Pose_Index[0]],
                //*ksi_j = pKsi[oM.m_Pose_Index[1]];
            //TQ_2_Rt(ksi_i, Ti);
            //TQ_2_Rt(ksi_j, Tj);
            memcpy(Ti, Camera[oM.m_Camera_Index[0]], 4 * 4 * sizeof(_T));
            memcpy(Tj, Camera[oM.m_Camera_Index[1]], 4 * 4 * sizeof(_T));
            TQ_2_Rt(oM.Delta_ksi, M_4x4);
            Get_Inv_Matrix_Row_Op_2(M_4x4, M_Inv_4x4, 4, &iResult);
            if (!iResult)break;
            Get_Inv_Matrix_Row_Op_2(Ti, Ti_Inv, 4, &iResult);
            if (!iResult)break;

            Matrix_Multiply(M_Inv_4x4, 4, 4, Ti_Inv, 4, E_4x4);
            Matrix_Multiply(E_4x4, 4, 4, Tj, 4, E_4x4);
            SE3_2_se3(E_4x4, E_6);

            //Disp(E_6, 6, 1, "E_6");
            for (j = 0; j < 6; j++)
                fSum_e += E_6[j] * E_6[j];

            memset(Jt, 0, iAdjust_Count * 36 * sizeof(_T));

            //接着求雅可比
            Get_J_Inv(E_6, J_Inv);  //此处搞定了一个近似的J(-1)
            Get_Inv_Matrix_Row_Op_2(Tj, Tj_Inv, 4, &iResult);
            if (!iResult)break;
            Get_Adj(Tj_Inv, Adj);

            //此时形成两个雅可比
            //第一个雅可比Jt
            Matrix_Multiply(J_Inv, 6, 6, Adj, 6, Temp);
            //∂eij/∂ξj
            Copy_Matrix_Partial(Temp, 6, 6, Jt, iAdjust_Count * 6, oM.m_Camera_Index[1] * 6, 0);

            //∂eij/∂ξi
            Matrix_Multiply(Temp, 6, 6, (_T)-1.f, Temp);
            //Transpose_Multiply(Temp, 6, 6, J1t,0);
            Copy_Matrix_Partial(Temp, 6, 6, Jt, iAdjust_Count * 6, oM.m_Camera_Index[0] * 6, 0);

            //printf("%d %d\n", oM.m_Pose_Index[0], oM.m_Pose_Index[1]);
            Matrix_Transpose(Jt, 6, iAdjust_Count * 6, J);

            Matrix_Multiply(J, iAdjust_Count * 6, 6, Jt, iAdjust_Count * 6, H);
            Matrix_Add(Sigma_H, H, iAdjust_Count * 6, Sigma_H);         //∑H, JJ'到位
            Matrix_Multiply(J, iAdjust_Count * 6, 6, E_6, 1, JEt);      //JE'到位

            //Disp(JEt, iAdjust_Count * 6, 1);
            Vector_Add(Sigma_JEt, JEt, iAdjust_Count * 6, Sigma_JEt);
        }
        printf("%f\n", fSum_e);

        Add_I_Matrix(Sigma_H, iAdjust_Count * 6);
        Solve_Linear_Gause(Sigma_H, iAdjust_Count * 6, Sigma_JEt, Delta_X, &iResult);
        Matrix_Multiply(Delta_X, 1, iAdjust_Count * 6, (_T)-1, Delta_X);
        if (fSum_e_Pre <= fSum_e || !iResult)
            break;

        for (i = 0; i < iAdjust_Count; i++)
        {
            se3_2_SE3(&Delta_X[6 * i], Delta_Pose);
            Matrix_Multiply(Delta_Pose, 4, 4, Camera[i], 4, Camera[i]);
        }
        fSum_e_Pre = fSum_e;
        printf("iIter:%d %.10f\n", iIter, fSum_e);
    }
    return;
}

static void Test_3()
{//稀疏矩阵，小规模
    typedef float _T;
    int i, j, iResult, iMeasurement_Count, iCamera_Count;
    _T(*pKsi)[7], (*Camera)[4 * 4];
    Measurement<_T>* pMeasurement;
    Pose_Graph_Sigma_H<_T> oPose_Graph;
    iResult = bTemp_Load_Data("D:\\Samp\\YBKJ\\Slam_Test\\Slam_Test\\Sample\\sphere.g2o", &pKsi, &iCamera_Count, &pMeasurement, &iMeasurement_Count);
    Camera = (_T(*)[4 * 4])pMalloc(&oMatrix_Mem, iCamera_Count * 4 * 4 * sizeof(_T));
    int iAdjust_Count = 10;
    for (i = 0; i < iAdjust_Count; i++)
        TQ_2_Rt(pKsi[i], Camera[i]);
    Free(&oMatrix_Mem, pKsi);
    Init_Pose_Graph(pMeasurement, Min(iAdjust_Count-1, iMeasurement_Count), Min(iAdjust_Count, iCamera_Count), &oPose_Graph);

    int iIter;
    union { _T Ti[4 * 4]; _T Ti_Inv[4 * 4]; };
    union { _T M_4x4[4 * 4]; _T M_Inv_4x4[4 * 4]; };
    union { _T Tj[4 * 4]; _T Tj_Inv[4 * 4]; };
    _T E_4x4[4 * 4], E_6[6], J_Inv[6 * 6], Adj[6 * 6],
        Jt[12 * 6], J[6 * 12], H[12 * 12], JEt[12], Delta_Pose[4 * 4], Temp[6 * 6],
        * Sigma_JEt, * Delta_X;
    _T fSum_e, fSum_e_Pre = (_T)1e10;
    Sparse_Matrix<_T> oSigma_H;
    Sigma_JEt = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
    Delta_X = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
    Init_Sparse_Matrix(&oSigma_H, oPose_Graph.m_iCamera_Data_Count * 2 * 6 * 6, iAdjust_Count * 6, iAdjust_Count * 6);
    
    for (iIter = 0;; iIter++)
    {
        fSum_e = 0;
        Reset_Pose_Graph(oPose_Graph);
        Reset_Sparse_Matrix(&oSigma_H);
        memset(Sigma_JEt, 0, iAdjust_Count * 6 * sizeof(_T));

        for (i = 0; i < iAdjust_Count - 1; i++)
        {//没一个测量都带来一个调整权重
            Measurement<_T> oM = pMeasurement[i];
            memcpy(Ti, Camera[oM.m_Camera_Index[0]], 4 * 4 * sizeof(_T));
            memcpy(Tj, Camera[oM.m_Camera_Index[1]], 4 * 4 * sizeof(_T));
            TQ_2_Rt(oM.Delta_ksi, M_4x4);
            Get_Inv_Matrix_Row_Op_2(M_4x4, M_Inv_4x4, 4, &iResult);
            if (!iResult)break;
            Get_Inv_Matrix_Row_Op_2(Ti, Ti_Inv, 4, &iResult);
            if (!iResult)break;

            Matrix_Multiply(Ti_Inv, 4, 4, Tj, 4, E_4x4);
            Matrix_Multiply(M_Inv_4x4, 4, 4, E_4x4, 4, E_4x4);
            Matrix_Multiply(M_Inv_4x4, 4, 4, Ti_Inv, 4, E_4x4);
            Matrix_Multiply(E_4x4, 4, 4, Tj, 4, E_4x4);
            SE3_2_se3(E_4x4, E_6);

            for (j = 0; j < 6; j++)
                fSum_e += E_6[j] * E_6[j];

            //接着求雅可比
            Get_J_Inv(E_6, J_Inv);  //此处搞定了一个近似的J(-1)
            Get_Inv_Matrix_Row_Op_2(Tj, Tj_Inv, 4, &iResult);
            if (!iResult)break;
            Get_Adj(Tj_Inv, Adj);

            //此时形成两个雅可比, Ti的雅可比放在Jt的0-5列，Tj的雅可比放在Jt的6-11列
            //第一个雅可比
            Matrix_Multiply(J_Inv, 6, 6, Adj, 6, Temp);

            //∂eij/∂ξj
            Copy_Matrix_Partial(Temp, 6, 6, Jt, 12, 6, 0);

            //∂eij/∂ξi
            Matrix_Multiply(Temp, 6, 6, (_T)-1.f, Temp);
            Copy_Matrix_Partial(Temp, 6, 6, Jt, 12, 0, 0);

            Matrix_Transpose(Jt, 6, 12, J);
            Matrix_Multiply(J, 12, 6, Jt, 12, H);
            //Disp(Jt, 6, 12);
            //此处要把H矩阵散发到稀疏矩阵Sigma_H中去
            if (i == 8)
                printf("here");
            Distribute_Data(oPose_Graph, H, oM.m_Camera_Index[0], oM.m_Camera_Index[1]);
            //Disp(H, 12, 12, "H");
            //轮到搞JEt
            Matrix_Multiply(J, 12, 6, E_6, 1, JEt);      //JE'到位
            Vector_Add(&Sigma_JEt[oM.m_Camera_Index[0] * 6], JEt, 6, &Sigma_JEt[oM.m_Camera_Index[0] * 6]);
            Vector_Add(&Sigma_JEt[oM.m_Camera_Index[1] * 6], &JEt[6], 6, &Sigma_JEt[oM.m_Camera_Index[1] * 6]);
        }
        printf("iIter:%d %.10f\n", iIter, fSum_e);
        Copy_Data_2_Sparse(oPose_Graph, &oSigma_H);
        //Disp_Fillness(oSigma_H);
        //解个方程
        Add_I_Matrix(&oSigma_H, &iResult);
        Solve_Linear_Gause_1(oSigma_H, Sigma_JEt, Delta_X, &iResult);
        Matrix_Multiply(Delta_X, 1, iAdjust_Count * 6, (_T)-1, Delta_X);
        if (fSum_e_Pre <= fSum_e || !iResult)
            break;

        for (i = 0; i < iAdjust_Count; i++)
        {
            se3_2_SE3(&Delta_X[6 * i], Delta_Pose);
            Matrix_Multiply(Delta_Pose, 4, 4, Camera[i], 4, Camera[i]);
        }
        fSum_e_Pre = fSum_e;
    }
    return;
}
static void Test_4()
{//试一下信息矩阵
    typedef float _T;
    _T(*pKsi)[7];
    Measurement<_T>* pMeasurement;
    int i, j, iIter, iResult, iPoint_Count, iMeasurement_Count;
    union { _T Ti[4 * 4]; _T Ti_Inv[4 * 4]; };
    union { _T M_4x4[4 * 4]; _T M_Inv_4x4[4 * 4]; };
    union { _T Tj[4 * 4]; _T Tj_Inv[4 * 4]; };

    _T E_6[6], E_4x4[4 * 4], //E可以是一个矩阵
        fSum_e, fSum_e_Pre = (_T)1e10;
    _T J_Inv[6 * 6], Adj[6 * 6], Temp[6 * 6], Delta_Pose[4 * 4];
    _T *Jt, *J, *Jt_Sigma,*Jt_Sigma_E, * H, * Sigma_H, *Sigma_Jt_Sigma_E,* Delta_X;

    _T(*Camera)[4 * 4];
    iResult = bTemp_Load_Data("D:\\Samp\\YBKJ\\Slam_Test\\Slam_Test\\Sample\\sphere.g2o", &pKsi, &iPoint_Count, &pMeasurement, &iMeasurement_Count);
    if (!iResult)return;
    int iAdjust_Count = 200;

    Jt = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 36 * sizeof(_T));
    J = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 36 * sizeof(_T));
    H = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
    Jt_Sigma = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 36 * sizeof(_T));
    Jt_Sigma_E = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
    Sigma_Jt_Sigma_E = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));

    Delta_X = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
    Sigma_H = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
    Camera = (_T(*)[4 * 4])pMalloc(&oMatrix_Mem, iAdjust_Count * 4 * 4 * sizeof(_T));
    for (i = 0; i < iAdjust_Count; i++)
        TQ_2_Rt(pKsi[i], Camera[i]);
    for (iIter = 0;; iIter++)
    {
        fSum_e = 0;
        memset(Sigma_H, 0, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
        memset(Sigma_Jt_Sigma_E, 0, iAdjust_Count * 6 * sizeof(_T));
        
        for (i = 0; i < iAdjust_Count - 1; i++)
        {//没一个测量都带来一个调整权重
            Measurement<_T> oM = pMeasurement[i];
            memcpy(Ti, Camera[oM.m_Camera_Index[0]], 4 * 4 * sizeof(_T));
            memcpy(Tj, Camera[oM.m_Camera_Index[1]], 4 * 4 * sizeof(_T));
            TQ_2_Rt(oM.Delta_ksi, M_4x4);
            Get_Inv_Matrix_Row_Op_2(M_4x4, M_Inv_4x4, 4, &iResult);
            if (!iResult)break;
            Get_Inv_Matrix_Row_Op_2(Ti, Ti_Inv, 4, &iResult);
            if (!iResult)break;
            //Disp(M_Inv_4x4, 4, 4, "M(-1)");
            //Disp(Ti_Inv, 4, 4, "Ti_Inv");
            Matrix_Multiply(M_Inv_4x4, 4, 4, Ti_Inv, 4, E_4x4);     //=Tij(-1) * Ti(-1)
            Matrix_Multiply(E_4x4, 4, 4, Tj, 4, E_4x4);             //=Tij(-1) * Ti(-1) * Tj
            SE3_2_se3(E_4x4, E_6);  //注意，书上是错的，正确的是eij = ln(Tij(-1) * Ti(-1) * Tj)
            for (j = 0; j < 6; j++)
                fSum_e += E_6[j] * E_6[j];

            memset(Jt, 0, iAdjust_Count * 36 * sizeof(_T));

            //接着求雅可比
            Get_J_Inv(E_6, J_Inv);  //此处搞定了一个近似的J(-1)
            Get_Inv_Matrix_Row_Op_2(Tj, Tj_Inv, 4, &iResult);
            if (!iResult)break;
            Get_Adj(Tj_Inv, Adj);

           
            //此时形成两个雅可比
            //第一个雅可比Jt
            Matrix_Multiply(J_Inv, 6, 6, Adj, 6, Temp);
            //∂eij/∂ξj
            Copy_Matrix_Partial(Temp, 6, 6, Jt, iAdjust_Count * 6, oM.m_Camera_Index[1] * 6, 0);

            //∂eij/∂ξi
            Matrix_Multiply(Temp, 6, 6, (_T)-1.f, Temp);
            //Transpose_Multiply(Temp, 6, 6, J1t,0);
            Copy_Matrix_Partial(Temp, 6, 6, Jt, iAdjust_Count * 6, oM.m_Camera_Index[0] * 6, 0);

            //printf("%d %d\n", oM.m_Pose_Index[0], oM.m_Pose_Index[1]);
            Matrix_Transpose(Jt, 6, iAdjust_Count * 6, J);
            
            //求 J * Z(-1)* Jt    
            _T JtJ[6 * 6];
            Matrix_Multiply(Jt, 6, iAdjust_Count * 6,J,6, JtJ);
            Matrix_Multiply(J, iAdjust_Count * 6, 6, JtJ, 6, Jt_Sigma);
            
            Matrix_Multiply(Jt_Sigma, iAdjust_Count * 6, 6, Jt, iAdjust_Count * 6, H);
            Matrix_Add(Sigma_H, H, iAdjust_Count * 6, Sigma_H);     //J * Z(-1)* Jt                       

            //求 J*Z(-1)*e
            Matrix_Multiply(Jt_Sigma, iAdjust_Count * 6, 6, E_6, 1, Jt_Sigma_E);
            Vector_Add(Sigma_Jt_Sigma_E, Jt_Sigma_E, iAdjust_Count * 6,Sigma_Jt_Sigma_E);
            //Disp_Fillness(Sigma_H, iAdjust_Count*6, iAdjust_Count* 6);
        }

        //解方程J1*J1' Δx = -J1 * f(x)'
        Add_I_Matrix(Sigma_H, iAdjust_Count * 6);
        Solve_Linear_Gause(Sigma_H, iAdjust_Count * 6, Sigma_Jt_Sigma_E, Delta_X, &iResult);
        Matrix_Multiply(Delta_X, 1, iAdjust_Count * 6, (_T)-1, Delta_X);
        //Disp(Delta_X, 1, iAdjust_Count * 6);

        if (fSum_e_Pre <= fSum_e || !iResult)
            break;

        for (i = 0; i < iAdjust_Count; i++)
        {
            se3_2_SE3(&Delta_X[6 * i], Delta_Pose);
            Matrix_Multiply(Delta_Pose, 4, 4, Camera[i], 4, Camera[i]);
        }
        fSum_e_Pre = fSum_e;
        printf("iIter:%d %.10f\n", iIter, fSum_e);
    }
}

static void Test_5()
{//引入信息矩阵，看看够不够丝滑
    typedef float _T;
    int i, j, iResult, iMeasurement_Count, iCamera_Count;
    _T(*pKsi)[7], (*Camera)[4 * 4];
    Measurement<_T>* pMeasurement;
    Pose_Graph_Sigma_H<_T> oPose_Graph;
    iResult = bTemp_Load_Data("D:\\Samp\\YBKJ\\Slam_Test\\Slam_Test\\Sample\\sphere.g2o", &pKsi, &iCamera_Count, &pMeasurement, &iMeasurement_Count);
    int iAdjust_Count = 200;
    iCamera_Count = 2500;
    Camera = (_T(*)[4 * 4])pMalloc(&oMatrix_Mem, iCamera_Count * 4 * 4 * sizeof(_T));
    //int iAdjust_Count = iMeasurement_Count;
    for (i = 0; i < iCamera_Count; i++)
        TQ_2_Rt(pKsi[i], Camera[i]);
    Free(&oMatrix_Mem, pKsi);
    Init_Pose_Graph(pMeasurement, iAdjust_Count-1, iAdjust_Count, &oPose_Graph);

    int iIter;
    union { _T Ti[4 * 4]; _T Ti_Inv[4 * 4]; };
    union { _T M_4x4[4 * 4]; _T M_Inv_4x4[4 * 4]; };
    union { _T Tj[4 * 4]; _T Tj_Inv[4 * 4]; };
    _T E_4x4[4 * 4], E_6[6], J_Inv[6 * 6], Adj[6 * 6],
        Jt[12 * 6], J[6 * 12], H[12 * 12], Delta_Pose[4 * 4], Temp[6 * 6],
        * Sigma_J_Z_Inv_E, * Delta_X;
    _T fSum_e, e, fSum_e_Pre = (_T)1e10;
    Sparse_Matrix<_T> oSigma_H;
    Sigma_J_Z_Inv_E = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * sizeof(_T));
    Delta_X = (_T*)pMalloc(&oMatrix_Mem, iCamera_Count * 6 * sizeof(_T));
    //Disp_Mem(&oMatrix_Mem, 0);
    _T(*pPoint_3D)[3] = (_T(*)[3])pMalloc(&oMatrix_Mem, iCamera_Count * 3 * sizeof(_T));

    for (iIter = 0;; iIter++)
    {
        fSum_e = 0;
        Reset_Pose_Graph(oPose_Graph);
        Init_Sparse_Matrix(&oSigma_H, iAdjust_Count * 3 * 6 * 6, iAdjust_Count * 6, iAdjust_Count * 6);
        memset(Sigma_J_Z_Inv_E, 0, iCamera_Count * 6 * sizeof(_T));

        for (i = 0; i < iAdjust_Count-1; i++)
        {//没一个测量都带来一个调整权重
            Measurement<_T> oM = pMeasurement[i];
            if (oM.m_Camera_Index[0] >= iCamera_Count || oM.m_Camera_Index[1] >= iCamera_Count)
                continue;
            memcpy(Ti, Camera[oM.m_Camera_Index[0]], 4 * 4 * sizeof(_T));
            memcpy(Tj, Camera[oM.m_Camera_Index[1]], 4 * 4 * sizeof(_T));
            TQ_2_Rt(oM.Delta_ksi, M_4x4);
            Get_Inv_Matrix_Row_Op_2(M_4x4, M_Inv_4x4, 4, &iResult);
            if (!iResult)break;
            Get_Inv_Matrix_Row_Op_2(Ti, Ti_Inv, 4, &iResult);
            if (!iResult)break;

            //注意，书上是错的
            Matrix_Multiply(M_Inv_4x4, 4, 4, Ti_Inv, 4, E_4x4);
            Matrix_Multiply(E_4x4, 4, 4, Tj, 4, E_4x4);			//=Tij(-1) * Ti(-1) * Tj

            SE3_2_se3(E_4x4, E_6);

            for (e = 0, j = 0; j < 3; j++)
                e += E_6[j] * E_6[j];
            fSum_e += e;
           
            //printf("Sum_e:%.10f\n", fSum_e);
            //接着求雅可比
            Get_J_Inv(E_6, J_Inv);  //此处搞定了一个近似的J(-1)
            Get_Inv_Matrix_Row_Op_2(Tj, Tj_Inv, 4, &iResult);
            if (!iResult)break;

            Get_Adj(Tj_Inv, Adj);
            //此时形成两个雅可比, Ti的雅可比放在Jt的0-5列，Tj的雅可比放在Jt的6-11列
            //第一个雅可比
            Matrix_Multiply(J_Inv, 6, 6, Adj, 6, Temp);
            //∂eij/∂ξj
            Copy_Matrix_Partial(Temp, 6, 6, Jt, 12, 6, 0);

            //∂eij/∂ξi  算第二个雅可比
            Matrix_Multiply(Temp, 6, 6, (_T)-1.f, Temp);
            Copy_Matrix_Partial(Temp, 6, 6, Jt, 12, 0, 0);
            Matrix_Transpose(Jt, 6, 12, J);
            //到此， J, Jt已经就绪，第一个第二个雅可比都紧凑放在Jt中

            //注意了，这里求的Z^-1 不是 H矩阵，H=JJt, 然而，Z^-1 = JtJ，矩阵是不可交换的！
            //所有的网上结论都是错的
            union {
                _T Z_Inv[6 * 6];
                _T J_Z_Inv[12 * 6];
                _T J_Z_Inv_E[12 * 1];
            };

            //先求H=J * Z^-1 * Jt
            Matrix_Multiply(Jt, 6, 12, J, 6, Z_Inv);        //=Z^-1 = JtJ
            Matrix_Multiply(J, 12, 6, Z_Inv, 6, J_Z_Inv);   //=J*Z^-1

            //H=J * Z^-1 * Jt   相当于取平方
            Matrix_Multiply(J_Z_Inv, 12, 6, Jt, 12, H);
            //此处要把H矩阵散发到稀疏矩阵Sigma_H中去
            Distribute_Data(oPose_Graph, H, oM.m_Camera_Index[0], oM.m_Camera_Index[1]);

            //再求 J * Z^1 * E
            Matrix_Multiply(J_Z_Inv, 12, 6, E_6, 1, J_Z_Inv_E);
            Vector_Add(&Sigma_J_Z_Inv_E[oM.m_Camera_Index[0] * 6], J_Z_Inv_E, 6, &Sigma_J_Z_Inv_E[oM.m_Camera_Index[0] * 6]);
            Vector_Add(&Sigma_J_Z_Inv_E[oM.m_Camera_Index[1] * 6], &J_Z_Inv_E[6], 6, &Sigma_J_Z_Inv_E[oM.m_Camera_Index[1] * 6]);
        }
        printf("iIter:%d %.10f\n", iIter, fSum_e);
        if (fSum_e_Pre <= fSum_e)
            break;

        Copy_Data_2_Sparse(oPose_Graph, &oSigma_H);
        //Add_I_Matrix(&oSigma_H, &iResult, (_T)100.f);   //此处终于需要改变ramda值了！
        Add_I_Matrix(&oSigma_H, &iResult, (_T)10.f);   //此处终于需要改变ramda值了！
        Compact_Sparse_Matrix(&oSigma_H);
        unsigned long long tStart = iGet_Tick_Count();
        Solve_Linear_Gause_1(oSigma_H, Sigma_J_Z_Inv_E, Delta_X, &iResult);
        printf("%lld\n", iGet_Tick_Count() - tStart);

        Free_Sparse_Matrix(&oSigma_H);
        Matrix_Multiply(Delta_X, 1, iCamera_Count * 6, (_T)-1, Delta_X);
        if (!iResult)
            break;
        
        if (!iResult)
            break;
        for (i = 0; i < iCamera_Count; i++)
        {
            se3_2_SE3(&Delta_X[6 * i], Delta_Pose);
            Matrix_Multiply(Delta_Pose, 4, 4, Camera[i], 4, Camera[i]);
        }
        fSum_e_Pre = fSum_e;
    }
}


int main()
{
    Init_Env();
    //Pose_Graph_Test_3();
	Test_Main();
    Free_Env();
#ifdef WIN32
    _CrtDumpMemoryLeaks();
#endif
}
