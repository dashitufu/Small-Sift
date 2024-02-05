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
    int i,j,iIter,iResult,iPoint_Count, iMeasurement_Count;
    union { _T Ti[4 * 4]; _T Ti_Inv[4 * 4]; };
    union { _T M_4x4[4 * 4]; _T M_Inv_4x4[4 * 4]; };
    union { _T Tj[4 * 4]; _T Tj_Inv[4 * 4]; };

    _T E_6[6], E_4x4[4 * 4], //E可以是一个矩阵
        fSum_e, fSum_e_Pre = (_T)1e10;
    _T J_Inv[6*6],Adj[6*6],Temp[6*6], Delta_Pose[4 * 4];
    _T *Jt,*J,*H,*JEt,*Sigma_H,*Sigma_JEt,*Delta_X;
    _T(*Camera)[4 * 4];
    iResult=bTemp_Load_Data("D:\\Samp\\YBKJ\\Slam_Test\\Slam_Test\\Sample\\sphere.g2o", &pKsi, &iPoint_Count,&pMeasurement,&iMeasurement_Count);
    if (!iResult)return;
    int iAdjust_Count = 10;
    Jt = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 36 * sizeof(_T));
    J = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 36 * sizeof(_T));
    H = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
    JEt = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
    Sigma_JEt = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
    Delta_X = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
    Sigma_H = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
    Camera = (_T(*)[4*4])pMalloc(&oMatrix_Mem, iAdjust_Count * 4 * 4 * sizeof(_T));
    for (i = 0; i < iAdjust_Count; i++)
        TQ_2_Rt(pKsi[i], Camera[i]);

    for (iIter = 0;; iIter++)
    {
        fSum_e = 0;
        memset(Sigma_H, 0, iAdjust_Count * 6 * iAdjust_Count * 6 * sizeof(_T));
        memset(Sigma_JEt, 0, iAdjust_Count * 6 * sizeof(_T));
        for (i = 0; i < iAdjust_Count-1; i++)
        {//没一个测量都带来一个调整权重
            Measurement<_T> oM = pMeasurement[i];
            //_T *ksi_i = pKsi[oM.m_Pose_Index[0]],
                //*ksi_j = pKsi[oM.m_Pose_Index[1]];
            //TQ_2_Rt(ksi_i, Ti);
            //TQ_2_Rt(ksi_j, Tj);
            memcpy(Ti, Camera[oM.m_Camera_Index[0]],4*4*sizeof(_T));
            memcpy(Tj, Camera[oM.m_Camera_Index[1]], 4 * 4 * sizeof(_T));
            TQ_2_Rt(oM.Delta_ksi, M_4x4);
            Get_Inv_Matrix_Row_Op_2(M_4x4, M_Inv_4x4, 4, &iResult);
            if (!iResult)break;
            Get_Inv_Matrix_Row_Op_2(Ti, Ti_Inv, 4, &iResult);
            if (!iResult)break;
            Matrix_Multiply(Ti_Inv, 4, 4, Tj,4, E_4x4);
            Matrix_Multiply(M_Inv_4x4, 4, 4, E_4x4, 4, E_4x4);
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
            //第一个雅可比
            Matrix_Multiply(J_Inv, 6, 6, Adj, 6, Temp);

            //∂eij/∂ξj
            Copy_Matrix_Partial(Temp, 6, 6, Jt, iAdjust_Count * 6, oM.m_Camera_Index[1] * 6, 0);

            //∂eij/∂ξi
            Matrix_Multiply(Temp, 6, 6, (_T) - 1.f, Temp);
            Copy_Matrix_Partial(Temp, 6, 6, Jt, iAdjust_Count * 6, oM.m_Camera_Index[0] * 6, 0);

            //printf("%d %d\n", oM.m_Pose_Index[0], oM.m_Pose_Index[1]);
            Matrix_Transpose(Jt, 6, iAdjust_Count * 6, J);
            Matrix_Multiply(J, iAdjust_Count * 6, 6, Jt, iAdjust_Count * 6, H);
            Matrix_Add(Sigma_H, H, iAdjust_Count * 6, Sigma_H);         //∑H, JJ'到位
            Matrix_Multiply(J, iAdjust_Count * 6, 6, E_6, 1, JEt);      //JE'到位
            Vector_Add(Sigma_JEt, JEt, iAdjust_Count * 6, Sigma_JEt);
        }
        printf("%f\n", fSum_e);

        /*Disp(Sigma_H, iAdjust_Count * 6, iAdjust_Count * 6,"Sigma_H");
        Cholosky_Decompose(Sigma_H, iAdjust_Count * 6, Sigma_H);
        Transpose_Multiply(Sigma_H, iAdjust_Count * 6, iAdjust_Count * 6,Sigma_H);
        Disp(Sigma_H, iAdjust_Count * 6, iAdjust_Count * 6,"Sigma_H");*/

        //Disp(Sigma_JEt, iAdjust_Count * 6, 1);
        
        Add_I_Matrix(Sigma_H,iAdjust_Count*6);
        Solve_Linear_Gause(Sigma_H, iAdjust_Count*6, Sigma_JEt, Delta_X, &iResult);
        //Conjugate_Gradient(Sigma_H, iAdjust_Count * 6, Sigma_JEt, Delta_X);

        Matrix_Multiply(Delta_X, 1, iAdjust_Count * 6, (_T)-1, Delta_X);
        if (fSum_e_Pre <= fSum_e || !iResult)
            break;

        for (i = 0; i < iAdjust_Count; i++)
        {
            se3_2_SE3(&Delta_X[6 * i], Delta_Pose);
            Matrix_Multiply(Delta_Pose, 4, 4, Camera[i], 4, Camera[i]);
        }
        fSum_e_Pre = fSum_e;
        printf("iIter:%d %f\n", iIter, fSum_e);
    }
    return;
}

static void Test_3()
{//稀疏矩阵，小规模
    typedef float _T;
    int i,j,iResult,iMeasurement_Count,iCamera_Count;
    _T(*pKsi)[7], (*Camera)[4 * 4];
    Measurement<_T>* pMeasurement;
    Pose_Graph_Sigma_H<_T> oPose_Graph;
    iResult = bTemp_Load_Data("D:\\Samp\\YBKJ\\Slam_Test\\Slam_Test\\Sample\\sphere.g2o", &pKsi, &iCamera_Count, &pMeasurement, &iMeasurement_Count);
    Camera = (_T(*)[4 * 4])pMalloc(&oMatrix_Mem, iCamera_Count * 4 * 4 * sizeof(_T));
    int iAdjust_Count = 10;
    for (i = 0; i < iAdjust_Count; i++)
        TQ_2_Rt(pKsi[i], Camera[i]);
    Free(&oMatrix_Mem, pKsi);
    Init_Pose_Graph(pMeasurement,Min(iAdjust_Count,iMeasurement_Count),Min(iAdjust_Count,iCamera_Count),&oPose_Graph);

    int iIter;
    union { _T Ti[4 * 4]; _T Ti_Inv[4 * 4]; };
    union { _T M_4x4[4 * 4]; _T M_Inv_4x4[4 * 4]; };
    union { _T Tj[4 * 4]; _T Tj_Inv[4 * 4]; };
    _T E_4x4[4*4], E_6[6], J_Inv[6*6], Adj[6*6],
        Jt[12*6], J[6*12], H[12*12], JEt[12], Delta_Pose[4 * 4],Temp[6*6],
        *Sigma_JEt, * Delta_X;
    _T fSum_e, fSum_e_Pre = (_T)1e10;
    Sparse_Matrix<_T> oSigma_H;
    Sigma_JEt = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
    Delta_X = (_T*)pMalloc(&oMatrix_Mem, iAdjust_Count * 6 * sizeof(_T));
    Init_Sparse_Matrix(&oSigma_H, oPose_Graph.m_iCamera_Data_Count*2 * 6 * 6, iAdjust_Count * 6, iAdjust_Count * 6);
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
            Distribute_Data(oPose_Graph, H, oM.m_Camera_Index[0], oM.m_Camera_Index[1]);
            //Disp(H, 12, 12, "H");
            //轮到搞JEt
            Matrix_Multiply(J, 12, 6, E_6, 1, JEt);      //JE'到位
            Vector_Add(&Sigma_JEt[oM.m_Camera_Index[0] * 6], JEt, 6, &Sigma_JEt[oM.m_Camera_Index[0] * 6]);
            Vector_Add(&Sigma_JEt[oM.m_Camera_Index[1] * 6], &JEt[6], 6, &Sigma_JEt[oM.m_Camera_Index[1] * 6]);
        }
        printf("iIter:%d %f\n", iIter, fSum_e);
        Copy_Data_2_Sparse(oPose_Graph, &oSigma_H);
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
void Gause_Test()
{
    typedef float _T;
    _T X[3],A[] = { 1,2,3,4,4,6,7,7,9 }, B[3] = { 6,-1,5 };
    int iResult;
    Sparse_Matrix<_T> oA;
    Init_Sparse_Matrix(&oA, 9, 3, 3);
    Dense_2_Sparse((_T*)A, 3, 3, &oA);
    //Disp((_T*)A, 3, 3, "A");
    Solve_Linear_Gause_1(oA, B, X, &iResult);
    Disp(X, 1, 3, "X");

    return;
}
int main()
{
    Init_Env();
    //Gause_Test();
    //Test_2();
	Test_Main();
    Free_Env();
#ifdef WIN32
    _CrtDumpMemoryLeaks();
#endif
}
