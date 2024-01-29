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
template<typename _T> struct Measurement {
    int m_Pose_Index[2];    //观察中的两个位姿
    _T Delta_ksi[7];        //存个ksi，前三项相当于位移，后4项为4元数
};
template<typename _T> int bTemp_Load_Data(const char *pcFile,_T (**ppT)[7], int* piPoint_Count,
    Measurement<_T> **ppMeasurement,int *piMeasure_Count)
{//装入2500个点
    _T(*pPose_7) [7];
    Measurement<_T>* pMeasurement;
    int i,iResult,iPoint_Count=2500;
    FILE* pFile = fopen(pcFile, "rb");
    if (!pFile)
        return 0;

    pPose_7 = (_T(*)[7])pMalloc(&oMatrix_Mem, 4000 * 7 * sizeof(_T));
    //for (i = 0; i < iPoint_Count; i++)
    for (i = 0; ; i++)
    {
        float Pose[7];
        int iCur;
        if ( (iResult=fscanf(pFile, "VERTEX_SE3:QUAT %d %f %f %f %f %f %f %f ", &iCur, &Pose[0], &Pose[1], &Pose[2], &Pose[4], &Pose[5], &Pose[6], &Pose[3])) <8)
            break;
        for (int j = 0; j < 7; j++)
            pPose_7[i][j] = Pose[j];
    }
    iPoint_Count = i;
    Shrink(&oMatrix_Mem,pPose_7, iPoint_Count * 7 * sizeof(_T));

    //再装入观察数据
    pMeasurement = (Measurement<_T>*)pMalloc(&oMatrix_Mem, 11000 * sizeof(Measurement<_T>));
    for (i = 0;; i++)
    {
        int j,iCur;
        Measurement<_T>* poM = &pMeasurement[i];
        float Pose[7];
        if ( (iResult=fscanf(pFile, "EDGE_SE3:QUAT %d %d %f %f %f %f %f %f %f ", &poM->m_Pose_Index[0], &poM->m_Pose_Index[1],
            &Pose[0], &Pose[1], &Pose[2],
            &Pose[4], &Pose[5], &Pose[6], &Pose[3]))<9 )
        break;
        for (j = 0; j < 7; j++)
            poM->Delta_ksi[j] = Pose[j];
        //Disp(&poM->Delta_ksi[3], 4, 1);
        for (j = 0; j < 21; j++)
            fscanf(pFile, "%d ", &iCur);

    }
    Shrink(&oMatrix_Mem, pMeasurement,i * sizeof(Measurement<_T>));
    fclose(pFile);

    *piPoint_Count = iPoint_Count;
    *ppT = pPose_7;
    *ppMeasurement = pMeasurement;
    *piMeasure_Count = i;


    return 1;
}
template<typename _T>void TQ_2_Rt(_T TQ[7], _T Rt[4 * 4])
{//将一个se3上的数据转换为Rt. se3数据前三项为位移，后4项为4元数
    //TQ中的T表示Translation Q表示为Quaternion
    //此函数跟sophus已经完全一致。需要留意的是四元数中实部与虚部各自的位置
    _T R[3 * 3];
    Quaternion_2_Rotation_Matrix(&TQ[3], R);
    Gen_Homo_Matrix(R, TQ, Rt);
    return;
}

void Test_1()
{//重做各种se3 <-> SE3的变换
    typedef float _T;
    ////第一步， e的由来，级数计算
    //printf("%.10f\n", fGet_e());

    ////试一下两种方法求exp(A)
    //{
    //    float theta[4] = { 0,0,1,PI / 6.f };
    //    float theta_3[3];
    //    float theta_hat[3 * 3],exp_theta[3*3];

    //    Rotation_Vector_4_2_3(theta, theta_3);
    //    Hat(theta_3, theta_hat);
    //    Disp(theta_hat, 3, 3, "theta_hat");
    //    Exp_Ref(theta_hat,3, exp_theta);
    //    Disp(exp_theta, 3, 3, "exp_theta");

    //    Rotation_Vector_2_Matrix(theta, exp_theta);
    //    Disp(exp_theta, 3, 3, "R");
    //}

    {//试试第十讲的数据
        _T ksi_1[] = { -0.125664f, (_T)-1.53894e-17, 99.9999f, (_T)-4.3325e-17,0.706662f, (_T)4.32706e-17, 0.707551f},
            ksi_2[] = { -0.250786f, -0.0328449f, 99.981f, -0.0465295f,0.705413f, 0.0432253f, 0.705946f },
            Delta_ksi_12[] = {-0.0187953f, 0.0328449f, -0.125146f,  0.997981f,0.0634648f, -0.000250128f, 0.00237634f};
        _T T1[4 * 4], T2[4 * 4], Delta_T12[4 * 4];
        int iResult;

        TQ_2_Rt(Delta_ksi_12, Delta_T12);
        Disp(Delta_T12, 4, 4, "Delta_T12");
        
        TQ_2_Rt(ksi_1, T1);
        TQ_2_Rt(ksi_2, T2);
        //Disp(T1, 4, 4, "T1");

        //假设 T1 * ΔT12 = T2 => ΔT12 = T1(-1)*T2
        Get_Inv_Matrix_Row_Op_2(T1, Delta_T12, 4, &iResult);    //-T1(-1)
        Matrix_Multiply(Delta_T12, 4, 4, T2, 4, Delta_T12);     //ΔT12=T1(-1)*T2
        //Disp(Delta_T12, 4, 4, "Delta_T12");
        //Disp(&Delta_ksi_12[3], 1, 4, "Q");
        //至此，终于对上了

        //把Homo_Matrix又变换到se3上
        _T R[3 * 3], t[3],Rotation_Vector[4];
        Get_R_t(Delta_T12, R, t);
        //Disp(Q, 1, 4, "Q");

        //如何表征eij?
        Quaternion_2_Rotation_Vector(&Delta_ksi_12[3], Rotation_Vector);
        //Disp(Rotation_Vector, 1, 4,"Rotation_Vector");
    }

    {//第三个实验，算个error
        _T Error[6],ksi_1[] = { -0.125664f,(_T)-1.53894e-17, 99.9999f, (_T)-4.3325e-17,0.706662f, (_T)4.32706e-17, 0.707551f },
            ksi_2[] = { -0.250786f, -0.0328449f, 99.981f, -0.0465295f,0.705413f, 0.0432253f, 0.705946f },
            Delta_ksi_12[] = { -0.0187953f, 0.0328449f, -0.125146f,  0.997981f,0.0634648f, -0.000250128f, 0.00237634f };
        _T T1[4 * 4], T2[4 * 4], Delta_T12[4 * 4], 
            T1_Inv[4 * 4], Delta_T12_Inv[4 * 4],Temp[4*4];
        int iResult;
        TQ_2_Rt(ksi_1, T1);
        TQ_2_Rt(ksi_2, T2);
        TQ_2_Rt(Delta_ksi_12, Delta_T12);
        //Disp(T1, 4, 4, "T1");
        //Disp(T2, 4, 4, "T2");
        //Disp(Delta_T12, 4, 4, "Delta_T12");

        Get_Inv_Matrix_Row_Op_2(T1, T1_Inv, 4, &iResult);
        Get_Inv_Matrix_Row_Op_2(Delta_T12, Delta_T12_Inv, 4, &iResult);
        Matrix_Multiply(Delta_T12_Inv, 4, 4, T1_Inv, 4, Temp);
        Matrix_Multiply(Temp, 4, 4, T2, 4, Temp);
        //Disp(Delta_T12_Inv, 4, 4, "Delta_T12_Inv");
        //Disp(T1_Inv, 4, 4, "T1_Inv");
        SE3_2_se3(Temp, Error);
        //Disp(Error, 6,1,"Error");
        //Disp(Temp, 4, 4, "T12(-1)*T1(-1)*T2");
    }    
    return;
}
static void Test_2()
{
    typedef float _T;
    _T(*pKsi)[7];
    Measurement<_T>* pMeasurement;
    int i,j,iIter,iResult,iPoint_Count, iMeasurement_Count;
    union {
        _T Ti[4 * 4];
        _T Ti_Inv[4 * 4];
    };
    _T Tj[4 * 4], M_4x4[4 * 4], M_Inv_4x4[4 * 4], E_6[6],
        E_4x4[4 * 4], //E可以是一个矩阵
        fSum_e, fSum_e_Pre = (_T)1e10;
    _T Sigma_H[12*12],Jt[6*6];
    
    iResult=bTemp_Load_Data("D:\\Samp\\YBKJ\\Slam_Test\\Slam_Test\\Sample\\sphere.g2o", &pKsi, &iPoint_Count,&pMeasurement,&iMeasurement_Count);
    if (!iResult)return;
    
    for (iIter = 0;; iIter++)
    {
        fSum_e = 0;
        for (i = 0; i < iMeasurement_Count; i++)
        {//没一个测量都带来一个调整权重
            Measurement<_T> oM = pMeasurement[i];
            _T *ksi_i = pKsi[oM.m_Pose_Index[0]],
                *ksi_j = pKsi[oM.m_Pose_Index[1]];
            TQ_2_Rt(ksi_i, Ti);
            TQ_2_Rt(ksi_j, Tj);
            TQ_2_Rt(oM.Delta_ksi, M_4x4);
            Get_Inv_Matrix_Row_Op_2(M_4x4, M_Inv_4x4, 4, &iResult);
            if (!iResult)
                break;
            Get_Inv_Matrix_Row_Op_2(Ti, Ti_Inv, 4, &iResult);
            if (!iResult)
                break;
            Matrix_Multiply(Ti_Inv, 4, 4, Tj,4, E_4x4);
            Matrix_Multiply(M_Inv_4x4, 4, 4, E_4x4, 4, E_4x4);
            Matrix_Multiply(M_Inv_4x4, 4, 4, Ti_Inv, 4, E_4x4);
            Matrix_Multiply(E_4x4, 4, 4, Tj, 4, E_4x4);
            for (j = 0; j < 6; j++)
                fSum_e += E_6[j] * E_6[j];
            
            //接着求雅可比

        }

        printf("%f\n", fSum_e);
    }
}

int main()
{
    Init_Env();
	Test_Main();
    Free_Env();
#ifdef WIN32
    _CrtDumpMemoryLeaks();
#endif
}
