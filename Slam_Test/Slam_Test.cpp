// Slam_Test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#pragma once
#include <iostream>
#include "Common.h"
#include "Image.h"
//#include "sift.h"
#include "Matrix.h"
#include "Reconstruct.h"

extern void Test_Main();
extern "C"
{
#include "Buddy_System.h"
}


static void Test_1()
{
    typedef double _T;
    _T A[3][3] = { {322688.000000, -400.000000,     116224.000000},
        {-400.000000,     306984.000000, -78240.000000},
        {116224.000000, -78240.000000,   70784.000000 } };
    _T fEigen_Value, Eigen_Vector[3];
    unsigned long long tStart;
    int i;

    //方法1，一般反幂法
    tStart = iGet_Tick_Count();
    for(i=0;i<1000000;i++)
        if (!bInverse_Power((_T*)A, 3, &fEigen_Value, Eigen_Vector))
            printf("err");
    printf("%lld\n", iGet_Tick_Count() - tStart);

    Disp(Eigen_Vector, 1, 3, "Eigen_Vector");
    bTest_Eigen((_T*)A, 3, fEigen_Value, Eigen_Vector);

    ////方法2，3x3反幂法
    //tStart = iGet_Tick_Count();
    //for (i = 0; i < 1000000; i++)
    //    if (!bInverse_Power(A, &fEigen_Value, Eigen_Vector))
    //	    printf("err");
    //printf("%lld\n", iGet_Tick_Count() - tStart);
    //Disp(Eigen_Vector, 1, 3, "Eigen_Vector");
    //bTest_Eigen((_T*)A,3, fEigen_Value, Eigen_Vector);

    ////printf("Eigen Value:%f\n", fEigen_Value);

    //_T B[3];
    //Matrix_Multiply((_T*)A, 3, 3, Eigen_Vector, 1, B);
    //Disp(B, 1, 3, "B");
    //Matrix_Multiply(Eigen_Vector, 1, 3, fEigen_Value, B);
    //Disp(B, 1, 3, "B");
    //printf("Error:%f\n", fGet_Error((_T*)A, 3, 3, Eigen_Vector));
    //SVD_Info oSVD;
    //int iResult;
   
    //SVD_Alloc(3, 3, &oSVD, (_T*)A);
    //svd_3((_T*)A, oSVD, &iResult);
    //Eigen_Vector[0] = ((_T*)oSVD.Vt)[6];
    //Eigen_Vector[1] = ((_T*)oSVD.Vt)[7];
    //Eigen_Vector[2] = ((_T*)oSVD.Vt)[8];
    //fEigen_Value = ((_T*)oSVD.S)[2];
    //Disp(Eigen_Vector, 1, 3, "Eigen_Vector");
    ////printf("Error:%f\n", fGet_Error((_T*)A, 3, 3, Eigen_Vector));
    //Free_SVD(&oSVD);
    return;
}

//static void Test_1()
//{
//    typedef float _T;
//    const int n = 12;
//    _T A[n * n], 
//        L[n * n]={},
//        U[n * n] = {};
//    int i,j,k;
//    for (i = 0; i < n; i++)
//        L[i * n + i] = 1;
//    for (i = 1, k = 1; i < n; i++)
//        for (j = 0; j < i; j++, k++)
//            L[i * n + j] = (_T)k;
//    
//    for (i = 0; i < n; i++)
//        for (j = i; j < n; j++, k++)
//            U[i * n + j] = (_T)k;
//
//    //Disp(L, n, n, "L");
//    //Disp(U, n, n, "U");
//    Matrix_Multiply(L, n, n, U, n, A);
//
//    unsigned long long tStart = iGet_Tick_Count();
//    for(int i=0;i<10000;i++)
//        LU_Decompose(A, L, U, n);
//    printf("%lld\n", iGet_Tick_Count() - tStart);
//
//    /*Disp(A, n, n, "A");
//    Disp(L, n, n, "L");
//    Disp(U, n, n, "U");*/
//    bTest_LU(A, L, U, n);
//    return;
//}
int main()
{
    Init_Env();
    Test_1();
	//Test_Main();
    Free_Env();
#ifdef WIN32
    _CrtDumpMemoryLeaks();
#endif
}
