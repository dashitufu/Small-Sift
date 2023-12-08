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

template<typename _T>void Temp_Load_Data(_T (**ppPoint_2D)[2], _T (**ppPoint_3D)[3], int* piCount)
{
    int iCount = (int)(iGet_File_Length((char*)"c:\\tmp\\1.bin") / (5 * sizeof(float)));
    FILE* pFile = fopen("c:\\tmp\\1.bin", "rb");
    _T(*pPoint_2D)[2], (*pPoint_3D)[3];
    if (!pFile)
    {
        printf("Fail to open file\n");
        return;
    }
    pPoint_2D = (_T(*)[2])malloc(iCount * 2 * sizeof(_T));
    pPoint_3D = (_T(*)[3])malloc(iCount * 3 * sizeof(_T));
    for (int i = 0; i < iCount; i++)
    {
        float Data[5];
        fread(Data, 1, 5 * sizeof(float), pFile);
        pPoint_2D[i][0] = Data[0];
        pPoint_2D[i][1] = Data[1];
        pPoint_3D[i][0] = Data[2];
        pPoint_3D[i][1] = Data[3];
        pPoint_3D[i][2] = Data[4];
    }
    fclose(pFile);
    *ppPoint_2D = pPoint_2D;
    *ppPoint_3D = pPoint_3D;
    *piCount = iCount;
    return;
}

void DLT_Test_2()
{//试一下自建数据进行DLT估计
    typedef double _T;
    const int iCount = 111;
    _T Point_3D_1[iCount][3], Point_2D_2[iCount][2];    //图A用3D数据，图2用归一化平面数据
    _T Point_3D_2[iCount][3];
    int i,x,y;
    //点云用 f(x,y) = x^2 +y^2
    for (i=0,y = -10; y <=10; y+=2)
    {
        for (x = -10; x <=10; x +=2,i++)
        {
            Point_3D_1[i][0] = x;
            Point_3D_1[i][1] = y;
            Point_3D_1[i][2] = (x * x + y * y)/10+10;
        }
    }
    
    //定义一个T，使得P2=T * P1
    _T T_Org[4 * 4],T[4*4], R_Org[3 * 3], t_Org[3] = {10,20,30}, Rotation_Vector_Org[4] = {0,1,0,PI / 6};;
    Rotation_Vector_2_Matrix(Rotation_Vector_Org, R_Org);
    Gen_Homo_Matrix(R_Org, t_Org, T_Org);
    Disp(T_Org, 4, 4, "T");
    for (i = 0; i < iCount; i++)
    {
        _T Pos[4];
        memcpy(Pos, Point_3D_1[i], 3 * sizeof(_T));
        Pos[3] = 1;
        Matrix_Multiply(T_Org, 4, 4, Pos,1, Point_3D_2[i]);
        //Disp(Pos, 1, 4);
        //再将新点投影到归一化平面
        Point_2D_2[i][0] = Point_3D_2[i][0] / Point_3D_2[i][2];
        Point_2D_2[i][1] = Point_3D_2[i][1] / Point_3D_2[i][2];

    }
    //bSave_PLY("c:\\tmp\\1.ply", Point_3D_1, iCount);
    //Disp((_T*)Point_2D_2, iCount, 2);

    //尝试无扰动情况下求解DLT问题
    //变成矩阵形式, 一下为A, 每对匹配点搞出两条式
    //  P0 P1 P2 1 0  0  0  0 u1 * P0 u1 * P1 u1 * p2 u1 
    //  0  0  0  0 P0 P1 P2 1 v1 * P0 v1 * P1 v1 * P2 v1
    _T* pA = (_T*)malloc(iCount * 12 * 2 * sizeof(_T)), * pCur, * pCur_Point_1, *pCur_UV;
    memset(pA, 0, iCount * 12 * 2 * sizeof(_T));
    pCur = pA;
    for (i = 0; i < iCount; i++)
    {
        pCur_Point_1 = Point_3D_1[i];
        pCur_UV = Point_2D_2[i];
        //  P0 P1 P2 1 0  0  0  0 u1 * P0 u1 * P1 u1 * p2 u1 
        pCur[0] = pCur_Point_1[0];
        pCur[1] = pCur_Point_1[1];
        pCur[2] = pCur_Point_1[2];
        pCur[3] = 1;
        pCur[8] = -pCur_UV[0] * pCur_Point_1[0];
        pCur[9] = -pCur_UV[0] * pCur_Point_1[1];
        pCur[10] = -pCur_UV[0] * pCur_Point_1[2];
        pCur[11] = -pCur_UV[0];
        pCur += 12;
        //  0  0  0  0 P0 P1 P2 1 v1 * P0 v1 * P1 v1 * P2 v1
        pCur[4] = pCur_Point_1[0];
        pCur[5] = pCur_Point_1[1];
        pCur[6] = pCur_Point_1[2];
        pCur[7] = 1;
        pCur[8] = -pCur_UV[1] * pCur_Point_1[0];
        pCur[9] = -pCur_UV[1] * pCur_Point_1[1];
        pCur[10] = -pCur_UV[1] * pCur_Point_1[2];
        pCur[11] = -pCur_UV[1];
        pCur += 12;
    }
    //Disp(pA, iCount * 2, 12);
    //然后求解 Ax=0 的矛盾方程组，用svd
    SVD_Info oSVD;
    int iResult;
    SVD_Alloc<_T>(iCount * 2, 12, &oSVD);
    svd_3(pA, oSVD, &iResult);
    //Test_SVD(pA, oSVD,NULL,0.0001f);
    //理论上Vt最后一行为矛盾方程组的解
    pCur = &((_T*)oSVD.Vt)[11 * 12];
    memcpy(T, pCur, 12 * sizeof(_T));
    Disp(T, 3, 4,"T");
    Disp(Point_3D_1[0], 1, 3,"Point_3D");
    Disp(Point_2D_2[0], 1, 2, "u1v1");

    _T b[iCount*2];
    Matrix_Multiply(pA, iCount * 2, 12, pCur, 1, b);
    //Disp(b, iCount*2,1);  //看看Ax 是否=0   感觉差不多了，但是，这个矩阵跟T不一样，要找出原因
    //再验算 (u,v,1)= T*P1
    
    for (i = 0; i < iCount; i++)
    {
        _T Pos[4],Point[4];
        memcpy(Point, Point_3D_1[i], 3 * sizeof(_T));
        Point[3] = 1;
        Matrix_Multiply(T, 3, 4, Point, 1, Pos);
        Matrix_Multiply(Pos, 1, 3, 1.f / Pos[2],Pos);
        //此处U,V已经和变换后的UV齐了。但只是UV上的齐，不是空间点的齐。所以这个变换有无数个
        Disp(Pos, 1, 3, "new uv");
        Disp(Point_2D_2[i], 1, 2, "org uv");

        //将uv乘上新点深度可恢复相机2的空间点
        Matrix_Multiply(Pos, 1, 3, Point_3D_2[i][2], Pos);
        Disp(Pos, 1, 3, "new point 3d");
        Disp(Point_3D_2[i], 1, 3, "org point 3d");
    }

    Free_SVD(&oSVD);
    return;
}

void DLT_Test_1()
{//直线线性变换方法，给定两张RGBD图1与图2，图1取(x,y,d)，图2只取(x,y)，尝试估计Rt，其中
    //x,y为像素平面坐标(x,y), d为深度，尽量保持第一手信息。另外，还需要相机参数K与深度量化
    // 参数 fDepth_Factor
//此处有个很迷惑的地方，图2为什么只取(x,y)而忽略d即可？因为(x,y)已经是d参与下的结果，已经隐含
    //这个例子不好，无从检验数据是否准确

    typedef double _T;
    //第一步，装入图1，图2的信息
    _T(*pPoint_3D_1)[3], (*pPoint_3D_2)[3],*pPoint_1,*pPoint_2;
    _T K[3 * 3]= {  520.9f,   0.f,      325.1f,     //内参必须有
                    0.f,      521.f,    249.7f, 
                    0.f,      0.f,      1.f },
        fDepth_Factor = 5000.f;                 //深度图量化参数

    int i,iCount,iResult;
    Temp_Load_File("sample\\7.8.2.bin", &pPoint_3D_1, &pPoint_3D_2, &iCount);
    //从相机参数和深度因子恢复图一的空间位置
    Image_Pos_2_3D(pPoint_3D_1, iCount, K, fDepth_Factor, pPoint_3D_1);
    
    //图二求归一化平面上的坐标
    for (i = 0; i < iCount; i++)
    {
        pPoint_3D_2[i][0] = (pPoint_3D_2[i][0] - K[2]) / K[0];
        pPoint_3D_2[i][1] = (pPoint_3D_2[i][1] - K[5]) / K[4];
    }
    //归一化平面坐标其实已经包含了d信息。所以现在缺了也可以继续搞

    //第二步，解齐次矛盾房车方程。先构造矩阵Ax=0
    //P0* t00 + P1 * t01 + P2 * t02 + 1 * t03 + 0  * t10 + 0  * t11 + 0  * t12 + 0 * t13 + u1 * P0 * t20 + u1 * P1 * t21 + u1 * P2 * t22 + u1 * 1 * t23 =   0
    //0 * t00 + 0 * t01  + 0 * t02  + 0 * t03 + P0 * t10 + P1 * t11 + p2 * t12 + 1 * t13 + v1 * P0 * t20 + v1 * P0 * t21 + v1 * P2 * t22 + v1 * 1 * t23 =   0
    //                                                                                                                                              ...	    ...
    //                                                                                                                                              t22	    0
    //先用点对构造A
    _T *pCur, * pA,T[12];
    pCur=pA = (_T*)malloc(iCount * 12 * 2 * sizeof(_T));
    memset(pA, 0, iCount * 12 * 2 * sizeof(_T));
    pPoint_1 = pPoint_3D_1[0];
    pPoint_2 = pPoint_3D_2[0];
    for (i = 0; i < iCount; i++,pPoint_1++,pPoint_2++)
    {
        pCur[0] = pPoint_1[0];  //P0 P1 P2
        pCur[1] = pPoint_1[1];
        pCur[2] = pPoint_1[2];
        pCur[3] = 1;
        
        //u1* P0* t20 + u1 * P1 * t21 + u1 * P2 * t22 + u1 * 1 * t23
        pCur[8] = pPoint_2[0] * pPoint_1[0];
        pCur[9] = pPoint_2[0] * pPoint_1[1];
        pCur[10] = pPoint_2[0] * pPoint_1[2];
        pCur[11] = pPoint_2[0];
        pCur += 12;

        pCur[4] = pPoint_1[0];  //P0 P1 P2 1
        pCur[5] = pPoint_1[1];
        pCur[6] = pPoint_1[2];
        pCur[7] = 1;
        
        //v1 * P0 * t20 + v1 * P0 * t21 + v1 * P2 * t22 + v1 * 1 * t23
        pCur[8] = pPoint_2[1] * pPoint_1[0];
        pCur[9] = pPoint_2[1] * pPoint_1[1];
        pCur[10] = pPoint_2[1] * pPoint_1[2];
        pCur[11] = pPoint_2[1];
        pCur += 12;
    }
    /*for (i = 0; i < iCount*2;i++)
    {
        for (int j = 0; j < 12; j++)
            printf("%f ", pA[i * 12 + j]);
        printf("\n");
    }*/
    iResult=iGet_Rank(pA, 12, 12);

    SVD_Info oSVD;
    SVD_Alloc<_T>(iCount * 2, 12, &oSVD);
    svd_3(pA, oSVD, &iResult);
    for (i = 0; i < 12; i++)
        T[i] = ((_T*)oSVD.Vt)[12*11+i];
    //Disp((_T*)oSVD.S, 1, 12);
    //Test_SVD(pA, oSVD, &iResult,0.0001);
    Free_SVD(&oSVD);

    Disp(T, 3, 4);
    _T* pResult = (_T*)malloc(iCount * 2*sizeof(_T));
    Matrix_Multiply(pA, iCount * 2, 12, T, 1, pResult);
    Disp(pResult, iCount*2, 1);
    free(pPoint_3D_1);
    return;
}

void Exp_Test()
{
    /*float A[3 * 3] = { 1,2,3,4,5,6,7,8,9 },B[3*3];
    Exp_Ref(A, 3, B);
    Disp(B, 3, 3, "B");*/
    return;
}



int main()
{
    Init_Env();
    
    //DLT_Test_1();
    Test_Main();
    Free_Env();
#ifdef WIN32
    _CrtDumpMemoryLeaks();
#endif
}
