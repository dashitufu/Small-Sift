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

//template<typename _T>void Temp_Load_Data(_T (**ppPoint_2D)[2], _T (**ppPoint_3D)[3], int* piCount)
//{
//    int iCount = (int)(iGet_File_Length((char*)"c:\\tmp\\1.bin") / (5 * sizeof(float)));
//    FILE* pFile = fopen("c:\\tmp\\1.bin", "rb");
//    _T(*pPoint_2D)[2], (*pPoint_3D)[3];
//    if (!pFile)
//    {
//        printf("Fail to open file\n");
//        return;
//    }
//    pPoint_2D = (_T(*)[2])malloc(iCount * 2 * sizeof(_T));
//    pPoint_3D = (_T(*)[3])malloc(iCount * 3 * sizeof(_T));
//    for (int i = 0; i < iCount; i++)
//    {
//        float Data[5];
//        fread(Data, 1, 5 * sizeof(float), pFile);
//        pPoint_2D[i][0] = Data[0];
//        pPoint_2D[i][1] = Data[1];
//        pPoint_3D[i][0] = Data[2];
//        pPoint_3D[i][1] = Data[3];
//        pPoint_3D[i][2] = Data[4];
//    }
//    fclose(pFile);
//    *ppPoint_2D = pPoint_2D;
//    *ppPoint_3D = pPoint_3D;
//    *piCount = iCount;
//    return;
//}

typedef struct Point_2D {
    unsigned int m_iCamera_Index;
    unsigned int m_iPoint_Index;
    float m_Pos[2];
}Point_2D;

void Temp_Load_File_2(int* piCameta_Count, int* piPoint_Count, int* piObservation_Count,
    Point_2D** ppPoint_2D, float(**ppPoint_3D)[3], float(**ppCamera)[3*3])
{//这次装入problem-16-22106-pre.txt。搞个好点的数据结构指出匹配关系

    int i, iCamera_Count, iPoint_Count, iObservation_Count;
    float(*pPoint_3D)[3], (*pCamera)[3*3];
    Point_2D* pPoint_2D;
    FILE* pFile = fopen("Sample\\problem-16-22106-pre.txt", "rb");
    i=fscanf(pFile, "%d %d %d\n", &iCamera_Count, &iPoint_Count, &iObservation_Count);
    pPoint_2D = (Point_2D*)malloc(iObservation_Count * 2 * sizeof(Point_2D));
    pPoint_3D = (float(*)[3])malloc(iPoint_Count * 3 * sizeof(float));
    pCamera = (float(*)[3*3])malloc(iCamera_Count * 16 * sizeof(float));

    for (i = 0; i < iObservation_Count; i++)
        fscanf(pFile, "%d %d %f %f", &pPoint_2D[i].m_iCamera_Index, &pPoint_2D[i].m_iPoint_Index, &pPoint_2D[i].m_Pos[0], &pPoint_2D[i].m_Pos[1]);
    for (i = 0; i < iCamera_Count; i++)
        for (int j = 0; j < 9; j++)
            fscanf(pFile, "%f ", &pCamera[i][j]);
    for (i = 0; i < iPoint_Count; i++)
        fscanf(pFile, "%f %f %f ", &pPoint_3D[i][0], &pPoint_3D[i][1], &pPoint_3D[i][2]);
    fclose(pFile);
    *piCameta_Count = iCamera_Count;
    *piPoint_Count = iPoint_Count;
    *piObservation_Count = iObservation_Count;
    *ppPoint_2D = pPoint_2D;
    *ppPoint_3D = pPoint_3D;
    *ppCamera = pCamera;
    return;
}

void BA_Test_3()
{
    int iCamera_Count, iPoint_Count, iObservation_Count;
    float(*pPoint_3D)[3], 
        (*pCamera)[3*3];    //只是个内参
    Point_2D* pPoint_2D;
    Temp_Load_File_2(&iCamera_Count, &iPoint_Count, &iObservation_Count, &pPoint_2D, &pPoint_3D,&pCamera);
}

void Inverse_Test()
{//Sparse的矩阵求逆
    typedef double _T;
    _T A[] = { 2,3,1,1,-1,2,1,2,-1 }, A_Inv[9];
    int iResult;
    Sparse_Matrix<_T> oA, oA_Inv;
    Get_Inv_Matrix_Row_Op_2(A, A_Inv, 3, &iResult);
    Init_Sparse_Matrix(&oA, 3 * 3, 3, 3);
    Dense_2_Sparse(A, 3, 3, &oA);
    Disp(A_Inv, 3, 3, "A_Inv");
    Matrix_Multiply(A, 3, 3, A_Inv, 3, A_Inv);
    Disp(A_Inv, 3, 3, "I");

    Get_Inv_Matrix_Row_Op(oA, &oA_Inv,&iResult);
    Disp(oA_Inv, "A_Inv");
    return;
}
int main()
{
    Init_Env();
    Inverse_Test();
	//Test_Main();
    Free_Env();
#ifdef WIN32
    _CrtDumpMemoryLeaks();
#endif
}
