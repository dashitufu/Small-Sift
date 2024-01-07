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


void Sphere_Test_2()
{//搞三维
	typedef float _T;
	_T Sphere_Center[4][3] = { { 200,300,400 },
								{ 300,400,410 } ,
								{ 350,300,420 } ,
								{ 200,400,430 } };

	_T d[4] = { 100,110,120,130 };	//可视为基站到物体距离
	int i;
	Image oImage;
	Init_Image(&oImage, 1000, 1000, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for (i = 0; i < 4; i++)
		Draw_Arc(oImage, (int)d[i], (int)Sphere_Center[i][0],(int)Sphere_Center[i][1]);

	_T P[3] = { 0,0,0 }, Pre_P[3];	//最后的解
	int iIter, iResult;

	_T Jt[1 * 3], J[3 * 1], JJt[3 * 3], Je[3 * 1], H_Inv[3 * 3], Delta_X[3],
		fSum_e, e, fSum_e_Pre = 1e10;
	_T Sigma_H[3 * 3], Sigma_Je[3 * 1];
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		//置零
		memset(Sigma_H, 0, 3 * 3 * sizeof(_T));	
		memset(Sigma_Je, 0, 3 * sizeof(_T));

		//到4张皮的距离和最小感觉最好，因为这才是题意
		for (i = 0; i < 4; i++)
		{	//e = [(x-a)^2 + (x-b)^2 + (z-c)^2]^1/2
			float fDist_Sqr = (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) +
						(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1])+
						(P[2] - Sphere_Center[i][2]) * (P[2] - Sphere_Center[i][2]);
			e= d[i] - sqrt(fDist_Sqr);
			fSum_e += abs(e);	// e* e;	//这里就必须用e的平方了，或者用绝对值，因为有正有负。要琢磨一下误差的度量对结果的影响
			//?e/?x = - [(x-a)^2 + (x-b)^2 + (z-a)^2] *(x-a)
			Jt[0] = -pow(fDist_Sqr,-0.5f) * (P[0] - Sphere_Center[i][0]);
			Jt[1] = -pow(fDist_Sqr,-0.5f) * (P[1] - Sphere_Center[i][1]);
			Jt[2] = -pow(fDist_Sqr, -0.5f) * (P[2] - Sphere_Center[i][2]);

			memcpy(J, Jt, 3 * sizeof(_T));
			Matrix_Multiply(J, 3, 1, Jt, 3, JJt);
			Matrix_Multiply(J, 3, 1, e, Je);
			//	//累加
			Matrix_Add(Sigma_H, JJt, 3, Sigma_H);
			Vector_Add(Sigma_Je, Je, 3, Sigma_Je);
		}

		////试一下,到圆心距离和最小
		//for (i = 0; i < 4; i++)
		//{	//e = [(x-a)^2 + (x-b)^2 + (z-c)^2]^1/2
		//	float fDist_Sqr = (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) +
		//		(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]) +
		//		(P[2] - Sphere_Center[i][2]) * (P[2] - Sphere_Center[i][2]);
		//	if (fDist_Sqr == 0)
		//		continue;
		//	e = sqrt(fDist_Sqr);
		//	fSum_e += e;	//此处到底用e还是e的平方？用e表示到点的距离之和，用e*e表示距离的平方和，微妙差别

		//	//?e/?x = [(x-a)^2 + (x-b)^2 + (z-c)^2] *(x-a) 
		//	Jt[0] = pow(fDist_Sqr, -0.5) * (P[0] - Sphere_Center[i][0]);
		//	Jt[1] = pow(fDist_Sqr, -0.5) * (P[1] - Sphere_Center[i][1]);
		//	Jt[2] = pow(fDist_Sqr, -0.5) * (P[2] - Sphere_Center[i][2]);

		//	memcpy(J, Jt, 3 * sizeof(_T));
		//	Matrix_Multiply(J, 3, 1, Jt, 3, JJt);
		//	Matrix_Multiply(J, 3, 1, e, Je);
		//	//	//累加
		//	Matrix_Add(Sigma_H, JJt, 3, Sigma_H);
		//	Vector_Add(Sigma_Je, Je, 3, Sigma_Je);
		//}

		//后面全一样，毫无差别毫无营养，搞利索以后改成解方程，
		//会比求个逆可能快一点
		Sigma_H[0] += 1, Sigma_H[4] += 1, Sigma_H[8] += 1;
		Get_Inv_Matrix_Row_Op(Sigma_H, H_Inv, 3, &iResult);
		Matrix_Multiply(H_Inv, 3, 3, Sigma_Je, 1, Delta_X);
		Matrix_Multiply(Delta_X, 3, 1, (_T)-1.f, Delta_X);

		if ((fSum_e_Pre <= fSum_e && iIter > 0) || !iResult)
			break;
		printf("Iter:%d Error:%f\n", iIter, fSum_e);

		Pre_P[0] = (P[0] += Delta_X[0]);
		Pre_P[1] = (P[1] += Delta_X[1]);
		Pre_P[2] = (P[2] += Delta_X[2]);
		fSum_e_Pre = fSum_e;
	}

	//Disp(Pre_P, 1, 3, "Point");
	Draw_Point(oImage, (int)Pre_P[0], (int)Pre_P[1], 2);
	bSave_Image("c:\\tmp\\1.bmp", oImage);
	Free_Image(&oImage);
}
void Sphere_Test_1()
{//先看看二元情况
	typedef float _T;
	_T Sphere_Center[4][3] = {	{ 200,300,400 },
								{ 300,400,410 } ,
								{ 350,300,420 } ,
								{ 200,400,430 } };

	_T R[4] = { 100,110,120,130 };
	Image oImage;
	int i;
	Init_Image(&oImage, 1000, 1000, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for(i=0;i<4;i++)
		Draw_Arc(oImage, (int)R[i], (int)Sphere_Center[i][0],(int)Sphere_Center[i][1]);
	
	_T P[3] = { 0,0,0 }, Pre_P[3];	//最后的解
	int iIter, iResult;
	
	_T Jt[1 * 2], J[2 * 1], JJt[2 * 2], Je[2 * 1], H_Inv[2 * 2], Delta_X[2],
		fSum_e, e, fSum_e_Pre = 1e10;
	_T Sigma_H[2 * 2], Sigma_Je[2 * 1];
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		memset(Sigma_H, 0, 2 * 2 * sizeof(_T));
		memset(Sigma_Je, 0, 2 * sizeof(_T));

		//这个条件不好
		//for (i = 0; i < 2; i++)
		//{
		//	e = R[i] * R[i] - (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) -
		//		(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]);
		//	fSum_e += e * e;
		//	//?e/?x= (-2x, -2y)	,第一步顺着这个梯度走，能走到目的地
		//	Jt[0] = -2 * (P[0]-Sphere_Center[i][0]), Jt[1] = -2 * (P[1]-Sphere_Center[i][1]);
		//	//Disp(Jt, 1, 2, "Jt");
		//	memcpy(J, Jt, 2 * sizeof(_T));
		//	//第二步，要求Δx
		//	Matrix_Multiply(J, 2, 1, Jt, 2, JJt);
		//	//Disp(JJt, 2, 2, "JJt");
		//	Matrix_Multiply(J, 2, 1, e, Je);

		//	//累加
		//	Matrix_Add(Sigma_H, JJt, 2, Sigma_H);
		//	Vector_Add(Sigma_Je, Je, 2, Sigma_Je);
		//}

		////以下为到4个圆边距离和最近
		//for (i = 0; i < 4; i++)
		//{//e(x) = Ri - [(x-a)^2 + (x-b)^2]^1/2
		//	float fDist_Sqr = (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) +
		//		(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]);
		//	e= R[i] - sqrt(fDist_Sqr);
		//	fSum_e += e * e;
		//	//?e/?x = - [(x-a)^2 + (x-b)^2] *(x-a)
		//	Jt[0] = -pow(fDist_Sqr,-0.5) * (P[0] - Sphere_Center[i][0]);
		//	Jt[1] = -pow(fDist_Sqr,-0.5) * (P[1] - Sphere_Center[i][1]);

		//	memcpy(J, Jt, 2 * sizeof(_T));
		//	Matrix_Multiply(J, 2, 1, Jt, 2, JJt);
		//	Matrix_Multiply(J, 2, 1, e, Je);
		//	//	//累加
		//	Matrix_Add(Sigma_H, JJt, 2, Sigma_H);
		//	Vector_Add(Sigma_Je, Je, 2, Sigma_Je);
		//}

		//试一下,到圆心距离和最小
		for (i = 0; i < 4; i++)
		{
			float fDist_Sqr = (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) +
				(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]);
			if (fDist_Sqr == 0)
				continue;
			e = sqrt(fDist_Sqr);
			fSum_e += e;
			//?e/?x = [(x-a)^2 + (x-b)^2] *(x-a)
			Jt[0] = pow(fDist_Sqr, -0.5f) * (P[0] - Sphere_Center[i][0]);
			Jt[1] = pow(fDist_Sqr, -0.5f) * (P[1] - Sphere_Center[i][1]);

			memcpy(J, Jt, 2 * sizeof(_T));
			Matrix_Multiply(J, 2, 1, Jt, 2, JJt);
			Matrix_Multiply(J, 2, 1, e, Je);
			//	//累加
			Matrix_Add(Sigma_H, JJt, 2, Sigma_H);
			Vector_Add(Sigma_Je, Je, 2, Sigma_Je);
		}
		
		Sigma_H[0] += 1, Sigma_H[3] += 1;
		Get_Inv_Matrix_Row_Op(Sigma_H, H_Inv,2,&iResult);
		Matrix_Multiply(H_Inv, 2, 2, Sigma_Je, 1, Delta_X);
		Matrix_Multiply(Delta_X, 2, 1,(_T)- 1.f, Delta_X);
		
		if ((fSum_e_Pre <= fSum_e && iIter > 0) || !iResult)
			break;
		printf("Iter:%d Error:%f\n", iIter, fSum_e);
		//Draw_Point(oImage, P[0], P[1],2);

		Pre_P[0] = (P[0] += Delta_X[0]);
		Pre_P[1] = (P[1] += Delta_X[1]);
		fSum_e_Pre = fSum_e;
	}
	Draw_Point(oImage, (int)Pre_P[0],(int)Pre_P[1], 2);
	Disp(Pre_P, 1, 3, "Point");
	bSave_Image("c:\\tmp\\1.bmp", oImage);
	Free_Image(&oImage);
	return;
}

int main()
{
    Init_Env();
	//Sphere_Test_2();

	Test_Main();
    Free_Env();
#ifdef WIN32
    _CrtDumpMemoryLeaks();
#endif
}
