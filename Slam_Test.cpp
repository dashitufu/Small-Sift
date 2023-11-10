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

void PnP_Test()
{//尚不知何物，干就完了
    typedef double _T;
    //已知相机二所观察到的点
    _T(*pPoint_2D)[2], (*pPoint_3D)[3];
    _T K[3 * 3] = { 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 };    //已知条件相机内参

    int iCount=0;
    Temp_Load_Data(&pPoint_2D, &pPoint_3D, &iCount);

    Disp((_T*)pPoint_3D, iCount, 3, "Point_3D");
    return;
}

int bSave_PLY(const char* pcFile, float Point[][3], int iPoint_Count, int bText)
{//存点云，最简形式，用于实验，连结构都不要
    FILE* pFile = fopen(pcFile, "wb");
    char Header[512];
    int i;
    float* pPos;

    if (!pFile)
    {
        printf("Fail to open file:%s\n", pcFile);
        return 0;
    }   

    //先写入Header
    sprintf(Header, "ply\r\n");
    if (bText)
        sprintf(Header + strlen(Header), "format ascii 1.0\r\n");
    else
        sprintf(Header + strlen(Header), "format binary_little_endian 1.0\r\n");
    sprintf(Header + strlen(Header), "comment HQYT generated\r\n");
    sprintf(Header + strlen(Header), "element vertex %d\r\n", iPoint_Count);
    sprintf(Header + strlen(Header), "property float x\r\n");
    sprintf(Header + strlen(Header), "property float y\r\n");
    sprintf(Header + strlen(Header), "property float z\r\n");

    sprintf(Header + strlen(Header), "end_header\r\n");
    fwrite(Header, 1, strlen(Header), pFile);

    for (i = 0; i <iPoint_Count; i++)
    {
        pPos = Point[i];
        if (bText)
            fprintf(pFile, "%f %f %f\r\n", pPos[0], pPos[1], pPos[2]);
        else
            printf("Not implemented\n");
    }
    fclose(pFile);
    return 1;
}
void Lease_Square_Test_3()
{//搞个二维曲面拟合实验， z= a (x/w)^2 + b(y/h)^2 搞一组添加了随机噪声的样本，用这些样本来做实验
//至此，由样本拟合参数的牛顿法OK了，但是梯度法还没行
    const int w = 10, h = 10;
    const int iSample_Count = w*h;
    const float eps = (float)1e-6;
    float *pCur_Point, (*pPoint_3D)[3] = (float(*)[3])malloc(iSample_Count * 3 * sizeof(float));
    float a0 = 4, b0 = 5, 
        xa,xb;  //待求参数
    
    //造样本集
    int y, x;
    for (y = 0; y < h; y++)
    {
        for (x = 0; x < w; x++)
        {
            pCur_Point = pPoint_3D[y * w + x];
            pCur_Point[0] = (float)x;
            pCur_Point[1] = (float)y;
            pCur_Point[2] = a0 * (float)pow((float)x/w,2) + b0 * (float)pow((float)y/h,2);
            //printf("x:%f y:%f z:%f\n", pCur_Point[0], pCur_Point[1], pCur_Point[2]);
            pCur_Point[2] += (float)pow(-1, y * w + x) * iGet_Random_No_cv(0, 10) / 50.f;
            //printf("x:%f y:%f z1:%f\n", pCur_Point[0], pCur_Point[1], pCur_Point[2]);
        }
    }
    //存个盘看图像
    //bSave_PLY("c:\\tmp\\1.ply", pPoint_3D, iSample_Count, 1);

    //目标是求 min(yi - fi(x))^2 这个是最小二乘的一般形式，yi好办，每个样本的函数值，问题是， xk在此处对应的是什么？
    //显然我们要求的是a,b，所以， f(x)不再视为关于向量x的函数，而是视为 关于a,b的函数，所以
    //此处，我们讲a,b改为 xa,xb，这样可能好看些。故此， xk就是(xa,xb)在迭代过程中的一个取值，
    //目标是建立迭代格 x=(xa,xb)= (J'J)(-1) * J'*e + xk
    //先对fi(x)在xk处进行一阶泰勒展开 = fi(xk) + J(xk)(x-xk) 二元为 fi(x,y)= fi(xk,yk) + f'xk(xk,yk)*(x-xk) + f'yk(xk,yk)(y-yk)
    //于是，原来求解问题变成 min Sigma[ (yi-fi(x))^2 ] , x=(xa,xb)
    //泰勒展开 = min Sigma [ yi- fi(xk) - J(xk)(x-xk)]^2 , 等于只将yi-fi(x)展开
    //将前面两项合并为一项 ei = yi - fi(xk) 则上式=
    // min Sigma [ (ei - J(xk)(x-xk))^2] 
    //即求解 ei - J(xk)(x-xk) = 0 矛盾方程组，求x, 此时，将J视为fi(x)关于x的一阶偏导矩阵，
    // J (x-xk)=ei => J'J (x-xk) = J'ei
    // (J'J)(-1)(J'J) (x-xk) = (J'J)(-1) * J' ei
    // x-xk=  (J'J)(-1) * J' ei
    //x = (J'J)(-1) * J' ei + xk

    xa = 1, xb = 1;               //xk初值
    float J[iSample_Count][2];  //Jacob
    float Jt[2][iSample_Count]; //J'
    float e[iSample_Count];     
    float Temp[iSample_Count * 2];  //搞个足够大的临时空间
    int i,iResult;
    while (1)
    {
        for (i = 0; i < iSample_Count; i++)
        {
            pCur_Point = pPoint_3D[i];
            //逐个求一阶偏导 f(a,b)= a (x/w)^2 + b(y/h)^2
            //fi'a (对a求偏导)  = (x/w)^2
            J[i][0] = (float)pow(pCur_Point[0] / w, 2);
            //fi'b (对b求偏导) = (y/h)^2
            J[i][1] = (float)pow(pCur_Point[1] / h, 2);

            //ei = yi - fi(xk) 要特别留意此处，fi(xk)作为一个整体，要加括号，否则发散
            e[i] = pCur_Point[2] - (xa * (float)pow(pCur_Point[0] / w, 2) + xb * (float)pow(pCur_Point[1] / h, 2));
        }

        Matrix_Transpose((float*)J, iSample_Count, 2, (float*)Jt);          //= J' 2*n
        Matrix_Multiply((float*)Jt, 2, iSample_Count, (float*)J, 2, Temp);  //=J'J 2x2
        Get_Inv_Matrix_Row_Op(Temp, Temp, 2, &iResult);                     //(J'J)(-1) 逆矩阵 2x2
        Matrix_Multiply(Temp, 2, 2,(float*)Jt, iSample_Count, Temp);        //=(J'J)(-1)*J' 2xn
        Matrix_Multiply(Temp, 2, iSample_Count, e, 1, Temp);                //=(J'J)(-1)*J'*e 2x1
                
        xa = Temp[0] + xa;
        xb = Temp[1] + xb;
        if (abs(Temp[0]) < eps && abs(Temp[1]) < eps)
            break;
        printf("%f %f\n", Temp[0], Temp[1]);
    }

    return;
}
int main()
{
    Init_Env();
    Lease_Square_Test_3();
    
    Free_Env();
#ifdef WIN32
    _CrtDumpMemoryLeaks();
#endif
}
