// Slam_Test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>
#include "Common.h"
#include "Image.h"
#include "sift.h"
#include "Reconstruct.h"

extern "C"
{
#include "Buddy_System.h"
}
void Free_Report(Report oReport, Mem_Mgr* poMem_Mgr=NULL)
{
    if (oReport.m_pInlier_Mask)
    {
        if (poMem_Mgr)
            Free(poMem_Mgr, oReport.m_pInlier_Mask);
        else
            free(oReport.m_pInlier_Mask);
    }
    return;
}
void Disp_Report(Report oReport)
{
    printf("%s\n", oReport.m_bSuccess ? "Success" : "Fail");
    printf("Sample Count:%d Trial_Count:%d\n", oReport.m_iSample_Count,oReport.m_iTrial_Count);
    printf("Inlier Count:%d Residual:%f\n", oReport.m_oSupport.m_iInlier_Count, oReport.m_oSupport.m_fResidual_Sum);
    if (oReport.m_iFloat_Size == 4)
        Disp(oReport.m_Modal_f, 3, 3, "Modal");
    else
        Disp(oReport.m_Modal_d, 3, 3, "Modal");
    return;
}
static void Sift_Test_1()
{//Sift实验，给出一张黑白图，求所有的特征点
    float(*pPoint)[2];
    int iCount;
    unsigned long long tStart = iGet_Tick_Count();
    Get_Sift_Feature("c:\\tmp\\Screen_Cut_A.bmp", &pPoint, &iCount, 0);
    printf("%lld\n", iGet_Tick_Count() - tStart);
    free(pPoint);
    return;
}
static void Sift_Test_2()
{//两张图的匹配，返回两组匹配点位置
    float(*pPoint_1)[2] = NULL, (*pPoint_2)[2];
    int iMatch_Count;

    Sift_Match_2_Image("C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\Ajay\\A16.bmp",
        "C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\Ajay\\A17.bmp", &pPoint_1, &pPoint_2, &iMatch_Count);

    for (int i = 0; i < iMatch_Count; i++)
        printf("%f %f %f %f\n", pPoint_1[i][0], pPoint_1[i][1], pPoint_2[i][0], pPoint_2[i][1]);
    free(pPoint_1);
    return;
}
static void Sift_Test_3()
{//整个目录遍历，此处用Mem_Mgr
    Sift_Match_Map oMatch_Map;
    Sift_Simple_Match_Item oMatch;
    Mem_Mgr oMem_Mgr;
    Init_Mem_Mgr(&oMem_Mgr, 128000000, 1024, 997);
    Sift_Match_Path("C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\ET\\bmp", &oMatch_Map, &oMem_Mgr);
    int y, x, iIndex;
    for (y = 0; y < oMatch_Map.m_iImage_Count; y++)
    {
        for (x = y + 1; x < oMatch_Map.m_iImage_Count; x++)
        {
            iIndex = iUpper_Triangle_Cord_2_Index(x, y, oMatch_Map.m_iImage_Count);
            oMatch = oMatch_Map.m_pMatch[iIndex];
            printf("Image Pair:%d %d Match Count:%d\n", oMatch.m_iImage_A, oMatch.m_iImage_B, oMatch.m_iMatch_Count);
            /* for (int k = 0; k < oMatch.m_iMatch_Count; k++)
                 printf("%f %f %f %f\n", oMatch.m_pPoint_1[k][0], oMatch.m_pPoint_1[k][1],
                     oMatch.m_pPoint_2[k][0], oMatch.m_pPoint_2[k][1]);*/
        }
    }
    Free_Mem_Mgr(&oMem_Mgr);
    return;
}
static void Sift_Test_4()
{//整个目录遍历，最简接口
    Sift_Match_Map oMatch_Map;
    Sift_Simple_Match_Item oMatch;

    Sift_Match_Path("C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\Ajay", &oMatch_Map);
    int y, x, iIndex;
    for (y = 0; y < oMatch_Map.m_iImage_Count; y++)
    {
        for (x = y + 1; x < oMatch_Map.m_iImage_Count; x++)
        {
            iIndex = iUpper_Triangle_Cord_2_Index(x, y, oMatch_Map.m_iImage_Count);
            oMatch = oMatch_Map.m_pMatch[iIndex];
            printf("Image Pair:%d %d Match Count:%d\n", oMatch.m_iImage_A, oMatch.m_iImage_B, oMatch.m_iMatch_Count);
            //for (int k = 0; k < oMatch.m_iMatch_Count; k++)
                //printf("%f %f %f %f\n", oMatch.m_pPoint_1[k][0], oMatch.m_pPoint_1[k][1], oMatch.m_pPoint_2[k][0], oMatch.m_pPoint_2[k][1]);
        }
    }
    free(oMatch_Map.m_pBuffer);
    return;
}
void Sift_Example()
{
    Sift_Test_1();
    Sift_Test_2();
    Sift_Test_3();
    Sift_Test_4();
}
void Ransac_Test()
{
    typedef double _T;
    _T (*pPoint_1)[2], (*pPoint_2)[2];
    int iCount;
    Report oReport_H;
    Mem_Mgr oMem_Mgr;
    Init_Mem_Mgr(&oMem_Mgr, 100000000, 1024, 997);
    //Temp_Load_Match_Point(&pPoint_1,&pPoint_2,&iCount);
    Sift_Match_2_Image("C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\ET\\bmp\\et000.bmp",
        "C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\ET\\bmp\\et003.bmp", &pPoint_1, &pPoint_2, &iCount);

    Ransac_Estimate_H(pPoint_1, pPoint_2, iCount, &oReport_H);
    Disp_Report(oReport_H);
    
    free(pPoint_1);
    //如果是读入的点，此处要释放
    //free(pPoint_2);
    Free_Report(oReport_H);
    Disp_Mem(&oMem_Mgr, 0);
    Free_Mem_Mgr(&oMem_Mgr);
}

void SVD_Test_1()
{//搞一个2000x10的矩阵
#define _T float
    const int w = 9, h = 8;
    int bFull_UV = 1;
    _T M[w * h]
    = { -1.11092001, 1.09082501, -1.00000000, 0.00000000, 0.00000000, 0.00000000, 0.34592309, -0.33966582, 0.31138433,
        0.44694880, -1.33252455, -1.00000000, 0.00000000, 0.00000000, 0.00000000, -0.07166018, 0.21364628, 0.16033197,
        1.61034985, 0.32395583, -1.00000000, 0.00000000, 0.00000000, 0.00000000, 3.06228775, 0.61604377, -1.90162886,
        -0.94637863, -0.08225629, -1.00000000, 0.00000000, 0.00000000, 0.00000000, 1.35323869, 0.11761930, 1.42991256,
        0.00000000, 0.00000000, 0.00000000, -1.11092001, 1.09082501, -1.00000000, -1.16227131, 1.14124742, -1.04622411,
        0.00000000, 0.00000000, 0.00000000, 0.44694880, -1.33252455, -1.00000000, -0.45111539, 1.34494674, 1.00932230,
        0.00000000, 0.00000000, 0.00000000, 1.61034985, 0.32395583, -1.00000000, 0.33483522, 0.06735917, -0.20792701,
        0.00000000, 0.00000000, 0.00000000, -0.94637863, -0.08225629, -1.00000000, 0.23170076, 0.02013871, 0.24482882 };

    //可以自动赋值，这是个不满秩的矩阵，能很好的测出问题
   /* for (int i = 0; i < w * h; i++)
        M[i] = i;*/

    SVD_Info oSVD;

    /*_T* U, * S, * Vt;
    int U_h, U_w, S_w, Vt_h, Vt_w;
    SVD_Allocate(h, w, &U, &S, &Vt, &U_h, &U_w, &S_w, &Vt_h, &Vt_w);*/
    //Disp_Mem(&oMatrix_Mem, 0);
    SVD_Allocate(M, h, w, &oSVD);

    //Disp(M, 8, 9, "M");
    int iResult;
      
    svd_3(M,oSVD,&iResult,0.000001);
    //Disp((_T*)oSVD.U, oSVD.h_Min_U, oSVD.w_Min_U, "U");
    //Disp((_T*)oSVD.S, 1, oSVD.w_Min_S, "S");
    //Disp((_T*)oSVD.Vt, oSVD.h_Min_Vt, oSVD.w_Min_Vt,"Vt");
    
    //验算SVD结果
    Test_SVD(M,oSVD,&iResult,0.000001);

    //基础行变换，可以看秩
    Elementary_Row_Operation_1(M, h, w, M, &iResult);

    //Disp((_T*)oSVD.U, oSVD.h_Min_U, oSVD.w_Min_U);
    return;
#undef _T
}
int main()
{
    Init_Env();

    Ransac_Test();
    Free_Env();
#ifdef WIN32
    _CrtDumpMemoryLeaks();
#endif
}
