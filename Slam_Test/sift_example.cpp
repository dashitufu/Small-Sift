#include "Common.h"
#include "Image.h"
#include "sift.h"

static void Sift_Test_1()
{//Siftʵ�飬����һ�źڰ�ͼ�������е�������
    float(*pPoint)[2];
    int iCount;
    unsigned long long tStart = iGet_Tick_Count();
    Get_Sift_Feature("c:\\tmp\\Screen_Cut_A.bmp", &pPoint, &iCount, 0);
    printf("%lld\n", iGet_Tick_Count() - tStart);
    free(pPoint);
    return;
}
static void Sift_Test_2()
{//����ͼ��ƥ�䣬��������ƥ���λ��
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
{//����Ŀ¼�������˴���Mem_Mgr
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
{//����Ŀ¼���������ӿ�
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