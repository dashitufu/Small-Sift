//这个文件管所有独立于特殊平台的通用函数
#pragma once
#include "stdio.h"
#include "stdlib.h"
#include "memory.h"
#include "sys/timeb.h"
#include "math.h"
#include <mutex>
using namespace std;

#ifndef WIN32
#include <unistd.h>	//给sleep用
#endif

#define Abs(A) ((A)>=0?(A):(-(A)))
#ifndef Min 
#define Min(A,B)( A<=B?A:B)
#endif
#ifndef Max
#define Max(A,B)(A>=B?A:B)
#endif

#define ALIGN_SIZE_128(iSize) ((( (int)(iSize)+127)>>7)<<7) 
#define ALIGN_SIZE_1024(iSize) ((( (int)(iSize)+1023)>>10)<<10) 

#define bGet_Bit(pBuffer, iBit_Pos) (pBuffer)[(iBit_Pos) >> 3] & (1 << ((iBit_Pos) & 0x7))
//int bGet_Bit(unsigned char* pBuffer, int iBit_Pos)
//{	return pBuffer[iBit_Pos >> 3] & (1 << (iBit_Pos & 0x7));}
#define Set_Bit(pBuffer, iBit_Pos) \
{\
	(pBuffer)[(iBit_Pos) >> 3] |= (1 << ((iBit_Pos) & 0x7)); \
}

template<typename _T> struct Point_2D {
	unsigned int m_iCamera_Index;
	unsigned int m_iPoint_Index;
	_T m_Pos[2];
};

//void Set_Bit(unsigned char* pBuffer, int iBit_Pos)
//{	pBuffer[iBit_Pos >> 3] |= (1 << (iBit_Pos & 0x7)); }

unsigned long long iGet_File_Length(char* pcFile);
unsigned long long iGet_Tick_Count();
int iGet_File_Count(const char* pcPath);	//获取一个目录所有文件
void Get_All_File(const char* pcPath, char* pBuffer);
int bSave_Bin(const char* pcFile, float* pData, int iSize);
int bLoad_Raw_Data(const char* pcFile, unsigned char** ppBuffer, int* piSize);

//上三角坐标转换为索引值
int iUpper_Triangle_Cord_2_Index(int x, int y, int w);

//上三角有效元数个数
int iGet_Upper_Triangle_Size(int w);

//一组有的没的随机数生成
int iRandom(int iStart, int iEnd);
int iRandom();

//早晚得废
//以后废弃的临时函数
template<typename _T>
void Temp_Load_Match_Point(_T(**ppPoint_1)[2], _T(**ppPoint_2)[2], int* piCount)
{
	_T(*pPoint_1)[2], (*pPoint_2)[2];
	int i, iCount = (int)iGet_File_Length((char*)"c:\\tmp\\2.bin") / (4 * sizeof(float));
	pPoint_1 = (_T(*)[2])malloc(iCount * 2 * sizeof(_T));
	pPoint_2 = (_T(*)[2])malloc(iCount * 2 * sizeof(_T));
	FILE* pFile = fopen("c:\\tmp\\1.bin", "rb");
	for (i = 0; i < iCount; i++)
	{
		float Data[2];
		fread(Data, 1, 2 * sizeof(float), pFile);
		pPoint_1[i][0] = (_T)Data[0];
		pPoint_1[i][1] = (_T)Data[1];
		fread(Data, 1, 2 * sizeof(float), pFile);
		pPoint_2[i][0] = (_T)Data[0];
		pPoint_2[i][1] = (_T)Data[1];		
	}
	fclose(pFile);
	*ppPoint_1 = pPoint_1, * ppPoint_2 = pPoint_2;
	*piCount = iCount;
}
//解决两组基本数据类型的第n大与及快速排序，待优化
template<typename _T> _T oGet_Nth_Elem(_T Seq[], int iCount, int iNth);
template<typename _T> void Quick_Sort(_T Seq[], int iStart, int iEnd);
template<typename _T>int bSave_PLY(const char* pcFile, _T Point[][3], int iPoint_Count,unsigned char Color[][3]=NULL, int bText=1);

//Temp code 
template<typename _T>void Temp_Load_File(const char* pcFile, _T(**ppPoint_3D_1)[3], _T(**ppPoint_3D_2)[3], int* piCount);
template<typename _T>void Temp_Load_File_1(const char* pcFile, _T(**ppPoint_3D_1)[3], _T(**ppPoint_3D_2)[3], int* piCount);
template<typename _T>void Temp_Load_File_2(int* piCameta_Count, int* piPoint_Count, int* piObservation_Count,
	Point_2D<_T>** ppPoint_2D, float(**ppPoint_3D)[3], float(**ppCamera)[3 * 3]);
//以下函数也没啥用，第九讲用
template<typename _T>void Normalize(_T Point_3D[][3], Point_2D<_T> Point_2D[], int iPoint_Count, _T Camera[][9], int iCamera_Count);