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

unsigned long long iGet_File_Length(char* pcFile);
unsigned long long iGet_Tick_Count();
int iGet_File_Count(const char* pcPath);	//获取一个目录所有文件
void Get_All_File(const char* pcPath, char* pBuffer);
int bSave_Bin(const char* pcFile, float* pData, int iSize);

//上三角坐标转换为索引值
int iUpper_Triangle_Cord_2_Index(int x, int y, int w);

//上三角有效元数个数
int iGet_Upper_Triangle_Size(int w);

//早晚得肥
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