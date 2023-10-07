#ifdef WIN32
	#include "io.h"
	#include "windows.h"
#endif

#include "Common.h"

//unsigned long long iGet_File_Length(char* pcFile)
//{//return: >-0 if success; -1 if fail
//	FILE* pFile = fopen(pcFile, "rb");
//	unsigned long long iLen;
//	if (!pFile)
//		return -1;
//
//#ifdef WIN32
//	_fseeki64(pFile, 0, SEEK_END);
//	iLen = _ftelli64(pFile);
//#else
//	fpos64_t oPos;
//	if (fgetpos64(pFile, &oPos) == 0)
//		iLen = oPos.__pos;
//	else
//		iLen = 0;
//#endif
//	fclose(pFile);
//	return iLen;
//}

#ifdef WIN32

void Get_All_File(const char* pcPath,char *pBuffer)
{//取一个目录所有文件 例: c:\\tmp\\*.bin
	intptr_t handle;
	_finddata_t findData;
	int iCount = 0;
	char* pCur = pBuffer;
	handle = _findfirst(pcPath, &findData);    // 查找目录中的第一个文件
	if (handle == -1)
	{
		printf("File not found\n");
		return;
	}
	do
	{
		if (findData.attrib & _A_ARCH)
		{
			strcpy(pCur, findData.name);
			pCur += strlen(pCur)+1;
			iCount++;
		}	
	} while (_findnext(handle, &findData) == 0);    // 查找目录中的下一个文件
	_findclose(handle);    // 关闭搜索句柄
}
int iGet_File_Count(const char* pcPath)
{//给定文件路径，求此路径下得文件数量
	intptr_t handle;
	_finddata_t findData;
	int iCount = 0;
	handle = _findfirst(pcPath, &findData);    // 查找目录中的第一个文件
	if (handle == -1)
	{
		printf("File not found\n");
		return 0;
	}
	do
	{
		if (findData.attrib & _A_ARCH)
			iCount++;
	} while (_findnext(handle, &findData) == 0);    // 查找目录中的下一个文件
	_findclose(handle);    // 关闭搜索句柄
	return iCount;
}
#endif

int bSave_Bin(const char* pcFile, float* pData, int iSize)
{
	FILE* pFile = fopen(pcFile, "wb");
	if (!pFile)
	{
		printf("Fail to save:%s\n", pcFile);
		return 0;
	}
	int iResult = (int)fwrite(pData, 1, iSize, pFile);
	if (iResult != iSize)
	{
		printf("Fail to save:%s\n", pcFile);
		iResult = 0;
	}
	else
		iResult = 1;
	return iResult;
}
int iGet_Upper_Triangle_Size(int w)
{
	int iSize;
	iSize = w * (w - 1) / 2;
	return iSize;
}

int iUpper_Triangle_Cord_2_Index(int x, int y, int w)
{//一个上三角矩阵，给定(x,y)坐标，转换为索引值, 注意，w为上三角矩形的列数, 不是上三角第一行有效元数个数
	int iPos;
	//比如 w=4, y=2 x=3 Index=7
	if (x <= y || y >= w - 1)
		return -1;
	iPos = ((w - 1) + (w - y)) * y / 2 + x - y - 1;
	return iPos;
}