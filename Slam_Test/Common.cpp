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
unsigned long long iGet_File_Length(char* pcFile)
{//return: >-0 if success; -1 if fail
	FILE* pFile = fopen(pcFile, "rb");
	__int64 iLen;
	if (!pFile)
	{
		int iResult = GetLastError();
		printf("Fail to get file length, error:%d\n", iResult);
		return -1;
	}
	_fseeki64(pFile, 0, SEEK_END);
	iLen = _ftelli64(pFile);
	fclose(pFile);
	return iLen;
}

unsigned long long iGet_Tick_Count()
{//求当前的毫秒级Tick Count。之所以不搞微秒级是因为存在时间片问题，即使毫秒级也不准，很有可能Round到16毫秒一个时间片，聊胜于无
	timeb tp;
	ftime(&tp);
	return (unsigned long long) ((unsigned long long)tp.time * 1000 + tp.millitm);
}

unsigned int iGet_Random_No()
{//伪随机	(789, 123)	(389,621) 已通过，可以自定义自己的种子
#define b 389
#define c 621
	static unsigned int iNum = 0xFFFFFFF;	//GetTickCount();	//种子
	return iNum = iNum * b + c;
#undef b
#undef c
}

int iRandom(int iStart, int iEnd)
{//从iStart到iEnd之间随机出一个数字，由于RandomInteger太傻逼，没有必要把时间浪费在傻逼身上
	return iStart + iGet_Random_No() % (iEnd - iStart + 1);
}
int iRandom()
{//尝试搞一个无参数获得随机数的函数，计算办法为 iRandom_No= iRandom_No*a + c; 最后取模 0x7FFFFFFF
#define m 1999999973			//基于模为素数能令散列更均匀的理论
	static unsigned int a = (unsigned int)iGet_Tick_Count(); //1103515245;		//1103515245为很好的初始值，但选取静态值变成了确定问题
	static unsigned int c = 2347;	// 12345;
	static unsigned long long iRandom_No = 1;
	iRandom_No = (iRandom_No * a + c) % m;	//求和不溢出，再求模
	return (int)iRandom_No;
#undef m
}
template<typename _T>int iQuick_Sort_Partition(_T pBuffer[], int left, int right)
{//小到大的顺序
	//Real fRef;
	_T iValue, oTemp;
	int pos = right;

	right--;
	iValue = pBuffer[pos];
	while (left <= right)
	{
		while (left < pos && pBuffer[left] <= iValue)
			left++;
		while (right >= 0 && pBuffer[right] > iValue)
			right--;
		if (left >= right)
			break;
		oTemp = pBuffer[left];
		pBuffer[left] = pBuffer[right];
		pBuffer[right] = oTemp;
	}

	oTemp = pBuffer[left];
	pBuffer[left] = pBuffer[pos];
	pBuffer[pos] = oTemp;

	return left;
}
template<typename _T>int iAdjust_Left(_T* pStart, _T* pEnd)
{
	_T oTemp, * pCur_Left = pEnd - 1,
		* pCur_Right;
	_T oRef = *pEnd;

	//为了减少一次判断，此处先扫过去
	while (pCur_Left >= pStart && *pCur_Left == oRef)
		pCur_Left--;
	pCur_Right = pCur_Left;
	pCur_Left--;

	while (pCur_Left >= pStart)
	{
		if (*pCur_Left == oRef)
		{
			oTemp = *pCur_Left;
			*pCur_Left = *pCur_Right;
			*pCur_Right = oTemp;
			pCur_Right--;
		}
		pCur_Left--;
	}
	return (int)(pCur_Right - pStart);
}
template<typename _T> _T oGet_Nth_Elem(_T Seq[],int iCount, int iStart, int iEnd, int iNth)
{//取第n大元素，用Quick_Sort的Partition做
	int iPos;
	if (iStart < iEnd)
	{
		iPos = iQuick_Sort_Partition(Seq, iStart, iEnd);
		if (iPos > iNth)
		{//那么第n大元素在左分区内
			return oGet_Nth_Elem(Seq,iCount, iStart, iPos - 1, iNth);
		}
		else if (iPos < iNth)
		{//第n大元素在右分区内
			return oGet_Nth_Elem(Seq,iCount, iPos + 1, iEnd, iNth);
		}
		else //找到了，正好在iPos中
			return Seq[iPos];
	}
	else
	{//此时又分奇偶两种情况
		if(iCount&1)
			return Seq[iStart];	//奇数好办，返回便是
		else //偶数的还要往前找最大值
		{
			_T fMax = Seq[iStart - 1];
			for (int i = iStart - 2; i >= 0; i--)
			{
				if (Seq[i] > fMax)
					fMax = Seq[i];
			}
			return (_T)((Seq[iStart] + fMax) / 2.f);
		}
	}
		
		
}
template<typename _T> void Quick_Sort(_T Seq[], int iStart, int iEnd)
{
	int iPos, iLeft;
	if (iStart < iEnd)
	{
		iPos = iQuick_Sort_Partition(Seq, iStart, iEnd);
		//如果iPos>=iEnd, 则遇上极差形态的组了，这时做一个推进，找出所有与Seq[iEnd]相等的项目，赶到右边
		if (iPos >= iEnd)
		{
			iLeft = iAdjust_Left(Seq + iStart, Seq + iPos);
			iLeft = iStart + iLeft;
		}
		else
			iLeft = iPos - 1;

		if (iStart < iLeft)
			Quick_Sort(Seq, iStart, iLeft);

		Quick_Sort(Seq, iPos + 1, iEnd);
	}
}
void SB_Common()
{//template实例化，只对vc有效
	bSave_PLY(NULL, (double(*)[3])NULL, 0);
	bSave_PLY(NULL, (float(*)[3])NULL, 0);

	oGet_Nth_Elem((double*)NULL, 0, 0, 0, 0);
	oGet_Nth_Elem((float*)NULL, 0, 0, 0, 0);
	oGet_Nth_Elem((int*)NULL, 0, 0, 0, 0);

	Quick_Sort((double*)NULL, 0, 0);
	Quick_Sort((float*)NULL, 0, 0);
	Quick_Sort((int*)NULL, 0, 0);
}

int bLoad_Raw_Data(const char* pcFile, unsigned char** ppBuffer,int *piSize)
{
	FILE* pFile = fopen(pcFile, "rb");
	int bRet = 0, iResult,iSize;
	unsigned char* pBuffer;
	iSize = (int)iGet_File_Length((char*)pcFile);
	pBuffer = (unsigned char*)malloc(iSize);

	if (!pFile)
	{
		printf("Fail to open file:%s\n", pcFile);
		goto END;
	}
	if (!pBuffer)
	{
		printf("Fail to allocate memory\n");
		goto END;
	}

	iResult = (int)fread(pBuffer, 1, iSize, pFile);
	if (iResult != iSize)
	{
		if (pBuffer)
			free(pBuffer);
		*ppBuffer = NULL;
		printf("Fail to read data\n");
		goto END;
	}
	*ppBuffer = pBuffer;
	if (piSize)
		*piSize = iSize;
	bRet = 1;
END:
	if (pFile)
		fclose(pFile);
	if (!bRet)
	{
		if (pBuffer)
			free(pBuffer);
	}
	return bRet;
}

template<typename _T>int bSave_PLY(const char* pcFile, _T Point[][3], int iPoint_Count, unsigned char Color[][3], int bText)
{//存点云，最简形式，用于实验，连结构都不要
	FILE* pFile = fopen(pcFile, "wb");
	char Header[512];
	int i;
	_T* pPos;

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

	if (Color)
	{
		sprintf(Header + strlen(Header), "property uchar red\r\n");
		sprintf(Header + strlen(Header), "property uchar green\r\n");
		sprintf(Header + strlen(Header), "property uchar blue\r\n");
	}

	sprintf(Header + strlen(Header), "end_header\r\n");
	fwrite(Header, 1, strlen(Header), pFile);

	for (i = 0; i < iPoint_Count; i++)
	{
		pPos = Point[i];
		if (bText)
		{
			fprintf(pFile, "%f %f %f ", pPos[0], pPos[1], pPos[2]);
			if (Color)
				fprintf(pFile, "%d %d %d\r\n", Color[i][0], Color[i][1], Color[i][2]);
			else
				fprintf(pFile, "\r\n");
		}
	}
	fclose(pFile);
	return 1;
}