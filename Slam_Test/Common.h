//����ļ������ж���������ƽ̨��ͨ�ú���
#include "stdio.h"
#include "stdlib.h"
#include "memory.h"
#include "sys/timeb.h"
#include "math.h"
#include <mutex>
using namespace std;

#ifndef WIN32
#include <unistd.h>	//��sleep��
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

//unsigned long long iGet_File_Length(char* pcFile);
int iGet_File_Count(const char* pcPath);	//��ȡһ��Ŀ¼�����ļ�
void Get_All_File(const char* pcPath, char* pBuffer);
int bSave_Bin(const char* pcFile, float* pData, int iSize);

//����������ת��Ϊ����ֵ
int iUpper_Triangle_Cord_2_Index(int x, int y, int w);

//��������ЧԪ������
int iGet_Upper_Triangle_Size(int w);