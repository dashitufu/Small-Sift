//由于数学库同时应对float/double两种类型，故此该版开始，全变为模板函数
#pragma once 
#include "Common.h"

extern "C"
{
#include "Buddy_System.h"
}


#define sign(x) (x>=0?1:-1)

#define Get_Max_1(V, fMax,i) \
{ \
	double fAbs_Max; \
	fAbs_Max = Abs(fMax=V[0]); \
	i=0; \
	if (Abs(V[1]) > fAbs_Max) \
	{ \
		fAbs_Max = Abs(V[1]), fMax = V[1]; \
		i=1;	\
	} \
	if (Abs(V[2] > fAbs_Max)) \
	{ \
		fAbs_Max = Abs(V[2]), fMax = V[2]; \
		i=2;	\
	} \
}

#define PI 3.1415926f
#define Abs(A) ((A)>=0?(A):(-(A)))
#define MAX_FLOAT ((float)0xFFFFFFFFFFFFFFFF)	//仅仅给一个足够大的数字，并不是iEEE的浮点数最大值
#define ZERO_APPROCIATE	0.00001f

#define Get_Homo_Pos(Pos, Homo_Pos) Homo_Pos[0]=Pos[0], Homo_Pos[1]=Pos[1], Homo_Pos[2]=Pos[2],Homo_Pos[3]=1;


typedef struct Complex_d {//双精度复数
	double real;	//实部
	double im;		//虚部 imagary part	
}Complex_d;
typedef struct Complex_f {//双精度复数
	float real;	//实部
	float im;		//虚部 imagary part	
}Complex_f;

typedef struct Polynormial {	//多元多项式，次数必须为正整数(>=0, =0表示没有该项
	unsigned char* m_pTerm;		//多项式中的项
	float* m_pCoeff;			//每项的系数
	int m_iElem_Count;			//变量（元）数
	int m_iTerm_Count;			//项数
	int m_iMax_Term_Count;		//目前最大项数，为日后扩展用
}Polynormial;

typedef struct SVD_Info {	//有必要给SVD做一个单独的参数结构
	void* A;		//原矩阵
	void* U;
	void* S;
	void* Vt;
	int h_A, w_A,
		h_Min_U, w_Min_U,
		w_Min_S,
		h_Min_Vt, w_Min_Vt;
	int m_bSuccess;
}SVD_Info;

template<typename _T> struct Sparse_Matrix {
public:
	typedef class Item {
	public:
		unsigned int x, y;	//0xFFFFFFFF未非法值
		_T m_fValue;
		int m_iRow_Next;	//行方向下一个，向右
		int m_iCol_Next;	//列方向下一个，向下
	}Item;
	unsigned int* m_pRow;	//行桶
	unsigned int* m_pCol;	//列桶
	Item* m_pBuffer;		//内容从[1]开始 [0]表示NULL
	int m_iCur_Item=1;		//当前在哪个Item
	int m_iMax_Item_Count=0;		//整个matrix共右多少个item
	int m_iRow_Count;
	int m_iCol_Count;
};

extern Mem_Mgr oMatrix_Mem;

unsigned long long iGet_Random_No_cv(unsigned long long* piState);

#define Malloc_1(oPtr, iSize,pBuffer) \
{ \
	(pBuffer)= (_T*)((oPtr).m_pBuffer+(oPtr).m_iCur); \
	(oPtr).m_iCur +=ALIGN_SIZE_128((iSize)); \
	if ( (oPtr).m_iCur > (oPtr).m_iMax_Buffer_Size) \
	{ \
		(oPtr).m_iCur -= ALIGN_SIZE_128((iSize));\
		(pBuffer)=NULL; \
		printf("Fail to allocate memory in Malloc, Total:%u Remain:%u Need:%d\n",(oPtr).m_iMax_Buffer_Size,(oPtr).m_iMax_Buffer_Size-(oPtr).m_iCur,(int)(iSize)); \
	} \
}

template<typename _T>void Disp(_T* M, int iHeight, int iWidth,const char* pcCaption=NULL)
{
	int i, j;
	if (pcCaption)
		printf("%s\n", pcCaption);

	for (i = 0; i < iHeight; i++)
	{
		for (j = 0; j < iWidth; j++)
		{
			if (std::is_same_v<_T, float> || std::is_same_v<_T,double>)
				printf("%.8f, ", M[i * iWidth + j]);
			else if (std::is_same_v<_T, int> || std::is_same_v<_T,short>)
				printf("%d,", (int)M[i * iWidth + j]);
		}
			
		printf("\n");
	}
	return;
}
template<typename _T>void Matrix_Transpose(_T* A, int ma, int na, _T* At)
{//矩阵转置
	int y, x;
	_T* At_1;
	//Light_Ptr oPtr = oMatrix_Mem;
	//Malloc_1(oPtr, ma * na * sizeof(_T), At_1);
	At_1 = (_T*)pMalloc(&oMatrix_Mem, ma * na * sizeof(_T));
	for (y = 0; y < ma; y++)
		for (x = 0; x < na; x++)
			At_1[x * ma + y] = A[y * na + x];
	memcpy(At, At_1, ma * na * sizeof(_T));
	Free(&oMatrix_Mem, At_1);
	return;
}
template<typename _T>
void Matrix_Multiply_3x3(_T A[3*3], _T B[3*3], _T C[3*3])
{//计算C=AxB
	Light_Ptr oPtr = oMatrix_Mem;
	_T* C_1;
	Malloc_1(oPtr, 3 * 3 * sizeof(_T),C_1);
	C_1[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
	C_1[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
	C_1[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

	C_1[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
	C_1[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
	C_1[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

	C_1[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
	C_1[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
	C_1[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
	memcpy(C, C_1, 9 * sizeof(_T));
	return;
}

void Init_Env();
void Free_Env();
//排列组合函数
unsigned int iFactorial(int n);

unsigned int iGet_Random_No();
unsigned long long iGet_Random_No_cv();
unsigned long long iGet_Random_No_cv(unsigned long long* piState);
int iGet_Random_No_cv(int a, int b);

//void Schmidt_Orthogon(float* A, int m, int n, float* B);	//施密特正交化
template<typename _T>void Schmidt_Orthogon(_T* A, int m, int n, _T* B);

void Polor_2_Rect_Coordinate(float rho, float theta, float phi, float* px, float* py, float* pz);	//极坐标转直角坐标系
void Polor_2_Rect_Coordinate(float rho, float theta, float* px, float* py);							//二维
void Rect_2_Polor_Coordinate(float x, float y, float z, float* prho, float* ptheta, float* pphi);	//直角坐标系转极坐标
void Rect_2_Screen_Coordinate(float x, float y, int* px_Screen, int* py_Screen, int iWidth = 1920, int iHeight = 1080);	//直角坐标系转屏幕坐标
void Screen_2_Coordinate(int x_Screen, int y_Screen, float* px, float* py, int iWidth = 1920, int iHeight = 1080);			//屏幕坐标转直角坐标系

//一组与线性方程有关的函数
void Cholosky_Decompose(float A[], int iOrder, float B[]);
void Conjugate_Gradient(float* A, const int n, float B[], float X[]);
void Get_Inv_Matrix(float* pM, float* pInv, int iOrder, int* pbSuccess);
void Solve_Linear_Cramer(float* A, int iOrder, float* B, float* X, int* pbSuccess); //克莱姆法则解线性方程组

template<typename _T>void Solve_Linear_Gause(_T* A, int iOrder, _T* B, _T* X, int* pbSuccess);	//高斯法求解线性方程组
//void Solve_Linear_Gause(float* A, int iOrder, float* B, float* X, int* pbSuccess=NULL);

void Solve_Linear_Jocabi(float A[], float B[], int iOrder, float X[], int* pbResult);				//雅可比迭代法求解线性方程组
void Solve_Linear_Gauss_Seidel(float A[], float B[], int iOrder, float X[], int* pbResult);
void Solve_Linear_Contradictory(float* A, int m, int n, float* B, float* X, int* pbSuccess);	//解矛盾方程组，求最小二乘解
//void Solve_Linear_Solution_Construction(float* A, int m, int n, float B[], int* pbSuccess, float* pBasic_Solution = NULL, int* piBasic_Solution_Count = NULL, float* pSpecial_Solution = NULL);
template<typename _T> void Solve_Linear_Solution_Construction(_T* A, int m, int n, _T B[], int* pbSuccess, _T* pBasic_Solution=NULL, int* piBasic_Solution_Count=NULL, _T* pSpecial_Solution=NULL);

void Get_Linear_Solution_Construction(float A[], int m, int n, float B[]);//看解的结构
//void Elementary_Row_Operation(float A[], int m, int n, float A_1[], int* piRank = NULL, float** ppBasic_Solution = NULL, float** ppSpecial_Solution = NULL);		//初等行变换至最简形
template<typename _T>void Elementary_Row_Operation(_T A[], int m, int n, _T A_1[], int* piRank=NULL, _T** ppBasic_Solution=NULL, _T** ppSpecial_Solution=NULL);

template<typename _T>void Elementary_Row_Operation_1(_T A[], int m, int n, _T A_1[], int* piRank=NULL, _T** ppBasic_Solution=NULL, _T** ppSpecial_Solution=NULL);

int bIs_Linearly_Dependent(float A[], int m, int n);	//判断向量组A是否线性相关
template<typename _T>int bIs_Orthogonal(_T* A, int h, int w=0);
template<typename _T>void Gen_I_Matrix(_T M[], int h, int w);	//生成对角阵

void Gen_Iterate_Matrix(float A[], int n, float B[], float B_1[], float C[]);	//生成迭代中的B
int bIs_Contrative_Mapping(float B[], int n);					//判断一个矩阵是否为压缩矩阵，这是一个线性方程组是否能用迭代法求解的必要条件
int bIs_Diagonal_Dominant(float A[], int iOrder);			//判断一个方阵是否为对角线占优而且是行占优矩阵
int bIs_Unit_Vector(float* V, int na);	//测试向量是否为单位向量
void Diagonalize(float A[], int iOrder, float Diag[], float P[], int* pbSuccess);	//对称矩阵对角化

void Solve_Cubic(double A[3][3], Complex_d Root[3], int* piRoot_Count);
void Solve_Eigen(double A[3][3], Complex_d Root[3], int* piRoot_Count);
void Solve_Eigen_3x3(float A[], Complex_f Root[3]);
void Solve_Eigen_Vector(float A[], int iOrder, float fEigen_Value, float Q[], int* piCount);	//根据给定的特征值代入A， 求解特征方程得到特征向量

void Solve_Poly(float Coeff[], int iCoeff_Count, Complex_f Root[], int* piRoot_Count = NULL);

//**************************稀疏矩阵*******************************
#define Get_Item(oMatrix, x1, y1, poItem) \
{ \
	int iCur; \
	iCur = oMatrix.m_pRow[y1]; \
	if (!iCur) \
		poItem = NULL; \
	else \
	{\
		poItem = &oMatrix.m_pBuffer[iCur];\
		while (poItem)\
		{\
			if (poItem->x == x1)\
				break;\
			if (poItem->m_iRow_Next && poItem->x < x1)\
				poItem = &oMatrix.m_pBuffer[poItem->m_iRow_Next];\
			else\
			{\
				poItem = NULL;\
				break;\
			}\
		}\
	}\
}

template<typename _T>void Set_Value(Sparse_Matrix<_T>* poMatrix, int x, int y, _T fValue)
{//对稀疏矩阵赋值某单元
	/*Sparse_Matrix<_T>::Item* poItem;
	Get_Item(oMatrix, x, y, poItem);
	if (poItem)
	{
		poItem->m_fValue = fValue;
		return;
	}
	if (oMatrix.m_iCur_Item >= oMatrix.m_iItem_Count)
	{
		printf("Exceed max Item _Count:%d in Set_Value\n", oMatrix.m_iItem_Count);
		return;
	}
	poItem =&oMatrix.m_pBuffer[oMatrix.m_iCur_Item++];
	poItem->x = x;
	poItem->y = y;
	poItem->m_fValue = fValue;*/

	Sparse_Matrix<_T> oMatrix = *poMatrix;
	//插入到相应为止
	Sparse_Matrix<_T>::Item* poCur, * poPrevious, oItem;
	int iCur_Link_Item;
	oItem.x = x;
	oItem.y = y;
	oItem.m_fValue = fValue;
	//先找到相应的行
	if (!(iCur_Link_Item = oMatrix.m_pRow[oItem.y]))
	{	//桶中没有指向，直接插入
		oMatrix.m_pRow[oItem.y] = oMatrix.m_iCur_Item;
		oItem.m_iRow_Next = 0;
	}
	else {
		poCur = &oMatrix.m_pBuffer[iCur_Link_Item];
		poPrevious = NULL;
		while (poCur->x < oItem.x && poCur->m_iRow_Next)
		{
			poPrevious = poCur;
			poCur = &oMatrix.m_pBuffer[poCur->m_iRow_Next];
		}
		if (poCur->x < oItem.x)
		{
			oItem.m_iRow_Next = poCur->m_iRow_Next;
			poCur->m_iRow_Next = oMatrix.m_iCur_Item;
		}
		else if (poCur->x == oItem.x)
		{//该位置已经有值，改写了事
			poCur->m_fValue = fValue;
			return;
		}
		else
		{
			if (poPrevious)
			{//插在previous与cur之间
				oItem.m_iRow_Next = poPrevious->m_iRow_Next;
				poPrevious->m_iRow_Next = oMatrix.m_iCur_Item;
			}
			else
			{
				oItem.m_iRow_Next = iCur_Link_Item;
				oMatrix.m_pRow[oItem.y] = oMatrix.m_iCur_Item;
			}
		}
	}

	//按顺序插入到列链表中
	if (!(iCur_Link_Item = oMatrix.m_pCol[oItem.x]))
	{	//桶中没有指向，直接插入
		oMatrix.m_pCol[oItem.x] = oMatrix.m_iCur_Item;
		oItem.m_iCol_Next = 0;
	}
	else {
		poCur = &oMatrix.m_pBuffer[iCur_Link_Item];
		poPrevious = NULL;
		while (poCur->y < oItem.y && poCur->m_iCol_Next)
		{
			poPrevious = poCur;
			poCur = &oMatrix.m_pBuffer[poCur->m_iCol_Next];
		}
		if (poCur->y < oItem.y)
		{
			oItem.m_iCol_Next = poCur->m_iCol_Next;
			poCur->m_iCol_Next = oMatrix.m_iCur_Item;
		}
		else
		{
			if (poPrevious)
			{//插在previous与cur之间
				oItem.m_iCol_Next = poPrevious->m_iCol_Next;
				poPrevious->m_iCol_Next = oMatrix.m_iCur_Item;
			}
			else
			{
				oItem.m_iCol_Next = iCur_Link_Item;
				oMatrix.m_pCol[oItem.x] = oMatrix.m_iCur_Item;
			}
		}
	}

	if (oItem.y >= (unsigned int)oMatrix.m_iRow_Count)
		oMatrix.m_iRow_Count = oItem.y + 1;
	if (oItem.x >= (unsigned int)oMatrix.m_iCol_Count)
		oMatrix.m_iCol_Count = oItem.x + 1;

	if (oMatrix.m_iCur_Item > oMatrix.m_iMax_Item_Count)
	{
		printf("Exceed max item count in Set_Value:%d %d\n", x, y);
		return;
	}
	oMatrix.m_pBuffer[oMatrix.m_iCur_Item++] = oItem;
	*poMatrix = oMatrix;
	return;
}

template<typename _T>void Resize_Matrix(Sparse_Matrix<_T>* poA, int iNew_Item_Count);
template<typename _T>void Matrix_Multiply(Sparse_Matrix<_T> A, Sparse_Matrix<_T> B, Sparse_Matrix<_T>* poC)
{
	int y, x, i0_Count = 0, iNew_Item_Count = 0;
	_T fValue;
	Sparse_Matrix<_T>::Item* poRow_Cur, * poCol_Cur, * poNew_Row_Pre = NULL, oNew_Item;//* poNew_Col_Pre = NULL,
	Sparse_Matrix<_T> oC;
	//pNew_Col_Pre实际上是一行Item, 从以存放上次各列的最后一个Item,以便加快速度
	Sparse_Matrix<_T>::Item** pNew_Col_Pre = (Sparse_Matrix<_T>::Item**)malloc(B.m_iCol_Count * sizeof(Sparse_Matrix<_T>::Item*));

	if (A.m_iCol_Count != B.m_iRow_Count || !poC)
		return;
	if (!poC->m_iMax_Item_Count)
		Init_Sparse_Matrix(&oC, A.m_iRow_Count * B.m_iCol_Count, Max(A.m_iRow_Count, B.m_iCol_Count));
	else
	{
		oC = *poC;
		memset(oC.m_pRow, 0, oC.m_iRow_Count * sizeof(int));
		memset(oC.m_pCol, 0, oC.m_iCol_Count * sizeof(int));
	}
	for (y = 0; y < A.m_iRow_Count; y++)
	{
		for (x = 0; x < B.m_iCol_Count; x++)
		{
			fValue = 0;
			if (A.m_pRow[y] && B.m_pCol[x])
			{
				poRow_Cur = &A.m_pBuffer[A.m_pRow[y]];
				poCol_Cur = &B.m_pBuffer[B.m_pCol[x]];
				do
				{
					if (poRow_Cur->x == poCol_Cur->y)
					{
						fValue += poRow_Cur->m_fValue * poCol_Cur->m_fValue;
						poRow_Cur = poRow_Cur->m_iRow_Next ? &A.m_pBuffer[poRow_Cur->m_iRow_Next] : NULL;
						poCol_Cur = poCol_Cur->m_iCol_Next ? &B.m_pBuffer[poCol_Cur->m_iCol_Next] : NULL;
					}
					else if (poRow_Cur->x < poCol_Cur->y)
						poRow_Cur = poRow_Cur->m_iRow_Next ? &A.m_pBuffer[poRow_Cur->m_iRow_Next] : NULL;
					else
						poCol_Cur = poCol_Cur->m_iCol_Next ? &B.m_pBuffer[poCol_Cur->m_iCol_Next] : NULL;
				} while (poRow_Cur && poCol_Cur);
			}

			if (fValue != 0)
			{//在目标矩阵c中加一个Item
				oNew_Item.x = x;
				oNew_Item.y = y;
				oNew_Item.m_fValue = fValue;
				oNew_Item.m_iRow_Next = oNew_Item.m_iCol_Next = 0;
				oC.m_pBuffer[++iNew_Item_Count] = oNew_Item;

				//先加入行链表
				if (!oC.m_pRow[y])
					oC.m_pRow[y] = iNew_Item_Count;
				else
					poNew_Row_Pre->m_iRow_Next = iNew_Item_Count;

				//再加入列链表
				if (!oC.m_pCol[x])
					oC.m_pCol[x] = iNew_Item_Count;
				else
					pNew_Col_Pre[x]->m_iCol_Next = iNew_Item_Count;
				pNew_Col_Pre[x] = poNew_Row_Pre = &oC.m_pBuffer[iNew_Item_Count];
			}
			else
				i0_Count++;
		}
	}
	oC.m_iRow_Count = A.m_iRow_Count;
	oC.m_iCol_Count = B.m_iCol_Count;

	oC.m_iCur_Item = iNew_Item_Count;
	*poC = oC;
	free(pNew_Col_Pre);
	return;	
}
template<typename _T>void Matrix_Multiply(Sparse_Matrix<_T> A, _T a)
{//C = aA
	int i;
	for (i = 1; i < A.m_iCur_Item; i++)
		A.m_pBuffer[i].m_fValue *= a;
	return;
}
template<typename _T>void Matrix_Transpose_1(Sparse_Matrix<_T> A, Sparse_Matrix<_T>* poAt)
{
	Sparse_Matrix<_T> At = *poAt;
	Sparse_Matrix<_T>::Item* poCur;
	unsigned int iTemp;
	//Init_Sparse_Matrix(&At, A.m_iItem_Count, Max(A.m_iRow_Count, A.m_iCol_Count));
	if (!At.m_pBuffer || At.m_iRow_Count != A.m_iCol_Count || At.m_iCol_Count != A.m_iRow_Count)
	{
		printf("Invalid parameter in Matrix_Transpose_1\n");
		return;
	}		
	At.m_iRow_Count = A.m_iCol_Count;
	At.m_iCol_Count = A.m_iRow_Count;

	memcpy(At.m_pBuffer + 1, A.m_pBuffer + 1, A.m_iMax_Item_Count * sizeof(Sparse_Matrix<_T>::Item));
	memcpy(At.m_pRow, A.m_pCol, At.m_iRow_Count * sizeof(unsigned int));
	memcpy(At.m_pCol, A.m_pRow, At.m_iCol_Count * sizeof(unsigned int));

	for (int i = 1; i <= At.m_iMax_Item_Count; i++)
	{
		poCur = &At.m_pBuffer[i];
		//交换行指针，列指针
		iTemp = poCur->m_iRow_Next;
		poCur->m_iRow_Next = poCur->m_iCol_Next;
		poCur->m_iCol_Next = iTemp;

		//交换行号，列号
		iTemp = poCur->x;
		poCur->x = poCur->y;
		poCur->y = iTemp;
	}
	*poAt = At;
	return;
}
template<typename _T>void Init_Sparse_Matrix(Sparse_Matrix<_T>* poMatrix, int iItem_Count, int iMax_Order)
{
	poMatrix->m_iMax_Item_Count = iItem_Count;
	int iSize = poMatrix->m_iMax_Item_Count * sizeof(Sparse_Matrix<_T>::Item);
	unsigned char* pBuffer = (unsigned char*)pMalloc(&oMatrix_Mem, iSize + iMax_Order * 2 * sizeof(unsigned int));
	poMatrix->m_pBuffer = (Sparse_Matrix<_T>::Item*)pBuffer;
	if (!pBuffer)
		return;
	poMatrix->m_pBuffer--;	//以便从[1]开始
	poMatrix->m_pRow = (unsigned int*)(pBuffer + iSize);
	memset(poMatrix->m_pRow, 0, iMax_Order * 2 * sizeof(unsigned int));
	poMatrix->m_pCol = poMatrix->m_pRow + iMax_Order;
	poMatrix->m_iRow_Count = poMatrix->m_iCol_Count = 0;
}
template<typename _T>void Init_Sparse_Matrix(Sparse_Matrix<_T>* poMatrix, int iItem_Count, int w,int h)
{
	poMatrix->m_iMax_Item_Count = iItem_Count;
	int iSize = poMatrix->m_iMax_Item_Count * sizeof(Sparse_Matrix<_T>::Item);
	unsigned char* pBuffer = (unsigned char*)pMalloc(&oMatrix_Mem, iSize + (w + h) * sizeof(unsigned int));
	poMatrix->m_pBuffer = (Sparse_Matrix<_T>::Item*)pBuffer;
	poMatrix->m_pBuffer--;	//以便从[1]开始
	poMatrix->m_pRow = (unsigned int*)(pBuffer + iSize);
	memset(poMatrix->m_pRow, 0,( w + h )* sizeof(unsigned int));
	poMatrix->m_pCol = poMatrix->m_pRow + h;
	poMatrix->m_iRow_Count = h;
	poMatrix->m_iCol_Count = w;
}
template<typename _T>void Compact_Sparse_Matrix(Sparse_Matrix<_T>* poMatrix);
template<typename _T>void Free_Sparse_Matrix(Sparse_Matrix<_T>* poMatrix);

template<typename _T>void Matrix_Add(Sparse_Matrix<_T>* poA, _T a, Sparse_Matrix<_T>* poB, _T b, _T* pC);
template<typename _T>void Disp(Sparse_Matrix<_T> oMatrix, const char Caption[] = NULL)
{
	int y, x;
	Sparse_Matrix<_T>::Item* poCur;
	if (Caption)
		printf("%s\n", Caption);
	for (y = 0; y < oMatrix.m_iRow_Count; y++)
	{
		if (!oMatrix.m_pRow[y])
			for (x = 0; x < oMatrix.m_iCol_Count; x++)
				printf("%f ", 0);
		else
		{
			poCur = &oMatrix.m_pBuffer[oMatrix.m_pRow[y]];
			x = 0;
			do {
				for (; x < poCur->x; x++)
					printf("%f ", 0);
				printf("%f ", poCur->m_fValue);
				x++;
				if (poCur->m_iRow_Next)
					poCur = &oMatrix.m_pBuffer[poCur->m_iRow_Next];
				else
					break;
			} while (1);
			for (; x < oMatrix.m_iCol_Count; x++)
				printf("%f ", 0);
		}
		printf("\n");
	}
}
//**************************稀疏矩阵*******************************

template<typename _T>void Vector_Add(_T A[], _T B[], int n, _T C[]);
template<typename _T>void Vector_Minus(_T A[], _T B[], int n, _T C[]);
template<typename _T> void Matrix_Multiply(_T* A, int ma, int na, _T a, _T* C);

template<typename _T> void Matrix_Multiply(_T* A, int ma, int na, _T* B, int nb, _T* C);
template<typename _T> void Transpose_Multiply(_T A[], int m, int n, _T B[], int bAAt=1);
template<typename _T>void Matrix_Add(_T A[], _T B[], int iOrder, _T C[]);

void Matrix_x_Vector(double A[3][3], double X[3], double Y[3]);
void QR_Decompose(double A1[3][3], double Q[3][3], double R[3][3]);
void QR_Decompose(float* A, int ma, int na, float* Q, float* R);
void QR_Decompose(float* A, int na, float* R, float* Q, int* pbSuccess = NULL, int* pbDup_Root = NULL);

template<typename _T> void Normalize(_T V[], int n, _T V_1[]);
template<typename _T>void Homo_Normalize(_T V0[], int n, _T V1[]);	//这就是个傻逼方法，用来欺骗template

template<typename _T> void Get_Inv_Matrix_Row_Op(_T* pM, _T* pInv, int iOrder, int* pbSuccess=NULL);	//矩阵求逆


void Get_Inv_Matrix(float* pM, float* pInv, int iOrder, int* pbSuccess);	//用伴随矩阵求逆
template<typename _T>_T fGet_Determinant(_T* A, int iOrder);			//求行列式

template<typename _T> _T fGet_Mod(_T V[], int n);//求向量的模

template<typename _T> _T fGet_Distance(_T V_1[], _T V_2[], int n);	//求两向量距离
float fGet_Theta(float v0[], float v1[], float Axis[], int n);		//求两个向量之间的夹角
int iRank(double A[3][3]);								//求3x3矩阵的秩
template<typename _T>int iGet_Rank(_T* A, int m, int n);				//求一般矩阵的秩，用初等行变换法
void Cross_Product(float V0[], float V1[], float V2[]);	//求外积
template<typename _T>_T fDot(_T V0[], _T V1[], int iDim);		//求内积，点积

//以下为一组仿射变换矩阵的生成
template<typename _T> void Gen_Roation_Matrix_2D(_T Rotation_Center[2], _T theta, _T R[3 * 3]);
template<typename _T>void Gen_Rotation_Matrix_2D(_T R[2 * 2], float fTheta);

void Gen_Rotation_Matrix(float Axis[3], float fTheta, float R[]);//根据旋转轴与旋转角度生成一个旋转矩阵，此处用列向量，往后一路左乘
template<typename _T>void Gen_Translation_Matrix(_T Offset[3], _T T[]);	//平移变换
void Gen_Scale_Matrix(float Scale[3], float T[]);			//比例变换

//四元数,旋转矩阵，旋转向量互换函数
void Quaternion_2_Rotation_Matrix(float Q[4], float R[]);	//四元数转旋转矩阵
void Quaternion_2_Rotation_Vector(float Q[4], float V[4]);	//四元数转旋转向量
void Quaternion_Add(float Q_1[], float Q_2[], float Q_3[]);	//四元数加
void Quaternion_Minus(float Q_1[], float Q_2[], float Q_3[]);	//四元数减，其实这两个函数可以用普通向量加减
void Quaternion_Conj(float Q_1[], float Q_2[]);				//简单求个共轭
void Quaternion_Multiply(float Q_1[], float Q_2[], float Q_3[]);	//四元数乘法
void Quaternion_Inv(float Q_1[], float Q_2[]);	//四元数求逆
template<typename _T>void Rotation_Matrix_2_Quaternion(_T R[], _T Q[]);		//旋转矩阵转换四元数
template<typename _T>void Rotation_Matrix_2_Vector(_T R[3*3], _T V[4]);		//旋转矩阵转旋转向量

template<typename _T>void Rotation_Vector_2_Matrix(_T V[4], _T R[3 * 3]);		//旋转向量转换旋转矩阵
template<typename _T>void Rotation_Vector_2_Quaternion(_T V[4], _T Q[4]);	//旋转向量转换四元数
template<typename _T>void Hat(_T V[], _T M[]);				//构造反对称矩阵

void Vee(float M[], float V[3]);			//反对称矩阵到旋转向量
template<typename _T>void se3_2_SE3(_T Ksi[6], _T T[]);		//从se3向量到SE3矩阵的转换
void SE3_2_se3(float Rotation_Vector[4], float t[3], float Ksi[6]);	//一个旋转向量加一个位移向量构造出一个Ksi

void SIM3_2_sim3(float Rotation_Vector[], float t[], float s, float zeta[7]);
void sim3_2_SIM3(float zeta[7], float Rotation_Vector[4], float t[], float* ps);
template<typename _T>void Get_J_by_Rotation_Vector(_T Rotation_Vector[4], _T J[]);
template<typename _T> void Gen_Ksi_by_Rotation_Vector_t(_T Rotation_Vector[4], _T t[3], _T Ksi[6], int n=4);

template<typename _T> void Exp_Ref(_T A[], int n, _T B[]);
template<typename _T>void Gen_Homo_Matrix_2D(_T R[2 * 2], _T t[2], _T T[3 * 3]);
template<typename _T>void Gen_Homo_Matrix(_T R[3*3], _T t[3], _T c2w[3*3]);			//用旋转坐标与位移坐标构成一个SE3变换矩阵
template<typename _T>void Gen_Homo_Matrix_1(_T Rotation_Vector[3], _T t[3], _T c2w[]);

template<typename _T>void Get_R_t(_T T[4 * 4], _T R[3 * 3]=NULL, _T t[3]=NULL);

void Gen_Homo_Matrix(float R[], float t[], float s, float M[]);	//用旋转,位移，缩放构成一个变换矩阵
template<typename _T> void Gen_Cube(_T Cube[][4], float fScale, _T x_Center=0, _T y_Center=0, _T z_Center=0);//生成一个Cube

//取代透视变换，一步计算
void Perspective(float Pos_Source[3], float h[3], float Pos_Dest[3]);
void Perspective_Camera(float Pos_Source[3], float h[3], float Pos_Dest[3]);

template<typename _T> void svd_3(_T* A,SVD_Info oInfo, int* pbSuccess=NULL, double eps= 2.2204460492503131e-15);
template<typename _T>void Test_SVD(_T A[], SVD_Info oSVD, int* piResult=NULL, double eps= 2.2204460492503131e-15);
void Free_SVD(SVD_Info* poInfo);

template<typename _T>void SVD_Alloc(int h, int w, SVD_Info* poInfo, _T* A=NULL);

//一组多元多项式函数
void Disp(Polynormial oPoly, int iElem_No=-1);
void Free_Polynormial(Polynormial* poPoly);				//释放多项式所占内存
void Init_Polynormial(Polynormial* poPoly, int iElem_Count, int iMax_Term_Count);	//初始化
void Add_Poly_Term(Polynormial* poPoly, float fCoeff, int first, ...);
void Get_Derivation(Polynormial* poSource, int iElem, Polynormial* poDest=NULL);	//多项式对xi求导
float fGet_Polynormial_Value(Polynormial oPoly, float x[]);	//代入x求多项式值