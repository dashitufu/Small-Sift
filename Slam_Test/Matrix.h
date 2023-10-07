//由于数学库同时应对float/double两种类型，故此改版开始，全变为模板函数
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


typedef struct Complex_d {//双精度复数
	double real;	//实部
	double im;		//虚部 imagary part	
}Complex_d;
typedef struct Complex_f {//双精度复数
	float real;	//实部
	float im;		//虚部 imagary part	
}Complex_f;
typedef struct Sparse_Matrix {
	typedef struct Item {
		int x, y;
		float m_fValue;
		int m_iRow_Next;	//行方向下一个，向右
		int m_iCol_Next;	//列方向下一个，向下
	}Item;
	unsigned int* m_pRow;	//行桶
	unsigned int* m_pCol;	//列桶
	Item* m_pBuffer;		//内容从[1]开始 [0]表示NULL
	int m_iItem_Count;		//整个matrix共右多少个matrix
	int m_iRow_Count;
	int m_iCol_Count;
}Sparse_Matrix;

extern Light_Ptr oStack_Mem;

unsigned long long iGet_Random_No_cv(unsigned long long* piState);

#define Malloc_1(oPtr, iSize,pBuffer) \
{ \
	(pBuffer)= (_T*)(oPtr).m_pBuffer+(oPtr).m_iCur; \
	(oPtr).m_iCur +=ALIGN_SIZE_128((iSize)); \
	if ( (oPtr).m_iCur > (oPtr).m_iMax_Buffer_Size) \
	{ \
		(oPtr).m_iCur -= ALIGN_SIZE_128((iSize));\
		(pBuffer)=NULL; \
		printf("Fail to allocate memory in Malloc, Total:%u Remain:%u Need:%d\n",(oPtr).m_iMax_Buffer_Size,(oPtr).m_iMax_Buffer_Size-(oPtr).m_iCur,(int)(iSize)); \
	} \
}

template<typename _T>
void Disp(_T* M, int iHeight, int iWidth,const char* pcCaption=NULL,int bDigit_8=1)
{
	int i, j;
	if (pcCaption)
		printf("%s\n", pcCaption);
	for (i = 0; i < iHeight; i++)
	{
		for (j = 0; j < iWidth; j++)
			if(bDigit_8)
				printf("%.8f, ", (float)M[i * iWidth + j]);
			else
				printf("%.2f, ", (float)M[i * iWidth + j]);

		printf("\n");
	}
	return;
}

template<typename _T>
void Matrix_Multiply(_T* A, int ma, int na, _T* B, int nb, _T* C)
{//Amn x Bno = Cmo
	int y, x, i;
	_T fValue, * C_Dup;	// = (_T*)malloc(ma * nb * sizeof(_T));
	Light_Ptr oPtr = oStack_Mem;
	Malloc_1(oPtr, ma * nb * sizeof(_T), C_Dup);
	if (!C_Dup)
	{
		printf("Fail to Malloc_1 in Matrix_Multiply\n");
		return;
	}
	for (y = 0; y < ma; y++)
	{
		for (x = 0; x < nb; x++)
		{
			for (fValue = 0, i = 0; i < na; i++)
				fValue += A[y * na + i] * B[i * nb + x];
			C_Dup[y * nb + x] = fValue;
		}
	}
	memcpy(C, C_Dup, ma * nb * sizeof(_T));
	return;
}
template<typename _T>
void _svd_3(_T A[], int m, int n, _T Sigma[], _T Vt[], int* pbSuccess, _T eps)
{//尝试按opencv写一个
//return pbSuccess: 1：表示收敛； 0：表示不收敛
	int i, j, k, iter, max_iter = 30;
	_T sd;
	_T s, c;
	int iMax_Size = Max(m, n);
	memset(Sigma, 0, iMax_Size * sizeof(_T));
	memset(Vt, 0, iMax_Size * iMax_Size * sizeof(_T));
	for (i = 0; i < m; i++)
	{
		for (sd = 0, j = 0; j < n; j++)
		{
			_T t = A[i * n + j];
			sd += t * t;	//求个平方和？
		}
		Sigma[i] = sd;		//Sigma[i]就是一行的平方和
		if (Vt)
		{
			for (j = 0; j < n; j++)
				Vt[i * n + j] = 0;
			Vt[i * n + i] = 1;		//此处给Vt一个单位矩阵
		}
	}
	_T a, p, b;
	if (pbSuccess)
		*pbSuccess = 0;	//默认不成功
	for (iter = 0; iter < max_iter; iter++)
	{
		int changed = 0;
		for (i = 0; i < m - 1; i++)
		{
			for (j = i + 1; j < m; j++)
			{
				_T* Ai = A + i * n, * Aj = A + j * n;	//去第i行和第j行？
				a = Sigma[i], p = 0, b = Sigma[j];
				/*if (iter == 1)
					printf("here");*/
				for (k = 0; k < n; k++)
					p += (_T)Ai[k] * Aj[k];
				float fTemp = eps * sqrt((_T)a * b);
				if (Abs(p) <= eps * sqrt((_T)a * b))
					continue;
				//printf("%f\n", abs(p));
				p *= 2;
				_T beta = a - b,
					gamma = hypot((_T)p, beta);	//勾股定理算斜边，毫无营养

				if (beta < 0)
				{
					_T delta = (gamma - beta) * 0.5f;
					s = (_T)sqrt(delta / gamma);
					c = (_T)(p / (gamma * s * 2));
				}
				else
				{
					c = (_T)sqrt((gamma + beta) / (gamma * 2));
					s = (_T)(p / (gamma * c * 2));
				}
				a = b = 0;
				for (k = 0; k < n; k++)
				{
					_T t0 = c * Ai[k] + s * Aj[k];
					_T t1 = -s * Ai[k] + c * Aj[k];
					Ai[k] = t0; Aj[k] = t1;
					a += (_T)t0 * t0; b += (_T)t1 * t1;
				}
				Sigma[i] = a; Sigma[j] = b;

				changed = 1;
				if (Vt)
				{
					_T* Vi = Vt + i * n, * Vj = Vt + j * n;
					k = 0;
					for (; k < n; k++)
					{
						_T t0 = c * Vi[k] + s * Vj[k];
						_T t1 = -s * Vi[k] + c * Vj[k];
						Vi[k] = t0; Vj[k] = t1;
					}
				}
			}
		}
		if (!changed)
		{
			if (pbSuccess)
				*pbSuccess = 1;
			break;
		}
	}

	for (i = 0; i < m; i++)
	{
		for (k = 0, sd = 0; k < n; k++)
		{
			_T t = A[i * n + k];
			sd += (_T)t * t;
		}
		Sigma[i] = sqrt(sd);
	}

	for (i = 0; i < m - 1; i++)
	{
		j = i;
		for (k = i + 1; k < m; k++)
		{
			if (Sigma[j] < Sigma[k])
				j = k;
		}
		if (i != j)
		{
			//std::swap(Sigma[i], Sigma[j]);
			_T fTemp = Sigma[i];
			Sigma[i] = Sigma[j];
			Sigma[j] = fTemp;

			if (Vt)
			{
				for (k = 0; k < n; k++)
					std::swap(A[i * n + k], A[j * n + k]);

				for (k = 0; k < m; k++)
					std::swap(Vt[i * n + k], Vt[j * n + k]);
			}
		}
	}
	//for (i = 0; i < n; i++)
		//printf("%f ", Sigma[i]);
	unsigned long long iRandom_State = 0x12345678;
	for (i = 0; i < n; i++)
	{
		sd = i < m ? Sigma[i] : 0;

		for (int ii = 0; ii < 100 && sd <= DBL_MIN; ii++)
		{
			_T val0 = (_T)(1. / n);
			for (k = 0; k < n; k++)
			{
				iGet_Random_No_cv(&iRandom_State);
				_T val = (iRandom_State & 256) != 0 ? val0 : -val0;
				A[i * n + k] = val;
			}
			for (iter = 0; iter < 2; iter++)
			{
				for (j = 0; j < i; j++)
				{
					sd = 0;
					for (k = 0; k < n; k++)
						sd += A[i * n + k] * A[j * n + k];
					_T asum = 0;
					for (k = 0; k < n; k++)
					{
						_T t = (_T)(A[i * n + k] - sd * A[j * n + k]);
						A[i * n + k] = t;
						asum += Abs(t);
					}
					asum = asum > eps * 100 ? 1 / asum : 0;
					for (k = 0; k < n; k++)
						A[i * n + k] *= asum;
				}
			}
			sd = 0;
			for (k = 0; k < n; k++)
			{
				_T t = A[i * n + k];
				sd += (_T)t * t;
			}
			sd = sqrt(sd);
		}
		s = (_T)(sd > DBL_MIN ? 1 / sd : 0.);
		for (k = 0; k < n; k++)
			A[i * n + k] *= s;
	}
	return;
}

template<typename _T>
void svd_3(_T* A, int m, int n, _T U[], _T S[], _T Vt[], int* pbSuccess = NULL, _T eps = 2.2204460492503131e-15)
{//此处为svd的入口。有需要时做个转置
	_T* A_1, * Vt_1, * Sigma;
	int x, y, iFlag;
	int iMax_Size = Max(m, n), iMin_Size = Min(m, n);

	A_1 = (_T*)malloc(iMax_Size * iMax_Size * sizeof(_T));
	Vt_1 = (_T*)malloc(iMax_Size * iMax_Size * sizeof(_T));
	Sigma = (_T*)malloc(iMax_Size * sizeof(_T));

	memset(A_1, 0, iMax_Size * iMax_Size * sizeof(_T));
	if (m < n)
	{
		memcpy(A_1, A, m * n * sizeof(_T));
		//swap(m, n);	//看来后面的svd只干高比宽大的情况
		iFlag = 1;
	}
	else
	{
		//Transpose
		//Matrix_Transpose(A, m, n, A_1);
		for (y = 0; y < m; y++)
			for (x = 0; x < n; x++)
				A_1[x * m + y] = A[y * n + x];
		int iTemp = m;
		m = n;
		n = iTemp;
		iFlag = 0;
	}
	//Disp(A_1, m, m);
	int iResult;
	_svd_3(A_1, m, n, Sigma, Vt_1, &iResult, eps);
	if (iFlag)
	{//m<n
		memcpy(S, Sigma, m * sizeof(_T));
		//出来以后，A_1就是U
		memcpy(Vt, A_1, n * n * sizeof(_T));
		if (U)
		{
			//将Vt_1转置就是U
			for (y = 0; y < m; y++)
				for (x = 0; x < m; x++)
					U[y * m + x] = Vt_1[x * n + y];
		}
	}
	else
	{
		memcpy(S, Sigma, m * sizeof(_T));
		//对A_1转置就是U
		for (y = 0; y < n; y++)
			for (x = 0; x < n; x++)
				U[y * n + x] = A_1[x * n + y];
		for (y = 0; y < m; y++)
			for (x = 0; x < m; x++)
				Vt[y * m + x] = Vt_1[y * n + x];
		//Disp(Vt, m, m);
	}
	if (pbSuccess)
		*pbSuccess = iResult;
	free(A_1);
	free(Vt_1);
	free(Sigma);
	return;
}
template<typename _T>
void Matrix_Transpose(_T* A, int ma, int na, _T* At)
{//矩阵转置
	int y, x;
	_T* At_1;
	Light_Ptr oPtr = oStack_Mem;
	Malloc_1(oPtr, ma * na * sizeof(_T), At_1);
	for (y = 0; y < ma; y++)
		for (x = 0; x < na; x++)
			At_1[x * ma + y] = A[y * na + x];
	memcpy(At, At_1, ma * na * sizeof(_T));
	return;
}
template<typename _T>
void Matrix_Multiply_3x3(_T A[3*3], _T B[3*3], _T C[3*3])
{//计算C=AxB
	Light_Ptr oPtr = oStack_Mem;
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

//排列组合函数
unsigned int iFactorial(int n);

void Schmidt_Orthogon(float* A, int m, int n, float* B);	//施密特正交化

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
void Solve_Linear_Gause(float* A, int iOrder, float* B, float* X, int* pbSuccess);	//高斯法求解线性方程组
void Solve_Linear_Jocabi(float A[], float B[], int iOrder, float X[], int* pbResult);				//雅可比迭代法求解线性方程组
void Solve_Linear_Gauss_Seidel(float A[], float B[], int iOrder, float X[], int* pbResult);
void Solve_Linear_Contradictory(float* A, int m, int n, float* B, float* X, int* pbSuccess);	//解矛盾方程组，求最小二乘解
void Solve_Linear_Solution_Construction(float* A, int m, int n, float B[], int* pbSuccess, float* pBasic_Solution = NULL, int* piBasic_Solution_Count = NULL, float* pSpecial_Solution = NULL);
void Get_Linear_Solution_Construction(float A[], int m, int n, float B[]);//看解的结构
void Elementary_Row_Operation(float A[], int m, int n, float A_1[], int* piRank = NULL, float** ppBasic_Solution = NULL, float** ppSpecial_Solution = NULL);		//初等行变换至最简形
int bIs_Linearly_Dependent(float A[], int m, int n);	//判断向量组A是否线性相关
int bIs_Orthogonal(float* A, int na);					//判断是否为正交矩阵

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

//稀疏矩阵
void Init_Sparse_Matrix(Sparse_Matrix* poMatrix, int iItem_Count, int iMax_Order);
void Compact_Sparse_Matrix(Sparse_Matrix* poMatrix);
void Free_Sparse_Matrix(Sparse_Matrix* poMatrix);
void Matrix_Add(Sparse_Matrix* poA, float a, Sparse_Matrix* poB, float b, float* pC);
void Matrix_Add(Sparse_Matrix* poA, float a, Sparse_Matrix* poB, float b);
void Vector_Add(float A[], float B[], int n, float C[]);
void Vector_Minus(float A[], float B[], int n, float C[]);
void Matrix_Multiply(Sparse_Matrix A, Sparse_Matrix B, Sparse_Matrix* poC);
void Matrix_Transpose_1(Sparse_Matrix A, Sparse_Matrix* poAt);

void Matrix_Add(float A[], float B[], int iOrder, float C[]);

void Matrix_x_Vector(double A[3][3], double X[3], double Y[3]);
void QR_Decompose(double A1[3][3], double Q[3][3], double R[3][3]);
void QR_Decompose(float* A, int ma, int na, float* Q, float* R);
void QR_Decompose(float* A, int na, float* R, float* Q, int* pbSuccess = NULL, int* pbDup_Root = NULL);
//对本质矩阵E进行分解，分解为不同可能的解
void Decompose_E(float E[], float R_1[], float R_2[], float t_1[], float t_2[]);

void Normalize(float V[], int n, float V_1[]);

void Get_Inv_Matrix_Row_Op(float* pM, float* pInv, int iOrder, int* pbSuccess);	//矩阵求逆
void Get_Inv_Matrix(float* pM, float* pInv, int iOrder, int* pbSuccess);	//用伴随矩阵求逆
float fGet_Determinant(float* A, int iOrder);			//求行列式
float fGet_Mod(float V[], int n);						//求向量的模
float fGet_Distance(float V_1[], float V_2[], int n);	//求两向量距离
float fGet_Theta(float v0[], float v1[], float Axis[], int n);		//求两个向量之间的夹角
int iRank(double A[3][3]);								//求3x3矩阵的秩
int iGet_Rank(float* A, int m, int n);					//求一般矩阵的秩，用初等行变换法
void Cross_Product(float V0[], float V1[], float V2[]);	//求外积
float fDot(float V0[], float V1[], int iDim);			//求内积，点积

//以下为一组仿射变换矩阵的生成
void Gen_Rotation_Matrix(float Axis[3], float fTheta, float R[]);//根据旋转轴与旋转角度生成一个旋转矩阵，此处用列向量，往后一路左乘
void Gen_Translation_Matrix(float Offset[3], float T[]);	//平移变换
void Gen_Scale_Matrix(float Scale[3], float T[]);			//比例变换

//四元数,旋转矩阵，旋转向量互换函数
void Quaternion_2_Rotation_Matrix(float Q[4], float R[]);	//四元数转旋转矩阵
void Quaternion_2_Rotation_Vector(float Q[4], float V[4]);	//四元数转旋转向量
void Quaternion_Add(float Q_1[], float Q_2[], float Q_3[]);	//四元数加
void Quaternion_Minus(float Q_1[], float Q_2[], float Q_3[]);	//四元数减，其实这两个函数可以用普通向量加减
void Quaternion_Conj(float Q_1[], float Q_2[]);				//简单求个共轭
void Quaternion_Multiply(float Q_1[], float Q_2[], float Q_3[]);	//四元数乘法
void Quaternion_Inv(float Q_1[], float Q_2[]);	//四元数求逆
void Rotation_Matrix_2_Quaternion(float R[], float Q[]);		//旋转矩阵转换四元数
void Rotation_Matrix_2_Vector(float R[], float V[]);		//旋转矩阵转旋转向量
void Rotation_Vector_2_Matrix_3D(float V[], float R[]);		//旋转向量转换旋转矩阵
void Rotation_Vector_2_Quaternion(float V[4], float Q[4]);	//旋转向量转换四元数
void Roataion_Vector_2_Angle_Axis(float V_4[4], float V_3[3]);	//旋转向量4维化为3维
void Hat(float V[], float M[]);				//构造反对称矩阵
void Vee(float M[], float V[3]);			//反对称矩阵到旋转向量
void se3_2_SE3(float Ksi[6], float T[]);		//从se3向量到SE3矩阵的转换
void SE3_2_se3(float Rotation_Vector[4], float t[3], float Ksi[6]);	//一个旋转向量加一个位移向量构造出一个Ksi

void SIM3_2_sim3(float Rotation_Vector[], float t[], float s, float zeta[7]);
void sim3_2_SIM3(float zeta[7], float Rotation_Vector[4], float t[], float* ps);

void Gen_Homo_Matrix(float R[], float t[], float M[]);			//用旋转坐标与位移坐标构成一个SE3变换矩阵
void Gen_Homo_Matrix(float R[], float t[], float s, float M[]);	//用旋转,位移，缩放构成一个变换矩阵
void Gen_Cube(float Cube[8][4], float fScale, float x_Center = 0, float y_Center = 0, float z_Center = 0);	//生成一个Cube

//取代透视变换，一步计算
void Perspective(float Pos_Source[3], float h[3], float Pos_Dest[3]);
void Perspective_Camera(float Pos_Source[3], float h[3], float Pos_Dest[3]);
void Resize_Matrix(Sparse_Matrix* poA, int iNew_Item_Count);