//������ѧ��ͬʱӦ��float/double�������ͣ��ʴ˸İ濪ʼ��ȫ��Ϊģ�庯��
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
#define MAX_FLOAT ((float)0xFFFFFFFFFFFFFFFF)	//������һ���㹻������֣�������iEEE�ĸ��������ֵ
#define ZERO_APPROCIATE	0.00001f


typedef struct Complex_d {//˫���ȸ���
	double real;	//ʵ��
	double im;		//�鲿 imagary part	
}Complex_d;
typedef struct Complex_f {//˫���ȸ���
	float real;	//ʵ��
	float im;		//�鲿 imagary part	
}Complex_f;
typedef struct Sparse_Matrix {
	typedef struct Item {
		int x, y;
		float m_fValue;
		int m_iRow_Next;	//�з�����һ��������
		int m_iCol_Next;	//�з�����һ��������
	}Item;
	unsigned int* m_pRow;	//��Ͱ
	unsigned int* m_pCol;	//��Ͱ
	Item* m_pBuffer;		//���ݴ�[1]��ʼ [0]��ʾNULL
	int m_iItem_Count;		//����matrix���Ҷ��ٸ�matrix
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
{//���԰�opencvдһ��
//return pbSuccess: 1����ʾ������ 0����ʾ������
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
			sd += t * t;	//���ƽ���ͣ�
		}
		Sigma[i] = sd;		//Sigma[i]����һ�е�ƽ����
		if (Vt)
		{
			for (j = 0; j < n; j++)
				Vt[i * n + j] = 0;
			Vt[i * n + i] = 1;		//�˴���Vtһ����λ����
		}
	}
	_T a, p, b;
	if (pbSuccess)
		*pbSuccess = 0;	//Ĭ�ϲ��ɹ�
	for (iter = 0; iter < max_iter; iter++)
	{
		int changed = 0;
		for (i = 0; i < m - 1; i++)
		{
			for (j = i + 1; j < m; j++)
			{
				_T* Ai = A + i * n, * Aj = A + j * n;	//ȥ��i�к͵�j�У�
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
					gamma = hypot((_T)p, beta);	//���ɶ�����б�ߣ�����Ӫ��

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
{//�˴�Ϊsvd����ڡ�����Ҫʱ����ת��
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
		//swap(m, n);	//���������svdֻ�ɸ߱ȿ������
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
		//�����Ժ�A_1����U
		memcpy(Vt, A_1, n * n * sizeof(_T));
		if (U)
		{
			//��Vt_1ת�þ���U
			for (y = 0; y < m; y++)
				for (x = 0; x < m; x++)
					U[y * m + x] = Vt_1[x * n + y];
		}
	}
	else
	{
		memcpy(S, Sigma, m * sizeof(_T));
		//��A_1ת�þ���U
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
{//����ת��
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
{//����C=AxB
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

//������Ϻ���
unsigned int iFactorial(int n);

void Schmidt_Orthogon(float* A, int m, int n, float* B);	//ʩ����������

void Polor_2_Rect_Coordinate(float rho, float theta, float phi, float* px, float* py, float* pz);	//������תֱ������ϵ
void Polor_2_Rect_Coordinate(float rho, float theta, float* px, float* py);							//��ά
void Rect_2_Polor_Coordinate(float x, float y, float z, float* prho, float* ptheta, float* pphi);	//ֱ������ϵת������
void Rect_2_Screen_Coordinate(float x, float y, int* px_Screen, int* py_Screen, int iWidth = 1920, int iHeight = 1080);	//ֱ������ϵת��Ļ����
void Screen_2_Coordinate(int x_Screen, int y_Screen, float* px, float* py, int iWidth = 1920, int iHeight = 1080);			//��Ļ����תֱ������ϵ

//һ�������Է����йصĺ���
void Cholosky_Decompose(float A[], int iOrder, float B[]);
void Conjugate_Gradient(float* A, const int n, float B[], float X[]);
void Get_Inv_Matrix(float* pM, float* pInv, int iOrder, int* pbSuccess);
void Solve_Linear_Cramer(float* A, int iOrder, float* B, float* X, int* pbSuccess); //����ķ��������Է�����
void Solve_Linear_Gause(float* A, int iOrder, float* B, float* X, int* pbSuccess);	//��˹��������Է�����
void Solve_Linear_Jocabi(float A[], float B[], int iOrder, float X[], int* pbResult);				//�ſɱȵ�����������Է�����
void Solve_Linear_Gauss_Seidel(float A[], float B[], int iOrder, float X[], int* pbResult);
void Solve_Linear_Contradictory(float* A, int m, int n, float* B, float* X, int* pbSuccess);	//��ì�ܷ����飬����С���˽�
void Solve_Linear_Solution_Construction(float* A, int m, int n, float B[], int* pbSuccess, float* pBasic_Solution = NULL, int* piBasic_Solution_Count = NULL, float* pSpecial_Solution = NULL);
void Get_Linear_Solution_Construction(float A[], int m, int n, float B[]);//����Ľṹ
void Elementary_Row_Operation(float A[], int m, int n, float A_1[], int* piRank = NULL, float** ppBasic_Solution = NULL, float** ppSpecial_Solution = NULL);		//�����б任�������
int bIs_Linearly_Dependent(float A[], int m, int n);	//�ж�������A�Ƿ��������
int bIs_Orthogonal(float* A, int na);					//�ж��Ƿ�Ϊ��������

void Gen_Iterate_Matrix(float A[], int n, float B[], float B_1[], float C[]);	//���ɵ����е�B
int bIs_Contrative_Mapping(float B[], int n);					//�ж�һ�������Ƿ�Ϊѹ����������һ�����Է������Ƿ����õ��������ı�Ҫ����
int bIs_Diagonal_Dominant(float A[], int iOrder);			//�ж�һ�������Ƿ�Ϊ�Խ���ռ�Ŷ�������ռ�ž���
int bIs_Unit_Vector(float* V, int na);	//���������Ƿ�Ϊ��λ����
void Diagonalize(float A[], int iOrder, float Diag[], float P[], int* pbSuccess);	//�Գƾ���Խǻ�

void Solve_Cubic(double A[3][3], Complex_d Root[3], int* piRoot_Count);
void Solve_Eigen(double A[3][3], Complex_d Root[3], int* piRoot_Count);
void Solve_Eigen_3x3(float A[], Complex_f Root[3]);
void Solve_Eigen_Vector(float A[], int iOrder, float fEigen_Value, float Q[], int* piCount);	//���ݸ���������ֵ����A�� ����������̵õ���������

void Solve_Poly(float Coeff[], int iCoeff_Count, Complex_f Root[], int* piRoot_Count = NULL);

//ϡ�����
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
//�Ա��ʾ���E���зֽ⣬�ֽ�Ϊ��ͬ���ܵĽ�
void Decompose_E(float E[], float R_1[], float R_2[], float t_1[], float t_2[]);

void Normalize(float V[], int n, float V_1[]);

void Get_Inv_Matrix_Row_Op(float* pM, float* pInv, int iOrder, int* pbSuccess);	//��������
void Get_Inv_Matrix(float* pM, float* pInv, int iOrder, int* pbSuccess);	//�ð����������
float fGet_Determinant(float* A, int iOrder);			//������ʽ
float fGet_Mod(float V[], int n);						//��������ģ
float fGet_Distance(float V_1[], float V_2[], int n);	//������������
float fGet_Theta(float v0[], float v1[], float Axis[], int n);		//����������֮��ļн�
int iRank(double A[3][3]);								//��3x3�������
int iGet_Rank(float* A, int m, int n);					//��һ�������ȣ��ó����б任��
void Cross_Product(float V0[], float V1[], float V2[]);	//�����
float fDot(float V0[], float V1[], int iDim);			//���ڻ������

//����Ϊһ�����任���������
void Gen_Rotation_Matrix(float Axis[3], float fTheta, float R[]);//������ת������ת�Ƕ�����һ����ת���󣬴˴���������������һ·���
void Gen_Translation_Matrix(float Offset[3], float T[]);	//ƽ�Ʊ任
void Gen_Scale_Matrix(float Scale[3], float T[]);			//�����任

//��Ԫ��,��ת������ת������������
void Quaternion_2_Rotation_Matrix(float Q[4], float R[]);	//��Ԫ��ת��ת����
void Quaternion_2_Rotation_Vector(float Q[4], float V[4]);	//��Ԫ��ת��ת����
void Quaternion_Add(float Q_1[], float Q_2[], float Q_3[]);	//��Ԫ����
void Quaternion_Minus(float Q_1[], float Q_2[], float Q_3[]);	//��Ԫ��������ʵ������������������ͨ�����Ӽ�
void Quaternion_Conj(float Q_1[], float Q_2[]);				//���������
void Quaternion_Multiply(float Q_1[], float Q_2[], float Q_3[]);	//��Ԫ���˷�
void Quaternion_Inv(float Q_1[], float Q_2[]);	//��Ԫ������
void Rotation_Matrix_2_Quaternion(float R[], float Q[]);		//��ת����ת����Ԫ��
void Rotation_Matrix_2_Vector(float R[], float V[]);		//��ת����ת��ת����
void Rotation_Vector_2_Matrix_3D(float V[], float R[]);		//��ת����ת����ת����
void Rotation_Vector_2_Quaternion(float V[4], float Q[4]);	//��ת����ת����Ԫ��
void Roataion_Vector_2_Angle_Axis(float V_4[4], float V_3[3]);	//��ת����4ά��Ϊ3ά
void Hat(float V[], float M[]);				//���췴�Գƾ���
void Vee(float M[], float V[3]);			//���Գƾ�����ת����
void se3_2_SE3(float Ksi[6], float T[]);		//��se3������SE3�����ת��
void SE3_2_se3(float Rotation_Vector[4], float t[3], float Ksi[6]);	//һ����ת������һ��λ�����������һ��Ksi

void SIM3_2_sim3(float Rotation_Vector[], float t[], float s, float zeta[7]);
void sim3_2_SIM3(float zeta[7], float Rotation_Vector[4], float t[], float* ps);

void Gen_Homo_Matrix(float R[], float t[], float M[]);			//����ת������λ�����깹��һ��SE3�任����
void Gen_Homo_Matrix(float R[], float t[], float s, float M[]);	//����ת,λ�ƣ����Ź���һ���任����
void Gen_Cube(float Cube[8][4], float fScale, float x_Center = 0, float y_Center = 0, float z_Center = 0);	//����һ��Cube

//ȡ��͸�ӱ任��һ������
void Perspective(float Pos_Source[3], float h[3], float Pos_Dest[3]);
void Perspective_Camera(float Pos_Source[3], float h[3], float Pos_Dest[3]);
void Resize_Matrix(Sparse_Matrix* poA, int iNew_Item_Count);