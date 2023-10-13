//���ļ����house holder�Ⱦ���
#include "stdio.h"
#include <iostream>
#include "assert.h"
#include "memory.h"
#include "math.h"
#include "Common.h"
#include "Matrix.h"

#define  Init_LU(L, U) \
{/*��ʼ��L(�����ǣ� U�������ǣ�*/ \
	L[0][0] = L[1][1] = L[2][2] = 1; \
	L[0][1] = L[0][2] = L[1][2] = 0; \
	U[1][0] = U[2][0] = U[2][1] = 0; \
}
int iCur = 0;

//Light_Ptr oMatrix_Mem = { 0 };	//����ڴ�ר����Matrix�������ݣ��뿪�������ͷ�
Mem_Mgr oMatrix_Mem = {};
//��ʱֻ��Ӧ�Ե��߳�

int iRandom()
{//���Ը�һ���޲������������ĺ���������취Ϊ iRandom_No= iRandom_No*a + c; ���ȡģ 0x7FFFFFFF
#define m 1999999973			//����ģΪ��������ɢ�и����ȵ�����
	static unsigned int a = (unsigned int)iGet_Tick_Count(); //1103515245;		//1103515245Ϊ�ܺõĳ�ʼֵ����ѡȡ��ֵ̬�����ȷ������
	static unsigned int c = 2347;	// 12345;
	static unsigned long long iRandom_No = 1;
	iRandom_No = (iRandom_No * a + c) % m;	//��Ͳ����������ģ
	return (int)iRandom_No;
#undef m
}
unsigned int iGet_Random_No()
{//α���	(789, 123)	(389,621) ��ͨ���������Զ����Լ�������
#define b 389
#define c 621
	static unsigned int iNum = 0xFFFFFFF;	//GetTickCount();	//����
	return iNum = iNum * b + c;
#undef b
#undef c
}
unsigned long long iGet_Random_No_cv(unsigned long long* piState)
{
	*piState = (unsigned long long)(unsigned) * piState * 4164903690U + (unsigned)(*piState >> 32);
	return *piState;
}
unsigned long long iGet_Random_No_cv()
{//α���������Opencv����
	static unsigned long long iNum = 0xFFFFFFFFFFFFFFFF;
	iNum = (unsigned long long)(unsigned)iNum * 4164903690U + (unsigned)(iNum >> 32);
	return iNum;
}
int iGet_Random_No_cv(int a, int b)
{//ȡ���a,b֮��
	unsigned iNum = (unsigned)iGet_Random_No_cv();
	return a == b ? a : (int)(iNum % (b - a) + a);
}
float fGet_Mod(float V[], int n)
{
	float fMod;
	int i;
	for (fMod = 0, i = 0; i < n; i++)
		fMod += V[i] * V[i];
	return sqrt(fMod);
}
float fGet_Distance(float V_1[], float V_2[], int n)
{//���������ľ��룬��ƽ����ʾ��������
	int i;
	float fSum;
	for (i = 0, fSum = 0; i < n; i++)
		fSum += (V_1[i] - V_2[i]) * (V_1[i] - V_2[i]);
	return fSum;
}
void Normalize_Col(float M[], int m, int n, float M_1[])
{//��M���н��й��
	int y, x;
	float fTotal, fMod;
	for (x = 0; x < n; x++)
	{
		//��һ�е�ģ
		fTotal = 0;
		for (y = 0; y < m; y++)
			fTotal += M[y * n + x] * M[y * n + x];
		fMod = sqrt(fTotal);
		if (fMod != 0)
			for (y = 0; y < m; y++)
				M_1[y * n + x] = M[y * n + x] / fMod;
		else
			M_1[y * n + x] = 0;

	}
	return;
}
void Normalize(float V[], int n, float V_1[])
{//���������й��
//����ÿ���������ܹ�񻯣���������0����ʱ���涨����0����
	float fMod;
	int i;
#define eps 1e-10
	fMod = fGet_Mod(V, n);
	if (fMod > eps)
	{
		for (i = 0; i < n; i++)
			V_1[i] = V[i] / fMod;
	}
	else
		memset(V_1, 0, n * sizeof(float));
#undef eps
}
void Disp(float* M, int iHeight, int iWidth, const char* pcCaption)
{
	int i, j;
	if (pcCaption)
		printf("%s\n", pcCaption);
	for (i = 0; i < iHeight; i++)
	{
		for (j = 0; j < iWidth; j++)
			printf("%.8f, ", (float)M[i * iWidth + j]);
		printf("\n");
	}
	return;
}
void Disp(double* M, int iHeight, int iWidth, const char* pcCaption)
{
	int i, j;
	if (pcCaption)
		printf("%s\n", pcCaption);
	for (i = 0; i < iHeight; i++)
	{
		for (j = 0; j < iWidth; j++)
			printf("%.8f, ", (double)M[i * iWidth + j]);
		printf("\n");
	}
	return;
}
void Disp(double V[3], const char* pcCaption)
{
	printf("%s\n", pcCaption);
	printf("%f %f %f\n", V[0], V[1], V[2]);
}
void Disp(float V[3], const char* pcCaption)
{
	printf("%s\n", pcCaption);
	printf("%f %f %f\n", V[0], V[1], V[2]);
}
void Disp(float A[3][3], const char* pcCaption = "")
{//��ʾһ������
	int y, x;
	printf("%s\n", pcCaption);
	for (y = 0; y < 3; y++)
	{
		for (x = 0; x < 3; x++)
			printf("%f\t", A[y][x]);
		printf("\n");
	}
	return;
}
void Disp(double A[3][3], const char* pcCaption)
{//��ʾһ������
	int y, x;
	printf("%s\n", pcCaption);
	for (y = 0; y < 3; y++)
	{
		for (x = 0; x < 3; x++)
			printf("%f\t", A[y][x]);
		printf("\n");
	}
	return;
}

void Matrix_x_Vector(double A[3][3], double X[3], double Y[3])
{//����Y=AX
	Y[0] = A[0][0] * X[0] + A[0][1] * X[1] + A[0][2] * X[2];
	Y[1] = A[1][0] * X[0] + A[1][1] * X[1] + A[1][2] * X[2];
	Y[2] = A[2][0] * X[0] + A[2][1] * X[1] + A[2][2] * X[2];
}
double fGet_Abs_Sum_3(double V[3])
{
	return Abs(V[0]) + Abs(V[1]) + Abs(V[2]);
}
float fGet_Next_Float(char** ppCur)
{
	char Value[256], * pCur = *ppCur;
	int i;
	for (i = 0; i < 256; i++, pCur++)
	{
		if (*pCur != '\t' && *pCur != '\0')
			Value[i] = *pCur;
		else
			break;
	}
	if (*pCur == '\0')
		*ppCur = pCur;
	else
		*ppCur = pCur + 1;
	return (float)atof(Value);
}
int bSave_Matrix(const char* pcFile, float* pMatrix, int iWidth, int iHeight)
{//����̣���\r\n����
	int i, j;
	FILE* pFile = fopen(pcFile, "wb");
	if (!pFile)
	{
		printf("Fail to open:%s\n", pcFile);
		return 0;
	}
	for (i = 0; i < iHeight; i++)
	{
		for (j = 0; j < iWidth; j++)
		{
			fprintf(pFile, "%f ", pMatrix[i * iWidth + j]);
		}
		fprintf(pFile, "\r\n");
	}
	fclose(pFile);
	return 1;
}


void Cholosky_Decompose(float A[], int iOrder, float B[])
{//������������Cholosky�ֽ⣬������ڰ�B A=BxB'
	int i, j, k, iPos_kk, iPos_ik, iPos_ij, iPos_jk;
	//Disp(A, iOrder, iOrder, "A");

	memcpy(B, A, iOrder * iOrder * sizeof(float));
	for (k = 0; k < iOrder; k++)
	{
		iPos_kk = k * iOrder + k;
		B[iPos_kk] = sqrt(B[iPos_kk]);	//�Խ��ߵ����俪�����Զ��׼�
		for (i = k + 1; i < iOrder; i++)
		{
			iPos_ik = i * iOrder + k;	//�ƺ��ڸ������ǵ�k��
			if (B[iPos_ik] != 0.f)
				B[iPos_ik] = B[iPos_ik] / B[iPos_kk];
		}
		//if (k == 2)
			//printf("here");
		for (j = k + 1; j < iOrder; j++)
		{
			iPos_jk = j * iOrder + k;
			for (i = j; i < iOrder; i++)
			{
				iPos_ij = i * iOrder + j;
				if (B[iPos_ij] != 0.f)
					B[iPos_ij] -= B[i * iOrder + k] * B[iPos_jk];
			}
		}
	}

	//��Ȼ������������0
	for (i = 0; i < iOrder; i++)
		for (j = i + 1; j < iOrder; j++)
			B[i * iOrder + j] = 0;
	Disp(B, iOrder, iOrder, "B");
	return;
}
void QR_Decompose(double A1[3][3], double Q[3][3], double R[3][3])
{//����һ��3x3����A������������Householder�����Q,R��R��������������Ǿ���
	//ע�⣺�п������������ݣ�����������������������ʵ���������²�����������
	double Q1[3][3], A2[3][3], Q2[3][3];
	double /*fMod,*/ fDelta, tau, v[3];
	//int x, y,i;

	//��һ������Q1����ʱ�������һ�����㣬��ȫ����������,��ʱ������I
	//�˴������Խ�һ��������ʱ����
	fDelta = sign(A1[0][0]) * sqrt(A1[0][0] * A1[0][0] + A1[1][0] * A1[1][0] + A1[2][0] * A1[2][0]);
	//v=(a11+delta,a21,a31...)
	v[0] = A1[0][0] + fDelta, v[1] = A1[1][0], v[2] = A1[2][0];
	tau = fDelta * (A1[0][0] + fDelta);
	tau = 1 / tau;	//�ܿ�������tau��Ϊ����
	//Q1=H1=I-vv'/tau
	Q1[0][0] = 1 - v[0] * v[0] * tau;
	Q1[1][0] = Q1[0][1] = -v[0] * v[1] * tau;
	Q1[2][0] = Q1[0][2] = -v[0] * v[2] * tau;
	Q1[1][1] = 1 - v[1] * v[1] * tau;
	Q1[2][1] = Q1[1][2] = -v[1] * v[2] * tau;
	Q1[2][2] = 1 - v[2] * v[2] * tau;

	//Disp(Q1,(char*)"Q1");
	//��A2=Q1*A
	A2[0][0] = Q1[0][0] * A1[0][0] + Q1[0][1] * A1[1][0] + Q1[0][2] * A1[2][0];
	A2[0][1] = Q1[0][0] * A1[0][1] + Q1[0][1] * A1[1][1] + Q1[0][2] * A1[2][1];
	A2[0][2] = Q1[0][0] * A1[0][2] + Q1[0][1] * A1[1][2] + Q1[0][2] * A1[2][2];
	A2[1][0] = A2[2][0] = 0;
	A2[1][1] = Q1[1][0] * A1[0][1] + Q1[1][1] * A1[1][1] + Q1[1][2] * A1[2][1];
	A2[1][2] = Q1[1][0] * A1[0][2] + Q1[1][1] * A1[1][2] + Q1[1][2] * A1[2][2];
	A2[2][1] = Q1[2][0] * A1[0][1] + Q1[2][1] * A1[1][1] + Q1[2][2] * A1[2][1];
	A2[2][2] = Q1[2][0] * A1[0][2] + Q1[2][1] * A1[1][2] + Q1[2][2] * A1[2][2];
	//Disp(A2, (char*)"A2");

	//�ڶ������˴��Ѿ�֤����Q[2][2]=-Q[1][1]��Ч��ţ�ƣ�
	//fDelta = sign(A2[1][1]) * sqrt(A2[1][1] * A2[1][1] + A2[2][1] * A2[2][1]);
	//v[0] = A2[1][1] + fDelta, v[1] = A2[2][1];
	//tau = 1/(fDelta * (A2[1][1] + fDelta));
	////Q=	I1	0
	////		0	H2
	//Q2[0][0] = 1, Q2[0][1] = Q2[0][2] = Q2[1][0] = Q2[2][0] = 0;
	//Q2[1][1] = 1 - tau * v[0] * v[0];
	//Q2[2][1]=Q2[1][2] = -tau * v[0] * v[1];
	//Q2[2][2] = 1 - tau * v[1] * v[1];
	//Disp(Q2,(char*)"Q2");

	//����Ϊ�ڶ����·���
	fDelta = 1 / (sign(A2[1][1]) * sqrt(A2[1][1] * A2[1][1] + A2[2][1] * A2[2][1]));
	//Q=	I1	0
	//		0	H2
	Q2[0][0] = 1, Q2[0][1] = Q2[0][2] = Q2[1][0] = Q2[2][0] = 0;
	Q2[1][1] = -(Q2[2][2] = A2[1][1] * fDelta);
	Q2[2][1] = Q2[1][2] = -A2[2][1] * fDelta;
	//Disp(Q2, (char*)"Q2");

	//Disp(Q2,(char*)"Q2");
	//A3����R���˴�����Q2,A2����Ԫ���򣬲��ң��������ۣ�R[1][0]=R[2]0]=R[2][1]=0
	R[0][0] =/* Q2[0][0] **/ A2[0][0];	//+Q2[0][1] * A2[1][0] + Q2[0][2] * A2[2][0];
	R[0][1] = /*Q2[0][0] **/ A2[0][1] /*+ Q2[0][1] * A2[1][1] + Q2[0][2] * A2[2][1]*/;
	R[0][2] =/* Q2[0][0] **/ A2[0][2] /*+ Q2[0][1] * A2[1][2] + Q2[0][2] * A2[2][2]*/;
	R[1][0] = R[2][0] = R[2][1] = 0;
	R[1][1] = /*Q2[1][0] * A2[0][1] +*/ Q2[1][1] * A2[1][1] + Q2[1][2] * A2[2][1];
	R[1][2] = /*Q2[1][0] * A2[0][2] +*/ Q2[1][1] * A2[1][2] + Q2[1][2] * A2[2][2];
	R[2][2] = /*Q2[2][0] * A2[0][2] +*/ Q2[2][1] * A2[1][2] + Q2[2][2] * A2[2][2];

	//Disp(R,(char*)"R=A3");
	//Q=(Q2*Q1)'
	//�˴���������Q2�����Լ��ټ������
	Q[0][0] = Q1[0][0];
	Q[1][0] = Q1[0][1];
	Q[2][0] = Q1[0][2];
	Q[0][1] = Q2[1][1] * Q1[1][0] + Q2[1][2] * Q1[2][0];
	Q[1][1] = Q2[1][1] * Q1[1][1] + Q2[1][2] * Q1[2][1];
	Q[2][1] = Q2[1][1] * Q1[1][2] + Q2[1][2] * Q1[2][2];
	Q[0][2] = Q2[2][1] * Q1[1][0] + Q2[2][2] * Q1[2][0];
	Q[1][2] = Q2[2][1] * Q1[1][1] + Q2[2][2] * Q1[2][1];
	Q[2][2] = Q2[2][1] * Q1[1][2] + Q2[2][2] * Q1[2][2];

	//Disp(Q,(char*)"Q");

	////������㿴���Ƿ�A=Q*R
	//for(y=0;y<3;y++)
	//	for(x=0;x<3;x++)
	//		for(A2[y][x]=0,i=0;i<3;i++)
	//			A2[y][x]+= Q[y][i] * R[i][x];
	return;
}
void Vector_Minus(float A[], float B[], int n, float C[])
{
	for (int i = 0; i < n; i++)
		C[i] = A[i] - B[i];
}
void Vector_Add(float A[], float B[], int n, float C[])
{
	for (int i = 0; i < n; i++)
		C[i] = A[i] + B[i];
}
void Matrix_Multiply_Symmetric(float* A, int m, int n, float* AAt)
{//�� AxA', �����Ȼ��һ���Գƾ���A(mxn) x A'(n*m) = AAt(m*m)
	float* AAt_1 = (float*)malloc(m * m * sizeof(float));
	int y, x, i;
	float fValue;
	for (y = 0; y < m; y++)
	{
		for (x = y; x < m; x++)
		{
			//if (y == 0 && x == 1)
				//printf("here");
			for (fValue = 0, i = 0; i < n; i++)
				fValue += A[y * n + i] * A[x * n + i];
			AAt_1[x * m + y] = AAt_1[y * m + x] = fValue;
		}
	}
	//Disp(AAt_1, m, m, "AAt");
	memcpy(AAt, AAt_1, m * m * sizeof(float));
	free(AAt_1);
	return;
}

//void Matrix_Multiply(float* A,int ma,int na,float *B, int nb,float *C)
//{//Amn x Bno = Cmo
//	int y, x,i;
//	float fValue,*C_Dup=(float*)malloc(ma*nb*sizeof(float));
//	for (y = 0; y < ma; y++)
//	{
//		for (x = 0; x < nb; x++)
//		{
//			//if (y == 2 && x == 1)
//				//printf("Here");
//			for (fValue = 0, i = 0; i < na; i++)
//				fValue += A[y * na + i] * B[i * nb + x];
//			C_Dup[y * nb + x] = fValue;
//		}
//	}
//	memcpy(C, C_Dup, ma * nb * sizeof(float));
//	free(C_Dup);
//	return;
//}

void RQ_Multiply_3x3(double R[3][3], double Q[3][3], double A[3][3])
{//����A=RxQ
	//�˴�Ӧ�ð���R[1][0]=R[2][0]=R[2][1]=0���л���
	A[0][0] = R[0][0] * Q[0][0] + R[0][1] * Q[1][0] + R[0][2] * Q[2][0];
	A[0][1] = R[0][0] * Q[0][1] + R[0][1] * Q[1][1] + R[0][2] * Q[2][1];
	A[0][2] = R[0][0] * Q[0][2] + R[0][1] * Q[1][2] + R[0][2] * Q[2][2];
	A[1][0] = /*R[1][0] * Q[0][0] +*/ R[1][1] * Q[1][0] + R[1][2] * Q[2][0];
	A[1][1] = /*R[1][0] * Q[0][1] +*/ R[1][1] * Q[1][1] + R[1][2] * Q[2][1];
	A[1][2] = /*R[1][0] * Q[0][2] +*/ R[1][1] * Q[1][2] + R[1][2] * Q[2][2];
	A[2][0] = /*R[2][0] * Q[0][0] + R[2][1] * Q[1][0] +*/ R[2][2] * Q[2][0];
	A[2][1] = /*R[2][0] * Q[0][1] + R[2][1] * Q[1][1] +*/ R[2][2] * Q[2][1];
	A[2][2] = /*R[2][0] * Q[0][2] + R[2][1] * Q[1][2] +*/ R[2][2] * Q[2][2];
}
#define Scale_Matrix(A, A1,fMax) \
{ \
	double fRecep; \
	fMax=Abs(A[0][0]);	\
	if (Abs(A[0][1]) > fMax)fMax = Abs(A[0][1]); \
	if (Abs(A[0][2]) > fMax)fMax = Abs(A[0][2]); \
	if (Abs(A[1][0]) > fMax)fMax = Abs(A[1][0]); \
	if (Abs(A[1][1]) > fMax)fMax = Abs(A[1][1]); \
	if (Abs(A[1][2]) > fMax)fMax = Abs(A[1][2]); \
	if (Abs(A[2][0]) > fMax)fMax = Abs(A[2][0]); \
	if (Abs(A[2][1]) > fMax)fMax = Abs(A[2][1]); \
	if (Abs(A[2][2]) > fMax)fMax = Abs(A[2][2]); \
	fRecep = 1 / fMax; \
	A1[0][0] = A[0][0] * fRecep; \
	A1[0][1] = A[0][1] * fRecep; \
	A1[0][2] = A[0][2] * fRecep; \
	A1[1][0] = A[1][0] * fRecep; \
	A1[1][1] = A[1][1] * fRecep; \
	A1[1][2] = A[1][2] * fRecep; \
	A1[2][0] = A[2][0] * fRecep; \
	A1[2][1] = A[2][1] * fRecep; \
	A1[2][2] = A[2][2] * fRecep; \
}
#define Scale_Matrix_1(A1,fMax) \
{ \
	A1[0][0] *= fMax; \
	A1[0][1] *= fMax; \
	A1[0][2] *= fMax; \
	A1[1][0] *= fMax; \
	A1[1][1] *= fMax; \
	A1[1][2] *= fMax; \
	A1[2][0] *= fMax; \
	A1[2][1] *= fMax; \
	A1[2][2] *= fMax; \
}
void Solve_Homo_3x3(double A[3][3], double fEigen_Value, double x[3])
{//Ϊ3x3���������η���Ax=0
	double fMod, Sub[2];	//substitution;
	double A1[3][3] = { {A[0][0] - fEigen_Value,A[0][1],A[0][2]},
						{A[1][0],A[1][1] - fEigen_Value,A[1][2]},
						{A[2][0],A[2][1],A[2][2] - fEigen_Value} };
	//Disp(A1, "A1"); 
	//��һ����ȥA[1][0],A[2][0]
	//�ҳ�x1������Ԫ,������ʽ
	if (A1[0][0] != 0)
	{
		Sub[0] = -(A1[0][1] /= A1[0][0]);
		Sub[1] = -(A1[0][2] /= A1[0][0]);
		A1[1][1] += A1[1][0] * Sub[0];
		A1[1][2] += A1[1][0] * Sub[1];
		A1[2][1] += A1[2][0] * Sub[0];
		A1[2][2] += A1[2][0] * Sub[1];
		A1[1][0] = A1[2][0] = 0;
		A1[0][0] = 1;
	}
	else if (A1[1][0] != 0)
	{
		Sub[0] = -(A1[1][1] /= A1[1][0]);
		Sub[1] = -(A1[1][2] /= A1[1][0]);
		A1[2][1] += A1[2][0] * Sub[0];
		A1[2][2] += A1[2][0] * Sub[1];
		//A1[1][0] = 1;
		A1[2][0] = 0;
		//����1�����0�н���λ��
		A1[0][0] = 1;
		A1[1][0] = 0;
		std::swap(A1[0][1], A1[1][1]);
		std::swap(A1[0][2], A1[1][2]);
	}
	else if (A1[2][0] != 0)
	{
		A1[2][1] /= A1[2][0];
		A1[2][2] /= A1[2][0];

		//����2�����0�н���λ��
		A1[0][0] = 1;
		A1[2][0] = 0;
		std::swap(A1[0][1], A1[2][1]);
		std::swap(A1[0][2], A1[2][2]);
	}
	//Disp(A1,"A1");
	//�ڶ�������ȥA[2][1],��ȥA[0][1]
	if (A1[1][1] != 0)
	{//A1[1][1]��Ϊ0������������2��
		A1[1][2] /= A1[1][1];
		A1[1][1] = 1;
		Sub[0] = -A1[1][2];
		A1[2][2] += A1[2][1] * Sub[0];
		A1[2][1] = 0;
	}
	else if (A1[2][1] != 0)
	{
		A1[2][2] /= A1[2][1];

		A1[1][1] = 1;
		A1[2][1] = 0;
		//����2�����1�н���
		std::swap(A1[1][2], A1[2][2]);
	}

	//Disp(A1, "A1");
	//����Ҫ����һ�������ʴ˴˴�����ֱ���ж�Ϊ0
	//if (A1[2][2] !=0)
	//{//���ȣ���·�˳�
	//	x[0] = x[1] = x[2] = 0;
	//	return;
	//}

	if (A1[1][1] == 1)
	{
		x[2] = 1;	//��x[2]��Ϊ����Ԫ
		x[1] = -A1[1][2] * x[2];
	}

	if (A1[0][0] == 1)
	{
		if (A1[1][1] == 0)
		{//����������Ԫ
			x[1] = 1;
			x[2] = 1;
		}
		//x[0]=-a01*x1-a02*x2
		x[0] = -A1[0][1] * x[1] - A1[0][2] * x[2];
	}

	////��ʱ�������� Ax=0
	//double Temp[3];
	double A2[3][3] = { {A[0][0] - fEigen_Value,A[0][1],A[0][2]},
						{A[1][0],A[1][1] - fEigen_Value,A[1][2]},
						{A[2][0],A[2][1],A[2][2] - fEigen_Value} };

	x[2] = 1;//ǿ�����㣬��Ϊ����Ԫ
	//Matrix_x_Vector(A2, x, Temp);
	//fMod = fGet_Abs_Sum_3(Temp);
	//if (fMod >= 0.5)
		//printf("Err:%f\n",fMod);

	//���һ����Normalize
	fMod = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];
	if (fMod < ZERO_APPROCIATE)
		return;
	else
		fMod = 1 / sqrt(fMod);

	x[0] *= fMod;
	x[1] *= fMod;
	x[2] *= fMod;

	//Disp(A2, "A2");
	//����Ϊʲôȫ������-1��������ǰ��ʲô���ˣ��о����ﲻӦ�ã������ǳ�ʼ��һ������
	/*x[0] *= -1;
	x[1] *= -1;
	x[2] *= -1;*/
	//Disp(x, "x");

	//double Temp[3];
	//Matrix_x_Vector(A2, x, Temp);
	//fMod = fGet_Abs_Sum_3(Temp);
	//if (fMod >= 0.5)
		//printf("Err:%f\n", fMod);
	//Disp(x, "x");
	return;
}
int bEigen_Value_3x3(double A[3][3], double Ramda[3], double Eigen_Vector[3][3])
{//�����������������ֵ����A���죬����һ������ֵ=0
//����ɹ�������1�����򣬷���0
#define MAX_ITER_COUNT 120
	double Q[3][3], A1[3][3], fMax;
	static double R[3][3] = { 0 };
	//ʵ��֤�����ֽ�ǰ����һ������/����������ٵ�������

	Scale_Matrix(A, A1, fMax);
	//Disp(A, "A");
	//Disp(A1, "A1");
	int iCount = 0;
	float /*fPre_Sum=MAX_FLOAT,*/fSum;

	//�˴���ʼ����
	while (1)
	{
		QR_Decompose(A1, Q, R);
		//Disp(R, (char*)"R="); Disp(Q, (char*)"Q=");
		//A1 = RQ
		RQ_Multiply_3x3(R, Q, A1);
		//Disp(A1, (char*)"RQ=");
		//�ж��Ƿ�������
		fSum = (float)(Abs(A1[1][0]) + Abs(A1[2][0]) + Abs(A1[2][1]));
		//printf("Sum:%f\n", fSum);
		if (fSum < 0.000001)
			break;
		else if (iCount >= MAX_ITER_COUNT)
		{
			//printf("Sum of Err:%f\n", fSum);
			return 0;
		}
		//if (Abs(A1[1][0]) < ZERO_APPROCIATE && Abs(A1[2][0]) < ZERO_APPROCIATE && Abs(A1[2][1]) < ZERO_APPROCIATE)
			//break;
		iCount++;
	}
	Ramda[0] = A1[0][0] * fMax;
	Ramda[1] = A1[1][1] * fMax;
	Ramda[2] = A1[2][2] * fMax;
	//Disp(A1, "A1");
	//Disp(Ramda, (char*)"Eigen Value=");
	//��ʱ���룬��A1
	//printf("%f %f %f\n", Ramda[0] * fMax, Ramda[1] * fMax, Ramda[2] * fMax);
	//Scale_Matrix(A, A1, fMax);
	//Solve_Homo_3x3(A, Ramda[0]*fMax, Eigen_Vector[0]);

	//Solve_Homo_3x3(A, Ramda[0]*=fMax, Eigen_Vector[0]);
	//Solve_Homo_3x3(A, Ramda[1] *= fMax, Eigen_Vector[1]);
	//Solve_Homo_3x3(A, Ramda[2] *= fMax, Eigen_Vector[2]);
	//Scale_Matrix_1(A1, fMax);	//ʵ���ϣ���һ�����п���ʡ�ԣ���ΪMatlab����������һ��
	//Disp(Eigen_Vector, "Eigen_Vector");
	return 1;
#undef MAX_ITER_COUNT
}
#define LU_Decompose_1(A, L, U, bSucceed) \
{ \
	if (A[0][0] == 0) \
	{ \
		bSucceed = 0; \
		goto END; \
	} \
	U[0][0] = A[0][0]; \
	L[1][0] = A[1][0] / U[0][0]; \
	U[0][1] = A[0][1]; \
	U[1][1] = A[1][1] - L[1][0] * U[0][1]; \
	if (U[1][1] == 0) \
	{ \
		bSucceed = 0; \
		goto END; \
	} \
	L[2][0] = A[2][0] / U[0][0]; \
	L[2][1] = (A[2][1] - L[2][0] * U[0][1]) / U[1][1]; \
	U[0][2] = A[0][2]; \
	U[1][2] = A[1][2] - L[1][0] * U[0][2]; \
	U[2][2] = A[2][2] - L[2][0] * U[0][2] - L[2][1] * U[1][2]; \
	bSucceed = 1; \
END: \
	; \
}

void LU_Decompose(double A[3][3], double L[3][3], double U[3][3], int* pbSucceed)
{//���Խ�һ������A�ֽ�ΪLxU=A���ֽ������λ��ɧ�������Կ����
//���ƣ����Է�����������ȣ���Ч
	if (A[0][0] == 0)
	{
		*pbSucceed = 0;
		return;
	}
	U[0][0] = A[0][0];
	L[1][0] = A[1][0] / U[0][0];
	U[0][1] = A[0][1];
	U[1][1] = A[1][1] - L[1][0] * U[0][1];
	if (U[1][1] == 0)
	{
		*pbSucceed = 0;
		return;
	}
	L[2][0] = A[2][0] / U[0][0];
	L[2][1] = (A[2][1] - L[2][0] * U[0][1]) / U[1][1];
	U[0][2] = A[0][2];
	U[1][2] = A[1][2] - L[1][0] * U[0][2];
	U[2][2] = A[2][2] - L[2][0] * U[0][2] - L[2][1] * U[1][2];
	/*double Temp[3][3];
	Matrix_Multiply_3x3(L, U, Temp);
	Disp(Temp, "Temp");
	Disp(L, (char*)"L");
	Disp(U, (char*)"U");*/
	*pbSucceed = 1;
	return;
}

void Solve_Uk(double L[3][3], double Z[3], double U[3])
{//���LU=Z�ֵ�U
	U[0] = Z[0];
	U[1] = Z[1] - L[1][0] * U[0];
	U[2] = Z[2] - L[2][0] * U[0] - L[2][1] * U[1];
	return;
}
void Solve_yk(double U[3][3], double yk[3], double uk[3])
{//���Uyk=uk�е�yk
	yk[2] = uk[2] / U[2][2];
	yk[1] = (uk[1] - U[1][2] * yk[2]) / U[1][1];
	yk[0] = (uk[0] - U[0][1] * yk[1] - U[0][2] * yk[2]) / U[0][0];
	return;
}
int iRank(double A[3][3])
{//�Ծ�������ȣ�ע�⣬δ���ԣ�ǧ�������
	double A1[3][3], Sub[2];
	//��һ����ȥA[1][0],A[2][0]
	//�ҳ�x1������Ԫ,������ʽ
	memcpy(A1, A, 3 * 3 * sizeof(double));
	if (A1[0][0] != 0)
	{
		Sub[0] = -(A1[0][1] /= A1[0][0]);
		Sub[1] = -(A1[0][2] /= A1[0][0]);
		A1[1][1] += A1[1][0] * Sub[0];
		A1[1][2] += A1[1][0] * Sub[1];
		A1[2][1] += A1[2][0] * Sub[0];
		A1[2][2] += A1[2][0] * Sub[1];
		A1[1][0] = A1[2][0] = 0;
		A1[0][0] = 1;
	}
	else if (A1[1][0] != 0)
	{
		Sub[0] = -(A1[1][1] /= A1[1][0]);
		Sub[1] = -(A1[1][2] /= A1[1][0]);
		A1[2][1] += A1[2][0] * Sub[0];
		A1[2][2] += A1[2][0] * Sub[1];
		//A1[1][0] = 1;
		A1[2][0] = 0;
		//����1�����0�н���λ��
		A1[0][0] = 1;
		A1[1][0] = 0;
		std::swap(A1[0][1], A1[1][1]);
		std::swap(A1[0][2], A1[1][2]);
	}
	else if (A1[2][0] != 0)
	{
		A1[2][1] /= A1[2][0];
		A1[2][2] /= A1[2][0];

		//����2�����0�н���λ��
		A1[0][0] = 1;
		A1[2][0] = 0;
		std::swap(A1[0][1], A1[2][1]);
		std::swap(A1[0][2], A1[2][2]);
	}
	//Disp(A1,"A1");
	//�ڶ�������ȥA[2][1],��ȥA[0][1]
	if (A1[1][1] != 0)
	{//A1[1][1]��Ϊ0������������2��
		A1[1][2] /= A1[1][1];
		A1[1][1] = 1;
		Sub[0] = -A1[1][2];
		A1[2][2] += A1[2][1] * Sub[0];
		A1[2][1] = 0;
	}
	else if (A1[2][1] != 0)
	{
		A1[2][2] /= A1[2][1];

		A1[1][1] = 1;
		A1[2][1] = 0;
		//����2�����1�н���
		std::swap(A1[1][2], A1[2][2]);
	}
	//Disp(A1, "A1");
	int i, iCount = 0;
	for (i = 0; i < 3; i++)
		if (A1[i][i] != 0)
			iCount++;
	return iCount;
}
int bInverse_Power(double A[3][3], double* pfEigen_Value, double Eigen_Vector[3])
{//�����С������ֵ����Ӧ������������С��Χ�����Ѿ��ɹ�����
//�ɹ�������1��ʧ��:����0
#define MAX_ITER_COUNT 100
	double L[3][3], U[3][3], yk[3], zk[3] = { 1,1,1 }, z_pre[3] = { MAX_FLOAT,MAX_FLOAT,MAX_FLOAT }, uk[3];
	double fDiff, fDiff_Total, tau, fSum;
	Init_LU(L, U);
	int bSucceed, i, iCount = 0;
	//double tm1[3][3],tv1[3];
	//iRank(A);

	//����㷨��ȱ�ݣ��������жϾ����Ƿ���棨���ȣ�
	LU_Decompose(A, L, U, &bSucceed);
	//�����Ժ�A=L*U

	//Disp(A,"A");
	if (!bSucceed || U[0][0] == 0 || U[1][1] == 0 || U[2][2] == 0)
	{//���г��ֳ���0
		*pfEigen_Value = Eigen_Vector[0] = Eigen_Vector[1] = Eigen_Vector[2] = 0;
		return 0;
	}

	while (1)
	{
		//�����Luk=Zk
		Solve_Uk(L, zk, uk);

		//���� Uyk=uk
		Solve_yk(U, yk, uk);

		//Get_Max(yk, tau);
		Get_Max_1(yk, tau, i);
		tau = zk[i] >= 0 ? 1 / tau : -1.f / tau;	//�˷��ȳ�����
		zk[0] = yk[0] * tau;
		zk[1] = yk[1] * tau;
		zk[2] = yk[2] * tau;

		//Disp(zk, "zk");
		fDiff_Total = 0;
		fDiff = zk[0] - z_pre[0];
		fDiff_Total = Abs(fDiff);
		fDiff = zk[1] - z_pre[1];
		fDiff_Total += Abs(fDiff);
		fDiff = zk[2] - z_pre[2];
		fDiff_Total += Abs(fDiff);
		//printf("Diff:%f\n", fDiff_Total);
		if (fDiff_Total < ZERO_APPROCIATE)
			//if(abs(zk[0] - z_pre[0])+abs(zk[1] - z_pre[1]) + abs(zk[2] - z_pre[2])< ZERO_APPROCIATE)
			break;
		else if ((iCount++) >= MAX_ITER_COUNT)
		{//��������֮�����������ף�������������
			//printf("Diff:%f\n", fDiff_Total);
			//*pfEigen_Value = Eigen_Vector[0] = Eigen_Vector[1] = Eigen_Vector[2] = 0;
			//return 0;
			break;
		}

		z_pre[0] = zk[0];
		z_pre[1] = zk[1];
		z_pre[2] = zk[2];
	}

	//���
	fSum = zk[0] * zk[0] + zk[1] * zk[1] + zk[2] * zk[2];
	if (fSum < ZERO_APPROCIATE)
		return 0;
	fSum = 1 / sqrt(fSum);
	Eigen_Vector[0] = zk[0] *= fSum;
	Eigen_Vector[1] = zk[1] *= fSum;
	Eigen_Vector[2] = zk[2] *= fSum;
	*pfEigen_Value = tau;

	////��ʱ��������
	//double A1[3][3],Temp[3],fErr;
	//memcpy(A1, A, 3 * 3 * sizeof(double));
	//A1[0][0] -= *pfEigen_Value;
	//A1[1][1] -= *pfEigen_Value;
	//A1[2][2] -= *pfEigen_Value;
	////Disp(A1, "A1");
	//Matrix_x_Vector(A1, Eigen_Vector,Temp);
	//if ((fErr=abs(Temp[0]) + abs(Temp[1]) + abs(Temp[2])) > 1)
	//	printf("err:%f\n",fErr);

	return 1;
#undef MAX_ITER_COUNT
}
int bInverse_Power(double A[3][3], double p, double* pfEigen_Value, double Eigen_Vector[3])
{//����λ����A�������p������ֵ������������
	double p1 = p + 0.1;	//��һ����У����������ֵ����
	double A1[3][3] = { {A[0][0] - p1, A[0][1],A[0][2] },
			{A[1][0],A[1][1] - p1,A[1][2]},
			{A[2][0],A[2][1],A[2][2] - p1} };	//ΪA-pI
	Disp((double*)A, "A");
	if (!bInverse_Power(A1, pfEigen_Value, Eigen_Vector))
		return 0;
	*pfEigen_Value += p1;

	return 1;
}
void Power_Method_1(double A[3][3], double Eigen_Vector[3], double* pfEigen_Value)
{//��һ���ݷ�
	double fErr, v1[3], v[3] = { 1,1,1 };	//��ȷ��һ��v0��ʹ��(x1,v0)!=0,�ʴˣ�1,1,1)��������
	//������vk=A*vk-1

	while (1)
	{
		Matrix_x_Vector(A, v, v1);
		fErr = abs(v1[0] - v[0]) + abs(v1[1] - v[1]) + abs(v1[2] - v[2]);
		printf("v=(%f,%f,%f) v1=(%f,%f,%f) err=%f\n", v[0], v[1], v[2], v1[0], v1[1], v1[2], fErr);
		v[0] = v1[0], v[1] = v1[1], v[2] = v1[2];
	}
	return;
}
void Power_Method(double A[3][3], double Eigen_Vector[3], double* pfEigen_Value)
{//����������ֵ����Ӧ����������
	double fDiff, fDiff_Total, fMod, tau, z_pre[3] = { MAX_FLOAT,MAX_FLOAT,MAX_FLOAT }, zk[3] = { 1,1,1 }, yk[3];
	int i;
	int iCount = 0;
	while (1)
	{//�˴�������֪������Ϊֹ
		Matrix_x_Vector(A, zk, yk);
		//Get_Max(yk, tau);
		Get_Max_1(yk, tau, i);
		printf("count:%d zk=(%f %f %f) yk=(%f %f %f) tau=%f\n", iCount++, zk[0], zk[1], zk[2], yk[0], yk[1], yk[2], tau);
		//�˴��޸ĵ�����ʹ��zk����
		tau = zk[i] >= 0 ? 1.f / tau : -1.f / tau;	//�˷��ȳ�����
		zk[0] = yk[0] * tau;
		zk[1] = yk[1] * tau;
		zk[2] = yk[2] * tau;
		//Disp(zk, "zk");
		fDiff_Total = 0;
		fDiff = zk[0] - z_pre[0];
		fDiff_Total = Abs(fDiff);
		fDiff = zk[1] - z_pre[1];
		fDiff_Total += Abs(fDiff);
		fDiff = zk[2] - z_pre[2];
		fDiff_Total += Abs(fDiff);
		if (fDiff_Total < ZERO_APPROCIATE)
			break;
		z_pre[0] = zk[0];
		z_pre[1] = zk[1];
		z_pre[2] = zk[2];
	}
	//��ʱ��1/tau��Ϊ��������ֵ��zkΪ��Ӧ����������
	*pfEigen_Value = 1 / tau;
	//Normalize zk
	fMod = sqrt(zk[0] * zk[0] + zk[1] * zk[1] + zk[2] * zk[2]);
	Eigen_Vector[0] = zk[0] / fMod;
	Eigen_Vector[1] = zk[1] / fMod;
	Eigen_Vector[2] = zk[2] / fMod;
	*pfEigen_Value = 1 / tau;
	//printf("Eigen Value:%f\n", *pfEigen_Value);
	//Disp(Eigen_Vector, "Eigen_Vector");
	return;
}
void Solve_Cubic_Eq_PQ(double p, double q, double Root[3], int* piRoot_Count)
{//��� x^3 + px + q=0��ע�⣬����ʧ�ܣ����Ľ�
	//double w=-1.f/2.f;	//�����Ǹ���������ʱ���ܸ���
	double fTemp_1, fTemp_2, fDelta;	// , w_square = -1.f / 2.f;
	double fRoot_Part_1;	// delta^(1/2)
	double fRoot_Part_2, fRoot_Part_3;
	//(q / 2) ^ 2 + (p / 3) ^ 3;
	fTemp_1 = q / 2;
	fTemp_2 = p / 3;
	fDelta = fTemp_2 * fTemp_2 * fTemp_2 + fTemp_1 * fTemp_1;

	//��delta�����б�����
	if (fDelta < 0)
	{
		double r;
		r = -p / 3.f;
		r = sqrt(r * r * r);
		double R = 2 * sqrt(-p / 3.f);
		if (abs(-0.5 * (q / r)) > 1)
			printf("err");
		double theta = acos(-0.5 * (q / r)) / 3;
		Root[0] = R * cos(theta);
		Root[1] = R * cos(theta + 3.1415926 * 2 / 3);
		Root[2] = R * cos(theta + 3.1415926 * 4 / 3);
		*piRoot_Count = 3;
		//printf("����������ʵ��\n");
	}
	else
	{
		if (p == 0 && q == 0)
		{//fDelta=0
			Root[0] = Root[1] = Root[2] = 0;
			*piRoot_Count = 0;
			printf("������0��\n");
			return;
		}
		fRoot_Part_1 = sqrt(fDelta);
		fRoot_Part_2 = -fTemp_1 + fRoot_Part_1;
		fRoot_Part_3 = -fTemp_1 - fRoot_Part_1;
		fRoot_Part_2 = fRoot_Part_2 >= 0 ? pow(fRoot_Part_2, 1.f / 3.f) : -pow(-fRoot_Part_2, 1.f / 3.f);;
		fRoot_Part_3 = fRoot_Part_3 >= 0 ? pow(fRoot_Part_3, 1.f / 3.f) : -pow(-fRoot_Part_3, 1.f / 3.f);
		Root[0] = fRoot_Part_2 + fRoot_Part_3;
		if (fDelta > 0)
		{
			*piRoot_Count = 1;
			printf("��һ��ʵ����������");
			//return;
		}
		else
		{//��ôq�ز�����0
			Root[1] = Root[2] = Root[0] * (-0.5);
			*piRoot_Count = 3;
			printf("����ʵ�������������\n");
		}
	}

	//����
	float fErr;
	for (int i = 0; i < 3; i++)
	{
		fErr = (float)(Root[i] * Root[i] * Root[i] + p * Root[i] + q);
		if (abs(fErr) > 0.1)
			printf("err");
	}

	return;
}
void Solve_Cubic(double a, double b, double c, double d, Complex_d Root[3], int* piReal_Root_Count)
{//	��ʢ��ʽ��һԪ�����������̣���������ͨ��
#define SQRT_3 1.7320508075688772935274463415059
	//Complex_d Comp[2];
	double A = b * b - 3 * a * c;
	double B = b * c - 9 * a * d;
	double C = c * c - 3 * b * d;
	double Y1, Y2, K, T, theta, Part_1, Part_2;
	double delta;
	memset(Root, 0, 3 * sizeof(Complex_d));
	if (A == 0 && B == 0)
	{
		if (b != 0)
			Root[0].real = Root[1].real = Root[2].real = -c / b;
		else if (a != 0)
			Root[0].real = Root[1].real = Root[2].real = -b / (3 * a);
		else if (c != 0)
			Root[0].real = Root[1].real = Root[2].real = -(3 * d) / c;
		printf("��һ������ʵ��\n");
	}
	else
	{
		delta = B * B - 4 * A * C;
		if (delta > 0)
		{//����·���Ѿ�����
			//�Ȱ�ʵ�������
			delta = 1.5 * sqrt(delta);

			Y1 = A * b - 1.5 * a * B + delta;
			Y2 = Y1 - 2 * delta;
			//��Y1,Y2�����η�
			Y1 = Y1 >= 0 ? pow(Y1, 1.f / 3.f) : -pow(-Y1, 1.f / 3.f);
			Y2 = Y2 >= 0 ? pow(Y2, 1.f / 3.f) : -pow(-Y2, 1.f / 3.f);
			Root[0].real = (-b - Y1 - Y2) / (3 * a);
			*piReal_Root_Count = 1;
			//����һ�Թ����
			Root[1].real = Root[2].real = (-2 * b + Y1 + Y2) / (6 * a);
			Root[1].im = SQRT_3 * (Y1 - Y2) / (6 * a);
			Root[2].im = -Root[1].im;	//����
			printf("��һ��ʵ����һ�Թ����\n");
		}
		else if (delta == 0)
		{
			K = B / A;
			Root[0].real = -b / a + K;
			Root[1].real = Root[2].real = -0.5 * K;
			printf("������ʵ����������һ�����ظ�\n");
		}
		else //delta<0
		{
			T = (2 * A * b - 3 * a * B) / (2 * sqrt(A * A * A));
			theta = acos(T) / 3;
			A = sqrt(A);
			Part_1 = A * cos(theta);
			Part_2 = SQRT_3 * A * sin(theta);
			Root[0].real = -b - 2 * Part_1;
			Root[1].real = -b + Part_1 + Part_2;
			Root[2].real = -b + Part_1 - Part_2;
			a = 1.f / (3.f * a);
			Root[0].real *= a;
			Root[1].real *= a;
			Root[2].real *= a;
			*piReal_Root_Count = 3;
			//printf("����������ȵ�ʵ��\n");
		}
	}
}
void Solve_Eigen_3x3(float A[], Complex_f Root[3])
{//�����������
	double a, b, c, d;	//����ϵ��
	float(*A_1)[3] = (float(*)[3])A;
	Complex_d Root_1[3];
	int i, iCount;

	a = -1;
	b = A_1[0][0] + A_1[1][1] + A_1[2][2];//+ (A_100 + A_111 + A_122) * r ^ 2
	c = A_1[0][1] * A_1[1][0] + A_1[0][2] * A_1[2][0] - A_1[0][0] * A_1[1][1] - A_1[0][0] * A_1[2][2]
		- A_1[1][1] * A_1[2][2] + A_1[1][2] * A_1[2][1];
	d = A_1[0][0] * A_1[1][1] * A_1[2][2] + A_1[0][1] * A_1[1][2] * A_1[2][0] + A_1[0][2] * A_1[1][0] * A_1[2][1]
		- A_1[0][0] * A_1[1][2] * A_1[2][1] - A_1[0][1] * A_1[1][0] * A_1[2][2] - A_1[0][2] * A_1[1][1] * A_1[2][0];


	//Ȼ�����A_1x^3 + bx^2+ cx + d=0�����߳���A_1,��ȡ�෴������
	a = 1;
	b = -b;
	c = -c;
	d = -d;

	//ע�⣬�˴���õĽ����ڴ�����ֵ�����Բ���ȫ�š�Ӧ�ý�һ���Ե�������߾���
	Solve_Cubic(a, b, c, d, Root_1, &iCount);
	for (i = 0; i < 3; i++)
		Root[i].im = (float)Root_1->im, Root[i].real = (float)Root_1[i].real;
	return;
}
void Solve_Eigen_Vector(float A[], int iOrder, float fEigen_Value, float Q[], int* piCount)
{//���ݸ���������ֵ����A�� ����������̵õ����������� Q�� ��������  *piCount: ��������������ϵ��
//�˴�����һ�����⣬��һ����ֵ����һ���ľ������⣬�п�������ⷽ�̵�����
	float* A_1 = (float*)malloc((iOrder + 1) * iOrder * sizeof(float)),
		* A_2 = (float*)malloc((iOrder + 1) * iOrder * sizeof(float)),
		* Q_1;

	int y, x, iRank;
	for (y = 0; y < iOrder; y++)
	{
		for (x = 0; x < iOrder; x++)
		{
			if (x == y)
				A_1[y * (iOrder + 1) + x] = A[y * iOrder + x] - fEigen_Value;
			else
				A_1[y * (iOrder + 1) + x] = A[y * iOrder + x];
		}
		A_1[y * (iOrder + 1) + iOrder] = 0;
	}
	//Disp(A_1, iOrder, iOrder+1, "A_1");


	Elementary_Row_Operation(A_1, iOrder, iOrder + 1, A_2, &iRank, &Q_1, NULL);
	*piCount = iOrder - iRank;
	Disp(Q_1, 1, 3, "Q_1");
	memcpy(Q, Q_1, *piCount * iOrder * sizeof(float));
	free(A_1);
	free(A_2);
	free(Q_1);
	return;
}
void Solve_Eigen(double A[3][3], Complex_d Root[3], int* piRoot_Count)
{
	double a, b, c, d;	//����ϵ��
	a = -1;
	b = A[0][0] + A[1][1] + A[2][2];//+ (a00 + a11 + a22) * r ^ 2
	c = A[0][1] * A[1][0] + A[0][2] * A[2][0] - A[0][0] * A[1][1] - A[0][0] * A[2][2]
		- A[1][1] * A[2][2] + A[1][2] * A[2][1];
	d = A[0][0] * A[1][1] * A[2][2] + A[0][1] * A[1][2] * A[2][0] + A[0][2] * A[1][0] * A[2][1]
		- A[0][0] * A[1][2] * A[2][1] - A[0][1] * A[1][0] * A[2][2] - A[0][2] * A[1][1] * A[2][0];

	//Ȼ�����ax^3 + bx^2+ cx + d=0�����߳���a,��ȡ�෴������
	a = 1;
	b = -b;
	c = -c;
	d = -d;

	//ע�⣬�˴���õĽ����ڴ�����ֵ�����Բ���ȫ�š�Ӧ�ý�һ���Ե�������߾���
	Solve_Cubic(a, b, c, d, Root, piRoot_Count);

	//����
	int i;
	float fResult;
	for (i = 0; i < 3; i++)
	{
		fResult = (float)(a * Root[i].real * Root[i].real * Root[i].real + b * Root[i].real * Root[i].real + c * Root[i].real + d);
		if (abs(fResult) > 0.1)
			printf("Err Result:%f\n", fResult);
	}

	return;
}
void Solve_Cubic(double A[3][3], double Root[3], int* piRoot_Count)
{//��һԪ���η��̣�δ�㶨
	double aa, a, b, bb, c, d, p, q/*, det*/;	//����ֵһԪ���η��̵�ϵ��
	double r = 2;	//����ramda
	double fPart_1;
	Disp((double*)A, "A");
	// -r ^ 3 + (a00 + a11 + a22) * r ^ 2 + (a01 * a10 - a00 * a11 - a00 * a22 + a02 * a20 - a11 * a22 + a12 * a21) * r + a00 * a11 * a22 - a00 * a12 * a21 - a01 * a10 * a22 + a01 * a12 * a20 + a02 * a10 * a21 - a02 * a11 * a20
	a = -1; //-r ^ 3
	b = A[0][0] + A[1][1] + A[2][2];//+ (a00 + a11 + a22) * r ^ 2
	//(a01 * a10 - a00 * a11 - a00 * a22 + a02 * a20 - a11 * a22 + a12 * a21)* r
	c = A[0][1] * A[1][0] + A[0][2] * A[2][0] - A[0][0] * A[1][1] - A[0][0] * A[2][2]
		- A[1][1] * A[2][2] + A[1][2] * A[2][1];
	//a00 * a11 * a22 - a00 * a12 * a21 
	// - a01 * a10 * a22 + a01 * a12 * a20 
	//+ a02 * a10 * a21 - a02 * a11 * a20
	d = A[0][0] * A[1][1] * A[2][2] + A[0][1] * A[1][2] * A[2][0] + A[0][2] * A[1][0] * A[2][1]
		- A[0][0] * A[1][2] * A[2][1] - A[0][1] * A[1][0] * A[2][2] - A[0][2] * A[1][1] * A[2][0];
	aa = a * a;
	bb = b * b;

	p = (3 * a * c - bb) / (3 * aa);
	q = (27 * aa * d - 9 * a * b * c + 2 * bb * b) / (27 * aa * a);

	//Ȼ�����ax^3 + bx^2+ cx + d=0�����߳���a,��ȡ�෴������
	a = 1;
	b = -b;
	c = -c;
	d = -d;
	//�� x=y-b/3a, ���ɻ�Ϊ y^3 + py + q =0
	//det = a * r * r * r + b * r * r + c * r + d;

	//�˴�OK�ˣ����˷���
	Solve_Cubic_Eq_PQ(p, q, Root, piRoot_Count);
	//Solve_Cubic_Eq_PQ(-3, 2, Root,piRoot_Count);

	fPart_1 = b / (3 * a);
	Root[0] -= fPart_1;
	Root[1] -= fPart_1;
	Root[2] -= fPart_1;
	//Disp(Root, "Root");

	//����
	float fErr;
	for (int i = 0; i < 3; i++)
	{
		fErr = (float)(a * (Root[i] * Root[i] * Root[i]) + b * Root[i] * Root[i] + c * Root[i] + d);
		if (abs(fErr) > 0.1)
			printf("err");
	}
	return;
}


void Matrix_Test()
{
	//Test_7();
	//return;
	int i;
	double Ramda[3],/* V[3],*/ A[3][3] = { {1, 2, 3}, { 4,5,6 }, { 7,8,9 } };
	unsigned long long tStart = iGet_Tick_Count();
	double fTotal = 0,/*fEigen_Value,*/Eigen_Vector[3][3];

	double Root[3];
	int iCount;
	Solve_Cubic(A, Root, &iCount);
	//�ݷ����������ֵ����������
	//Power_Method(A, V,&fEigen_Value);
	//���ݷ�����С����ֵ����������
	//bInverse_Power(A, V);
	//Disp(V, "V");
	for (i = 0; i < 10000000; i++)
	{
		bEigen_Value_3x3(A, Ramda, Eigen_Vector);
		fTotal += Ramda[0];
		//Disp(Ramda, "Ramda");
	}
	printf("%lld %f\n", iGet_Tick_Count() - tStart, fTotal);
	Disp(Ramda, "Ramda");
	Disp(Eigen_Vector, "Eigen_Vector");
	return;
}
float fGet_Determinant(float* A, int iOrder)
{//������ʽ��MΪ����iOrderΪ����ʽ�Ľ�,iStrikeΪM���д�С��һ��Ԫ�ظ����� Get n-order determinant
//���㷨�����ٶȣ�������֤��ѧԭ�����Բ��ð���չ������ D=ai1*Ai1+ai2*Ai2+...+ainAin
//�Ż�˼·��������ʽ������Ԫ�����г����б任��ת��Ϊ�����Ƿ���������Խ��߳˻�
	float* pCur, * pCofactor; //����ʽ
	int i, j, k;
	float fDeterminant, fTotal;
	if (iOrder == 2)//�ݹ鵽����һ�㣬 2������ʽ
		return A[0] * A[3] - A[1] * A[2];

	pCofactor = (float*)malloc((iOrder - 1) * (iOrder - 1) * sizeof(float)); //����ʽ
	A -= (iOrder + 1);	//Ϊ������ѧԭ�����Ӧ���˴��ı��ַ���±��1��ʼ
	//����һ��չ��
	for (fTotal = 0, i = 1; i <= iOrder; i++)
	{
		//���һ�е�i��Ԫ�ص�����ʽ
		for (pCur = pCofactor, j = 2; j <= iOrder; j++)
		{
			for (k = 1; k <= iOrder; k++)
			{
				if (k == i)
					continue;	//��ȥ
				*pCur++ = A[j * iOrder + k];
			}
		}
		fDeterminant = fGet_Determinant(pCofactor, iOrder - 1);
		fTotal += (float)(A[1 * iOrder + i] * pow(-1, 1 + i) * fDeterminant);
		//Disp((float*)pCofactor, iOrder - 1, iOrder - 1);
	}
	free(pCofactor);
	return fTotal;
}
void Gen_Cofactor(float* pM, int iOrder, float* pCofactor, int i, int j)
{//����aij��Ӧ������ʽAij, iOrrderΪpM�Ľ�
	int x, y;
	float* pCur = pCofactor;
	for (y = 0; y < iOrder; y++)
	{
		for (x = 0; x < iOrder; x++)
		{
			if (y == i || x == j)
				continue;	//����i�У���j�л�ȥ
			*pCur++ = pM[y * iOrder + x];
		}
	}
	return;
}
void Get_Adjoint_Matrix(float* pA, int iOrder, float* pAdjoint_Matrix)
{//�������󡣰�������ɾ���A��Ԫ�صĴ������������
	float* pCofactor = (float*)malloc((iOrder - 1) * (iOrder - 1) * sizeof(float));
	int i, j, iFlag;
	float* pAdj = (float*)malloc(iOrder * iOrder * sizeof(float));
	//Disp(pA, iOrder, iOrder);
	//���������󣬼���iOrder*iOrder����������ʽ
	for (i = 0; i < iOrder; i++)
	{
		for (j = 0; j < iOrder; j++)
		{
			iFlag = (int)pow(-1, (i + j) % 2);
			if (iOrder >= 3)
			{
				Gen_Cofactor(pA, iOrder, pCofactor, i, j);
				//printf("%d %d\n", i, j);
				//Disp(pCofactor, iOrder - 1, iOrder - 1);
				//printf("Determinant:%f\n\n", fGet_Determinant(pCofactor, iOrder - 1));
				pAdj[i * iOrder + j] = iFlag * fGet_Determinant(pCofactor, iOrder - 1);
			}
			else
				pAdj[i * iOrder + j] = iFlag * pA[((i + 1) % 2) * 2 + (j + 1) % 2];

		}
	}

	//Disp(pAdj, iOrder, iOrder);
	memcpy(pAdjoint_Matrix, pAdj, iOrder * iOrder * sizeof(float));
	free(pAdj);
	return;
}

template<typename _T> void Get_Inv_Matrix_Row_Op(_T* pM, _T* pInv, int iOrder, int* pbSuccess)
{//�����ð�������������̫�����˴��ó����б任�㣬��ʵ���Ǹ�˹�󷽳̵�һ��
	//��һ����Bufferװ����M���б任
	_T* pAux;
	int y, x, i, iRow_Size = iOrder * 2, iPos;
	_T fMax, * pfCur_Row, fValue;
	//Light_Ptr oPtr = oMatrix_Mem;
	if (iOrder > 256)
	{
		printf("Too large order:%d\n", iOrder);
		*pbSuccess = 0;
		return;
	}
	//pAux = (float*)malloc(iOrder * 2 * iOrder * sizeof(float));
	//Malloc_1(oPtr,iOrder * 2 * iOrder * sizeof(_T),pAux);
	pAux = (_T*)pMalloc(&oMatrix_Mem, iOrder * 2 * iOrder * sizeof(_T));

	//��ʼ��ֵ
	for (y = 0; y < iOrder; y++)
	{
		iPos = y * iRow_Size;
		for (x = 0; x < iOrder; x++)
			pAux[iPos + x] = pM[y * iOrder + x];
		iPos += iOrder;
		for (x = 0; x < iOrder; x++)
			pAux[iPos + x] = y == x ? 1.f : 0.f;
	}
	//Disp(pAux, iOrder, iOrder * 2,"\n");

	for (y = 0; y < iOrder; y++)
	{
		pfCur_Row = &pAux[y * iRow_Size];
		fMax = pAux[y * iRow_Size + y];
		pfCur_Row[y] = 1.f;
		for (x = y + 1; x < iRow_Size; x++)
			pfCur_Row[x] /= fMax;
		//Disp(pAux, iOrder, iOrder * 2, "\n");

		//�Ժ��������д���
		for (i = y + 1; i < iOrder; i++)
		{//i��ʾ��i��
			//iPos = Q[i] * iRow_Size;
			iPos = i * iRow_Size;
			if (fValue = pAux[iPos + y])
			{//���ڶ�ӦԪ��Ϊ0�����������
				for (x = y + 1; x < iRow_Size; x++)
					pAux[iPos + x] -= fValue * pfCur_Row[x];
				pAux[iPos + y] = 0;	//�˴�Ҳ���Ǳ���ģ�����ֻ�Ǻÿ�
			}
			//Disp(pAux, iOrder, iOrder * 2, "\n");
		}
	}
	//Disp(pAux, iOrder, iOrder * 2, "\n");

	_T* pfBottom_Row;
	int y_1;
	//Ȼ��˳������һ�����ϱ任����θ���˹�����Է��̲�һ��
	for (y = iOrder - 1; y > 0; y--)
	{//�߼��ϴ�����һ�����ϣ�ʵ������Qָ·
		pfBottom_Row = &pAux[y * iRow_Size];
		for (y_1 = y - 1; y_1 >= 0; y_1--)
		{
			//iPos = Q[y_1] * iRow_Size;
			iPos = y_1 * iRow_Size;
			x = y;	//�����е�xλ��
			fValue = pAux[iPos + x];
			pAux[iPos + x] = 0;
			for (x++; x < iRow_Size; x++)
				pAux[iPos + x] -= fValue * pfBottom_Row[x];
			//Disp(pAux, iOrder, iRow_Size, "\n");
		}
	}
	//��󳭵�Ŀ�����

	for (y = 0; y < iOrder; y++)
	{
		iPos = y * iRow_Size + iOrder;
		for (x = 0; x < iOrder; x++, iPos++, pInv++)
			*pInv = pAux[iPos];
	}
	*pbSuccess = 1;	//�˴������⣬��������������ô��Ȼ�Ҳ��������
	Free(&oMatrix_Mem, pAux);
	//free(pAux);
	return;
}

void Get_Inv_Matrix(float* pM, float* pInv, int iOrder, int* pbSuccess)
{//��pM��������ð������  Inv(A)= (A*)/|A| ��ʱֻ��3�����ϵľ���
	float fDet = fGet_Determinant(pM, iOrder);

	int i, j;
	if (abs(fDet) < ZERO_APPROCIATE)
	{
		*pbSuccess = 0;
		return;
	}
	float* pAdjoint_Matrix = (float*)malloc(iOrder * iOrder * sizeof(float));
	Get_Adjoint_Matrix(pM, iOrder, pAdjoint_Matrix);

	//���ˣ���������Ѿ�OK������pAdjoint_Matrix�Ȼ�������ս�����ܸ��̿���
	//׼ȷ���㷨Ӧ���� A(-1)= (A*)'/|A|	, �������Ҫת��һ�·���
	for (i = 0; i < iOrder; i++)
		for (j = 0; j < iOrder; j++)
			pInv[i * iOrder + j] = pAdjoint_Matrix[j * iOrder + i] / fDet;
	*pbSuccess = 1;

	free(pAdjoint_Matrix);
	return;
}
void Linear_Equation_Check(float* A, int iOrder, float* B, float* X, float eps)
{//�����Է���Ax=B����, ��<epsΪ��
	float* X1, fError;
	int i;
	X1 = (float*)malloc(iOrder * sizeof(float));

	Matrix_Multiply(A, iOrder, iOrder, X, 1, X1);
	for (i = 0, fError = 0; i < iOrder; i++)
		fError += abs(B[i] - X1[i]);

	if (fError < eps)
		printf("Correct\n");
	else
		printf("Incorrect, Error Sum:%f\n", fError);
	free(X1);
	return;
}

void Solve_Linear_Contradictory(float* A, int m, int n, float* B, float* X, int* pbSuccess)
{//��������С���˷���ì�ܷ����顣�ؼ����� A'Ax=A'B, ���н⣬��xΪ Ax=B����С���˽�
//����������⾡����ͨ�����߱��λ�Ϊ�������⣬Ȼ���ý�ì�ܷ�����ķ�������ͺð�

	float* At = (float*)malloc((m * n + n * n + m) * sizeof(float));
	float* AtA = At + m * n;
	float* AtB = AtA + n * n;
	int bResult = 0;

	Matrix_Transpose(A, m, n, At);
	Matrix_Multiply(At, n, m, A, n, AtA);
	Matrix_Multiply(At, n, m, B, 1, AtB);

	Solve_Linear_Gause(AtA, n, AtB, X, &bResult);
	/*Disp(AtA, n, n, "AtA");
	Disp(AtB, n, 1, "AtB");
	Disp(X, n, 1, "X");*/
	if (bResult)
	{//����
		float fTotal = 0, * B_1 = (float*)malloc(m * sizeof(float));
		int i;
		Matrix_Multiply(A, m, n, X, 1, B_1);
		for (i = 0; i < m; i++)
			fTotal += (B[i] - B_1[i]) * (B[i] - B_1[i]);
		printf("Error Sum:%f\n", fTotal);
		free(B_1);
		Disp(X, 1, n, "x");
	}
	*pbSuccess = bResult;
	free(At);
}
int iGet_Rank(float* A, int m, int n)
{//���ø�˹����Ԫ���������ȣ��ó����б任��
	int iRank = 0;
	int iMax, iTemp, iPos, Q[256];
	int y, x, i, iRow_To_Test;
	float* Ai = (float*)malloc(m * n * sizeof(float));
	float fMax, * pfMax_Row, fValue;
	iPos = 0;
	if (Ai)
		memcpy(Ai, A, m * n * sizeof(float));
	else
	{
		printf("Fail to malloc in fGet_Rank\n");
		return -1;
	}
	iRow_To_Test = Min(m, n);
	for (y = 0; y < m; y++)
		Q[y] = y;	//ÿ����Ԫ���ڵ���

	for (y = 0; y < iRow_To_Test; y++)
	{
		iMax = y;
		fMax = Ai[Q[iMax] * n + y];

		for (i = y + 1; i < m; i++)
		{//Ѱ������Ԫ
			if (abs(Ai[iPos = Q[i] * n + y]) > abs(fMax))
			{
				fMax = Ai[iPos];
				iMax = i;
			}
		}
		if (abs(fMax) < ZERO_APPROCIATE)
		{//����ԪΪ0����Ȼ�����ȣ��÷���û��Ψһ��
			printf("������,����ԪΪ��%f\n", fMax);
			Disp(Ai, m, n, "Ai");
			continue;
		}
		else
			iRank++;

		//�����ԪSWAP��Q�ĵ�ǰλ����
		iTemp = Q[y];
		Q[y] = Q[iMax];
		Q[iMax] = iTemp;

		//��iMax���ڵ��н���ϵ�����㣬��ϵ��/=A[y][y]
		pfMax_Row = &Ai[Q[y] * n];
		pfMax_Row[y] = 1.f;
		for (x = y + 1; x < n; x++)
			pfMax_Row[x] /= fMax;

		//�Ժ��������д���
		for (i = y + 1; i < m; i++)
		{//i��ʾ��i��
			iPos = Q[i] * n;
			if (fValue = Ai[iPos + y])
			{//���ڶ�ӦԪ��Ϊ0�����������
				for (x = y + 1; x < n; x++)
					Ai[iPos + x] -= fValue * pfMax_Row[x];
				Ai[iPos + y] = 0;	//�˴�Ҳ���Ǳ���ģ�����ֻ�Ǻÿ�
			}
			//Disp(Ai, iOrder, iOrder + 1, "\n");
		}
	}
	free(Ai);
	return iRank;
}
void Solve_Linear_Gause(float* A, int iOrder, float* B, float* X, int* pbSuccess)
{//�ø�˹����Ԫ��������Է�����, Ҫ�㣺
	//1����˹����ȫ�ȼ������б任��ֻ����û��������Ĺ������������ǲ������ǽ���Ԫ��Ϊ1
	//2��ѡ����Ԫ��ԭ���Ǳ�֤�� ����ϵ��/aijʱ��ĸ������̫С����ĸС����
	//3,������Ԫ̫С��<eps)��������㲻���ȣ��˳���ʵ������Ԫ̫С�ᵼ�º���ĳ����������
	int y, x, i, iRow_Size;
	int iMax, iTemp, iPos, Q[256];
	float fMax, * pfMax_Row, fValue;
	if (iOrder > 256)
	{
		printf("Too large order:%d\n", iOrder);
		*pbSuccess = 0;
		return;
	}

	float* Ai = (float*)malloc((iOrder + 1) * iOrder * sizeof(float));
	iPos = 0;
	for (y = 0; y < iOrder; y++)
	{
		for (x = 0; x < iOrder; x++, iPos++)
			Ai[iPos] = A[y * iOrder + x];
		Ai[iPos++] = B[y];
		Q[y] = y;	//ÿ����Ԫ���ڵ���
	}
	//Disp(Ai, iOrder, iOrder + 1,"\n");
	iRow_Size = iOrder + 1;
	//Ϊ�˱�����⣬����iMax��ʾQ�е�������������Ai�е��к�
	for (y = 0; y < iOrder; y++)
	{
		iMax = y;
		fMax = Ai[Q[iMax] * iRow_Size + y];
		for (i = y + 1; i < iOrder; i++)
		{//Ѱ������Ԫ
			if (abs(Ai[iPos = Q[i] * iRow_Size + y]) > abs(fMax))
			{
				fMax = Ai[iPos];
				iMax = i;
			}
		}
		if (abs(fMax) < ZERO_APPROCIATE)
		{//����ԪΪ0����Ȼ�����ȣ��÷���û��Ψһ��
			printf("������,����ԪΪ��%f\n", fMax);
			*pbSuccess = 0;
			free(Ai);
			return;
		}

		//�����ԪSWAP��Q�ĵ�ǰλ����
		iTemp = Q[y];
		Q[y] = Q[iMax];
		Q[iMax] = iTemp;

		//��iMax���ڵ��н���ϵ�����㣬��ϵ��/=A[y][y]
		pfMax_Row = &Ai[Q[y] * iRow_Size];
		pfMax_Row[y] = 1.f;
		for (x = y + 1; x < iRow_Size; x++)
			pfMax_Row[x] /= fMax;

		//Disp(Ai, iOrder, iOrder + 1, "\n");

		//�Ժ��������д���
		for (i = y + 1; i < iOrder; i++)
		{//i��ʾ��i��
			iPos = Q[i] * iRow_Size;
			if (fValue = Ai[iPos + y])
			{//���ڶ�ӦԪ��Ϊ0�����������
				for (x = y + 1; x < iRow_Size; x++)
					Ai[iPos + x] -= fValue * pfMax_Row[x];
				Ai[iPos + y] = 0;	//�˴�Ҳ���Ǳ���ģ�����ֻ�Ǻÿ�
			}
			//Disp(Ai, iOrder, iOrder + 1, "\n");
		}
	}

	//��һ����
	X[iOrder - 1] = Ai[Q[iOrder - 1] * iRow_Size + iOrder];

	//�ش�����Q[iOrder-1]��ʼ�ش���������һ�����ϻش�
	for (y = iOrder - 2; y >= 0; y--)
	{
		//�������ϻش�
		iPos = Q[y] * iRow_Size;
		//fValue=b
		fValue = Ai[iPos + iOrder];
		for (x = y + 1; x < iOrder; x++)
		{
			fValue -= Ai[iPos + x] * X[x];
			Ai[iPos + x] = 0;		//�˴����Ǳ���ģ�������0���ÿ�һЩ����
		}
		X[y] = fValue;
		Ai[iPos + iOrder] = fValue;	//�˴����Ǳ��룬�ÿ�����
	}

	//Disp(Ai, iOrder, iOrder + 1, "\n");
	*pbSuccess = 1;
	free(Ai);
	//free(X1);

	//����
	Linear_Equation_Check(A, iOrder, B, X, ZERO_APPROCIATE);
	return;
}
void Solve_Linear_Cramer(float* A, int iOrder, float* B, float* X, int* pbSuccess)
{//�ÿ���ķ��������Է���AX=B���˷��ȼ۸�˹����ֻ�ܽ����η�����Ψһ�⣬�����Աܿ�
//��˹���г���Ϊ0����������������ձ��ԡ���ʱ�临�ӶȺܲ�������ۼ�ֵ
	int i, j;
	float fDet = fGet_Determinant(A, iOrder);
	float* Ai = (float*)malloc(iOrder * iOrder * sizeof(float));
	if (abs(fDet) < ZERO_APPROCIATE)
	{
		free(Ai);
		*pbSuccess = 0;
		return;
	}
	for (i = 0; i < iOrder; i++)
	{//��A�ĵ�i�л��ɳ�������
		memcpy(Ai, A, iOrder * iOrder * sizeof(float));
		for (j = 0; j < iOrder; j++)
			Ai[j * iOrder + i] = B[j];
		X[i] = fGet_Determinant(Ai, iOrder) / fDet;
	}

	free(Ai);
	*pbSuccess = 1;
	return;
}
float fDot(float V0[], float V1[], int iDim)
{//���ڻ�
	float fTotal = 0;
	for (int i = 0; i < iDim; i++)
		fTotal += V0[i] * V1[i];
	return fTotal;
}
void Cross_Product(float V0[], float V1[], float V2[])
{//�����ˣ�������������� V2=V0xV1��ֻ����ά a��b=��aybz-azby)i + (azbx-axbz)j + (axby-aybx)k
	float Temp[3];
	Temp[0] = V0[1] * V1[2] - V0[2] * V1[1];
	Temp[1] = V0[2] * V1[0] - V0[0] * V1[2];
	Temp[2] = V0[0] * V1[1] - V0[1] * V1[0];
	V2[0] = Temp[0], V2[1] = Temp[1], V2[2] = Temp[2];
	return;
}

void Schmidt_Orthogon(float* A, int m, int n, float* B)
{//m��n�У�m������������
	float fValue, * bi, * pB = (float*)malloc(m * n * sizeof(float));
	int i, j, k;
	//Disp(A, m, n);
	for (i = 0; i < m; i++)
	{//ÿ����һ��Bi
		bi = &pB[i * n];
		for (j = 0; j < n; j++)
			bi[j] = A[i * n + j];	//bi=ai;
		for (j = 0; j < i; j++)
		{
			fValue = fDot(&pB[j * n], &A[i * n], n);
			fValue /= fDot(&pB[j * n], &pB[j * n], n);
			for (k = 0; k < n; k++)
				bi[k] -= fValue * pB[j * n + k];
		}
	}

	//�ٵ�λ��
	for (i = 0; i < m; i++)
	{
		bi = &pB[i * n];
		Normalize(bi, n, bi);
		//Disp(bi, 1, n);
	}
	memcpy(B, pB, m * n * sizeof(float));
	free(pB);
	return;
}
void Schmidt_Orthogon(float* A, int iOrder, float* B)
{//ʩ����������.Ϊ��򵥻������������Ծ�����ʽ���롣����������Ҳ�Ծ�����ʽ���
	float fValue, * bi, * pB = (float*)malloc(iOrder * iOrder * sizeof(float));
	int i, j, k;
	//b1=a1;
	//for (i = 0; i < iOrder; i++)
		//pTemp[i] = A[i];
	for (i = 0; i < iOrder; i++)
	{//ÿ����һ��Bi
		bi = &pB[i * iOrder];
		for (j = 0; j < iOrder; j++)
			bi[j] = A[i * iOrder + j];	//bi=ai;
		for (j = 0; j < i; j++)
		{
			fValue = fDot(&pB[j * iOrder], &A[i * iOrder], iOrder);
			fValue /= fDot(&pB[j * iOrder], &pB[j * iOrder], iOrder);
			for (k = 0; k < iOrder; k++)
				bi[k] -= fValue * pB[j * iOrder + k];
		}
		//Disp(bi, 1, 3);
	}
	//�ٵ�λ��
	for (i = 0; i < iOrder; i++)
	{
		bi = &pB[i * iOrder];
		fValue = sqrt(fDot(bi, bi, iOrder));
		for (j = 0; j < 3; j++)
			bi[j] /= fValue;
	}
	memcpy(B, pB, iOrder * iOrder * sizeof(float));
	free(pB);
	return;
}
template<typename _T>int bIs_Orthogonal(_T* A, int h, int w)
{//�ж�һ�������Ƿ�Ϊ�������������������Ƿ��󣬵��ǿ��Խ�һ���ſ���չ��������Ϊһ�����
	int y, x,iMin;
	_T* At, *S;
	//Light_Ptr oPtr = oMatrix_Mem;
	if (w == 0)
		w = h;
	//Malloc_1(oPtr, w * h * sizeof(_T), At);
	At = (_T*)pMalloc(&oMatrix_Mem, w * h * sizeof(_T));
	if (!At)
		return 0;
	Matrix_Transpose(A, h, w, At);
	iMin = Min(h, w);
	//Malloc_1(oPtr, iMin * iMin * sizeof(_T), S);
	S = (_T*)pMalloc(&oMatrix_Mem, iMin * iMin * sizeof(_T));
	if (w > h)	//����ڸ�
		Matrix_Multiply(A, h, w, At, h, S);
	else
		Matrix_Multiply(At, w, h, A, w, S);
	//Disp(S, iMin, iMin, "S");
	for (y = 0; y < iMin; y++)
	{
		for (x = 0; x < iMin; x++)
		{
			if (y == x)
			{//�Խ���
				if (abs(S[y * iMin + x] - 1) > ZERO_APPROCIATE)
					return 0;
			}
			else
			{
				if (abs(S[y * iMin + x]) > ZERO_APPROCIATE)
					return 0;
			}
		}
	}
	Free(&oMatrix_Mem, At);
	Free(&oMatrix_Mem, S);
	return 1;			
}
//template<typename _T>int bIs_Orthogonal(_T* A, int na)
//{//�жϾ���A�Ƿ�Ϊ�������жϷ���AA'=E
//	int y, x;
//	//_T* At = (_T*)malloc(na * na * sizeof(_T));
//	Light_Ptr oPtr = oMatrix_Mem;
//	_T* At;
//	Malloc_1(oPtr, na * na * sizeof(_T), At);
//	if (!At)
//		return 0;
//
//	//float fTotal;
//	for (y = 0; y < na; y++)
//		for (x = 0; x < na; x++)
//			At[y * na + x] = A[x * na + y];	//ת��
//	Matrix_Multiply(A, na, na, At, na, At);
//
//	//Disp(At, na, na, "AAt");
//	for (y = 0; y < na; y++)
//	{
//		for (x = 0; x < na; x++)
//		{
//			if (y == x)
//			{
//				if (abs(At[y * na + x] - 1) > ZERO_APPROCIATE)
//					return 0;
//			}
//			else if (abs(At[y * na + x]) > ZERO_APPROCIATE)
//				return 0;
//		}
//	}
//
//	/* ʵ��֤�������������ÿһ�л�ÿ�в���Ϊ��λ����
//	for (y = 0; y < na; y++)
//	{
//		for (fTotal = 0, x = 0; x < na; x++)
//			fTotal += A[y * na + x];
//		if (abs(fTotal- 1.f)>ZERO_APPROCIATE)
//			printf("Here");
//	}
//	for (x = 0; x < na; x++)
//	{
//		for (fTotal = 0, y = 0; y < na; y++)
//			fTotal += A[y * na + x];
//		if (abs(fTotal - 1.f) > ZERO_APPROCIATE)
//			printf("Here");
//	}*/
//
//	//Disp(At, na, na, "AAt");
//	//free(At);
//	return 1;
//}

void Get_Householder(float* X, float* Y, int iDim, float* H, int* pbSuccess)
{//���� |X|==|Y|,��ģ��ȵ���������������Hx=y�ľ������
//���������һ���Գƾ�����������,H=H',ͬʱ��
	int i, j;
	float fMod_X, fMod_Y, fDenominator;
	float* u = (float*)malloc(iDim * sizeof(float));
	for (fDenominator = fMod_X = fMod_Y = 0, i = 0; i < iDim; i++)
	{
		fMod_X += X[i] * X[i], fMod_Y += Y[i] * Y[i];
		fDenominator += (X[i] - Y[i]) * (X[i] - Y[i]);
	}
	fDenominator = sqrt(fDenominator);
	if (abs(fMod_X - fMod_Y) > ZERO_APPROCIATE)
	{
		printf("x��y��ģ��һ��,|x|=%f |y|=%f", sqrt(fMod_X), sqrt(fMod_Y));
		*pbSuccess = 0;
		return;
	}

	//u�����˵�λ����
	for (i = 0; i < iDim; i++)
		u[i] = (X[i] - Y[i]) / fDenominator;

	//H=I-2uu', ����u�Ѿ��������������ʹ
	for (i = 0; i < iDim; i++)
		for (j = 0; j < iDim; j++)
			H[i * iDim + j] = (i == j ? 1 : 0) - 2 * u[i] * u[j];
	*pbSuccess = 1;

	//////����
	//float *y1 = (float*)malloc(iDim * sizeof(float));
	//memset(y1, 0, iDim * sizeof(float));
	////for (i = 0; i < iDim; i++)
	////	for (j = 0; j < iDim; j++)
	////		y1[i] += H[i*iDim+j] * X[j];
	//for (i = 0; i < iDim; i++)
	//	for (j = 0; j < iDim; j++)
	//		y1[i] += H[i * iDim + j];

	Disp(H, iDim, iDim, "H");
	//Disp(y1, 1, iDim, "y1");		//ȫ��1�Ŷ�

	free(u);
	return;
}

void Get_Householder(float* X, int iDim, int l, float* Q, int* pbSuccess)
{//�����Ѱ��Hʹ��Hx=y��һ�㣬ֻ����X�� Y��Xֱ���Ƶ�����,ʹ�� Qx=y
	//Q��Ȼ�Ǿ�����󣬵����Ǹ�����ľ������
	//l������������λ�ã��հ涨ΪX�������ٸ�����
	//�ȹ���һ��Y��ʹ��ǰl-1����x��ǰi-1���l�һ����ʹ��|X|=|y|
	float fMod, * Y = (float*)malloc(iDim * sizeof(float));
	int i;
	//for (fMod_Total = 0, i = 0; i < iDim; i++)
		//fMod_Total += X[i] * X[i];
	//if (bFront)
	{
		for (i = 0; i < l; i++)
			Y[i] = X[i];
		for (fMod = 0; i < iDim; i++)
			fMod += X[i] * X[i];
		Y[l] = sqrt(fMod);
		for (i = l + 1; i < iDim; i++)
			Y[i] = 0;
	}
	/*else
	{
		for (fMod=0,i = 0; i < iDim - l; i++)
		{
			fMod += X[i] * X[i];
			Y[i] = 0;
		}
		Y[i] = sqrt(fMod+X[i]*X[i]);
		for (i++; i < iDim; i++)
			Y[i] = X[i];
		Disp(Y, 1, 6);
	}*/

	//��������Y��������Qx=y
	Get_Householder(X, Y, iDim, Q, pbSuccess);
	//���ǣ�Q����һ������ľ������ ��I��H���ɣ���������һ���������
	Matrix_Multiply(Q, iDim, iDim, X, 1, Y);
	Disp(Y, 1, iDim);
	//Disp<float>(Y, 1, iDim);
	free(Y);
	return;
}
void Get_Householder_2(float X[], int iDim, int l, float* Q)
{//��һ��Householder�㷨
	float delta, tau;
	float* V = (float*)malloc((iDim - l) * sizeof(float));
	float* H_Dup = (float*)malloc((iDim - l) * (iDim - l) * sizeof(float));
	int i, j, iH_Dim;
	iH_Dim = iDim - l;

	//���delta
	for (delta = 0, i = l; i < iDim; i++)
		delta += X[i] * X[i];
	delta = sign(X[l]) * sqrt(delta);
	for (i = l, j = 0; i < iDim; i++, j++)
		V[j] = X[i];
	V[0] += delta;
	tau = V[0] * delta;
	//Disp(V, 1, iDim-l);
	Matrix_Multiply((float*)V, iDim - l, 1, (float*)V, iDim - l, (float*)H_Dup);

	/*if (abs(tau) < 2.2204460492503131e-15 && tau != 0)
		printf("Here");*/
	for (i = 0; i < iDim - l; i++)
		for (j = 0; j < iDim - l; j++)
			H_Dup[i * (iDim - l) + j] = (i == j ? 1 : 0) - (tau != 0 ? H_Dup[i * (iDim - l) + j] / tau : 0);
	//if (abs(tau) < 2.2204460492503131e-15 && tau != 0)
		//Disp(H_Dup, iDim - 1, iDim - 1);
	//���Թ���H��
	memset(Q, 0, iDim * iDim * sizeof(float));
	for (i = 0; i < l; i++)
		Q[i * iDim + i] = 1;		//�ȶ�I

	for (i = l; i < iDim; i++)
		for (j = l; j < iDim; j++)
			Q[i * iDim + j] = H_Dup[(i - l) * (iDim - l) + (j - l)];

	//for (i = 0; i < iDim * iDim; i++)
		//Q[i] *= 35;

	//Disp(Q, iDim, iDim);
	free(V);
	free(H_Dup);
	return;
}
void QR_Decompose(float* A, int ma, int na, float* Q, float* R)
{//�򵥷ֽ⣺ A=QnQn-1..Q1*R
	int i, j;
	float* An = (float*)malloc(ma * na * sizeof(float));
	float* v = (float*)malloc(ma * sizeof(float));
	float* Qn = (float*)malloc(ma * ma * sizeof(float));
	float* Q_Dup = (float*)malloc(ma * ma * sizeof(float));

	//�Ƚ�Q��ΪI
	memset(Q_Dup, 0, ma * ma * sizeof(float));
	for (i = 0; i < ma; i++)
		Q_Dup[i * ma + i] = 1;

	memcpy(An, A, ma * na * sizeof(float));
	for (i = 0; i < na; i++)
	{
		for (j = 0; j < ma; j++)
			v[j] = An[j * na + i];
		//if (iCur == 101 && i == 7)
		//{
		//	//Disp(Qn, na, na, "Qn");
		//	//Disp(Q_Dup, ma, na);
		//	Disp(v, ma, 1);
		//	//Disp(Qn, ma, ma);
		//}
		Get_Householder_2(v, ma, i, (float*)Qn);
		//��Ai=Qi-1*Ai-1
		Matrix_Multiply((float*)Qn, ma, ma, (float*)An, na, (float*)An);

		//����Q_Dup*Qn�ǣ��൱������Qn*...Q2*Q1
		Matrix_Multiply((float*)Q_Dup, ma, ma, (float*)Qn, ma, Q_Dup);
		//bIs_Orthogonal(Q_Dup, na);
		//Disp(Q_Dup, na, na, "Q");		
	}
	//��ʱ��R����һ�������Ǿ���
	memcpy(R, An, ma * ma * sizeof(float));
	//Q����һ���������
	memcpy(Q, Q_Dup, ma * ma * sizeof(float));
	//for (i = 0; i < ma * ma; i++)
		//Q[i] *= 21;
	//Disp(Q, ma, ma,"Q");
	//Disp(An, ma, na, "R");
	free(An);
	free(v);
	free(Qn);
	free(Q_Dup);
	return;
}
int bIs_Upper_Tri(float* A, int ma, int na)
{//����һ������A���ж����Ƿ�Ϊ�����Ǿ���
#define eps 2.2204460492503131e-15
	int y, x;
	for (x = 0; x < na; x++)
	{
		for (y = x + 1; y < ma; y++)
		{
			if (abs(A[y * na + x]) >= eps)	// ZERO_APPROCIATE)
				return 0;
		}
	}
	return 1;
#undef eps
}

int bIs_Unit_Vector(float* V, int na)
{
	int i;
	float fSum = 0;
	for (i = 0; i < na; i++)
		fSum += V[i] * V[i];
	if (fSum == 0 || abs(fSum - 1) < ZERO_APPROCIATE)
		return 1;
	else
		return 0;
}
int bIs_Unit_Vector(float* V, int na, int iStrike)
{
	int i;
	float fTotal;
	for (fTotal = 0, i = 0; i < na; i++)
	{
		fTotal += V[i * iStrike] * V[i * iStrike];
		//printf("%f ", V[i * iStrike]);
	}
	//printf(" Total:%f\n",fTotal);
	if (abs(fTotal - 1) < ZERO_APPROCIATE)
		return 1;
	else
		return 0;
}

void QR_Decompose(float* A, int na, float* R, float* Q, int* pbSuccess, int* pbDup_Root)
{//�������ֵR,��������Q. A=R*Q
#define MAX_ITERATE_COUNT 200
//#define eps 2.2204460492503131e-15
	float* R_Dup = (float*)malloc(na * na * sizeof(float));
	float* Q_Dup = (float*)malloc(na * na * sizeof(float));
	float* An = (float*)malloc(na * na * sizeof(float));
	//float* Q_Dup_1 = (float*)malloc(na * na * sizeof(float));
	int i;
	memcpy(An, A, na * na * sizeof(float));

	//��Q_Dup_1��ΪI��������������
	//memset(Q_Dup_1, 0, na * na * sizeof(float));
	//for (i = 0; i < na; i++)
		//Q_Dup_1[i * na + i] = 1;

	float* Q1_2_Qn_Product = (float*)malloc(na * na * sizeof(float));
	//*Temp_3 = (float*)malloc(na * na * sizeof(float));
//int bSuccess;
	memset(Q1_2_Qn_Product, 0, na * na * sizeof(float));
	for (i = 0; i < na; i++)
		Q1_2_Qn_Product[i * na + i] = 1;

	i = 0;
	while (1)
	{
		/*for (int j = 0; j < na; j++)
			if (abs(An[j * na + j]) < ZERO_APPROCIATE)
				An[j * na + j] = 0;*/
				//�ֽ�ΪA=QR
		QR_Decompose(An, na, na, Q_Dup, R_Dup);
		//�ֽ��Q��������

		//printf("%d\n", bIs_Orthogonal(Q_Dup, na));
		//Matrix_Multiply(Q_Dup, na, na, R_Dup, na, An);
		//��An=RQ
		Matrix_Multiply(R_Dup, na, na, Q_Dup, na, An);

		//�۳�Qn*...*Q2*Q1,���ս��������������
		Matrix_Multiply(Q1_2_Qn_Product, na, na, Q_Dup, na, Q1_2_Qn_Product);

		////��һ�ѿ��� A= Q*An*Qt
		//Matrix_Multiply(Q1_2_Qn_Product, na, na, An, na, Temp_2);
		//Get_Inv_Matrix(Q1_2_Qn_Product, Temp_3, na, &bSuccess);
		//Matrix_Multiply(Temp_2, na, na, Temp_3,na, Temp_3);
		//Disp(A, na, na, "A");
		//Disp(Q_Dup, na, na, "Q");
		//Disp(Temp_3, na, na, "QAnQt");

		//����An�Ƿ�Ϊ�����Ǿ���
		//Disp(An, na, na, "An");
		if (bIs_Upper_Tri(An, na, na))
			break;
		if (i++ >= MAX_ITERATE_COUNT)
			break;
	}

	////�˴�������һ�����㿴���Ƿ� A= Q*An*Q'
	//float* Temp_1 = (float*)malloc(na * na * sizeof(float));
	//float* Temp_2 = (float*)malloc(na * na * sizeof(float));
	//printf("Orthogonal:%d\n", bIs_Orthogonal(Q1_2_Qn_Product,na));
	//Disp(Q1_2_Qn_Product, na, na, "Q");
	//Disp(An, na, na, "R");
	//Matrix_Multiply(Q1_2_Qn_Product, na, na, An, na, Temp_1);
	//Disp(Temp_1, na, na, "QR");
	//Matrix_Transpose(Q1_2_Qn_Product, na, na, Temp_2);
	//Matrix_Multiply(Temp_1, na, na, Temp_2, na, Temp_2);
	//Disp(A, na, na, "A");
	//Disp(Temp_2, na, na, "QRQt");

	////�����������Ƿ�Ϊ��λ����
	//printf("Is unit vector:\t");
	//for (i = 0; i < na; i++)
	//	printf("%d\t", bIs_Unit_Vector(&Q1_2_Qn_Product[i], na, na));
	//printf("\n");

	memcpy(R, An, na * na * sizeof(float));
	memcpy(Q, Q1_2_Qn_Product, na * na * sizeof(float));

	if (pbDup_Root)
	{
		*pbDup_Root = 0;
		for (i = 1; i < na; i++)
		{
			if (abs(R[(i - 1) * na + i - 1] - R[i * na + i]) < 0.01f)
				//if (abs(R[(i - 1) * na + i - 1] - R[i * na + i]) < eps)
				*pbDup_Root = 1;
		}
	}
	free(Q_Dup);
	free(R_Dup);
	free(An);
	free(Q1_2_Qn_Product);
	*pbSuccess = 1;
	//free(Q_Dup_1);
	return;
#undef MAX_ITERATE_COUNT
#undef eps
}

void svd_2(float* A, int ma, int na, float* U, float* S, float* Vt)
{//��������A'A��������������V, ����AA'��Ӧ�����������
	float* At = (float*)malloc(na * ma * sizeof(float));
	float* AtA = (float*)malloc(na * na * sizeof(float));
	float* V = (float*)malloc(na * na * sizeof(float));
	float* StS = (float*)malloc(na * na * sizeof(float));

	int i, bSuccess;
	memset(V, 0, na * na * sizeof(float));
	Matrix_Transpose(A, ma, na, At);
	Matrix_Multiply(At, na, ma, A, na, AtA);
	//Disp(AtA, na, na, "AtA");
	QR_Decompose(AtA, na, StS, V, &bSuccess);
	Disp(StS, na, na, "StS");
	Disp(V, na, na, "V");

	memset(S, 0, na * na * sizeof(float));
	//�������ɵõ�Sigma������ֵ
	for (i = 0; i < na; i++)
	{
		if (abs(StS[i * na + i]) < ZERO_APPROCIATE)
			StS[i * na + i] = 0;
		S[i * na + i] = sqrt(StS[i * na + i]);
	}

	//Disp(S, na, na, "S");
	//���²��ֿ��������ˣ�Ҫ�Ƶ���U= AV������Ҫ���壡
	//�����Ƶ�һ�Σ� �Ѿ�֪��A'A������������ɵľ���ΪV�� ��������yΪV�е�һ������������ 
	// rΪ��Ӧ������ֵ���� A'A y= ry,  ע�⣬����ҧ��Ҫ�� AA'����������������ǰ�澭�飬 ����ͬʱ���A
	//�� AA'Ay=rAy, ��ʱ�� ��Ay����һ�����壬 ����
	//	AA' ����������ΪAy, ������ֵ��r
	//��ô  U = AV, ֤�ϣ�

	float* U_1 = (float*)malloc(ma * na * sizeof(float));
	memset(U, 0, sizeof(ma * ma));

	//����AV, ��ΪU
	Matrix_Multiply(A, ma, na, V, na, U_1);
	//Ȼ�����ù�񻯣�����U_1����������ɣ��ʴ˱�����н��й��
	Normalize_Col(U_1, ma, na, U_1);
	//Disp(U_1, 9, 9);
	//�����Ժ�U_1�Ǹ�mxn���󣬸����ۻ���U����һ�������з���������
	//���Զ��ڵ�n����m-1�У�������0

	int y, x;
	for (y = 0; y < ma; y++)
	{
		for (x = 0; x < na; x++)
			U[y * ma + x] = U_1[y * na + x];
		for (; x < ma; x++)
			U[y * ma + x] = 0;
	}
	//Disp(U, ma, ma, "U");

	Matrix_Transpose(V, na, na, Vt);
	//Disp(Vt, na, na, "Vt");
	free(At);
	free(AtA);
	free(V);
	free(StS);
	return;
}

void svd_1(float* A, int ma, int na, float* U, float* S, float* Vt)
{//��mxn����A������ֵ�ֽ⣬�ֽ��U(mxm)Ϊ AA'������ֵ��V(nxn)ΪA'A������ֵ, S(mxn)ΪSigma
//��ʱֻ�㵽n>m�����
	float* AAt = (float*)malloc(ma * ma * sizeof(float));
	int y, i, bSuccess;

	Matrix_Multiply_Symmetric(A, ma, na, AAt);
	float* SSt = (float*)malloc(ma * ma * sizeof(float));
	memset(S, 0, ma * na * sizeof(float));
	QR_Decompose(AAt, ma, SSt, U, &bSuccess);
	//�����Ժ� AA' = U x SS' x U', �����￪ʼ���Ͳ��ܿ�������Щǧƪһ�ɵĶ��ѵ�

	//�������ɵõ�Sigma������ֵ
	for (i = 0; i < ma; i++)
		S[i * na + i] = sqrt(SSt[i * ma + i]);

	//����Ҫ��V, Ϊ�˼��Ƶ���ֱ�Ӹ���������֤���� V=A'U�� ��ô��ʱ��������Ҫ����ΪV��������������⣬���ַ����Ǵ��
	//֤��	A'A A'U = A' (AA')U = A' (r U) , ���� rΪ AA'������ֵ�� �����׺��沿��
	//����	A'A (A'U)= r (A'U)�� ��ʱ,�� A'U������������ɵľ�����ô��Ȼ�� (A'U)��Ϊһ�����壬��������������
	//����  V= A'U, ֤��
	//�ټ�һ��ת�� V'= (A'U)'= U'(mxm)A(mxn)

	//�˴�ֱ�������˷�
	int iSize = Max(ma, na) * Max(ma, na);
	float* Vt_1 = (float*)malloc(iSize * sizeof(float));
	memset(Vt_1, 0, iSize * sizeof(float));

	float* Ut = (float*)malloc(ma * ma * sizeof(float));
	Matrix_Transpose(U, ma, ma, Ut);
	Matrix_Multiply(Ut, ma, ma, A, na, Vt_1);

	for (y = 0; y < ma; y++)	//���
		Normalize(&Vt_1[y * na], na, &Vt_1[y * na]);

	//�������ͣ����ö�AtA����һ��QR�ֽ⣬����ò���һ��ä����ָ�⣬����Vt�����һ��
	float* AtA = (float*)malloc(na * na * sizeof(float));
	float* V_2 = (float*)malloc(na * na * sizeof(float)),
		* Vt_2 = (float*)malloc(na * na * sizeof(float));
	float* S_1 = (float*)malloc(na * na * sizeof(float));
	Matrix_Transpose(A, ma, na, AtA);
	Matrix_Multiply(AtA, na, ma, A, na, AtA);
	QR_Decompose(AtA, na, S_1, V_2, &bSuccess);
	Matrix_Transpose(V_2, na, na, Vt_2);
	Normalize(&Vt_2[(na - 1) * na], na, &Vt_1[(na - 1) * na]);
	//for (i = 0; i < na; i++)
		//Vt_1[i * na + na - 1] = Vt_2[(na - 1) * na + i];

	//Disp(Vt_1, na, na, "Vt");
	//Disp(Vt_2, na, na, "V_2");
	free(AtA);
	free(V_2);
	free(Vt_2);
	free(S_1);

	memcpy(Vt, Vt_1, na * na * sizeof(float));
	free(AAt);
	free(Vt_1);
	free(Ut);
	return;
}
void svd(float* A, int ma, int na, float* U, float* S, float* Vt)
{//�ӿڻ����ã��ڴ����˷�
//A = U * S * V' ������ U �� V������������ ������U���ܻ�ܴ󣬹ʴ˲����ж�������

	if (ma < na)
		svd_1(A, ma, na, U, S, Vt);
	else
		svd_2(A, ma, na, U, S, Vt);
}
//void svd(float* A, int ma, int na, float* U, float* S, float* Vt_1)
//{//��Amn��U,V �ֽ�
//	float* AAt = (float*)malloc(ma * ma * sizeof(float));
//	float* AtA = (float*)malloc(na * na * sizeof(float));
//	float fValue;
//	int x, y, i;
//
//	//�ȼ���AAt;AAt�Ǹ��Գƾ��󣬹ʴ˿��Ի���
//	for (y = 0; y < ma; y++)
//	{
//		for (x = 0; x < ma; x++)
//		{
//			//if (y == 0 && x == 1)
//				//printf("here");
//			for (fValue = 0, i = 0; i < na; i++)
//				fValue += A[y * na + i] * A[x * na + i];
//			AAt[y * ma + x] = fValue;
//		}
//	}
//
//	//�ټ���AtA��AtA�Ǹ��Գƾ��󣬹ʴ˿��Ի���
//	for (y = 0; y < na; y++)
//	{
//		for (x = 0; x < na; x++)
//		{
//			for (fValue = 0, i = 0; i < ma; i++)
//				fValue += A[y + i * na] * A[x + i * na];
//			AtA[y * na + x] = fValue;
//		}
//	}
//	//Disp(AtA, na, na,"AtA");
//	//Disp(AAt, ma, ma, "AAt");
//
//	int bSuccess;
//	float* SSt = (float*)malloc(ma * ma * sizeof(float));
//	float* StS = (float*)malloc(na * na * sizeof(float));
//	
//	memset(S, 0, ma * na * sizeof(float));
//	QR_Decompose(AAt, ma, SSt, U, &bSuccess);
//
//	//���·ֽ����ã���Ȼ�õ�������ֵ��SStһ�����ܲ�ס����������һ�������Էϵ�
//	//ԭ����ʹ��ڴ˴�
//	//QR_Decompose(AtA, na, StS, Vt_1, &bSuccess);	//����ֽ�û��
//	//Disp(Vt_1, na, na, "Incorrect V\n");
//	Disp(AAt, ma, ma, "AAt");
//
//	//��VҪ�ر���գ�
//	float* At = (float*)malloc(na * ma * sizeof(float));
//	float* Vt = (float*)malloc(na * na * sizeof(float));
//	Matrix_Transpose(A, ma, na, At);
//	Matrix_Multiply(At, na, ma, U, ma, Vt);
//	Matrix_Transpose(Vt, na, ma, Vt_1);
//	Disp(U, ma, ma, "U");
//
//	//��AtU�� ����AtΪ n*m����  UΪmxm����
//	
//	for (i = 0; i < ma; i++)
//		Normalize(&Vt_1[i * na], na, &Vt_1[i * na]);
//	Matrix_Transpose(Vt_1, na, na, Vt_1);
//	//Disp(Vt_1, na, na, "V");
//	Matrix_Transpose(Vt_1, ma, na, Vt);
//
//
//	//Disp(AAt, 3, 3, "AAt");
//	//Disp(SSt, 3, 3, "SSt");
//	//Disp(U, 3, 3, "U");
//	//Disp(AtA, 5, 5, "AtA");
//	//Disp(StS, 5, 5, "StS");
//	//Disp(Vt_1, 5, 5, "V");
//	for (i = 0; i < ma; i++)
//		S[i * na + i] = sqrt(SSt[i * ma + i]);
//	//for (i = 0; i < ma; i++)
//		//S[i * na + i] = sqrt(StS[i * na + i]);
//	
//
//	//Disp(U, ma, ma, "U");
//	//Disp(Vt_1, na, na, "V");
//	//printf("%d\n", bIs_Orthogonal((float*)V,na));
//	//Disp(SSt, ma, ma, "SSt");
//	//Disp(StS, na, na, "StS");
//	//Disp(S, ma, na, "Sigma");
//
//	//����һ��
//	float* Temp_1 = (float*)malloc(ma * na* sizeof(float));
//	Matrix_Transpose(Vt_1, na, na, Vt);
//	Matrix_Multiply(U, ma, ma, S, na, Temp_1);
//	//Disp(Temp_1, ma, na, "UxS");
//	Matrix_Multiply(Temp_1, ma, na, Vt, na, Temp_1);
//	
//	//Disp(Vt, na, na, "Vt");
//	//Disp(Temp_1, ma, na, "USVt");
//
//	memcpy(Vt_1, Vt, na * na * sizeof(float));
//
//	free(AAt);
//	free(AtA);
//	free(SSt);
//	free(StS);
//	free(Vt);
//	free(At);
//}

void Conjugate_Gradient(float* A, const int n, float B[], float X[])
{//�����ݶȷ������Է���Ax=b
#define eps 0.0001
	float* pBuffer, * r0, r0_dot_r0,	//r0.r0���ڻ�
		r1_dot_r1,	//r1.r1, �ڻ�
		* r1, * p0, * p1, * Aux, alpha, beta, fValue_0;

	int i, iCount = 0;
	pBuffer = (float*)malloc(n * 5 * sizeof(float));
	r0 = pBuffer;
	r1 = pBuffer + n;
	p0 = r1 + n;
	p1 = p0 + n;
	Aux = r0 + n;

	for (i = 0; i < n; i++)
		X[i] = 0;		//��ʼx0Ϊ[0, 0, 0]

	//p0=r0= b-Ax0
	Matrix_Multiply(A, n, n, X, 1, Aux);
	for (i = 0; i < n; i++)
		p0[i] = r0[i] = B[i] - Aux[i];

	while (1)
	{
		r0_dot_r0 = fDot(r0, r0, n);

		//ak= rk'rk / pk'A pk;
		Matrix_Multiply(p0, 1, n, A, n, Aux);
		for (fValue_0 = 0, i = 0; i < n; i++)
			fValue_0 += Aux[i] * p0[i];
		alpha = r0_dot_r0 / fValue_0;

		//xk+1= xk+ak*pk;
		for (i = 0; i < n; i++)
			X[i] += alpha * p0[i];

		//rk+1= rk-ak.A.pk
		Matrix_Multiply(A, n, n, p0, 1, Aux);
		for (r1_dot_r1 = 0, i = 0; i < n; i++)
		{
			r1[i] = r0[i] - alpha * Aux[i];
			r1_dot_r1 += r1[i] * r1[i];
		}

		fValue_0 = fDot(r1, r1, n);
		if (fValue_0 < eps)
			return;
		printf("iterate:%d %f\n", iCount, fValue_0);

		//beta= r1.r1/r0.r0
		beta = r1_dot_r1 / r0_dot_r0;

		for (i = 0; i < n; i++)
		{
			p0[i] = r1[i] + beta * p0[i];	//pk+1= r1 + beta*pk
			r0[i] = r1[i];					//r0=r1
		}
		iCount++;
	}

	free(pBuffer);
	return;
#undef eps
}




void Free_Sparse_Matrix(Sparse_Matrix* poMatrix)
{
	if (poMatrix->m_pRow)
		free(poMatrix->m_pRow);
	if (poMatrix->m_pBuffer)
		free(poMatrix->m_pBuffer + 1);
}
void Init_Sparse_Matrix(Sparse_Matrix* poMatrix, int iItem_Count, int iMax_Order)
{
	poMatrix->m_iItem_Count = iItem_Count;
	poMatrix->m_pBuffer = (Sparse_Matrix::Item*)malloc(poMatrix->m_iItem_Count * sizeof(Sparse_Matrix::Item));
	poMatrix->m_pBuffer--;	//�Ա��[1]��ʼ
	poMatrix->m_pRow = (unsigned int*)malloc(iMax_Order * 2 * sizeof(unsigned int));
	memset(poMatrix->m_pRow, 0, iMax_Order * 2 * sizeof(unsigned int));
	poMatrix->m_pCol = poMatrix->m_pRow + iMax_Order;
	poMatrix->m_iRow_Count = poMatrix->m_iCol_Count = 0;
}
void Compact_Sparse_Matrix(Sparse_Matrix* poMatrix)
{
	unsigned int* pNew_Addr = poMatrix->m_pRow + poMatrix->m_iRow_Count;
	memmove(pNew_Addr, poMatrix->m_pCol, poMatrix->m_iCol_Count * sizeof(unsigned int));
	poMatrix->m_pCol = pNew_Addr;
	poMatrix->m_pRow = (unsigned int*)realloc(poMatrix->m_pRow, (poMatrix->m_iRow_Count + poMatrix->m_iCol_Count) * sizeof(unsigned int));

	return;
}
void Disp_Link_Col(Sparse_Matrix oMatrix, int x)
{
	Sparse_Matrix::Item* poCur;
	printf("x:%d y:", x);
	if (!oMatrix.m_pCol[x])
	{
		printf("Null\n");
		return;
	}
	poCur = &oMatrix.m_pBuffer[oMatrix.m_pCol[x]];
	do
	{
		printf("%d ", poCur->y);
		poCur = poCur->m_iCol_Next ? &oMatrix.m_pBuffer[poCur->m_iCol_Next] : NULL;
	} while (poCur);
	printf("\n");
	return;
}
void Disp_Link_Row(Sparse_Matrix oMatrix, int y)
{
	Sparse_Matrix::Item* poCur;
	printf("y:%d x:", y);
	if (!oMatrix.m_pRow[y])
	{
		printf("Null\n");
		return;
	}
	poCur = &oMatrix.m_pBuffer[oMatrix.m_pRow[y]];
	do
	{
		printf("%d ", poCur->x);
		poCur = poCur->m_iRow_Next ? &oMatrix.m_pBuffer[poCur->m_iRow_Next] : NULL;
	} while (poCur);
	printf("\n");
	return;
}


void Resize_Matrix(Sparse_Matrix* poA, int iNew_Item_Count)
{
	//��С�ڴ�
	poA->m_pBuffer++;
	poA->m_pBuffer = (Sparse_Matrix::Item*)realloc(poA->m_pBuffer, iNew_Item_Count * sizeof(Sparse_Matrix::Item));
	poA->m_iItem_Count = iNew_Item_Count;
	poA->m_pBuffer--;
}
void Matrix_Multiply(Sparse_Matrix A, Sparse_Matrix B, Sparse_Matrix* poC)
{
	int y, x, i0_Count = 0, iNew_Item_Count = 0;
	float fValue;
	Sparse_Matrix::Item* poRow_Cur, * poCol_Cur, * poNew_Col_Pre = NULL, * poNew_Row_Pre = NULL, oNew_Item;
	Sparse_Matrix oC;
	//pNew_Col_Preʵ������һ��Item, ���Դ���ϴθ��е����һ��Item,�Ա�ӿ��ٶ�
	Sparse_Matrix::Item** pNew_Col_Pre = (Sparse_Matrix::Item**)malloc(B.m_iCol_Count * sizeof(Sparse_Matrix::Item*));

	if (A.m_iCol_Count != B.m_iRow_Count)
		return;
	Init_Sparse_Matrix(&oC, A.m_iRow_Count * B.m_iCol_Count, Max(A.m_iRow_Count, B.m_iCol_Count));
	for (y = 0; y < A.m_iRow_Count; y++)
	{
		for (x = 0; x < B.m_iCol_Count; x++)
		{
			//if (y == 1 && x == 0)
				//printf("Here");
			poRow_Cur = &A.m_pBuffer[A.m_pRow[y]];
			poCol_Cur = &B.m_pBuffer[B.m_pCol[x]];
			fValue = 0;
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
			if (fValue != 0)
			{//��Ŀ�����c�м�һ��Item
				oNew_Item.x = x;
				oNew_Item.y = y;
				oNew_Item.m_fValue = fValue;
				oNew_Item.m_iRow_Next = oNew_Item.m_iCol_Next = 0;
				oC.m_pBuffer[++iNew_Item_Count] = oNew_Item;
				//�ȼ���������
				if (!oC.m_pRow[y])
					oC.m_pRow[y] = iNew_Item_Count;
				else
					poNew_Row_Pre->m_iRow_Next = iNew_Item_Count;

				//�ټ���������
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

	oC.m_iItem_Count = iNew_Item_Count;
	*poC = oC;
	free(pNew_Col_Pre);
	return;
}
void Matrix_Transpose_1(Sparse_Matrix A, Sparse_Matrix* poAt)
{
	Sparse_Matrix At;
	Sparse_Matrix::Item* poCur;
	unsigned int iTemp;
	At.m_pBuffer = (Sparse_Matrix::Item*)malloc(A.m_iItem_Count * sizeof(Sparse_Matrix::Item));
	At.m_pRow = (unsigned int*)malloc((A.m_iRow_Count + A.m_iCol_Count) * sizeof(unsigned int));
	At.m_iItem_Count = A.m_iItem_Count;
	At.m_iRow_Count = A.m_iCol_Count;
	At.m_iCol_Count = A.m_iRow_Count;
	At.m_pCol = At.m_pRow + At.m_iRow_Count;
	At.m_pBuffer--;
	memcpy(At.m_pBuffer + 1, A.m_pBuffer + 1, A.m_iItem_Count * sizeof(Sparse_Matrix::Item));
	memcpy(At.m_pRow, A.m_pCol, At.m_iRow_Count * sizeof(unsigned int));
	memcpy(At.m_pCol, A.m_pRow, At.m_iCol_Count * sizeof(unsigned int));

	for (int i = 1; i <= At.m_iItem_Count; i++)
	{
		poCur = &At.m_pBuffer[i];
		//������ָ�룬��ָ��
		iTemp = poCur->m_iRow_Next;
		poCur->m_iRow_Next = poCur->m_iCol_Next;
		poCur->m_iCol_Next = iTemp;

		//�����кţ��к�
		iTemp = poCur->x;
		poCur->x = poCur->y;
		poCur->y = iTemp;
	}
	*poAt = At;
	return;
}

void Compare_Transpose(Sparse_Matrix A, Sparse_Matrix B)
{//��������ת���Ƿ��
	int x, y;
	Sparse_Matrix::Item oCur_Row, oCur_Col;
	for (y = 0; y < A.m_iRow_Count; y++)
	{
		oCur_Row = A.m_pBuffer[A.m_pRow[y]];
		x = y;
		oCur_Col = B.m_pBuffer[B.m_pCol[x]];
		while (1)
		{
			if (oCur_Row.m_fValue != oCur_Col.m_fValue || oCur_Row.y != oCur_Col.x ||
				oCur_Row.m_iCol_Next != oCur_Col.m_iRow_Next || oCur_Row.m_iRow_Next != oCur_Col.m_iCol_Next)
			{
				printf("err");
				break;
			}
			if (oCur_Row.m_iRow_Next)
				oCur_Row = A.m_pBuffer[oCur_Row.m_iRow_Next];
			else
				break;
			if (oCur_Col.m_iCol_Next)
				oCur_Col = B.m_pBuffer[oCur_Col.m_iCol_Next];
			else
				break;
		}
	}
	return;
}

void Matrix_Add(Sparse_Matrix* poA, float a, Sparse_Matrix* poB, float b, float* pC)
{
	Sparse_Matrix A, B;
	Sparse_Matrix::Item oItem;
	float* pCur;
	int y;
	A = *poA;
	B = *poB;
	memset(pC, 0, A.m_iRow_Count * A.m_iCol_Count * sizeof(float));
	for (y = 0; y < A.m_iRow_Count; y++)
	{
		if (A.m_pRow[y])
		{
			oItem = A.m_pBuffer[A.m_pRow[y]];
			pCur = &pC[oItem.y * A.m_iCol_Count];
			while (1)
			{
				pCur[oItem.x] = a * oItem.m_fValue;
				if (oItem.m_iRow_Next)
					oItem = A.m_pBuffer[oItem.m_iRow_Next];
				else
					break;
			}
		}

		if (B.m_pRow[y])
		{
			oItem = B.m_pBuffer[B.m_pRow[y]];
			pCur = &pC[oItem.y * B.m_iCol_Count];
			while (1)
			{
				pCur[oItem.x] = b * oItem.m_fValue;
				if (oItem.m_iRow_Next)
					oItem = B.m_pBuffer[oItem.m_iRow_Next];
				else
					break;
			}
		}
	}
	return;
}
void Matrix_Add(float A[], float B[], int iOrder, float C[])
{//����ӷ����˴��Ƿ���
	int i;
	for (i = 0; i < iOrder * iOrder; i++)
		C[i] = A[i] + B[i];
	return;
}
void Matrix_Add(Sparse_Matrix* poA, float a, Sparse_Matrix* poB, float b)
{//��������ӣ� aA+bB=>B�� �������B��
	Sparse_Matrix A = *poA, B = *poB;
	//pNew_Col_Preʵ������һ��Item, ���Դ���ϴθ��е����һ��Item,�Ա�ӿ��ٶ�
	Sparse_Matrix::Item** pNew_Col_Pre = (Sparse_Matrix::Item**)malloc(B.m_iCol_Count * sizeof(Sparse_Matrix::Item*));
	Sparse_Matrix::Item oNew, * poNew, * poItem_A, * poItem_B, * poRow_Pre_B, oHead = { 0 };
	int y/*,iItem_Count=B.m_iItem_Count*/, bAdd;
	memset(pNew_Col_Pre, 0, B.m_iCol_Count * sizeof(Sparse_Matrix::Item*));
	for (y = 0; y < A.m_iRow_Count; y++)
	{
		//if (y == 114)
			//printf("here");
		poItem_A = &A.m_pBuffer[A.m_pRow[y]];
		poRow_Pre_B = poItem_B = &B.m_pBuffer[B.m_pRow[y]];

		while (1)
		{
			bAdd = 0;
			//if (y == 32 && poItem_A->x == 131)
				//printf("Here");
			//poItem_A->m_fValue *= a;
			//poItem_B->m_fValue *= b;
			if (poItem_A->x == poItem_B->x)
			{//���������
				poItem_B->m_fValue = a * poItem_A->m_fValue + b * poItem_B->m_fValue;
				if (poItem_A->m_iRow_Next)
					poItem_A = &A.m_pBuffer[poItem_A->m_iRow_Next];
				else
					break;
				poRow_Pre_B = poItem_B;
				if (poItem_B->m_iRow_Next)
					poItem_B = &B.m_pBuffer[poItem_B->m_iRow_Next];
			}
			else if (poItem_A->x < poItem_B->x)
			{//��new_Item=aA[]���뵽B��������
				oNew = *poItem_A;
				oNew.m_iRow_Next = (int)(poItem_B - B.m_pBuffer);
				B.m_pBuffer[++B.m_iItem_Count] = oNew;
				bAdd = 1;
				if (poRow_Pre_B != poItem_B)
				{//������ͷ
					poRow_Pre_B->m_iRow_Next = B.m_iItem_Count;
				}
				else
				{//����ͷ
					B.m_pRow[y] = B.m_iItem_Count;
				}
				poRow_Pre_B = &B.m_pBuffer[B.m_iItem_Count];
				//poItem_A���ƽ�
				if (!poItem_A->m_iRow_Next)
					break;
				else
					poItem_A = &A.m_pBuffer[poItem_A->m_iRow_Next];
			}
			else
			{//poItem_A->x > poItem_B->x, poItem_B����ƽ�һ��
				poRow_Pre_B = poItem_B;
				if (!poItem_B->m_iRow_Next)
				{//��Ҫ������Ԫ��
					oNew = *poItem_A;
					oNew.m_iRow_Next = 0;
					bAdd = 1;
					B.m_pBuffer[++B.m_iItem_Count] = oNew;
					poItem_B->m_iRow_Next = B.m_iItem_Count;
					poItem_B = &B.m_pBuffer[B.m_iItem_Count];
					if (poItem_A->m_iRow_Next)
						poItem_A = &A.m_pBuffer[poItem_A->m_iRow_Next];
					else
						break;
				}
				else
				{
					poItem_B = &B.m_pBuffer[poItem_B->m_iRow_Next];
				}

			}
			if (bAdd)
			{//�Ѿ����룬�ٸ��з���
				Sparse_Matrix::Item* poCur, * poPrevious;
				poNew = &B.m_pBuffer[B.m_iItem_Count];
				if (!pNew_Col_Pre[oNew.x])
				{
					if (!(B.m_pCol[oNew.x]))
					{	//Ͱ��û��ָ��ֱ�Ӳ���
						B.m_pCol[oNew.x] = B.m_iItem_Count;
					}
					else {
						poCur = &B.m_pBuffer[B.m_pCol[oNew.x]];
						poPrevious = NULL;
						while (poCur->y < oNew.y && poCur->m_iCol_Next)
						{
							poPrevious = poCur;
							poCur = &B.m_pBuffer[poCur->m_iCol_Next];
						}
						if (poCur->y < oNew.y)
						{//�嵽ǰ��
							poNew->m_iCol_Next = poCur->m_iCol_Next;
							poCur->m_iCol_Next = B.m_iItem_Count;
						}
						else
						{
							if (poPrevious)
							{//����previous��cur֮��
								poNew->m_iCol_Next = poPrevious->m_iCol_Next;
								poPrevious->m_iCol_Next = B.m_iItem_Count;
							}
							else
							{//�鵽����ͷ
								poNew->m_iCol_Next = B.m_pCol[oNew.x];
								B.m_pCol[oNew.x] = B.m_iItem_Count;
							}
						}
					}
				}
				else
				{//һֱ�����ң�ֱ���ҵ�����λ��
					poCur = pNew_Col_Pre[oNew.x];
					poPrevious = NULL;

					while (poCur->y < oNew.y && poCur->m_iCol_Next)
					{
						poPrevious = poCur;
						poCur = &B.m_pBuffer[poCur->m_iCol_Next];
					}
					if (poCur->y < oNew.y)
					{//�嵽poCur֮��
						poNew->m_iCol_Next = poCur->m_iCol_Next;
						poCur->m_iCol_Next = B.m_iItem_Count;
					}
					else
					{
						if (poPrevious)
						{//����previous��cur֮��
							poNew->m_iCol_Next = poPrevious->m_iCol_Next;
							poPrevious->m_iCol_Next = B.m_iItem_Count;
						}
					}
				}
				pNew_Col_Pre[oNew.x] = &B.m_pBuffer[B.m_iItem_Count];
			}
			//if (B.m_pRow[0] != 1)
				//printf("here");
		}
	}

	free(pNew_Col_Pre);
	poB->m_iItem_Count = B.m_iItem_Count;
	return;
}
void Solve_abc(float x0, float x1, float x2, float y0, float y1, float y2, float abc[3])
{//�����ֵ�㷨
	float x0_Square = x0 * x0, x1_Square = x1 * x1, x2_Square = x2 * x2;
	float a, b, c;
	float fValue_1 = x1_Square * x2 - x1 * x2_Square - x0_Square * x1 + x0 * x1 * x2;
	a = (y1 * x2 - y2 * x1) / fValue_1;
	b = y0 / x0 - (y2 * x2 * x0 - y2 * x1 * x2) / fValue_1 - (y0 * x1 - y1 * x0) / (x0 * x1 - x0_Square) - (x1 * (y1 * x2 - y2 * x1)) / fValue_1;
	c = (y0 * x1 - y1 * x0) / (x1 - x0) + x0 * x1 * (y0 * x2 - y2 * x1) / fValue_1;
	abc[0] = a;
	abc[1] = b;
	abc[2] = c;
}

unsigned int iFactorial(int n)
{//��n��ȫ����,�׳�
	unsigned int iResult = 1;
	if (n == 0)
		return 1;

	for (int i = 2; i <= n; i++)
		iResult *= i;
	return iResult;
}
unsigned int iGet_Perm(int n, int m)
{//��n��ȡm��������= n!/(n-m)! mСn��
	if (m > n)
		return 0;
	int iEnd = n - m + 1;
	unsigned int iResult = 1;
	while (n >= iEnd)
		iResult *= n--;
	return iResult;
}
unsigned int iGet_Combination(int n, int m)
{//������� n��ȡm = p(n,m)/m!
	if (m > n)
		return 0;
	return iGet_Perm(n, m) / iFactorial(m);
}

float fGet_Tr(float M[], int iOrder)
{//��N�׷���ļ������Խ���֮��
	int i;
	float fTotal = 0;
	for (i = 0; i < iOrder; i++)
		fTotal += M[i * iOrder + i];
	return fTotal;
}
void Vee(float M[], float V[3])
{//���Գƾ�������
	V[0] = M[7];
	V[1] = M[2];
	V[2] = M[3];
	return;
}
void Hat(float V[], float M[])
{//���ݸ������������췴�Գƾ��󣬸Ļ�������һ��
	M[0] = M[4] = M[8] = 0;
	M[1] = -V[2], M[3] = V[2];
	M[2] = V[1], M[6] = -V[1];
	M[5] = -V[0], M[7] = V[0];
	return;
}
void Rotation_Matrix_2_Vector(float R[], float V[])
{//����ת������ת�������ǽ� Rn=n������n���Ǵ����ת�ᣬ��Ȼ����ֵΪ1�� �����������
	//��������ֵ=1�� ����(A-rI)x=0, ���x����������������r=1,���Խ�(A-I)x=0����
	float I[3][3] = { 1,0,0,0,1,0,0,0,1 };
	double R_1[3][3], B[3] = { 0 }, V_1[3];
	int y, x, iPos;
	//R_1=R-I
	for (iPos = 0, y = 0; y < 3; y++)
		for (x = 0; x < 3; x++, iPos++)
			R_1[y][x] = R[iPos];
	//R_1[y][x] = y == x ? R[iPos] - 1.f: R[iPos];

//Ȼ�����η��� R_1x=0
	Solve_Homo_3x3(R_1, 1, V_1);
	V[0] = (float)V_1[0];
	V[1] = (float)V_1[1];
	V[2] = (float)V_1[2];

	//������ת�Ƕ�
	float fTr = fGet_Tr(R, 3);
	V[3] = acos((fTr - 1.f) / 2.f);

	return;
}
void Rotation_Vector_2_Quaternion(float V[4], float Q[4])
{//��ת����ת��Ϊ��Ԫ�飬�������ն��壬һ����ת������һ����׼��������Ϊ��ת����һ����ת�Ƕȹ���
	float fSin_Theta_Div_2;
	float V_1[3];
	float fTheta = V[3];
	Normalize(V, 3, V_1);
	Q[0] = cos(fTheta / 2.f);
	fSin_Theta_Div_2 = sin(fTheta / 2.f);
	Q[1] = V_1[0] * fSin_Theta_Div_2;
	Q[2] = V_1[1] * fSin_Theta_Div_2;
	Q[3] = V_1[2] * fSin_Theta_Div_2;
	return;
}
void Rotation_Matrix_2_Quaternion(float R[], float Q[])
{//��ת������Ԫ�顣�˴�����ȫʵ�֣�����ȱ��ֱ���㷨���ʴ˿�һ����ת������Ϊ�м��̵��ڹ�ȥ
	float V[4];
	Rotation_Matrix_2_Vector(R, V);
	Rotation_Vector_2_Quaternion(V, Q);
	return;
}
void Rotation_Vector_2_Matrix_3D(float V[4], float R[3 * 3])
{//��ת��������ת������һ�ѿ���׼��׼, 3���Ѿ�һ��һ��
//������ǰ��������Ϊ��׼����ת�ᣬ���һ������Ϊ��ת��
	float fCos_Theta, fSin_Theta;
	float fTheta = V[3];
	float V_1[3];
	//fTheta��Ҫ������ģ��
	fCos_Theta = cos(fTheta);
	fSin_Theta = sin(fTheta);
	float nnt[3][3], I[3][3] = { {1,0,0},{0,1,0},{0,0,1} };
	float Skew_Sym[3][3];

	//������������һ��
	Normalize(V, 3, V_1);

	Matrix_Multiply(V_1, 3, 1, V_1, 3, (float*)nnt);
	Scale_Matrix_1(I, fCos_Theta);
	Scale_Matrix_1(nnt, (1 - fCos_Theta));
	Hat(V_1, (float*)Skew_Sym);
	Scale_Matrix_1(Skew_Sym, fSin_Theta);

	Matrix_Add((float*)I, (float*)nnt, 3, (float*)R);
	Matrix_Add((float*)R, (float*)Skew_Sym, 3, (float*)R);

	return;
}


void Quaternion_Add(float Q_1[], float Q_2[], float Q_3[])
{
	for (int i = 0; i < 4; i++)
		Q_3[i] = Q_1[i] + Q_2[i];
	return;
}
void Quaternion_Minus(float Q_1[], float Q_2[], float Q_3[])
{
	for (int i = 0; i < 4; i++)
		Q_3[i] = Q_1[i] - Q_2[i];
	return;
}
void Quaternion_Conj(float Q_1[], float Q_2[])
{//���������
	Q_2[0] = Q_1[0];
	Q_2[1] = -Q_1[1];
	Q_2[2] = -Q_1[2];
	Q_2[3] = -Q_1[3];
}
void Quaternion_Multiply(float Q_1[], float Q_2[], float Q_3[])
{//�˷��Ȳ��ǵ��Ҳ������������䶨��
	Q_3[0] = Q_1[0] * Q_2[0] - Q_1[1] * Q_2[1] - Q_1[2] * Q_2[2] - Q_1[3] * Q_2[3];
	Q_3[1] = Q_1[0] * Q_2[1] + Q_1[1] * Q_2[0] + Q_1[2] * Q_2[3] - Q_1[3] * Q_2[2];
	Q_3[2] = Q_1[0] * Q_2[2] - Q_1[1] * Q_2[3] + Q_1[2] * Q_2[0] + Q_1[3] * Q_2[1];
	Q_3[3] = Q_1[0] * Q_2[3] + Q_1[1] * Q_2[2] - Q_1[2] * Q_2[1] + Q_1[3] * Q_2[0];
	return;
}
void Quaternion_Inv(float Q_1[], float Q_2[])
{//����Ԫ������
	float fMod = fGet_Mod(Q_1, 4);
	int i;
	Quaternion_Conj(Q_1, Q_2);
	fMod *= fMod;
	for (i = 0; i < 4; i++)
		Q_2[i] /= fMod;
	return;
}
void Quaternion_2_Rotation_Matrix(float Q[4], float R[])
{//��Ԫ��ת��Ϊ��ת���� R= vv' + s^2*I + 2sv^ + (v^)^2
	float fValue, M_2[3][3], M_1[3][3] = { 1,0,0,0,1,0,0,0,1 };	//��ʱ����	
	//�����vv' ֱ�ӷ�R����
	Matrix_Multiply(&Q[1], 3, 1, &Q[1], 3, R);

	//���� s^2*I
	fValue = Q[0] * Q[0];
	Scale_Matrix_1(M_1, fValue);
	Matrix_Add(R, (float*)M_1, 3, R);

	//����2sv
	Hat(&Q[1], (float*)M_1);
	fValue = 2.f * Q[0];
	memcpy(M_2, M_1, 3 * 3 * sizeof(float));
	Scale_Matrix_1(M_2, fValue);
	Matrix_Add(R, (float*)M_2, 3, R);

	//����(v^) ^ 2�������Ѿ��㶨��M_1= v^
	Matrix_Multiply((float*)M_1, 3, 3, (float*)M_1, 3, (float*)M_2);
	Matrix_Add(R, (float*)M_2, 3, R);

	//Disp((float*)R, 3, 3);
	return;
}
void Quaternion_2_Rotation_Vector(float Q[4], float V[4])
{//����Ԫ��ת��Ϊ��ת����
	float fSin_Theta_Div_2;
	V[3] = 2.f * acos(Q[0]);
	fSin_Theta_Div_2 = sin(V[3] / 2.f);
	V[0] = Q[1] / fSin_Theta_Div_2;
	V[1] = Q[2] / fSin_Theta_Div_2;
	V[2] = Q[3] / fSin_Theta_Div_2;
	return;
}
void Gen_Homo_Matrix(float R[], float t[], float s, float M[])
{//����һ��SIM3��������ת��ƽ�ƣ����Ź���
	int y, x;
	for (y = 0; y < 3; y++)
		for (x = 0; x < 3; x++)
			M[y * 4 + x] = s * R[y * 3 + x];
	M[15] = 1;

	M[3] = t[0];
	M[7] = t[1];
	M[11] = t[2];
	M[12] = M[13] = M[14] = 0;
	return;
}
void Gen_Homo_Matrix(float R[], float t[], float M[])
{//����ת������λ�����깹��һ����α任���󣬴˴�����ת��ƽ�ƹ���
	int y, x;
	for (y = 0; y < 3; y++)
		for (x = 0; x < 3; x++)
			M[y * 4 + x] = R[y * 3 + x];
	M[15] = 1;

	////�˴������һ����ֵ,�е�����
	//M[3] = M[7] = M[11] = 0;
	//M[12] = T[0];
	//M[13] = T[1];
	//M[14] = T[2];

	if (t)
	{
		M[3] = t[0];
		M[7] = t[1];
		M[11] = t[2];
	}
	else
		M[3] = M[7] = M[11] = 0;

	M[12] = M[13] = M[14] = 0;

	return;
}
void Roataion_Vector_2_Angle_Axis(float V_4[4], float V_3[3])
{
	float V_3_1[3], fTheta = V_4[3];
	//��4ά��ת��������Ϊ3ά�����᱾��Ϊ��λ���������Ƶ�ģ��Ϊ�Ƕ�
	//�ܼ򵥣���ÿ������*theta����
	V_3_1[0] = V_4[0] * fTheta;
	V_3_1[1] = V_4[1] * fTheta;
	V_3_1[2] = V_4[2] * fTheta;
	V_3[0] = V_3_1[0];
	V_3[1] = V_3_1[1];
	V_3[2] = V_3_1[2];
	return;
}
void Ksi_Get_J(float Rotation_Vector[4], float J[])
{//��������ת���������J����
	//����λ��t,�������J��aΪ��ת������ת��
	float fValue, Temp_1[3][3], I[3][3] = { 1,0,0,0,1,0,0,0,1 };
	int i;
	//����J��һ���� (sin(theta)/theta)*I
	fValue = sin(Rotation_Vector[3]) / Rotation_Vector[3];
	for (i = 0; i < 9; i++)
		((float*)J)[i] = fValue * ((float*)I)[i];

	//����J�ڶ����� (1-sin(theta)/theta) * axa'
	fValue = 1.f - fValue;
	Matrix_Multiply(Rotation_Vector, 3, 1, Rotation_Vector, 3, (float*)Temp_1);
	//Disp((float*)Temp_1, 3, 3);
	for (i = 0; i < 9; i++)
		((float*)J)[i] += fValue * ((float*)Temp_1)[i];

	//����������� (1-cos(theta))/theta * a^
	fValue = (1 - cos(Rotation_Vector[3])) / Rotation_Vector[3];
	Hat(Rotation_Vector, (float*)Temp_1);
	//Disp((float*)Temp_1, 3, 3);
	for (i = 0; i < 9; i++)
		((float*)J)[i] += fValue * ((float*)Temp_1)[i];
	//Disp((float*)J, 3, 3);
}
void SE3_2_se3(float Rotation_Vector[4], float t[3], float Ksi[6])
{//��һ����ת������һ��λ��ת��Ϊse3�ϵ�Ksi
//������SE3��һ��4x4���󣬰�����ת��λ�ơ� SE3->se3����4x4����ת��Ϊ6ά����
//Ȼ������ת������ʾ��ת��ֻ�����࣬�ʴ˴˴�����ת������λ������
//�ܽᣬ SE3�е���ת����se3�е���ά��������������ת��λ��
	float J[3][3], J_Inv[3][3];
	int bResult;
	Ksi_Get_J(Rotation_Vector, (float*)J);

	//��J�����
	Get_Inv_Matrix_Row_Op((float*)J, (float*)J_Inv, 3, &bResult);
	Matrix_Multiply((float*)J_Inv, 3, 3, t, 1, Ksi);

	//�ٰ�PhiҲ����һ�£���ʵ����Rotation_Vector 4άת��ά
	Roataion_Vector_2_Angle_Axis(Rotation_Vector, &Ksi[3]);
	Disp(Ksi, 1, 6);	//Rho�������

	return;
}
void se3_2_SE3(float Ksi[6], float T[])
{//T��6ά se3����Ksi��Ӧ��4x4����, Ksiǰrho��phi
//ת����Ϻ�T��ȫ��ͼ��ѧ����άת������һ��
//�ܽᣬ SE3�е���ת����se3�е���ά��������������ת��λ��

	//������R
	float R[3][3];
	float Rotation_Vector[4];

	Normalize(&Ksi[3], 3, Rotation_Vector);
	Rotation_Vector[3] = fGet_Mod(&Ksi[3], 3);	//�˴��Ѿ�����ת������Ϊ4ά��ʾ

	Rotation_Vector_2_Matrix_3D(Rotation_Vector, (float*)R);
	//Disp((float*)R, 3, 3,"R");

	float J[3][3], J_Rho[3];
	Ksi_Get_J(Rotation_Vector, (float*)J);
	Matrix_Multiply((float*)J, 3, 3, Ksi, 1, J_Rho);

	//Ȼ�� R,J_Rho, 0', 1��ϳ�T
	T[0] = R[0][0], T[1] = R[0][1], T[2] = R[0][2], T[3] = J_Rho[0];
	T[4] = R[1][0], T[5] = R[1][1], T[6] = R[1][2], T[7] = J_Rho[1];
	T[8] = R[2][0], T[9] = R[2][1], T[10] = R[2][2], T[11] = J_Rho[2];
	T[12] = T[13] = T[14] = 0, T[15] = 1;

	return;
}
void Gen_Iterate_Matrix(float A[], int n, float B[], float B_1[], float C[])
{//��Ax=B ��д�� x= Bx+C��BΪ��������cΪϵ������
	//�ܼ򵥣�����i��/aii����
	int y, x, iPos, iRow_Size = n + 1;
	float aii;
	//�ɴ��B,C�ո������������
	float* pB_C = (float*)malloc(n * (n + 1) * sizeof(float));
	//Disp(A, n, n, "\n");

	for (iPos = 0, y = 0; y < n; y++)
	{
		aii = 1.f / A[y * n + y];
		for (x = 0; x < n; x++, iPos++)
			pB_C[y * iRow_Size + x] = y == x ? 0.f : A[iPos] * aii;
		pB_C[y * iRow_Size + n] = B[y] * aii;
		//Disp(pB_C, n, iRow_Size, "\n");
	}
	for (iPos = y = 0; y < n; y++)
	{
		for (x = 0; x < n; x++, iPos++)
			B_1[iPos] = pB_C[y * iRow_Size + x];
		C[y] = pB_C[y * iRow_Size + n];
	}
	free(pB_C);
	return;
}
int bIs_Contrative_Mapping(float B[], int n)
{//����һ����������ͨ���������ļ��ֶ����ж����Ƿ�ѹ������
	float fF, fRow_Max = -1.f, //�ж���
		fCol_Max = -1.f,		//�ж���
		fTotal;
	int y, x, iPos;
	fF = 0;	//F����
	for (iPos = y = 0; y < n; y++)
	{
		fTotal = 0;
		for (x = 0; x < n; x++, iPos++)
		{
			fTotal += abs(B[iPos]);
			fF += B[iPos] * B[iPos];
		}
		if (fTotal > fRow_Max)
			fRow_Max = fTotal;

	}
	fF = sqrt(fF);

	for (x = 0; x < n; x++)
	{
		fTotal = 0;
		for (y = 0; y < n; y++)
			fTotal += B[y * n + x];
		if (fTotal > fCol_Max)
			fCol_Max = fTotal;
	}
	if (fRow_Max < 1.f || fCol_Max < 1.f || fF < 1.f)
		return 1;	//������������ֻҪ��һ��С��1��BΪѹ��ӳ��
	else
		return 0;
}

int bIs_Diagonal_Dominant(float A[], int iOrder)
{//�ж�һ�������Ƿ�Ϊ�Խ���ռ�Ŷ�������ռ�ž���
//�ж϶Խ���ռ�ź�������ȸ��죬���Եÿ���������취ȡ���ж����ȵķ���
	int y, x;
	float* pCur_Line, fSum;
	pCur_Line = A;
	for (y = 0; y < iOrder; y++, pCur_Line += iOrder)
	{
		for (fSum = 0, x = 0; x < iOrder; x++)
			if (y != x)
				fSum += abs(pCur_Line[x]);
		if (fSum > abs(pCur_Line[y]))
			return 0;
	}
	return 1;
}
void Solve_Linear_Gauss_Seidel(float A[], float B[], int iOrder, float X[], int* pbResult)
{//���ſɱȵ�����һ����ˣ���˼��������ϼ�������Ͷ����һ�����㣬����ȥ�ܺã�ʵ���ϲ���
//���ſɱȵ������������٣���Ϊ��ʹ�����ſɱ�����Ҳ�����Ҳ���ܷ�ɢ�����������ٶ�Ҳ�����ÿ���
#define ITERATE_COUNT 200
	int i;
	float* pBuffer = (float*)malloc((iOrder * iOrder + iOrder * 3) * sizeof(float));
	float* J = pBuffer;
	float* C = J + iOrder * iOrder;
	float* Xk_1 = C + iOrder;
	float* Temp_1 = Xk_1 + iOrder;
	float fValue;

	*pbResult = 0;
	//J��������ѹ������
	Gen_Iterate_Matrix(A, iOrder, B, J, C);
	if (!pBuffer)
		goto END;
	for (i = 0; i < iOrder; i++)
		Xk_1[i] = X[i] = 0;
	int y, x;
	for (i = 0; i < ITERATE_COUNT; i++)
	{
		for (y = 0; y < iOrder; y++)
		{
			for (fValue = C[y], x = 0; x < iOrder; x++)
				fValue += J[y * iOrder + x] * X[x];
			X[y] = fValue;	//ÿһ�����ϳ�Ϊ�⣬�˴��ǹؼ�
		}
		Disp(X, 1, 3, "Xk");
		if ((fValue = fGet_Distance(X, Xk_1, 3)) < 0.00001)
		{
			*pbResult = 1;
			goto END;
		}
		memcpy(Xk_1, X, 3 * sizeof(float));
	}
	//�ʹ�һ����������ſɱ��ţ��������ȷ
END:
	if (pBuffer)
		free(pBuffer);
#undef ITERATE_COUNT
}

void Solve_Linear_Jocabi(float A[], float B[], int iOrder, float X[], int* pbResult)
{//�ſɱȵ����������Է����飬ͬ���о������⣬�����ⲻ��
#define ITERATE_COUNT 200
	int i;
	float* pBuffer = (float*)malloc((iOrder * iOrder + iOrder * 3) * sizeof(float));
	float* J = pBuffer;
	float* C = J + iOrder * iOrder;
	float* Xk_1 = C + iOrder;
	float* Temp_1 = Xk_1 + iOrder;
	float* Xk = X;
	float fValue;

	*pbResult = 0;
	//J��������ѹ������
	Gen_Iterate_Matrix(A, iOrder, B, J, C);

	if (Xk_1)
		for (i = 0; i < iOrder; i++)
			Xk_1[i] = 0;
	for (i = 0; i < ITERATE_COUNT; i++)
	{
		Matrix_Multiply((float*)J, 3, 3, Xk_1, 1, Temp_1);
		Vector_Add(Temp_1, C, 3, Xk);

		Disp(Xk, 1, 3, "Xk");
		//������������飬����Ϊ����ԭʽ�ȽϽ������
		if ((fValue = fGet_Distance(Xk, Xk_1, 3)) < 0.00001)
		{
			*pbResult = 1;
			goto END;
		}
		//memcpy(Xk_1, Xk, 3 * sizeof(float));
		std::swap(Xk, Xk_1);
	}

	//������������ԭ�����ַ�����һ�ֲ�����������һ�����װ뾶��һ����ϵ�������Ƿ�Խ�ռ��
	if (!bIs_Contrative_Mapping(J, iOrder))
		printf("J������ѹ������\n");
	if (!bIs_Diagonal_Dominant(A, iOrder))
		printf("ϵ������ǶԽ�ռ��\n");
	//�װ뾶���ڻ�û�㶨QR�ֽ�ĸ�����ʾ����
END:
	if (Xk != X)
		memcpy(X, Xk, 3 * sizeof(float));
	free(pBuffer);

#undef  ITERATE_COUNT
}
void SIM3_Get_Js(float s, float sigma, float theta, float Phi[], float* Js)
{
	float Temp_1[3][3], Temp_2[3][3], I[3][3] = { 1,0,0,0,1,0,0,0,1 };
	float fValue;
	float hat[3][3];
	float a;	//=s*sin(theta)
	float b;	//=s*cos(theta)
	float c;	//=theta*theta + sigma*sigma
	float C;	//=(s-1)/sigma
	int i;

	a = s * sin(theta);
	b = s * cos(theta);
	c = theta * theta + sigma * sigma;

	C = (s - 1) / sigma;	//����
	for (i = 0; i < 9; i++)
		((float*)Temp_1)[i] = ((float*)I)[i] * C;	//����

	//ע�⣬�˴�����a^�� ������phi^
	//����и����ϣ�����ʹ
	//fValue = (sigma * s * sin(theta) + (1 - s * cos(theta)) * theta) / (sigma * sigma + theta * theta);
	fValue = (a * sigma + (1 - b) * theta) / (theta * c);
	Hat(Phi, (float*)hat);
	for (i = 0; i < 9; i++)
		((float*)Temp_1)[i] += fValue * ((float*)hat)[i];

	//fValue = (s - 1) / sigma - ((s * cos(theta) - 1) * sigma + s * sin(theta) * theta) / (sigma * sigma + theta * theta);
	fValue = (C - ((b - 1) * sigma + a * theta) / c) / (theta * theta);
	Matrix_Multiply((float*)hat, 3, 3, (float*)hat, 3, (float*)Temp_2);
	for (i = 0; i < 9; i++)
		((float*)Temp_2)[i] *= fValue;
	Matrix_Add((float*)Temp_1, (float*)Temp_2, 3, Js);
	return;
}
void sim3_2_SIM3(float zeta[7], float Rotation_Vector[4], float t[], float* ps)
{//��7άsim3������ԭ����ת������λ����������������s
//�ܽᣬsim3�а������ֱ任������׼ȷ˵���任��Ҫ����˳��R->s->t
	float s, theta, sigma;
	float Js[3][3];
	//��ת����ûʲô�ø��,��zeta����һ��
	Normalize(&zeta[3], 3, Rotation_Vector);
	Rotation_Vector[3] = theta = Rotation_Vector[3];
	sigma = zeta[6];
	//s����zigma��

	*ps = s = exp(zeta[6]);

	//���Ѹ���t, �����Js����
	SIM3_Get_Js(s, sigma, theta, &zeta[3], (float*)Js);
	Disp((float*)Js, 3, 3, "Js");
	Matrix_Multiply((float*)Js, 3, 3, zeta, 1, t);

	return;
}
void SIM3_2_sim3(float Rotation_Vector[], float t[], float s, float zeta[7])
{//����SIM3Ӧ����һ��4x4����Ȼ�������4x4����ʵ������R,t,s���ɡ�
//����sΪ����ϵ����ת��Ϊһ��7ά�������ֱ�λ��rho,  ��תphi��sigma��Ӧs
//�ܽᣬsim3�а������ֱ任������׼ȷ˵���任��Ҫ����˳��R->s->t

	//s=e^sigma�������sigma
	float sigma = log(s);
	float rho[3];	//t= Js * rho. �Ӵ˴����Ƴ�rho
	float Js[3][3], Js_Inv[3][3];
	float Phi[3];
	int bResult;

	Roataion_Vector_2_Angle_Axis(Rotation_Vector, Phi);

	SIM3_Get_Js(s, sigma, Rotation_Vector[3], Phi, (float*)Js);
	Disp((float*)Js, 3, 3, "Js");
	//�����þ�������ķ���Ҳûë������������Ӧ������Sophus����ѧ�ķ�����ֱ��
	//Ӧ�������㷨
	Get_Inv_Matrix_Row_Op((float*)Js, (float*)Js_Inv, 3, &bResult);

	//Disp((float*)Js_Inv, 3, 3, "Js");
	Matrix_Multiply((float*)Js_Inv, 3, 3, t, 1, rho);
	zeta[0] = rho[0];
	zeta[1] = rho[1];
	zeta[2] = rho[2];
	zeta[3] = Phi[0];
	zeta[4] = Phi[1];
	zeta[5] = Phi[2];
	zeta[6] = sigma;
	Disp(zeta, 1, 7, "zeta");
	return;
}

//��������һ�η���任��ȫ�����б任��һ·���
void Gen_Rotation_Matrix(float Axis[3], float fTheta, float T[])
{//������ת������ת�Ƕ�����һ����ת���󣬴˴�T��4x4����
//��Rotation_Vector_2_Matrix_3D�ȼۣ����ǿ϶���ܶ࣬��Ϊ����Ҫ�漰����˷�

	float V_1[3];	//��񻯷�������
	float fSin_Theta = sin(fTheta),
		fCos_Theta = cos(fTheta),
		f1_Cos_Theta = 1 - fCos_Theta;
	float fPart_1, fPart_2;

	Normalize(Axis, 3, V_1);

	memset(T, 0, 4 * 4 * sizeof(float));
	T[0] = fCos_Theta + f1_Cos_Theta * V_1[0] * V_1[0];
	T[5] = fCos_Theta + f1_Cos_Theta * V_1[1] * V_1[1];
	T[10] = fCos_Theta + f1_Cos_Theta * V_1[2] * V_1[2];
	T[15] = 1;

	//���öԳ��Լ��㣬Ҳ����
	fPart_1 = f1_Cos_Theta * V_1[0] * V_1[1];
	fPart_2 = fSin_Theta * V_1[2];
	T[1] = fPart_1 - fPart_2;
	T[4] = fPart_1 + fPart_2;

	fPart_1 = f1_Cos_Theta * V_1[0] * V_1[2];
	fPart_2 = fSin_Theta * V_1[1];
	T[2] = fPart_1 + fPart_2;
	T[8] = fPart_1 - fPart_2;

	fPart_1 = f1_Cos_Theta * V_1[1] * V_1[2];
	fPart_2 = fSin_Theta * V_1[0];
	T[6] = fPart_1 - fPart_2;
	T[9] = fPart_1 + fPart_2;
	return;
}
void Gen_Translation_Matrix(float Offset[3], float T[])
{//��ʾһ��λ��
	memset(T, 0, 4 * 4 * sizeof(float));
	T[0] = T[5] = T[10] = T[15] = 1;
	T[3] = Offset[0];
	T[7] = Offset[1];
	T[11] = Offset[2];
	return;
}
void Gen_Scale_Matrix(float Scale[3], float T[])
{//���������ϵı����任
	memset(T, 0, 4 * 4 * sizeof(float));
	T[0] = Scale[0];	//Scale_x
	T[5] = Scale[1];	//Scale_y
	T[10] = Scale[2];	//Scale_z
	T[15] = 1;
}
void Rect_2_Polor_Cordinate(float x, float y, float* prho, float* ptheta)
{//2d�µ�ֱ�����굽�����꣬�Ѿ��Ƚ�׼ȷ
	*prho = sqrt(x * x + y * y);
	if (*prho == 0)		//��r=0ʱ��ת���Ѿ�û�����壬������Ϊ0
		*ptheta = 0;
	else if (y > 0)
		*ptheta = acos(x / (*prho));
	else
		*ptheta = -acos(x / (*prho));
	return;
}
void Rect_2_Polor_Coordinate(float x, float y, float z, float* prho, float* ptheta, float* pphi)
{//��ֱ������ϵת��Ϊ�������ʾ���˴���δ���ã����ڸ����������⣬�����ٵ�
//�˴�δ���ƣ���δ��������
	*prho = sqrt(x * x + y * y + z * z);
	*pphi = acos(z / (*prho));
	float theta = atan(y / x);
	return;
}
void Screen_2_Coordinate(int x_Screen, int y_Screen, float* px, float* py, int iWidth, int iHeight)
{
	int iWidth_Half = iWidth >> 1,
		iHeight_Half = iHeight >> 1;
	*px = (float)(x_Screen - iWidth_Half);
	*py = (float)(-y_Screen + iHeight_Half);
}
void Rect_2_Screen_Coordinate(float x, float y, int* px_Screen, int* py_Screen, int iWidth, int iHeight)
{//ֱ������ϵ����ת��Ϊ��Ļ����
	int iWidth_Half = iWidth >> 1,
		iHeight_Half = iHeight >> 1;
	*px_Screen = (int)(x + iWidth_Half);
	*py_Screen = (int)(iHeight_Half - y);
	return;
}

void Polor_2_Rect_Coordinate(float rho, float theta, float* px, float* py)
{//��ά�µļ�����ת��Ϊֱ������
	*px = rho * cos(theta);
	*py = rho * sin(theta);
	return;
}
void Polor_2_Rect_Coordinate(float rho, float theta, float phi, float* px, float* py, float* pz)
{//��������(rho, theta, phi)��Ϊֱ������ϵ���˴��ϸ������϶��壬thetaΪrho��xyƽ����ͶӰ��x�н�
//phiΪrho��z��ļн�
	float fSin_Phi = sin(phi),
		fSin_Theta = sin(theta),
		fCos_Theta = cos(theta);
	*px = rho * fSin_Phi * fCos_Theta;
	*py = rho * fSin_Phi * fSin_Theta;
	*pz = rho * cos(phi);
	return;
}
template<typename _T>void Elementary_Row_Operation_1(_T A[], int m, int n, _T A_2[], int* piRank, _T** ppBasic_Solution, _T** ppSpecial_Solution)
{
	typedef struct Q_Item {
		unsigned short m_iRow_Index;	//��ǰ�ж�Ӧ������Ԫ����������
		unsigned short m_iCol_Index;	//			����Ԫ��Ӧ����������x����
	}Q_Item;

	int y, x, x_1, i, iRank = 0, iPos, iMax;
	Q_Item *Q, iTemp;
	_T* pBasic_Solution = NULL, * pSpecial_Solution = NULL;
	short* pMap_Row_2_x_Index = NULL, * pMap_x_2_Basic_Solution_Index = NULL;
	int j, iRank_Basic_Solution;

	_T fValue, fMax, * A_1 = NULL;
	union {
		_T* pfMax_Row;
		_T* pfBottom_Row;
		_T* pfCur_Row;
	};
	if(piRank)
		*piRank = 0;
	if (m > 65535)
	{
		printf("Too large row count:%d\n", m);
		goto END;
	}
	A_1 = (_T*)pMalloc(&oMatrix_Mem, m * n * sizeof(_T));
	Q = (Q_Item*)pMalloc(&oMatrix_Mem, m * sizeof(Q_Item));
	if (A_1)
		memcpy(A_1, A, m * n * sizeof(_T));
	iPos = 0;
	for (y = 0; y < m; y++)
		Q[y] = { (unsigned short)y };	//ÿ����Ԫ���ڵ���

	//Disp(A_1, m, n, "\n");
	for (x_1 = 0, y = 0; y < m; y++)
	{//�������y��x�����ƽ����������
		while (1)
		{
			iMax = y;
			fMax = A_1[Q[iMax].m_iRow_Index * n + x_1];
			for (i = y + 1; i < m; i++)
			{
				if (abs(A_1[iPos = Q[i].m_iRow_Index * n + x_1]) > abs(fMax))
				{
					fMax = A_1[iPos];
					iMax = i;
				}
			}
			if (abs(fMax) <= ZERO_APPROCIATE && x_1 < n - 1)
				x_1++;
			else
				break;
		}

		if (abs(fMax) < ZERO_APPROCIATE)
		{//����ԪΪ0����Ȼ�����ȣ��÷���û��Ψһ��
			//Disp(A_1, m, n,"\n");
			break;
		}

		//�����ԪSWAP��Q�ĵ�ǰλ����
		iTemp = Q[y];
		Q[y] = Q[iMax];
		Q[iMax] = iTemp;
		Q[y].m_iCol_Index = x_1;
		iRank++;

		//Disp(A_1, m, n, "\n");
		pfMax_Row = &A_1[Q[y].m_iRow_Index * n];
		pfMax_Row[x_1] = 1.f;
		for (x = x_1 + 1; x < n; x++)
			pfMax_Row[x] /= fMax;
		//Disp(A_1, m, n, "\n");

		//�Ժ��������д���
		for (i = y + 1; i < m; i++)
			//for (i = 0; i < m; i++)
		{//i��ʾ��i��
			iPos = Q[i].m_iRow_Index * n;
			if (((fValue = A_1[iPos + x_1]) != 0) && i != y)
			{//���ڶ�ӦԪ��Ϊ0�����������
				for (x = x_1; x < n; x++)
					A_1[iPos + x] -= fValue * pfMax_Row[x];
				A_1[iPos + x_1] = 0;	//�˴�Ҳ���Ǳ���ģ�����ֻ�Ǻÿ�
			}
			//Disp(Ai, iOrder, iOrder + 1, "\n");
		}
		//Disp(A_1, m, n, "\n");
		x_1++;
	}

	//Disp(A_1, m, n,"�����б任");
	int y1;	//�Ѿ���֪�������
	//Ȼ��˳������һ�����ϱ任����θ���˹�����Է��̲�һ��
	//����Ժ�A_1����������
	for (y = iRank - 1; y > 0; y--)
	{//�߼��ϴ�����һ�����ϣ�ʵ������Qָ·
		pfBottom_Row = &A_1[Q[y].m_iRow_Index * n];
		x_1 = Q[y].m_iCol_Index;	//ǰ���Ѿ��õ���������Ԫλ��

		for (y1 = y - 1; y1 >= 0; y1--)
		{
			//iPos = Q[y_1] * iRow_Size;
			iPos = Q[y1].m_iRow_Index * n;
			x = x_1;	//�����е�xλ��
			fValue = A_1[iPos + x];
			A_1[iPos + x] = 0;
			for (x++; x < n; x++)
				A_1[iPos + x] -= fValue * pfBottom_Row[x];
			//Disp(A_1, m, n, "\n");
		}
	}
	Disp(A_1, m, n, "A_1");
	if (piRank)
		*piRank = iRank;

	iRank_Basic_Solution = n - 1 - iRank;	//������ϵ����
	if (!ppBasic_Solution || !ppSpecial_Solution)
		goto END;
	
	//���һ��������η����������ϵ��������������������Ϊn-1ά����������n-1- Rank��
	pBasic_Solution = (_T*)pMalloc(&oMatrix_Mem,iRank_Basic_Solution * n * sizeof(_T));
	//����Ϊ�����Ľ�x��Ӧ�ĸ���������һ��Map
	pMap_Row_2_x_Index = (short*)pMalloc(&oMatrix_Mem,(n - 1) * sizeof(short));
	pMap_x_2_Basic_Solution_Index = (short*)pMalloc(&oMatrix_Mem,(n - 1) * sizeof(short));
	
	memset(pBasic_Solution, 0, iRank_Basic_Solution * n * sizeof(_T));
	memset(pMap_Row_2_x_Index, 0, (n - 1) * sizeof(short));
	memset(pMap_x_2_Basic_Solution_Index, 0, (n - 1) * sizeof(short));

	//����ǰ������Ԫ��xλ��Ϊ-1
	for (i = 0; i < iRank; i++)
		pMap_Row_2_x_Index[Q[i].m_iCol_Index] = -1;

	//ʣ�µ�ֵΪ0�ľ��ǻ�����ϵ��������Ӧ��xλ��
	for (j = 0, i = 0; i < n - 1; i++)
	{
		if (pMap_Row_2_x_Index[i] == 0)
		{
			pMap_x_2_Basic_Solution_Index[i] = j;
			pBasic_Solution[i * iRank_Basic_Solution + j] = 1;
			j++;
		}
	}

	//��󣬹�����η��̻�������
	for (y = 0; y < n; y++)
	{
		iPos = Q[y].m_iRow_Index * n;
		pfCur_Row = &A_1[iPos];
		x_1 = Q[y].m_iCol_Index + 1;
		for (; x_1 < n - 1; x_1++)
		{
			if (Abs(pfCur_Row[x_1]) > ZERO_APPROCIATE)
			{
				//�������кţ������к������к���أ���Ȼȡ�����кž����к�
				pBasic_Solution[Q[y].m_iCol_Index * iRank_Basic_Solution + pMap_x_2_Basic_Solution_Index[x_1]] = -A_1[iPos + x_1];
			}
		}
	}

	memset(pSpecial_Solution, 0, (n - 1) * sizeof(_T));
	for (i = 0; i < iRank; i++)
		pSpecial_Solution[Q[i].m_iCol_Index] = A_1[Q[i].m_iRow_Index * n + (n - 1)];

END:
	if(A_1)
		Free(&oMatrix_Mem,A_1);
	if (pBasic_Solution)
		Free(&oMatrix_Mem, pBasic_Solution);
	if (pMap_Row_2_x_Index)
		Free(&oMatrix_Mem, pMap_Row_2_x_Index);
	if (pMap_x_2_Basic_Solution_Index)
		Free(&oMatrix_Mem, pMap_x_2_Basic_Solution_Index);
	if (pSpecial_Solution)
		Free(&oMatrix_Mem, &pSpecial_Solution);

	return;
}
void Elementary_Row_Operation(float A[], int m, int n, float A_1[], int* piRank, float** ppBasic_Solution, float** ppSpecial_Solution)
{//��A�������б任����Ϊ����Σ�Ҫ������Ԫ��
//����A����
	typedef struct Q_Item {
		unsigned char m_iRow_Index;	//��ǰ�ж�Ӧ������Ԫ����������
		unsigned char m_iCol_Index;	//			����Ԫ��Ӧ����������x����
	}Q_Item;

	int y, x, x_1, i, iRank = 0, iPos, iMax;
	Q_Item Q[256], iTemp;
	float fValue, fMax;
	union {
		float* pfMax_Row;
		float* pfBottom_Row;
		float* pfCur_Row;
	};

	if (m > 256)
	{
		printf("Too large row count:%d\n", m);
		return;
	}
	if (A_1)
		memcpy(A_1, A, m * n * sizeof(float));
	iPos = 0;
	for (y = 0; y < m; y++)
		Q[y] = { (unsigned char)y };	//ÿ����Ԫ���ڵ���

	//Disp(A_1, m, n, "\n");
	for (x_1 = 0, y = 0; y < m; y++)
	{//�������y��x�����ƽ����������
		while (1)
		{
			iMax = y;
			fMax = A_1[Q[iMax].m_iRow_Index * n + x_1];
			for (i = y + 1; i < m; i++)
			{
				if (abs(A_1[iPos = Q[i].m_iRow_Index * n + x_1]) > abs(fMax))
				{
					fMax = A_1[iPos];
					iMax = i;
				}
			}
			if (abs(fMax) <= ZERO_APPROCIATE && x_1 < n - 1)
				x_1++;
			else
				break;
		}

		if (abs(fMax) < ZERO_APPROCIATE)
		{//����ԪΪ0����Ȼ�����ȣ��÷���û��Ψһ��
			//Disp(A_1, m, n,"\n");
			break;
		}

		//�����ԪSWAP��Q�ĵ�ǰλ����
		iTemp = Q[y];
		Q[y] = Q[iMax];
		Q[iMax] = iTemp;
		Q[y].m_iCol_Index = x_1;
		iRank++;

		//Disp(A_1, m, n, "\n");
		pfMax_Row = &A_1[Q[y].m_iRow_Index * n];
		pfMax_Row[x_1] = 1.f;
		for (x = x_1 + 1; x < n; x++)
			pfMax_Row[x] /= fMax;
		//Disp(A_1, m, n, "\n");

		//�Ժ��������д���
		for (i = y + 1; i < m; i++)
			//for (i = 0; i < m; i++)
		{//i��ʾ��i��
			iPos = Q[i].m_iRow_Index * n;
			if (((fValue = A_1[iPos + x_1]) != 0) && i != y)
			{//���ڶ�ӦԪ��Ϊ0�����������
				for (x = x_1; x < n; x++)
					A_1[iPos + x] -= fValue * pfMax_Row[x];
				A_1[iPos + x_1] = 0;	//�˴�Ҳ���Ǳ���ģ�����ֻ�Ǻÿ�
			}
			//Disp(Ai, iOrder, iOrder + 1, "\n");
		}
		//Disp(A_1, m, n, "\n");
		x_1++;
	}

	//Disp(A_1, m, n,"�����б任");
	int y1;	//�Ѿ���֪�������
	//Ȼ��˳������һ�����ϱ任����θ���˹�����Է��̲�һ��
	//����Ժ�A_1����������
	for (y = iRank - 1; y > 0; y--)
	{//�߼��ϴ�����һ�����ϣ�ʵ������Qָ·
		pfBottom_Row = &A_1[Q[y].m_iRow_Index * n];
		x_1 = Q[y].m_iCol_Index;	//ǰ���Ѿ��õ���������Ԫλ��

		for (y1 = y - 1; y1 >= 0; y1--)
		{
			//iPos = Q[y_1] * iRow_Size;
			iPos = Q[y1].m_iRow_Index * n;
			x = x_1;	//�����е�xλ��
			fValue = A_1[iPos + x];
			A_1[iPos + x] = 0;
			for (x++; x < n; x++)
				A_1[iPos + x] -= fValue * pfBottom_Row[x];
			//Disp(A_1, m, n, "\n");
		}
	}

	/*if (iRank == n-1)
	{
		*ppBasic_Solution = NULL;
		return;
	}*/


	int j, iRank_Basic_Solution = n - 1 - iRank;	//������ϵ����
	//���һ��������η����������ϵ��������������������Ϊn-1ά����������n-1- Rank��
	float* pBasic_Solution = (float*)malloc(iRank_Basic_Solution * n * sizeof(float));
	//����Ϊ�����Ľ�x��Ӧ�ĸ���������һ��Map
	short* pMap_Row_2_x_Index = (short*)malloc((n - 1) * sizeof(short));
	short* pMap_x_2_Basic_Solution_Index = (short*)malloc((n - 1) * sizeof(short));

	memset(pBasic_Solution, 0, iRank_Basic_Solution * n * sizeof(float));
	memset(pMap_Row_2_x_Index, 0, (n - 1) * sizeof(short));
	memset(pMap_x_2_Basic_Solution_Index, 0, (n - 1) * sizeof(short));

	//����ǰ������Ԫ��xλ��Ϊ-1
	for (i = 0; i < iRank; i++)
		pMap_Row_2_x_Index[Q[i].m_iCol_Index] = -1;

	//ʣ�µ�ֵΪ0�ľ��ǻ�����ϵ��������Ӧ��xλ��
	for (j = 0, i = 0; i < n - 1; i++)
	{
		if (pMap_Row_2_x_Index[i] == 0)
		{
			pMap_x_2_Basic_Solution_Index[i] = j;
			pBasic_Solution[i * iRank_Basic_Solution + j] = 1;
			j++;
		}
	}

	//��󣬹�����η��̻�������
	for (y = 0; y < n; y++)
	{
		iPos = Q[y].m_iRow_Index * n;
		pfCur_Row = &A_1[iPos];
		x_1 = Q[y].m_iCol_Index + 1;
		for (; x_1 < n - 1; x_1++)
		{
			if (Abs(pfCur_Row[x_1]) > ZERO_APPROCIATE)
			{
				//�������кţ������к������к���أ���Ȼȡ�����кž����к�
				pBasic_Solution[Q[y].m_iCol_Index * iRank_Basic_Solution + pMap_x_2_Basic_Solution_Index[x_1]] = -A_1[iPos + x_1];
			}
		}
	}
	//Disp(pBasic_Solution, n - 1, iRank_Basic_Solution);
	//����һ���ؽ�
	float* pSpecial_Solution = (float*)malloc((n - 1) * sizeof(float));
	memset(pSpecial_Solution, 0, (n - 1) * sizeof(float));
	for (i = 0; i < iRank; i++)
		pSpecial_Solution[Q[i].m_iCol_Index] = A_1[Q[i].m_iRow_Index * n + (n - 1)];

	if (ppSpecial_Solution)
		*ppSpecial_Solution = pSpecial_Solution;
	else
		free(pSpecial_Solution);
	//Disp(A_1, m, n);
	if (piRank)
		*piRank = iRank;
	if (ppBasic_Solution)
		*ppBasic_Solution = pBasic_Solution;
	else
		if (pBasic_Solution)
			free(pBasic_Solution);

	/*if (pQ)
		for (y = 0; y < m; y++)
			pQ[y] = Q[y].m_iRow_Index;*/
	free(pMap_Row_2_x_Index);
	free(pMap_x_2_Basic_Solution_Index);
	return;
}
void Solve_Linear_Solution_Construction(float* A, int m, int n, float B[], int* pbSuccess, float* pBasic_Solution, int* piBasic_Solution_Count, float* pSpecial_Solution)
{//�����Է��̣������������������ʽ
	float* Ai = (float*)malloc(m * (n + 1) * sizeof(float));
	float* pBasic_Solution_1, * pSpecial_Solution_1;
	int y, x, iRow_Size = n + 1;
	int iRank;	//ϵ���������

	for (y = 0; y < m; y++)
	{//AiΪ�������
		for (x = 0; x < n; x++)
			Ai[y * iRow_Size + x] = A[y * n + x];
		Ai[y * iRow_Size + n] = B[y];
	}
	//�˴����г����б任�������ø����׵�����Ԫ
	Elementary_Row_Operation(Ai, m, n + 1, Ai, &iRank, &pBasic_Solution_1, &pSpecial_Solution_1);
	for (y = 0; y < m; y++)
	{
		int bIs_Zero = 1;
		for (x = 0; x < n; x++)
		{
			if (abs(Ai[y * (n + 1) + x]) > ZERO_APPROCIATE)
			{
				bIs_Zero = 0;
				break;
			}
		}
		if (bIs_Zero && Ai[y * (n + 1) + n] > ZERO_APPROCIATE)
		{
			printf("�÷����޽⣬���������б任�Ժ󣬵�%d�еĳ�����Ϊ��%f\n", y, Ai[y * (n + 1) + n]);
			Disp(Ai, m, n + 1);
			*pbSuccess = 0;
			return;
		}
	}
	int bIs_Homo = 1;
	for (y = 0; y < m; y++)
	{
		if (B[y])
		{//���������b��Ϊ0����Ϊ����η���
			bIs_Homo = 0;
			break;
		}
	}
	if (bIs_Homo)
	{//��η�����Ľ⣬���� X= c0*v0 + c1*v1 + ... + c(n-r) * v(n-r)
		if (iRank == n)
		{
			printf("ϵ���������ȣ�ֻ�����\n");
			if (piBasic_Solution_Count)
				*piBasic_Solution_Count = 0;
			goto END;
		}
	}
	if (piBasic_Solution_Count)
		*piBasic_Solution_Count = n - iRank;
	if (pBasic_Solution)
	{
		Matrix_Transpose(pBasic_Solution_1, n, n - iRank, pBasic_Solution_1);
		Schmidt_Orthogon(pBasic_Solution_1, n - iRank, n, pBasic_Solution_1);
		memcpy(pBasic_Solution, pBasic_Solution_1, (n - iRank) * n * sizeof(float));
	}
	if (!bIs_Homo && pSpecial_Solution)
		memcpy(pSpecial_Solution, pSpecial_Solution_1, n * sizeof(float));

END:
	if (pSpecial_Solution_1)
		free(pSpecial_Solution_1);
	if (pBasic_Solution_1)
		free(pBasic_Solution_1);
	if (Ai)
		free(Ai);
	*pbSuccess = 1;
}
void Get_Linear_Solution_Construction(float A[], const int m, int n, float B[])
{//�����Է������Ľṹ
	float* Ai = (float*)malloc(m * (n + 1) * sizeof(float));
	int y, x, iRow_Size = n + 1;
	int iRank;	//ϵ���������
	for (y = 0; y < m; y++)
	{//AiΪ�������
		for (x = 0; x < n; x++)
			Ai[y * iRow_Size + x] = A[y * n + x];
		Ai[y * iRow_Size + n] = B[y];
	}
	Disp(Ai, m, n + 1, "Aumented Matrix");
	//�˴����г����б任�������ø����׵�����Ԫ
	float* pBasic_Solution = NULL,
		* pSpecial_Solution = NULL;

	Elementary_Row_Operation(Ai, m, n + 1, Ai, &iRank, &pBasic_Solution, &pSpecial_Solution);
	for (y = 0; y < m; y++)
	{
		int bIs_Zero = 1;
		for (x = 0; x < n; x++)
		{
			if (abs(Ai[y * (n + 1) + x]) > ZERO_APPROCIATE)
			{
				bIs_Zero = 0;
				break;
			}
		}
		if (bIs_Zero && Ai[y * (n + 1) + n] > ZERO_APPROCIATE)
		{
			printf("�÷����޽⣬���������б任�Ժ󣬵�%d�еĳ�����Ϊ��%f\n", y, Ai[y * (n + 1) + n]);
			Disp(Ai, m, n + 1);
			return;
		}
	}
	int bIs_Homo = 1;
	for (y = 0; y < m; y++)
	{
		if (B[y])
		{//���������b��Ϊ0����Ϊ����η���
			bIs_Homo = 0;
			break;
		}
	}
	//������������Ӳ�磬����Ԫ�������Ժ󲢲�������Ρ���զŪ
	Disp(Ai, m, n + 1, "����ξ�������");
	if (bIs_Homo)
	{//��η�����Ľ⣬���� X= c0*v0 + c1*v1 + ... + c(n-r) * v(n-r)
		if (iRank == n)
		{
			printf("ϵ���������ȣ�ֻ�����\n");
			goto END;
		}

		////����
		float* pB_1 = (float*)malloc(m * (n - iRank) * sizeof(float));
		Disp(pBasic_Solution, n, n - iRank, "������ϵ");
		Matrix_Multiply(A, m, n, pBasic_Solution, n - iRank, pB_1);

		float* pBasic_Solution_1 = (float*)malloc((n - iRank) * n * sizeof(float));
		Matrix_Transpose(pBasic_Solution, n, n - iRank, pBasic_Solution_1);
		Schmidt_Orthogon(pBasic_Solution_1, n - iRank, n, pBasic_Solution_1);
		Disp(pBasic_Solution_1, n - iRank, n, "������");
		free(pBasic_Solution_1);

		Disp(pB_1, m, n - iRank, "Axb");
		free(pB_1);

		////��һ��
		//float* pBasic_Solution_1 = (float*)malloc((n - iRank) * n * sizeof(float));
		//Matrix_Transpose(pBasic_Solution, n, n-iRank, pBasic_Solution_1);
		//
		//for (y = 0; y < n - iRank; y++)
		//	Normalize(&pBasic_Solution_1[y*n], n, &pBasic_Solution_1[y*n]);

		//Disp(pBasic_Solution_1, n - iRank, n);
		//free(pBasic_Solution_1);

		//float Matlab_Value[] = {0.3876f, -0.1577f ,0.3444f ,-0.1989f , -0.8165f};
		//printf("Mod:%f\n", fGet_Mod(Matlab_Value, n));
	}
	else
	{
		Disp(pSpecial_Solution, n, 1, "�ؽ�");
		if (iRank == n)
		{
			printf("ϵ���������ȣ�ֻ��Ψһ��\n");
			goto END;
		}
		Disp(pBasic_Solution, n, n - iRank, "������ϵ");
	}

	//����Ӹ������������
END:
	if (pSpecial_Solution)
		free(pSpecial_Solution);
	if (pBasic_Solution)
		free(pBasic_Solution);
	if (Ai)
		free(Ai);
	return;
}
int bIs_Linearly_Dependent(float A[], int m, int n)
{//��A��m�� nά�������Ƿ��������
	if (m > n)
		return 1;	//��������ά������Ȼ�������

	float* B = (float*)malloc(m * sizeof(float));
	float* A1 = (float*)malloc(m * n * sizeof(float));

	//memset(B, 0, m * sizeof(float));
	int iRank;
	Elementary_Row_Operation(A, m, n, A1, &iRank);

	if (iRank == m)
		return 0;
	else
		return 1;
}
void Perspective(float Pos_Source[3], float h[3], float Pos_Dest[3])
{//h�����������ϵ���㣬��Ӧ͸��ԭ���е�h����
	//���ڿռ��е�һ��(x,y,z), ���ձ�����͸�ӱ任��Ϊ (x',y',z'). x'/x0= h/(h-z0) => x'= x * h/(h-z0)
	//͸��Ҫ�㣬1���ӵ㼴���
	//			2����λ��z��0��ʱ������ԭλ��
	//			3, ��zΪ��ʱ����ԭͼ�󣬵�zΪ��ʱ����ԭͼС
	//			4, ������û�п���������꣬�ʴ���֮��Щ����
	//			5�����ڱ任֮��
	//			6, ��zλ��Ϊ0ʱ�������Ѿ��任�����޴󣬸�ֵ������
	//			7����zλ�ô����ӵ�hλ��ʱ��ͬ�������壬��Ϊ����󿴲���
	if ((h[2] > 0 && Pos_Source[2] >= h[2]) || (h[2] < 0 && Pos_Source[2] <= h[2]))
	{
		printf("Invalid Pos: z>=h\n");
		return;
	}

	float H =/*h[0]/(h[0]-Pos_Source[0]) +*/ h[2] / (h[2] - Pos_Source[2]);
	Pos_Dest[0] = Pos_Source[0] * H;
	Pos_Dest[1] = Pos_Source[1] * H;
	Pos_Dest[2] = 0;
	return;
}
void Perspective_Camera(float Pos_Source[3], float h[3], float Pos_Dest[3])
{	//Pos_Source: ���λ�ã�fCamera_z������Լ���z���ϵ�λ�ã�Ϊ��
	//float z = h[2] + Pos_Source[2];
	//float H = h[2] / (h[2] - z);
	float p = 1.f / (h[2] * 2),
		q = 1.f / (h[2]),
		r = 1.f / (-h[2]);
	/*float p = 0.00215193816,
		r = -0.00186189834;*/
	float H = 1 / (/*p * abs(Pos_Source[0])*/ /*+ q * abs(Pos_Source[1])*/ +r * Pos_Source[2] + 1);
	//float H = 1.f / (p + r + 1);
	//float H =  ( h[2]/(h[2] + abs(Pos_Source[0]))) * (-h[2] / Pos_Source[2]);
	Pos_Dest[0] = Pos_Source[0] * H;
	Pos_Dest[1] = Pos_Source[1] * H;
	Pos_Dest[2] = 0;
}

float fGet_Theta(float v0[], float v1[], float Axis[], int n)
{//����������֮��ļнǣ�nΪά�� �õ������  v0.v1= |v0|*|v1|*cos(theta)
	//�ȵ��
	float fMod_v0 = fGet_Mod(v0, n),
		fMod_v1 = fGet_Mod(v1, n);
	float theta = acos(fDot(v0, v1, n) / (fMod_v0 * fMod_v1));
	//float v2[3];
	//������δ���theta ����������

	//�����ɵ�Ƶķ���
#define eps 0.0001f
	float v0_1[3], v1_1[3], R[3][3], Temp[4];
	float Axis_1[4] = { Axis[0],Axis[1],Axis[2],theta };
	Normalize(v0, 3, v0_1);
	Normalize(v1, 3, v1_1);
	Rotation_Vector_2_Matrix_3D(Axis_1, (float*)R);
	Matrix_Multiply((float*)R, 3, 3, v0_1, 1, Temp);
	if (abs(Temp[0] - v1_1[0]) > eps || abs(Temp[1] - v1_1[1]) > eps || abs(Temp[2] - v1_1[2]) > eps)
		theta = -theta;
#undef eps
	//Cross_Product(v0, v1, v2);
	////���ݲ�˶��壬 |v2|=|v0|*|v1|*sin(theta)
	//theta = asin(fGet_Mod(v2, 3) / (fGet_Mod(v0, n) * fGet_Mod(v1, n)));

	return theta;
}
void Gen_Cube(float Cube[8][4], float fScale, float x_Center, float y_Center, float z_Center)
{//���²���ϲ�
	float Cube_1[8][4] = { {-1,-1,-1,1},	//���º�
						{1,-1,-1,1},	//���º�
						{1,-1,1,1},		//����ǰ
						{-1,-1,1,1},	//����ǰ
						{-1,1,-1,1},	//���Ϻ�
						{1,1,-1,1},		//���Ϻ�
						{1,1,1,1},		//����ǰ
						{-1,1,1,1} };	//����ǰ
	int i;
	for (i = 0; i < 8; i++)
	{
		Cube[i][0] = Cube_1[i][0] * fScale + x_Center;
		Cube[i][1] = Cube_1[i][1] * fScale + y_Center;
		Cube[i][2] = Cube_1[i][2] * fScale + z_Center;
		Cube[i][3] = 1;
	}
	return;
}
void Diagonalize(float A[], int iOrder, float Diag[], float P[], int* pbSuccess)
{//���һ���Գƾ���ĶԽǻ�������󽫶Խǻ��������Diag[]�У������ƾ���ı任�����P��
//ע�⣬��õĽ��������ϼ���˳�� A = P x Diag x P'
// ��P��һ���������󣬹ʴ�P'�������棬ת�ü���
	float* Diag_1 = (float*)malloc(iOrder * iOrder * sizeof(float)),
		* P_1 = (float*)malloc(iOrder * iOrder * sizeof(float));
	int y, x;
	QR_Decompose(A, iOrder, Diag_1, P_1, pbSuccess);
	if (!(*pbSuccess))
	{
		printf("�þ����޷�������ֵ����������\n");
		goto END;
	}

	//ֻ�����Խ��ߣ�˳�㿴��ԭ�����Ƿ�Գ�
	for (y = 0; y < iOrder; y++)
	{
		for (x = 0; x < iOrder; x++)
		{
			if (x != y)
				Diag_1[y * iOrder + x] = 0;
			else
			{
				if (A[y * iOrder + x] != A[x * iOrder + y])
				{
					printf("ԭ����ǶԳƾ��󣬲��ܶԽǻ�\n");
					*pbSuccess = 0;
					goto END;
				}
			}
		}
	}
	memcpy(Diag, Diag_1, iOrder * iOrder * sizeof(float));
	memcpy(P, P_1, iOrder * iOrder * sizeof(float));

END:
	free(Diag_1);
	free(P_1);
	return;
}
void Disp_Poly(float Coeff[], int iCoeff_Count, char* pCaption = NULL)
{
	int i, n = iCoeff_Count - 1;
	if (pCaption)
		printf("%s\n", pCaption);
	if (n >= 2)
		printf("%fx^%d ", Coeff[n], n);
	else if (n == 1)
		printf("%fx ", Coeff[n]);
	n--;

	for (i = n; i >= 0; i--, n--)
	{
		if (n >= 2)
			printf("+ %fx^%d ", Coeff[i], n);
		else if (n == 1)
			printf("+ %fx ", Coeff[i]);
		else
			printf("+ %f ", Coeff[i]);
	}
	printf("\n");
	return;
}
void Get_Poly_Derivative(float Coeff[], int iCoeff_Count, float Der_Coeff[])
{//�õ�һ������ʽ�ĵ�������ߴ�����[0]��
	int i;
	for (i = 1; i < iCoeff_Count; i++)
		Der_Coeff[i - 1] = i * Coeff[i];
}
void Solve_Poly(float Coeff[], int iCoeff_Count, Complex_f Root[], int* piRoot_Count)
{//iCount��ϵ�����������������Ϊn, ����ߴ�Ϊn-1
//Ϊ�˺��±�����ƥ�䣬Coeff��Ĵ�����С������
	Complex_f* pRoot_1 = (Complex_f*)malloc((iCoeff_Count - 1) * sizeof(Complex_f));
	memset(pRoot_1, 0, (iCoeff_Count - 1) * sizeof(Complex_f));
	if (iCoeff_Count == 3)
	{//���η��̣��н�
		float a = Coeff[2], b = Coeff[1], c = Coeff[0];
		float fDelta = b * b - 4 * a * c;
		if (fDelta > ZERO_APPROCIATE)
		{
			if (piRoot_Count)
				*piRoot_Count = 2;
			fDelta = sqrt(fDelta);
			pRoot_1[0] = { (-b + fDelta) / (2.f * a),0.f };
			pRoot_1[1] = { (-b - fDelta) / (2.f * a),0.f };
		}
		else if (fDelta < -ZERO_APPROCIATE)
		{
			if (piRoot_Count)
				*piRoot_Count = 2;
			fDelta = sqrt(-fDelta);
			pRoot_1[0] = { -b / (2.f * a), fDelta / (2.f * a) };
			pRoot_1[1] = { -b / (2.f * a), -fDelta / (2.f * a) };
		}
		else
		{
			if (piRoot_Count)
				*piRoot_Count = 1;
			pRoot_1[0] = pRoot_1[1] = { -b / (2.f * a),0 };
		}
	}
	else if (iCoeff_Count == 2)
	{//���� ax+b=0
		float a = Coeff[1], b = Coeff[0];
		*piRoot_Count = 1;
		pRoot_1[0] = { -b / a,0.f };
	}
	else
	{//�ߴη����ˣ���ʱҪ�õݹ��½������ǳ��鷳
		int n = iCoeff_Count - 1;
		//�ȶ�ԭ����ʽ�󵼣���ø���Ӷ����Ρ������ʽ��=0�����������̣��õ�һ���
		float* pDer_Coeff = (float*)malloc(n * sizeof(float));
		Complex_f* pDer_Root = (Complex_f*)malloc((n - 1) * sizeof(Complex_f));
		int iDer_Root_Count;
		//Disp_Poly(Coeff, iCoeff_Count);
		Get_Poly_Derivative(Coeff, iCoeff_Count, pDer_Coeff);
		//Disp_Poly(pDer_Coeff, iCoeff_Count - 1);
		Solve_Poly(pDer_Coeff, iCoeff_Count - 1, pDer_Root, &iDer_Root_Count);

		return;	//û��
	}

	memcpy(Root, pRoot_1, *piRoot_Count * sizeof(Complex_f));
	for (int i = 0; i < *piRoot_Count; i++)
	{
		if (Root[i].im == 0)
			printf("ʵ��%d:%f\n", i, Root[i].real);
		else
			if (Root[i].im > 0)
				printf("����%d:%f\t+%fi\n", i, Root[i].real, Root[i].im);
			else
				printf("����%d:%f\t%fi\n", i, Root[i].real, Root[i].im);
	}
	free(pRoot_1);
	return;
}

void Decompose_E(float E[], float R_1[], float R_2[], float t_1[], float t_2[])
{//��һ��E �ֽ�Ϊ һ������ R[3x3], ����t[3]
	float R_z_t_1[3 * 3],		//Rz(pi/2)		�������ɳ���
		R_z_t_2[3 * 3],		//Rz(-pi/2)
		U[3 * 3], Vt[3 * 3], Ut[3 * 3],
		S[3], Temp[3 * 3];
	//int iResult;
	//(E, 3, 3, U, S, Vt);
	Matrix_Transpose(U, 3, 3, Ut);

	float V[] = { 0,0,1,PI / 2.f };
	float Sigma[3 * 3] = { S[0],0,0,
						0,S[1],0,
						0,0,S[2] };
	Rotation_Vector_2_Matrix_3D(V, R_z_t_1);
	V[3] = -V[3];
	Rotation_Vector_2_Matrix_3D(V, R_z_t_2);

	Matrix_Multiply(U, 3, 3, Sigma, 3, Temp);
	Matrix_Multiply(Temp, 3, 3, Vt, 3, Temp);
	/*Disp(Temp, 3, 3,"Temp");
	Disp(U, 3, 3, "U");
	Disp(S, 3, 1, "S");
	Disp(Vt, 3, 3, "Vt");*/

	//svd_3(E, 3, 3, U, S, Vt);
	Disp(U, 3, 3, "U");
	//Disp(Sigma, 3, 3, "Sigma");
	Disp(Vt, 3, 3, "Vt");
	Matrix_Multiply(U, 3, 3, Sigma, 3, Temp);
	Matrix_Multiply(Temp, 3, 3, Vt, 3, Temp);
	Disp(Temp, 3, 3, "Temp");

	printf("%d", bIs_Orthogonal(U, 3));
	printf("%d", bIs_Orthogonal(Vt, 3));

	return;
}

void Init_Env()
{//��ʼ��������������Щ�ڴ湩һ�к�����ʱʹ��
	int iSize = 100000000;
	/*unsigned char* pBuffer = (unsigned char*)malloc(iSize);
	if (!pBuffer)
	{
		printf("Fail to malloc in Init_Env\n");
		return;
	}
	Attach_Light_Ptr(oMatrix_Mem, pBuffer, iSize, -1);*/
	Init_Mem_Mgr(&oMatrix_Mem, iSize, 1024, 997);
	return;
}
void Free_Env()
{
	if (oMatrix_Mem.m_pBuffer)
		Free_Mem_Mgr(&oMatrix_Mem);
	oMatrix_Mem = { 0 };
}
template<typename _T> void _svd_3(_T At[], int m, int n, int n1, _T Sigma[], _T Vt[], int* pbSuccess, double eps)
{
	int i, j, k, iter, max_iter = 30;
	_T sd;
	_T s, c;
	int iMax_Size = Max(m, n);
	memset(Sigma, 0, iMax_Size * sizeof(_T));
	memset(Vt, 0, m * n * sizeof(_T));

	int astep = m, vstep = n;

	for (i = 0; i < n; i++)
	{
		for (k = 0, sd = 0; k < m; k++)
		{
			_T t = At[i * astep + k];
			sd += (_T)t * t;
		}
		Sigma[i] = sd;

		if (Vt)
		{
			for (k = 0; k < n; k++)
				Vt[i * vstep + k] = 0;
			Vt[i * vstep + i] = 1;
		}
	}

	for (iter = 0; iter < max_iter; iter++)
	{
		int changed = false;

		for (i = 0; i < n - 1; i++)
		{
			for (j = i + 1; j < n; j++)
			{
				_T* Ai = At + i * astep, * Aj = At + j * astep;
				_T a = Sigma[i], p = 0, b = Sigma[j];

				for (k = 0; k < m; k++)
					p += (double)Ai[k] * Aj[k];

				if (std::abs(p) <= eps * std::sqrt((double)a * b))
					continue;

				p *= 2;
				double beta = a - b, gamma = hypot((double)p, beta);
				if (beta < 0)
				{
					double delta = (gamma - beta) * 0.5;
					s = (_T)std::sqrt(delta / gamma);
					c = (_T)(p / (gamma * s * 2));
				}
				else
				{
					c = (_T)std::sqrt((gamma + beta) / (gamma * 2));
					s = (_T)(p / (gamma * c * 2));
				}

				a = b = 0;
				for (k = 0; k < m; k++)
				{
					_T t0 = c * Ai[k] + s * Aj[k];
					_T t1 = -s * Ai[k] + c * Aj[k];
					Ai[k] = t0; Aj[k] = t1;
					a += (_T)t0 * t0; b += (_T)t1 * t1;
				}
				Sigma[i] = a; Sigma[j] = b;

				changed = true;

				if (Vt)
				{
					_T* Vi = Vt + i * vstep, * Vj = Vt + j * vstep;
					k = 0;	//vblas.givens(Vi, Vj, n, c, s);

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
			break;
	}

	for (i = 0; i < n; i++)
	{
		for (k = 0, sd = 0; k < m; k++)
		{
			_T t = At[i * astep + k];
			sd += (_T)t * t;
		}
		Sigma[i] = std::sqrt(sd);
	}

	for (i = 0; i < n - 1; i++)
	{
		j = i;
		for (k = i + 1; k < n; k++)
		{
			if (Sigma[j] < Sigma[k])
				j = k;
		}
		if (i != j)
		{
			std::swap(Sigma[i], Sigma[j]);
			if (Vt)
			{
				for (k = 0; k < m; k++)
					std::swap(At[i * astep + k], At[j * astep + k]);

				for (k = 0; k < n; k++)
					std::swap(Vt[i * vstep + k], Vt[j * vstep + k]);
			}
		}
	}
	if (!Vt)
		return;
	//Disp(Vt, m, vstep);

	unsigned long long iRandom_State = 0x12345678;
	for (i = 0; i < n1; i++)
	{
		sd = i < n ?Sigma[i] : 0;

		for (int ii = 0; ii < 100 && sd <= DBL_MIN; ii++)
		{
			// if we got a zero singular value, then in order to get the corresponding left singular vector
			// we generate a random vector, project it to the previously computed left singular vectors,
			// subtract the projection and normalize the difference.
			const _T val0 = (_T)(1. / m);
			for (k = 0; k < m; k++)
			{
				iGet_Random_No_cv(&iRandom_State);
				_T val = (iRandom_State & 256) != 0 ? val0 : -val0;
				At[i * astep + k] = val;
			}
			for (iter = 0; iter < 2; iter++)
			{
				for (j = 0; j < i; j++)
				{
					sd = 0;
					for (k = 0; k < m; k++)
						sd += At[i * astep + k] * At[j * astep + k];
					_T asum = 0;
					for (k = 0; k < m; k++)
					{
						_T t = (_T)(At[i * astep + k] - sd * At[j * astep + k]);
						At[i * astep + k] = t;
						asum += std::abs(t);
					}
					asum = asum > eps * 100 ? 1 / asum : 0;
					for (k = 0; k < m; k++)
						At[i * astep + k] *= asum;
				}
			}
			sd = 0;
			for (k = 0; k < m; k++)
			{
				_T t = At[i * astep + k];
				sd += (_T)t * t;
			}
			sd = std::sqrt(sd);
		}

		s = (_T)(sd > DBL_MIN ? 1 / sd : 0.);
		for (k = 0; k < m; k++)
			At[i * astep + k] *= s;
	}
}

//template<typename _T> void _svd_4(_T A[], int h, int w, int n1,_T Sigma[], _T Vt[], int* pbSuccess, double eps)
//{//���԰�opencvдһ��
////return pbSuccess: 1����ʾ������ 0����ʾ������
//	int i, j, k, iter, max_iter = 30;
//	_T sd;
//	_T s, c;
//	int iMax_Size = Max(h, w);
//	memset(Sigma, 0, iMax_Size * sizeof(_T));
//	memset(Vt, 0, h * w * sizeof(_T));
//	
//	for (i = 0; i < h; i++)
//	{
//		/*if (i == 5)
//			printf("here");*/
//		for (sd = 0, j = 0; j < w; j++)
//		{
//			_T t = A[i * w + j];
//			sd += t * t;	//���ƽ���ͣ�
//		}
//		Sigma[i] = sd;		//Sigma[i]����һ�е�ƽ����
//		if (Vt)
//		{
//			for (j = 0; j < h; j++)
//				Vt[i * w + j] = 0;
//			Vt[i * w + i] = 1;		//�˴���Vtһ����λ����
//		}
//	}
//	_T a, p, b;
//	if (pbSuccess)
//		*pbSuccess = 0;	//Ĭ�ϲ��ɹ�
//	for (iter = 0; iter < max_iter; iter++)
//	{
//		int changed = 0;
//		for (i = 0; i < h - 1; i++)
//		{
//			for (j = i + 1; j < h; j++)
//			{
//				_T* Ai = A + i * w, * Aj = A + j * w;	//ȥ��i�к͵�j�У�
//				a = Sigma[i], p = 0, b = Sigma[j];
//				//printf("i:%d j:%d %f\n",i,j, Sigma[5]);
//				for (k = 0; k < w; k++)
//					p += (_T)Ai[k] * Aj[k];
//				//float fTemp = eps * sqrt((_T)a * b);
//				if (Abs(p) <= eps * sqrt((_T)a * b))
//					continue;
//				//printf("%f\n", abs(p));
//				p *= 2;
//				_T beta = a - b,
//					gamma = (_T)hypot((_T)p, beta);	//���ɶ�����б�ߣ�����Ӫ��
//			
//				if (beta < 0)
//				{
//					_T delta = (gamma - beta) * 0.5f;
//					s = (_T)sqrt(delta / gamma);
//					c = (_T)(p / (gamma * s * 2));
//				}
//				else
//				{
//					c = (_T)sqrt((gamma + beta) / (gamma * 2));
//					s = (_T)(p / (gamma * c * 2));
//				}
//				a = b = 0;
//				for (k = 0; k < w; k++)
//				{
//					_T t0 = c * Ai[k] + s * Aj[k];
//					_T t1 = -s * Ai[k] + c * Aj[k];
//					Ai[k] = t0; Aj[k] = t1;
//					a += (_T)t0 * t0; b += (_T)t1 * t1;
//				}
//				Sigma[i] = a; Sigma[j] = b;
//
//				changed = 1;
//				if (Vt)
//				{
//					_T* Vi = Vt + i * w, * Vj = Vt + j * w;
//					_T t0, t1;
//					k = 0;
//					for (; k < w; k++)
//					{
//						t0 = c * Vi[k] + s * Vj[k];
//						t1 = -s * Vi[k] + c * Vj[k];
//						Vi[k] = t0; Vj[k] = t1;
//					}
//					//printf("i:%d j:%d Vt[0]:%f\n", i, j, Vt[1*w]);
//				}
//			}
//		}
//		if (!changed)
//		{
//			if (pbSuccess)
//				*pbSuccess = 1;
//			break;
//		}
//	}
//
//	for (i = 0; i < h; i++)
//	{
//		for (k = 0, sd = 0; k < w; k++)
//		{
//			_T t = A[i * w + k];
//			sd += (_T)t * t;
//		}
//		Sigma[i] = (_T)sqrt(sd);
//	}
//
//	for (i = 0; i < h - 1; i++)
//	{
//		j = i;
//		for (k = i + 1; k < h; k++)
//		{
//			if (Sigma[j] < Sigma[k])
//				j = k;
//		}
//		if (i != j)
//		{
//			//std::swap(Sigma[i], Sigma[j]);
//			_T fTemp = Sigma[i];
//			Sigma[i] = Sigma[j];
//			Sigma[j] = fTemp;
//
//			if (Vt)
//			{
//				for (k = 0; k < w; k++)
//					std::swap(A[i * w + k], A[j * w + k]);
//
//				for (k = 0; k < h; k++)
//					std::swap(Vt[i * w + k], Vt[j * w + k]);
//			}
//		}
//	}
//	/*for (int i = 0; i < 10; i++)
//	{
//		for (int j = 0; j < 10; j++)
//			printf("%f ", Vt[i * h + j]);
//		printf("\n");
//	}*/
//	unsigned long long iRandom_State = 0x12345678;
//	for (i = 0; i < n1; i++)
//	{
//		sd = i < h ? Sigma[i] : 0;
//
//		for (int ii = 0; ii < 100 && sd <= DBL_MIN; ii++)
//		{
//			_T val0 = (_T)(1. / w);
//			for (k = 0; k < w; k++)
//			{
//				iGet_Random_No_cv(&iRandom_State);
//				_T val = (iRandom_State & 256) != 0 ? val0 : -val0;
//				A[i * w + k] = val;
//			}
//			for (iter = 0; iter < 2; iter++)
//			{
//				for (j = 0; j < i; j++)
//				{
//					sd = 0;
//					for (k = 0; k < w; k++)
//						sd += A[i * w + k] * A[j * w + k];
//					_T asum = 0;
//					for (k = 0; k < w; k++)
//					{
//						_T t = (_T)(A[i * w + k] - sd * A[j * w + k]);
//						A[i * w + k] = t;
//						asum += Abs(t);
//					}
//					asum = asum > eps * 100 ? 1 / asum : 0;
//					for (k = 0; k < w; k++)
//						A[i * w + k] *= asum;
//				}
//			}
//			sd = 0;
//			for (k = 0; k < w; k++)
//			{
//				_T t = A[i * w + k];
//				sd += (_T)t * t;
//			}
//			sd = (_T)sqrt(sd);
//		}
//		s = (_T)(sd > DBL_MIN ? 1 / sd : 0.);
//		for (k = 0; k < w; k++)
//			A[i * w + k] *= s;
//	}
//	//Disp(A, h, w, "A");
//	return;
//}

template<typename _T> void svd_3(_T *A, SVD_Info oSVD, int* pbSuccess, double eps)
{/*��дһ��SVD������Ҫ�㣺
1�����еľ���ͨ��ת��ͳһΪ		nnnnn	���߱ȿ�����״
									nnnnn
									nnnnn
									nnnnn
									nnnnn
2, ����ԭ�����Ѿ��Ǹ߱ȿ�����״�������ϣ�����Ahxw, ��ʱVtֻ��wxw, U����wxh
3���������� mmmmmmmmmmmmmmm	����ȸߴ�ľ��󣬰�ʱVtֻ���� (h+1)*w, U����wxw
			mmmmmmmmmmmmmmm
			mmmmmmmmmmmmmmm
4,���ڷ�����ʱU Vt���Ƿ���
5,S����ʱֻ����һ��min(m,n)
*/
	_T* A_1, * Vt_1 = NULL, * Sigma;
	int x, y, bHor,iResult=1;
	int h = oSVD.h_A, w = oSVD.w_A;
	int iTemp,iMax_Size = Max(h, w), iMin_Size = Min(h, w);
	
	//���A����Ҫ���뵽��������������У���û�й����������ڴ氲��
	A_1 = (_T*)pMalloc(&oMatrix_Mem, iMax_Size * iMax_Size * sizeof(_T));
	Sigma = (_T*)pMalloc(&oMatrix_Mem, iMax_Size * sizeof(_T));
	if (!A_1 || !Sigma)
	{
		iResult = 0;
		goto END;
	}
	memset(A_1, 0, iMax_Size * iMax_Size * sizeof(_T));
	if (h < w)
	{//Ҫת�ó�Ϊ�߱ȿ��
		bHor = 1;
		iTemp = h;
		h = w;
		w = iTemp;
		memcpy(A_1, A, h * w * sizeof(_T));
	}
	else
	{//���� ��ʱ��A_1=At
		Matrix_Transpose(A, h, w, A_1);
		bHor = 0;
	}

	Vt_1 = (_T*)pMalloc(&oMatrix_Mem, w*(w+1) * sizeof(_T));
	memset(Vt_1, 0, w * (w + 1) * sizeof(_T));
	//U�������ţ�����ֱ��д��U��
	_svd_3(A_1, h, w, w+1, Sigma, Vt_1, &iResult, eps);
	//Disp(A_1, h, h, "U");
	//Disp(Sigma,1,w,"S");
	//Disp(Vt_1, w, w, "Vt");
	//Disp(&A_1[9*2000], 1, oInfo.h_A);
	memcpy(oSVD.S, Sigma, iMin_Size * sizeof(_T));	
	if (!bHor)
	{//��A_1ת�þ���U��Ȼ������ʱUֻȡ������	mmmm
	//											mmmm
	//											mmmm
	//											mmmm
		if(oSVD.U)
			Matrix_Transpose(A_1, w, h, (_T*)oSVD.U);
		//Disp((_T*)oInfo.U, oInfo.h_Min_U, oInfo.w_Min_U, "U");
		if (oSVD.Vt)
			memcpy(oSVD.Vt, Vt_1, oSVD.h_Min_Vt * oSVD.h_Min_Vt * sizeof(_T));
	}else
	{//
		//��ʱ��Vt��ֵ��U
		//Disp((_T*)oInfo.Vt, h, h);
		Matrix_Transpose(Vt_1, w,w, (_T*)oSVD.U);
		memcpy(oSVD.Vt, A_1, (w + 1) * h * sizeof(_T));

	}
END:
	if (A_1)
		Free(&oMatrix_Mem, A_1);
	if (Sigma)
		Free(&oMatrix_Mem, Sigma);
	if (Vt_1)
		Free(&oMatrix_Mem, Vt_1);
}

//template<typename _T> void svd_4(_T* A, int h, int w, _T U[], _T S[], _T Vt[], int* pbSuccess, _T eps)
//{//�˴�Ϊsvd����ڡ�����Ҫʱ����ת��
//	_T* A_1, * Vt_1=NULL, * Sigma;
//	int x, y, iFlag;
//	int iMax_Size = Max(h, w), iMin_Size = Min(h, w);
//
//	A_1 = (_T*)pMalloc(&oMatrix_Mem, iMax_Size * iMax_Size * sizeof(_T));
//	Sigma = (_T*)pMalloc(&oMatrix_Mem, iMax_Size * sizeof(_T));
//	if (!A_1 || !Sigma)
//		goto END;
//	memset(A_1, 0, iMax_Size * iMax_Size * sizeof(_T));
//	if (h < w)
//	{
//		memcpy(A_1, A, h * w * sizeof(_T));
//		iFlag = 1;
//		//Malloc_1(oPtr, w * h * sizeof(_T), Vt_1);
//		Vt_1 = (_T*)pMalloc(&oMatrix_Mem, w * h * sizeof(_T));
//	}
//	else
//	{
//		Matrix_Transpose(A, h, w, A_1);
//		int iTemp = h;
//		h = w;
//		w = iTemp;
//		iFlag = 0;
//		//Vt_1 = (_T*)malloc(h * h * sizeof(_T));
//		//Malloc_1(oPtr, h * h * sizeof(_T), Vt_1);
//		Vt_1 = (_T*)pMalloc(&oMatrix_Mem,h * h * sizeof(_T));
//		//Transpose
//		/*for (y = 0; y < h; y++)
//			for (x = 0; x < w; x++)
//				A_1[x * h + y] = A[y * w + x];*/
//	}
//	if (!Vt_1)
//		goto END;
//	//�����Ժ����ǿ���С������
//	//  **********************
//	//  **********************
//	//  **********************
//	//  **********************
//
//	int iResult;
//	_svd_3(A_1, h, w,w, Sigma, Vt_1, &iResult, eps);
//	//Disp(A_1, w, w);
//	if (iFlag)
//	{//h<w
//		memcpy(S, Sigma, h * sizeof(_T));
//		//�����Ժ�A_1����U
//		memcpy(Vt, A_1, w * w * sizeof(_T));
//		if (U)
//		{
//			//��Vt_1ת�þ���U
//			for (y = 0; y < h; y++)
//				for (x = 0; x < h; x++)
//					U[y * h + x] = Vt_1[x * w + y];
//		}
//	}
//	else
//	{
//		memcpy(S, Sigma, h * sizeof(_T));
//		//��A_1ת�þ���U
//		if(U)
//			for (y = 0; y < w; y++)
//				for (x = 0; x < w; x++)
//					U[y * w + x] = A_1[x * w + y];
//		//Matrix_Transpose(Vt_1, h, w, Vt);
//		//memcpy(Vt, Vt_1, h*h*sizeof(_T));
//
//		//Disp(Vt, m, m);
//	}
//	if (pbSuccess)
//		*pbSuccess = iResult;
//
//END:
//	if(A_1)
//		Free(&oMatrix_Mem, A_1);
//	if(Sigma)
//		Free(&oMatrix_Mem, Sigma);
//	if(Vt_1)
//		Free(&oMatrix_Mem, Vt_1);
//	return;
//}

template<typename _T> void Matrix_Multiply(_T* A, int ma, int na, _T* B, int nb, _T* C)
{//Amn x Bno = Cmo
	int y, x, i;
	_T fValue, * C_Dup;	// = (_T*)malloc(ma * nb * sizeof(_T));
	//Light_Ptr oPtr = oMatrix_Mem;
	//Malloc_1(oPtr, ma * nb * sizeof(_T), C_Dup);
	C_Dup = (_T*)pMalloc(&oMatrix_Mem, ma * nb * sizeof(_T));
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
	Free(&oMatrix_Mem, C_Dup);
	return;
}

void SB_Matrix()
{//����Ǹ�ɵ�Ʒ�����������ƭtemplate
	Get_Inv_Matrix_Row_Op((float*)NULL, (float*)NULL, 0, NULL);
	Get_Inv_Matrix_Row_Op((double*)NULL, (double*)NULL, 0, NULL);

	Matrix_Multiply((float*)NULL, 0, 0, (float*)NULL, 0, (float*)NULL);
	Matrix_Multiply((double*)NULL, 0, 0, (double*)NULL, 0, (double*)NULL);

	bIs_Orthogonal((float*)NULL, 0);
	bIs_Orthogonal((double*)NULL, 0);

	svd_3((float*)NULL, {});
	svd_3((double*)NULL, {});

	Test_SVD((double*)NULL, {});
	Test_SVD((float*)NULL, {});

	SVD_Allocate((double*)NULL, 0, 0, NULL);
	SVD_Allocate((float*)NULL, 0, 0, NULL);

	Elementary_Row_Operation_1((double*)NULL, 0, 0, (double*)NULL);
	Elementary_Row_Operation_1((float*)NULL, 0, 0, (float*)NULL);

}
template<typename _T>void Test_SVD(_T A[], SVD_Info oSVD, int* piResult, double eps)
{//��һ�ּ򻯱�ʾ���˴�����һ�·ֽ����Ƿ����Ԥ��
	int y, x,iResult=1;
	_T* A_1=NULL;
	union {
		_T* S;
		_T* SVt;
		_T* US;
	};
	if (oSVD.h_A > oSVD.w_A)
	{//���Σ�A= U x (S x Vt) ������棬����ǰ��
		
		SVt = (_T*)pMalloc(&oMatrix_Mem, oSVD.w_Min_S * oSVD.w_Min_S * sizeof(_T));
		memset(S, 0, oSVD.w_Min_S* oSVD.w_Min_S * sizeof(_T));
		for (y = 0; y < oSVD.w_Min_S; y++)
			S[y * oSVD.w_Min_S + y] = ((_T*)oSVD.S)[y];
		Disp(S,oSVD.w_Min_S, oSVD.w_Min_S, "S");
		Matrix_Multiply(S, oSVD.w_Min_S, oSVD.w_Min_S, (_T*)oSVD.Vt,oSVD.h_Min_Vt, SVt);
		Disp(SVt, oSVD.w_Min_S, oSVD.w_Min_S, "SVt");
		A_1 = (_T*)pMalloc(&oMatrix_Mem, oSVD.h_A * oSVD.w_A * sizeof(_T));
		Matrix_Multiply((_T*)oSVD.U, oSVD.h_Min_U, oSVD.w_Min_U, SVt, oSVD.w_Min_Vt,A_1);
		Disp(A_1, oSVD.h_A, oSVD.w_A, "A_1");
		Free(&oMatrix_Mem, SVt);
	}else
	{//����
		S= (_T*)pMalloc(&oMatrix_Mem, oSVD.w_Min_S * oSVD.w_Min_S * sizeof(_T));
		memset(S, 0, oSVD.w_Min_S* oSVD.w_Min_S * sizeof(_T));
		for (y = 0; y < oSVD.w_Min_S; y++)
			((_T*)S)[y * oSVD.w_Min_S + y] =  ((_T*)oSVD.S)[y];
		//Disp(S, oSVD.w_Min_S, oSVD.w_Min_S, "S");
		Matrix_Multiply((_T*)oSVD.U, oSVD.h_Min_U, oSVD.w_Min_U, S,oSVD.w_Min_S,US);
		Disp(US, oSVD.h_Min_U, oSVD.w_Min_S,"UxS");
		A_1 = (_T*)pMalloc(&oMatrix_Mem, oSVD.h_A * oSVD.w_A * sizeof(_T));
		Matrix_Multiply(US, oSVD.h_Min_U, oSVD.w_Min_S, (_T*)oSVD.Vt, oSVD.w_Min_Vt, A_1);
		Disp(A_1, oSVD.h_A, oSVD.w_A, "A_1");
		Free(&oMatrix_Mem, US);
	}

	if (bIs_Orthogonal((_T*)oSVD.U, oSVD.h_Min_U, oSVD.w_Min_U))
		printf("U ����\n");
	else
		printf("U ������\n");

	if (bIs_Orthogonal((_T*)oSVD.Vt, oSVD.h_Min_Vt, oSVD.w_Min_Vt))
		printf("Vt ����\n");
	else
		printf("Vt ������\n");

	for (y = 0; y < oSVD.h_A; y++)
	{
		for (x = 0; x < oSVD.w_A; x++)
		{
			if (abs(A_1[y * oSVD.w_A + x] - A[y * oSVD.w_A + x]) > eps)
			{
				printf("Correct:%f Error:%f\n", A[y * oSVD.w_A + x], A_1[y * oSVD.w_A + x]);
				iResult = 0;
			}
		}
	}

	if(A_1)
		Free(&oMatrix_Mem, A_1);
	if (piResult)
		*piResult = iResult;
}
template<typename _T>void Test_SVD(_T A[], int h, int w, _T U[], _T S[], _T Vt[],int *piResult,double eps)
{//����SVD�ֽ���
	_T* pTemp = (_T*)pMalloc(&oMatrix_Mem, h * w * sizeof(_T));
	int y, x,iResult;
	Disp(S, h, w, "S,���Ĺ۲�����ֵ,����0ֵ��λ�ÿ�������Vt��Ӧ����");
	Matrix_Multiply(U, h, h, S, w, pTemp);
	Disp(pTemp, h, w, "UxS");
	Matrix_Multiply(pTemp, h, w, Vt, w,pTemp);
	Disp(pTemp, h, w, "USVt");
	iResult = 1;
	if (bIs_Orthogonal(U, h))
		printf("U ����\n");
	else
		printf("U ������\n");

	if (bIs_Orthogonal(Vt, w))
		printf("Vt ����\n");
	else
		printf("Vt ������\n");
	
	for (y = 0; y < h; y++)
	{
		for (x = 0; x < w; x++)
		{
			if (abs(pTemp[y * w + x] - A[y * w + x]) > eps)
			{
				printf("Correct:%f Error:%f\n", A[y * w + x], pTemp[y * w + x]);
				iResult = 0;
				//goto END;
			}
		}
	}		

	Free(&oMatrix_Mem, pTemp);
	if (piResult)
		*piResult = iResult;
	return;
}
void Free_SVD(SVD_Info* poInfo)
{
	Free(&oMatrix_Mem, poInfo->U);
	Free(&oMatrix_Mem, poInfo->S);
	Free(&oMatrix_Mem, poInfo->Vt);
}

template<typename _T>void SVD_Allocate(_T* A, int h, int w, SVD_Info* poInfo)
{
	_T* U, * S, * Vt;
	SVD_Info oInfo;
	oInfo.A = A;
	oInfo.h_A = h;
	oInfo.w_A = w;
	if (h >= w)
	{//��׼��
		U = (_T*)pMalloc(&oMatrix_Mem, w * h * sizeof(_T));
		oInfo.h_Min_U = h;
		oInfo.w_Min_U = w;

		Vt = (_T*)pMalloc(&oMatrix_Mem, w * w * sizeof(_T));
		oInfo.h_Min_Vt = oInfo.w_Min_Vt = w;
	}
	else
	{//
		U = (_T*)pMalloc(&oMatrix_Mem, h * h * sizeof(_T));
		oInfo.h_Min_U = oInfo.w_Min_U = h;
		Vt = (_T*)pMalloc(&oMatrix_Mem, (h + 1) * w * sizeof(_T));
		oInfo.h_Min_Vt = h + 1;
		oInfo.w_Min_Vt = w;
	}
	oInfo.U = U;
	oInfo.Vt = Vt;
	S = (_T*)pMalloc(&oMatrix_Mem, oInfo.w_Min_S = min(h, w));
	oInfo.S = S;
	*poInfo = oInfo;
}
