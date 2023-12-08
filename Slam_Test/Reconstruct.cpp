//�ؽ��õĴ��� 
#pragma once
#include "Reconstruct.h"

#define SQR(Value) (Value)*(Value)

template<typename _T> void Sample_XY(_T Point_1[][2], _T Point_2[][2], short Sample_Index[], int iCount, _T X_rand[][2], _T Y_rand[][2], int iRan_Count = 8)
{//��һ�ѵ㼯�г���˸��������ǰ��
	int i, iNo_1,iNo_2, iLast_Index = iCount - 1;
	for (i = 0; i < iRan_Count; i++)
	{
		iNo_2 = iRandom(i, iLast_Index);
		iNo_1 = Sample_Index[iNo_2];
		X_rand[i][0] = Point_1[iNo_1][0];
		X_rand[i][1] = Point_1[iNo_1][1];
		Y_rand[i][0] = Point_2[iNo_1][0];
		Y_rand[i][1] = Point_2[iNo_1][1];
		swap(Sample_Index[i], Sample_Index[iNo_2]);		
	}
	//Disp(Sample_Index, iRan_Count, 1);
	return;
}

template<typename _T> void Normalize_Point_3(_T Point[][2], int iCount, _T M[3][3], _T Norm_Point[][2])
{//��һ��㣬ͶӰ���߳�Ϊ 2* sqrt(2)��ͶӰƽ����,��ô���еĵ㶼�Ѿ�ʵ��ĳ�������ϵĹ�һ��
	_T Centroid[2] = { 0 };
	int i;
	//���������
	for (i = 0; i < iCount; i++)
		Centroid[0] += Point[i][0], Centroid[1] += Point[i][1];
	Centroid[0] /= iCount, Centroid[1] /= iCount;

	//ÿһ�㵽�����и����룬��8��ľ���ƽ����
	_T rms_mean_dist = 0;
	for (i = 0; i < iCount; i++)
		rms_mean_dist += SQR(Point[i][0] - Centroid[0]) + SQR(Point[i][1] - Centroid[1]);
	rms_mean_dist = (_T)sqrt(rms_mean_dist / iCount);	//�˴��ȳ���8�ٿ����Ŷԣ���Ϲ��
	//�����Ժ�rms_mean_dist����8�㵽���ĵ�ƽ������

	//������norm_factor���ȼ�һ�£����еĵ㵽���ľ��붼��rms_mean_dist��
	//�����һ��������,�߳�Ϊ2*rms_mean_dist�����ǽ���������Σ�����ƽ�棩
	//��Ϊ2*sqrt(2)��С�� ��ôrms_mean_dist��Ӧsqrt(2)��ô��Ȼ��norm_factor
	//����ָÿһ��ԭ�������������.�ع�һ���Լ�������ƽ��ķ�����
	//a = 1920 / (fMax_x - fMin_x);	
	//b = -1080 / (fMax_y - fMin_y);	������֮������������Ļ�Ĵ�С��Ϊ����
	_T norm_factor = (_T)sqrt(2.0) / rms_mean_dist;
	//���M���󳤵ú�������ڲΣ�������㷨���һ��ӡ֤
	// fx 0  cx
	//	0 fy cy
	//	0 0  1
	M[0][0] = norm_factor, M[0][1] = 0, M[0][2] = -norm_factor * Centroid[0];
	M[1][0] = 0, M[1][1] = norm_factor, M[1][2] = -norm_factor * Centroid[1];
	M[2][0] = 0, M[2][1] = 0, M[2][2] = 1;
	//Disp((float*)M, 3, 3, "M");

	//���
	for (i = 0; i < iCount; i++)
	{
		Norm_Point[i][0] = Point[i][0] * M[0][0] + M[0][2];
		Norm_Point[i][1] = Point[i][1] * M[1][1] + M[1][2];
		//printf("%f %f\n", Norm_Point[i][0], Norm_Point[i][1]);
	}
	return;
}
template<typename _T> void Normalize_Point_1(_T Point[][2], int iCount, _T M[3][3], _T Norm_Point[][2])
{//������һ�ι�񻯣��γ�һ������ڲ�M
	_T Centroid[2] = { 0 };
	int i;
	//���������
	for (i = 0; i < iCount; i++)
		Centroid[0] += Point[i][0], Centroid[1] += Point[i][1];
	Centroid[0] /= iCount, Centroid[1] /= iCount;

	//ÿһ�㵽�����и����룬��8��ľ���ƽ����
	_T rms_mean_dist = 0;
	for (i = 0; i < iCount; i++)
		rms_mean_dist += SQR(Point[i][0] - Centroid[0]) + SQR(Point[i][1] - Centroid[1]);
	rms_mean_dist = (_T)sqrt(rms_mean_dist / iCount);	//�˴��ȳ���8�ٿ����Ŷԣ���Ϲ��
	//�����Ժ�rms_mean_dist����8�㵽���ĵ�ƽ������

	//������norm_factor���ȼ�һ�£����еĵ㵽���ľ��붼��rms_mean_dist��
	//�����һ��������,�߳�Ϊ2*rms_mean_dist�����ǽ���������Σ�����ƽ�棩
	//��Ϊ2*sqrt(2)��С�� ��ôrms_mean_dist��Ӧsqrt(2)��ô��Ȼ��norm_factor
	//����ָÿһ��ԭ�������������.�ع�һ���Լ�������ƽ��ķ�����
	//a = 1920 / (fMax_x - fMin_x);	
	//b = -1080 / (fMax_y - fMin_y);	������֮������������Ļ�Ĵ�С��Ϊ����
	_T norm_factor = (_T)sqrt(2.0) / rms_mean_dist;
	//���M���󳤵ú�������ڲΣ�������㷨���һ��ӡ֤
	// fx 0  cx
	//	0 fy cy
	//	0 0  1
	M[0][0] = norm_factor, M[0][1] = 0, M[0][2] = -norm_factor * Centroid[0];
	M[1][0] = 0, M[1][1] = norm_factor, M[1][2] = -norm_factor * Centroid[1];
	M[2][0] = 0, M[2][1] = 0, M[2][2] = 1;
	//Disp((float*)M, 3, 3, "M");

	//���
	for (i = 0; i < iCount; i++)
	{
		Norm_Point[i][0] = Point[i][0] * M[0][0] + M[0][2];
		Norm_Point[i][1] = Point[i][1] * M[1][1] + M[1][2];
		//printf("%f %f\n", Norm_Point[i][0], Norm_Point[i][1]);
	}
	//Disp((float*)Norm_Point, iCount, 2, "Norm Point");
	return;
}
template<typename _T> void Estimate_F(_T Point_1[][2], _T Point_2[][2], int iCount, _T F[3 * 3], _T(*Norm_Point_1)[2], _T(*Norm_Point_2)[2], Light_Ptr oPtr)
{//��������������Ѿ��걸��������һ����Ѿ����ҵ�һ������F��ʹ��p2'*F*p1=0���Ѿ��������ʽ������ҪNorm_Point�ռ�
	_T M_1[3][3], M_2[3][3];
	unsigned char* pCur;
	int i, iSize, bUse_Matrix_Mem;
	
	bUse_Matrix_Mem = (!Norm_Point_1 || !Norm_Point_2 || !oPtr.m_pBuffer) ? 1 : 0;
	if (bUse_Matrix_Mem)
	{
		iSize = iCount * 4 * sizeof(_T) +	//Norm_Point
			iCount * 9 * sizeof(_T) +		//A
			128 * 3;
		Attach_Light_Ptr(oPtr, (unsigned char*)pMalloc(&oMatrix_Mem, iSize), iSize, -1);
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);
		Norm_Point_1 = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);
		Norm_Point_2 = (_T(*)[2])pCur;
	}
	Normalize_Point_1(Point_1, iCount, M_1, Norm_Point_1);
	Normalize_Point_1(Point_2, iCount, M_2, Norm_Point_2);

	//�����¿��Կ���������������ĵĹ�񻯺�������ֵ��һ���ӽ�
	//Disp((_T*)Norm_Point_1, iCount, 2, "Norm_Point_1");
	//Disp((_T*)Norm_Point_2, iCount, 2, "Norm_Point_2");
	//Disp((_T*)Point_2, iCount, 2, "Point_2");
	//����ϵ��ϵ������
	//Point_1 x F(3x3) x Point_2' չ����Ϊ������
	//Disp((float*)pNorm_Point_1, iCount, 2);
	_T(*A)[9];	// , B[8] = { 0 };
	Malloc(oPtr, iCount * 9 * sizeof(_T), pCur);
	A = (_T(*)[9])pCur;
	for (i = 0; i < iCount; i++)
	{
		A[i][0] = Norm_Point_2[i][0] * Norm_Point_1[i][0];
		A[i][1] = Norm_Point_2[i][0] * Norm_Point_1[i][1];
		A[i][2] = Norm_Point_2[i][0];
		A[i][3] = Norm_Point_2[i][1] * Norm_Point_1[i][0];
		A[i][4] = Norm_Point_2[i][1] * Norm_Point_1[i][1];
		A[i][5] = Norm_Point_2[i][1];
		A[i][6] = Norm_Point_1[i][0];
		A[i][7] = Norm_Point_1[i][1];
		A[i][8] = 1;
	}

	//��ʽ���
	_T Basic_Solution[7 * 9];
	//int iBasic_Solution_Count, iResult;
	//ע�⣬��Ax=0 �е�AΪ���Σ���ʱ���Բ���svd��⣬���Ǹ���η����������ϵ�Ĺ��̣�Ҳ������׼
	//Solve_Linear_Solution_Construction((float*)A, iCount, 9, B, &iResult, Basic_Solution, &iBasic_Solution_Count, NULL);
	SVD_Info oSVD;
	SVD_Alloc<_T>(iCount, 9, &oSVD);
	//Disp((_T*)A, 8, 9, "A");
	svd_3((_T*)A, oSVD, &oSVD.m_bSuccess);
	//Disp((_T*)oSVD.Vt, 9, 9, "Vt");
	memcpy(Basic_Solution, ((_T(*)[9])oSVD.Vt)[8], 9 * sizeof(_T));
	//��һ��SVD�Ѿ�����ʹ��
	Free_SVD(&oSVD);

	//�ɽ�����ϳ�һ��F����
	_T Temp_1[3 * 3], 
		F_1[3 * 3] = { Basic_Solution[0],Basic_Solution[1],Basic_Solution[2],
		Basic_Solution[3],Basic_Solution[4],Basic_Solution[5],
		Basic_Solution[6],Basic_Solution[7],Basic_Solution[8] };
	//Disp((_T*)E_t, 3, 3, "Et");
	SVD_Alloc<_T>(3, 3, &oSVD);
	svd_3(F_1, oSVD,&oSVD.m_bSuccess);

	//����F��������ֵ�Ĵ�����΢û��ô�࣬���������һ������ֵ����
	_T S[3 * 3] = { ((_T*)oSVD.S)[0],0,0,
				0,((_T*)oSVD.S)[1],0,
				0,0,0 };
	Matrix_Multiply((_T*)oSVD.U, 3, 3 ,S, 3, Temp_1);
	Matrix_Multiply(Temp_1, 3, 3, (_T*)oSVD.Vt, 3, F);

	//���� M2' * F * M1�ָ�ԭ����
	Matrix_Transpose((_T*)M_2, 3, 3, (_T*)M_2);
	Matrix_Multiply((_T*)M_2, 3, 3, F, 3, Temp_1);
	Matrix_Multiply(Temp_1, 3, 3, (_T*)M_1, 3, F);
	Free_SVD(&oSVD);
	if (bUse_Matrix_Mem)
		Free(&oMatrix_Mem, oPtr.m_pBuffer);
	//Disp(F, 3, 3, "F");
	//Disp_Mem(&oMatrix_Mem, 0);
}
template<typename _T> void Estimate_E(_T Point_1[][2], _T Point_2[][2], int iCount, _T E[3 * 3], _T (*Norm_Point_1)[2], _T (*Norm_Point_2)[2], Light_Ptr oPtr)
{//��һ������ˬ�Ľӿڣ�Norm_Point_1 ��Norm_Point_2���Բ����ռ䣬oPtr����û��
//���������Ѿ����걸�ӿڣ�����һ��� x1,x2�����һ������E�� ʹ�� x2'*E*x1=0
	_T M_1[3][3], M_2[3][3];
	unsigned char* pCur;
	int i, iSize,bUse_Matrix_Mem;

	bUse_Matrix_Mem = (!Norm_Point_1 || !Norm_Point_2 || !oPtr.m_pBuffer) ? 1 : 0;
	if (bUse_Matrix_Mem)
	{
		iSize = iCount * 4 * sizeof(_T) +	//Norm_Point
			iCount * 9 * sizeof(_T) +		//A
			128 * 3;

		Attach_Light_Ptr(oPtr,(unsigned char*)pMalloc(&oMatrix_Mem,iSize), iSize, -1);
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);
		Norm_Point_1 = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);
		Norm_Point_2= (_T(*)[2])pCur;
	}
	
	Normalize_Point_1(Point_1, iCount, M_1, Norm_Point_1);
	Normalize_Point_1(Point_2, iCount, M_2, Norm_Point_2);
	
	//�����¿��Կ���������������ĵĹ�񻯺�������ֵ��һ���ӽ�
	//Disp((_T*)Norm_Point_1, iCount, 2, "Norm_Point_1");
	//Disp((_T*)Norm_Point_2, iCount, 2, "Norm_Point_2");
	//Disp((_T*)Point_2, iCount, 2, "Point_2");
	//����ϵ��ϵ������
	//Point_1 x E(3x3) x Point_2' չ����Ϊ������
	//Disp((float*)pNorm_Point_1, iCount, 2);
	_T(*A)[9];	//, B[8] = { 0 };
	Malloc(oPtr, iCount * 9 * sizeof(_T), pCur);
	A = (_T(*)[9])pCur;
	for (i = 0; i < iCount; i++)
	{
		A[i][0] = Norm_Point_2[i][0] * Norm_Point_1[i][0];
		A[i][1] = Norm_Point_2[i][0] * Norm_Point_1[i][1];
		A[i][2] = Norm_Point_2[i][0];
		A[i][3] = Norm_Point_2[i][1] * Norm_Point_1[i][0];
		A[i][4] = Norm_Point_2[i][1] * Norm_Point_1[i][1];
		A[i][5] = Norm_Point_2[i][1];
		A[i][6] = Norm_Point_1[i][0];
		A[i][7] = Norm_Point_1[i][1];
		A[i][8] = 1;
	}
	
	//Disp((_T*)A, 8, 9);
	//��ʽ���
	_T Basic_Solution[9 * 9];
	
	////����,��һ�λ���Ч�����ԭ�򣬿�������������
	//int iRank;
	//iRank = iGet_Rank((_T*)A, 8, 9);
	//if (iRank != 8)
	//	printf("��С��8:%d\n", iRank);
	//else
	//	printf("Done");

	
	//int iBasic_Solution_Count, iResult;

	//ע�⣬��Ax=0 �е�AΪ���Σ���ʱ���Բ���svd��⣬���Ǹ���η����������ϵ�Ĺ��̣�Ҳ������׼
	//Solve_Linear_Solution_Construction((float*)A, iCount, 9, B, &iResult, Basic_Solution, &iBasic_Solution_Count, NULL);
	/*float* U_A = (float*)malloc(iCount * iCount * sizeof(float)),
		* S_A = (float*)malloc(iCount * 9 * sizeof(float)),
		* Sigma = (float*)malloc(3 * sizeof(float)),
		Vt_A[9][9];*/
	SVD_Info oSVD;
	SVD_Alloc<_T>(iCount, 9, &oSVD);
	//Disp((_T*)A, 8, 9, "A");
	svd_3((_T*)A, oSVD,&oSVD.m_bSuccess);
	//Disp((_T*)oSVD.Vt, 9, 9, "Vt");
	memcpy(Basic_Solution, ((_T(*)[9])oSVD.Vt)[8], 9 * sizeof(_T));

	//Matrix_Multiply((_T*)A, iCount, 9, Basic_Solution, 1, Basic_Solution);
	//Disp(Basic_Solution, 1, 8, "Solution");	//ע�⣬[8]��û��

	/*_T Temp_3[3];
	for (int i = 0; i < 3; i++)
	{	
		_T Point_back_1[3] = { Norm_Point_1[i][0],Norm_Point_1[i][1],(_T)1};
		_T Point_back_2[3] = { Norm_Point_2[i][0],Norm_Point_2[i][1],(_T)1};

		Matrix_Multiply((_T*)Point_back_2, 1, 3, Basic_Solution, 3, Temp_3);
		Matrix_Multiply(Temp_3, 1, 3, (_T*)Point_back_2, 1,Temp_3);		
	}*/

	//��һ��SVD�Ѿ�����ʹ��
	Free_SVD(&oSVD);	
	
	//�ɽ�����ϳ�һ��E'����ע�⣬������˵���һ�ٵ�ת����ɺ���ѽ�
	//��ȷ��E = [	e1 e2 e3		���˴��� [	e1 e4 e7	��������E'
	//				e3 e4 e5					e2 e5 e8
	//				e6 e7 e8 ]					e3 e6 e9]
	_T E_raw[3*3], Temp_1[3 * 3], Temp_2[3 * 3],
		E_t[3 * 3] = { Basic_Solution[0],Basic_Solution[3],Basic_Solution[6],
		Basic_Solution[1],Basic_Solution[4],Basic_Solution[7],
		Basic_Solution[2],Basic_Solution[5],Basic_Solution[8] };
	//Disp((_T*)E_t, 3, 3, "Et");

	//Ҫ�Ƶ���������E_raw�Ǹ�ʲô����
	// NP_2' * E * NP_1 =0		������������Ż����⣬�ڹ�񻯵�����
	//չ����񻯵�  (M_2 * P2)' * E * M_1 * P1=0  �ָ��ݾ���˵�ת�õ���ת�ú����ĳ���
	//=>  P2' * M_2' * E * M_1 * P1=0	���Խ��м���Ϊ����E_raw
	//=�� E_raw = M_2' * E * M_1
	// De-normalize to image points.
	Matrix_Transpose((_T*)M_2, 3, 3, Temp_1);
	Matrix_Transpose(E_t, 3, 3, Temp_2);
	Matrix_Multiply(Temp_1, 3, 3, Temp_2, 3, E_raw);
	Matrix_Multiply(E_raw, 3, 3, (_T*)M_1, 3, E_raw);
	
	//Disp(E_raw, 3, 3, "E_raw");
	//�ڶ���SVD, ΪE_raw
	SVD_Alloc<_T>(3, 3, &oSVD);
	svd_3(E_raw, oSVD);
	//Disp((_T*)oSVD.S, 1, 3,"S");
	//Disp((_T*)oSVD.Vt, 3, 3, "Vt");

	_T S[3 * 3] = {};
	//Ȼ�����һ��ɧ�������Խǻ�������Ԫ�ؽ��е�������ʱ��֪����
	//������QR�ֽ�������һ���������Ϊ������������ظ�����Щ����ʱ
	//Ӧ�ý������ظ������ƽ������ʱ�����С
	S[0] = S[4] = ( ((_T*)oSVD.S)[0]  + ((_T*)oSVD.S)[1]) / 2.f;
	S[8] = 0.f;

	//Disp(S, 3, 3, "S");
	Matrix_Multiply((_T*)oSVD.U, 3, 3, S, 3, Temp_1);
	Matrix_Multiply(Temp_1, 3, 3, (_T*)oSVD.Vt, 3, E);
	Free_SVD(&oSVD);
	if (bUse_Matrix_Mem)
		Free(&oMatrix_Mem, oPtr.m_pBuffer);
	//Disp(E, 3, 3, "E");
	return;

}

template<typename _T> void Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, _T H[3 * 3], _T (*Norm_Point_1)[2], _T (*Norm_Point_2)[2], Light_Ptr oPtr)
{//�������������һ��Homograph�������� min ( ||P1*X-P2||^2)
//�����ϸýӿ��Ѿ��걸������һ��㣬����һ������H��s.t. H*p1=p2
	_T M_1[3][3], M_2[3][3];
	int i, j,iResult,bUse_Matrix_Mem;
	
	bUse_Matrix_Mem = (!Norm_Point_1 || !Norm_Point_2 || !oPtr.m_pBuffer) ? 1 : 0;
	if (bUse_Matrix_Mem)
	{
		unsigned char* pCur;
		int iSize = iCount * 4 * sizeof(_T) +	//Norm_Point
			iCount * 9 * sizeof(_T) +		//A
			128 * 3;

		Attach_Light_Ptr(oPtr, (unsigned char*)pMalloc(&oMatrix_Mem, iSize), iSize, -1);
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);
		Norm_Point_1 = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);
		Norm_Point_2 = (_T(*)[2])pCur;
	}

	//�����¿��Կ���������������ĵĹ�񻯺�������ֵ��һ���ӽ�
	Normalize_Point_3(Point_1, iCount, M_1, Norm_Point_1);
	Normalize_Point_3(Point_2, iCount, M_2, Norm_Point_2);
	//Disp((_T*)Norm_Point_1, iCount, 2, "Norm_Point_1");
	//Disp((_T*)Norm_Point_2, 4, 2, "Norm_Point_2");

	_T(*A)[9];	// , * S, * Vt;
	unsigned char* pCur;

	Malloc(oPtr, iCount * 9 * sizeof(_T), pCur);
	A = (_T(*)[9])pCur;

	memset(A, 0, iCount * 2 * 9 * sizeof(_T));
	for (i = 0, j = iCount; i < iCount; i++, j++)
	{
		_T s_0 = Norm_Point_1[i][0];
		_T s_1 = Norm_Point_1[i][1];
		_T d_0 = Norm_Point_2[i][0];
		_T d_1 = Norm_Point_2[i][1];

		A[i][0] = -s_0;
		A[i][1] = -s_1;
		A[i][2] = -1;
		A[i][6] = s_0 * d_0;
		A[i][7] = s_1 * d_0;
		A[i][8] = d_0;

		A[j][3] = -s_0;
		A[j][4] = -s_1;
		A[j][5] = -1;
		A[j][6] = s_0 * d_1;
		A[j][7] = s_1 * d_1;
		A[j][8] = d_1;
	}
	//Disp((_T*)A, 8, 9, "A");
	SVD_Info oSVD;
	//SVD_Allocate( (_T*)(A) , iCount*2, 9, &oSVD);
	SVD_Alloc<_T>(iCount * 2, 9, &oSVD);

	svd_3( (_T*)A, oSVD, &iResult);
	//Disp((_T*)oSVD.Vt, oSVD.h_Min_Vt, oSVD.w_Min_Vt, "Vt");
	
	_T H_t[3 * 3];
	memcpy(H_t, &((_T*)oSVD.Vt)[8 * 9], 9 * sizeof(_T));
	Free_SVD(&oSVD);
	_T Temp_1[3 * 3];
	Get_Inv_Matrix_Row_Op((_T*)M_2, (_T*)M_2, 3, &iResult);
	Matrix_Multiply((_T*)M_2, 3, 3, H_t, 3, Temp_1);
	Matrix_Multiply(Temp_1, 3, 3, (_T*)M_1, 3, H);
	if (bUse_Matrix_Mem)
		Free(&oMatrix_Mem, oPtr.m_pBuffer);
	return;
}

template<typename _T>
void Get_Residual_H(_T Point_1[][2], _T Point_2[][2], int iCount, _T H[3 * 3], _T Residual[])
{//��H����������Ʋ�����Sampson���룬�õĻ���ŷ�Ͼ��룬����֤
	_T H_00 = H[0], H_01 = H[1], H_02 = H[2], H_10 = H[3],
		H_11 = H[4], H_12 = H[5], H_20 = H[6], H_21 = H[7], H_22 = H[8];

	for (int i = 0; i < iCount; i++)
	{
		_T s_0 = Point_1[i][0];
		_T s_1 = Point_1[i][1];
		_T d_0 = Point_2[i][0];
		_T d_1 = Point_2[i][1];

		//�˴������˸�H * Point_1 �ı任
		_T pd_0 = H_00 * s_0 + H_01 * s_1 + H_02;
		_T pd_1 = H_10 * s_0 + H_11 * s_1 + H_12;
		_T pd_2 = H_20 * s_0 + H_21 * s_1 + H_22;

		//�˴���ͶӰ
		_T inv_pd_2 = 1.f / pd_2;
		_T dd_0 = d_0 - pd_0 * inv_pd_2;
		_T dd_1 = d_1 - pd_1 * inv_pd_2;

		//ŷ�Ͼ���
		Residual[i] = dd_0 * dd_0 + dd_1 * dd_1;
	}
}
template<typename _T>
void Get_Support(_T Residual[], int iCount, _T fMax_Residual, int* piInlier_Count, float* pfResidual_Sum)
{//����ûʲôӪ�������Ǳȸ���
	int iInlier_Count = 0;
	_T fSum = 0;
	for (int i = 0; i < iCount; i++)
		if (Residual[i] <= fMax_Residual)
			iInlier_Count++, fSum += Residual[i];
	if (piInlier_Count)
		*piInlier_Count = iInlier_Count;
	if (pfResidual_Sum)
		*pfResidual_Sum = (float)fSum;
}
int iCompute_Num_Trials(int iInlier_Count, int iSample_Count, int kMinNumSamples)
{//����dyn_max_num_trials,��̬���Դ���
	//��Inlier����fInlier_Ratio����fInlier_Ratio�����ҵ�ƥ���ı���
	float fInlier_Ratio = (float)iInlier_Count / iSample_Count;
	float nom = 1.f - 0.99f;
	//Inlier����denom���١� fInlier_Ration ^ (4|8)���ߴκ���x^(4|8)��[0,1]����ͼ�񼴿�
	float denom = 1.f - (float)pow(fInlier_Ratio, kMinNumSamples);
	int iRet;
	if (denom <= 0)
		return 1;
	if (denom == 1.0)	//��ʱ��ʾInlier_Ratio=0���Ҳ����κ�ƥ���
		return (int)(std::numeric_limits<size_t>::max());	//���ظ����ֵ
	
	//printf("nom:%f denom:%f log(nom):%f log(denom):%f\n",nom,denom,log(nom),log(denom));
	//log(nom)= -4.605171, ��Ϊ�� denom<1, log��denom) ��Ϊ��
	//inlier ����denom�ݼ���|log(denom)| ����������ݼ�
	//��ʵ���������������޷Ǿ���ƥ��ĵ���Խ�࣬���������Ĵ�����Լ�١�����Ȥ�����Լ��������
	iRet = (int)ceil(log(nom) / log(denom) * 3.f);
	return iRet;
}
template<typename _T>void Normalize_Point(_T(*pPoint_1)[2], _T(*pPoint_2)[2], int iCount, _T(*pNorm_Point_1)[2], _T(*pNorm_Point_2)[2], float f, float c1, float c2)
{//������ڲν���Ļ����ͶӰ����һ��ƽ���ϣ����ǽ���Ϊ1�ĳ���ƽ��
	for (int i = 0; i < iCount; i++)
	{	//����u= a*f*x/z + cx => x= z* (u-cx) /af	//����a=1
		//Ȼ����Դ�����õ�ʽ���ǣ� (x - c1) / f�� ��ôz=1.�˴�����
		//����ٶ�ÿ�����z������1��
		//OK�ˣ�������̾���ͨ������ڲν����е����ص�ͶӰ����һ��ƽ���ϡ�
		// ������ʽ���� x= K(-1) * p	����PΪ���ص�λ�ã�xΪ��һ����λ�ã� K(-1)Ϊ K����
		//Ŀǰ�������󵽵ĵı任���������������Ϊ1���ء�����Ϊ768���أ��ָ����ĵ�
		//�ʴ������������������Ϊ (x-c1)/f ���� �� -0.5,0.5֮��
		pNorm_Point_1[i][0] = (pPoint_1[i][0] - c1) / f;
		pNorm_Point_1[i][1] = (pPoint_1[i][1] - c2) / f;

		pNorm_Point_2[i][0] = (pPoint_2[i][0] - c1) / f;
		pNorm_Point_2[i][1] = (pPoint_2[i][1] - c2) / f;
		//printf("%f %f\n", pNorm_Point_1[i][0], pNorm_Point_1[i][1]);
	}	
	//Disp((float*)pNorm_Point_2, iMatch_Count, 2, "Point_2");
}

template<typename _T> void Compute_Squared_Sampson_Error(_T Point_1[][2], _T Point_2[][2], int iCount, _T E[3 * 3], _T Residual[])
{//����Sampson����
	const _T E_00 = E[0 * 3 + 0], E_01 = E[0 * 3 + 1], E_02 = E[0 * 3 + 2],
		E_10 = E[1 * 3 + 0], E_11 = E[1 * 3 + 1], E_12 = E[1 * 3 + 2],
		E_20 = E[2 * 3 + 0], E_21 = E[2 * 3 + 1], E_22 = E[2 * 3 + 2];
	//float fSum = 0;
	for (int i = 0; i < iCount; i++)
	{
		_T x1_0 = Point_1[i][0], x1_1 = Point_1[i][1],
			x2_0 = Point_2[i][0], x2_1 = Point_2[i][1];

		//�� E*x1
		// Ex1 = E * points1[i].homogeneous();
		_T Ex1_0 = E_00 * x1_0 + E_01 * x1_1 + E_02,
			Ex1_1 = E_10 * x1_0 + E_11 * x1_1 + E_12,
			Ex1_2 = E_20 * x1_0 + E_21 * x1_1 + E_22;

		//�� E'*x2
		// Etx2 = E.transpose() * points2[i].homogeneous();
		_T Etx2_0 = E_00 * x2_0 + E_10 * x2_1 + E_20,
			Etx2_1 = E_01 * x2_0 + E_11 * x2_1 + E_21;

		//�� x2'*E*x1 �����E�Ķ���
		// x2tEx1 = points2[i].homogeneous().transpose() * Ex1;
		_T x2tEx1 = x2_0 * Ex1_0 + x2_1 * Ex1_1 + Ex1_2;

		// Sampson distance �����������
		Residual[i] = x2tEx1 * x2tEx1 / (Ex1_0 * Ex1_0 + Ex1_1 * Ex1_1 + Etx2_0 * Etx2_0 + Etx2_1 * Etx2_1);
	}
}
template<typename _T> void Ransac_Estimate_F(_T Point_1[][2], _T Point_2[][2], int iCount, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr)
{//�����㷨������� p0*E*x1=0�� ����p0,p1��ԭλ��
#define SAMPLE_COUNT 8	//�������ģ������Ҫ����С������
	Mem_Mgr* poMem_Mgr;
	short* pSample_Index;
	_T* pResidual, (*pX_Inlier)[2], (*pY_Inlier)[2],
		(*pNorm_Point_1)[2], (*pNorm_Point_2)[2];	//Ϊ���ٶȣ�Norm_Point �ȷ����ڴ�

	_T X_rand[SAMPLE_COUNT][2], Y_rand[SAMPLE_COUNT][2], F[3 * 3];
	Light_Ptr oPtr;
	int i, j, iTrial, dyn_max_num_trials = 0, bAbort = 0;
	//��νLocalָ�����ôֲڵ�8�㷨����ı任Ϊ�����������еĵ㶼���������һ����������õı任
	Ransac_Support oLocal_Support = {}, oCur_Support = {};	//�������
	Ransac_Report oReport = {};

	//��ν Best ָ�������Ž�
	_T Best_Modal[3 * 3];
	//int bBest_Model_is_Local;
	Ransac_Support oBest_Support = { 0,(float)1e30 };

	//RansacҪ��1, ����ָ��һ����������ֵ�����ڴ���ֵ��������������
	_T fMax_Residual = 4 * 4;
	{//���·����ڴ棬�е�������ֱ���
		unsigned char* pCur;
		int iSize = ALIGN_SIZE_128(iCount * sizeof(short) +	//Sample Index
			iCount * 5 * sizeof(_T) +					//Residual, Norm_Point_1,Norm_Point_2,pDup_Point_1,pDup_Point_2
			iCount * 9 * sizeof(_T) +					//Estimate_E
			128 * 4);
		if (pMem_Mgr)
			poMem_Mgr = pMem_Mgr;
		else
		{
			poMem_Mgr = (Mem_Mgr*)malloc(sizeof(Mem_Mgr));
			//iSize = ALIGN_SIZE_128(iCount * sizeof(short) + iCount * 5 * sizeof(_T) + iCount * iCount + 128 * 4);
			Init_Mem_Mgr(poMem_Mgr, iSize, 1024, 997);
			if (!poMem_Mgr->m_pBuffer)
				return;
		}
		//Report�ǵ�ռ�������
		if (pMem_Mgr)
			oReport.m_pInlier_Mask = (unsigned char*)pMalloc(poMem_Mgr, iCount * sizeof(unsigned char));	//oReport.m_pInlier_Mask
		else
			oReport.m_pInlier_Mask = (unsigned char*)malloc(iCount * sizeof(unsigned char));
		oReport.m_iFloat_Size = sizeof(_T);

		Attach_Light_Ptr(oPtr, (unsigned char*)pMalloc(poMem_Mgr, iSize), iSize, -1);

		if (!oPtr.m_pBuffer)
		{
			printf("Fail to allocate memory\n");
			return;
		}
		Malloc(oPtr, iCount * sizeof(short), pCur);		//Sample Index
		pSample_Index = (short*)pCur;
		Malloc_1(oPtr, iCount * sizeof(_T), pResidual);		//Residual
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//Norm_Point_1
		pNorm_Point_1 = pX_Inlier = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//Norm_Point_2
		pNorm_Point_2 = pY_Inlier = (_T(*)[2])pCur;
	}

	for (i = 0; i < iCount; i++)	//��ʼ���������������������ѡȡ���������ϴ��ҡ����ﱣ֤�����ҵ㲻��
		pSample_Index[i] = i;

	for (oReport.m_iTrial_Count = 0; oReport.m_iTrial_Count < 1381551042; oReport.m_iTrial_Count++)
	{
		if (bAbort)
		{
			oReport.m_iTrial_Count++;
			break;
		}
		//RansacҪ��2�����ȡһ��㹹��һ���ֲڵĽ⡣����������С�������ǳ����ģ�
		//�������е������㼯�Ǵֲڵ�		
		Sample_XY(Point_1, Point_2, pSample_Index, iCount, X_rand, Y_rand, SAMPLE_COUNT);
		//RansacҪ��3������㷨��ÿ�����ݶ��в�ͬ����ⷽ������һ���㣬�ʴ�Ransacֻ��
		//����ṹ�ϵ����壬û��ϸ���ϵ����壬�˴�Ҫ�Լ�д�Լ�����ⷽ��
		Estimate_F(X_rand, Y_rand, SAMPLE_COUNT, F, pNorm_Point_1, pNorm_Point_2, oPtr);
		//Estimate_F(X_rand, Y_rand, SAMPLE_COUNT, F);
		
		//��Sampson���
		Compute_Squared_Sampson_Error(Point_1, Point_2, iCount, F, pResidual);
		Get_Support(pResidual, iCount, fMax_Residual, &oCur_Support.m_iInlier_Count, &oCur_Support.m_fResidual_Sum);
		if (oCur_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oCur_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oCur_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
		{
			oBest_Support = oCur_Support;
			//bBest_Model_is_Local = 0;
			memcpy(Best_Modal, F, 9 * sizeof(_T));
			int prev_best_num_inliers = oBest_Support.m_iInlier_Count;
			if (oCur_Support.m_iInlier_Count > SAMPLE_COUNT)
			{
				for (iTrial = 0; iTrial < 10; iTrial++)
				{
					//�ٰ�m_iInlier_Count��������һ��E���ƣ���ξͲ����ð˵���
					for (j = i = 0; i < iCount; i++)
					{
						if (pResidual[i] < fMax_Residual)
						{
							pX_Inlier[j][0] = Point_1[i][0];
							pX_Inlier[j][1] = Point_1[i][1];
							pY_Inlier[j][0] = Point_2[i][0];
							pY_Inlier[j][1] = Point_2[i][1];
							j++;
						}
					}
					prev_best_num_inliers = oBest_Support.m_iInlier_Count;
					Estimate_F(pX_Inlier, pY_Inlier, oBest_Support.m_iInlier_Count, F, pNorm_Point_1, pNorm_Point_2, oPtr);
					//Estimate_F(pX_Inlier, pY_Inlier, oBest_Support.m_iInlier_Count, F);
					Compute_Squared_Sampson_Error(Point_1, Point_2, iCount, F, pResidual);
					Get_Support(pResidual, iCount, fMax_Residual, &oLocal_Support.m_iInlier_Count, &oLocal_Support.m_fResidual_Sum);
					if (oLocal_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oLocal_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oLocal_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
					{
						oBest_Support = oLocal_Support;
						memcpy(Best_Modal, F, 9 * sizeof(_T));
						//bBest_Model_is_Local = 1;
					}
					if (oBest_Support.m_iInlier_Count <= prev_best_num_inliers)
						break;
				}
				dyn_max_num_trials = iCompute_Num_Trials(oBest_Support.m_iInlier_Count, iCount, SAMPLE_COUNT);
			}			
		}
		if (oReport.m_iTrial_Count >= dyn_max_num_trials && oReport.m_iTrial_Count >= 30)
		{
			bAbort = true;
			break;
		}
	}
	oReport.m_oSupport = oBest_Support;
	memcpy(oReport.m_Modal, Best_Modal, 9 * sizeof(_T));

	if (oReport.m_oSupport.m_iInlier_Count > 8)
		oReport.m_bSuccess = 1;

	Compute_Squared_Sampson_Error(Point_1, Point_2, iCount, Best_Modal, pResidual);
	Get_Support(pResidual, iCount, fMax_Residual, &oLocal_Support.m_iInlier_Count, &oLocal_Support.m_fResidual_Sum);
	for (i = 0; i < iCount; i++)
	{
		if (pResidual[i] <= fMax_Residual)
			oReport.m_pInlier_Mask[i] = 1;
		else
			oReport.m_pInlier_Mask[i] = 0;
	}
	oReport.m_iSample_Count = iCount;
	if (poReport)
		*poReport = oReport;

	//Disp((_T*)oReport.m_Modal, 3, 3);
	//����ͷ�
	if (pMem_Mgr)
		Free(poMem_Mgr, oPtr.m_pBuffer);
	else
	{//�Լ�����Mem_Mgr�Լ������ͷ�
		Free_Mem_Mgr(poMem_Mgr);
		free(poMem_Mgr);
	}
}
template<typename _T> void Ransac_Estimate_E(_T Point_1[][2], _T Point_2[][2], int iCount, float f,float c1,  float c2, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr)
{//ע�⣬�˴�Ӧ�������H����ķ����б�����Point_1��Point_2�Ѿ���Normalized
//�����㷨������� x0*E*x1=0, ���� x0,x1�ǹ�һ����ĵ�

#define SAMPLE_COUNT 8	//�������ģ������Ҫ����С������
	Mem_Mgr* poMem_Mgr;
	short* pSample_Index;
	_T* pResidual, (*pX_Inlier)[2], (*pY_Inlier)[2],
		(*pDup_Point_1)[2], (*pDup_Point_2)[2],		//����Point_1, Point_2�ָ�����һ��ƽ�������
		(*pNorm_Point_1)[2], (*pNorm_Point_2)[2];	//Ϊ���ٶȣ�Norm_Point �ȷ����ڴ�

	_T Best_Modal[3 * 3], X_rand[SAMPLE_COUNT][2], Y_rand[SAMPLE_COUNT][2], E[3 * 3];
	Light_Ptr oPtr;
	int i, j, iTrial,/* bBest_Model_is_Local,*/ dyn_max_num_trials = 0, bAbort = 0;
	Ransac_Support oLocal_Support = {}, oCur_Support = {}, oBest_Support = { 0,(float)1e30 };	//�������
	Ransac_Report oReport = {};

	//RansacҪ��1, ����ָ��һ����������ֵ�����ڴ���ֵ��������������
	_T fMax_Residual = 0.005208333333333333f * 0.005208333333333333f;

	{//���·����ڴ棬�е�������ֱ���
		unsigned char* pCur;
		int iSize = ALIGN_SIZE_128(iCount * sizeof(short) +	//Sample Index
			iCount * 9 * sizeof(_T) +					//Residual, Norm_Point_1,Norm_Point_2,pDup_Point_1,pDup_Point_2
			iCount * 9 * sizeof(_T) +					//Estimate_E
			128 * 4);	
		if (pMem_Mgr)
			poMem_Mgr = pMem_Mgr;
		else
		{
			poMem_Mgr = (Mem_Mgr*)malloc(sizeof(Mem_Mgr));
			//iSize = ALIGN_SIZE_128(iCount * sizeof(short) + iCount * 5 * sizeof(_T) + iCount * iCount + 128 * 4);
			Init_Mem_Mgr(poMem_Mgr, iSize, 1024, 997);
			if (!poMem_Mgr->m_pBuffer)
				return;
		}
		//Report�ǵ�ռ�������
		if (pMem_Mgr)
			oReport.m_pInlier_Mask = (unsigned char*)pMalloc(poMem_Mgr, iCount * sizeof(unsigned char));	//oReport.m_pInlier_Mask
		else
			oReport.m_pInlier_Mask = (unsigned char*)malloc(iCount * sizeof(unsigned char));
		oReport.m_iFloat_Size = sizeof(_T);

		Attach_Light_Ptr(oPtr, (unsigned char*)pMalloc(poMem_Mgr, iSize), iSize, -1);
		
		if (!oPtr.m_pBuffer)
		{
			printf("Fail to allocate memory\n");
			return;
		}
		Malloc(oPtr, iCount * sizeof(short), pCur);		//Sample Index
		pSample_Index = (short*)pCur;
		Malloc_1(oPtr, iCount * sizeof(_T), pResidual);		//Residual
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//Norm_Point_1
		pNorm_Point_1 = pX_Inlier = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//Norm_Point_2
		pNorm_Point_2 = pY_Inlier = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//pDup_Point_1
		pDup_Point_1 = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//pDup_Point_2
		pDup_Point_2 = (_T(*)[2])pCur;
	}
	for (i = 0; i < iCount; i++)	//��ʼ���������������������ѡȡ���������ϴ��ҡ����ﱣ֤�����ҵ㲻��
		pSample_Index[i] = i;

	//����ָ�����һ��ƽ��
	Normalize_Point(Point_1, Point_2, iCount, pDup_Point_1, pDup_Point_2, f, c1, c2);

	for (oReport.m_iTrial_Count = 0; oReport.m_iTrial_Count < 1381551042; oReport.m_iTrial_Count++)
	{
		if (bAbort)
		{
			oReport.m_iTrial_Count++;
			break;
		}
		//RansacҪ��2�����ȡһ��㹹��һ���ֲڵĽ⡣����������С�������ǳ����ģ�
		//�������е������㼯�Ǵֲڵ�		
		Sample_XY(pDup_Point_1, pDup_Point_2, pSample_Index, iCount, X_rand, Y_rand, SAMPLE_COUNT);

		//RansacҪ��3������㷨��ÿ�����ݶ��в�ͬ����ⷽ������һ���㣬�ʴ�Ransacֻ��
		//����ṹ�ϵ����壬û��ϸ���ϵ����壬�˴�Ҫ�Լ�д�Լ�����ⷽ��
		//Estimate_E(X_rand, Y_rand, SAMPLE_COUNT, E, pNorm_Point_1, pNorm_Point_2, oPtr);
		Estimate_E(X_rand, Y_rand, SAMPLE_COUNT, E);
		//��Sampson���
		Compute_Squared_Sampson_Error(pDup_Point_1, pDup_Point_2, iCount, E, pResidual);
		Get_Support(pResidual, iCount, fMax_Residual, &oCur_Support.m_iInlier_Count, &oCur_Support.m_fResidual_Sum);
		//if (oCur_Support.m_iInlier_Count == 8)
			//printf("Here");
		if (oCur_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oCur_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oCur_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
		{
			oBest_Support = oCur_Support;
			//bBest_Model_is_Local = 0;
			memcpy(Best_Modal, E, 9 * sizeof(_T));
			int prev_best_num_inliers = oBest_Support.m_iInlier_Count;
			if (oCur_Support.m_iInlier_Count > SAMPLE_COUNT)
			{
				for (iTrial = 0; iTrial < 10; iTrial++)
				{
					//�ٰ�m_iInlier_Count��������һ��E���ƣ���ξͲ����ð˵���
					for (j = i = 0; i < iCount; i++)
					{
						if (pResidual[i] < fMax_Residual)
						{
							pX_Inlier[j][0] = pDup_Point_1[i][0];
							pX_Inlier[j][1] = pDup_Point_1[i][1];
							pY_Inlier[j][0] = pDup_Point_2[i][0];
							pY_Inlier[j][1] = pDup_Point_2[i][1];
							j++;
						}
					}
					prev_best_num_inliers = oBest_Support.m_iInlier_Count;
					//Estimate_E(pX_Inlier, pY_Inlier, oBest_Support.m_iInlier_Count, E, pNorm_Point_1, pNorm_Point_2, oPtr);
					Estimate_E(pX_Inlier, pY_Inlier, oBest_Support.m_iInlier_Count, E);
					//Disp(E, 3, 3, "E");
					Compute_Squared_Sampson_Error(pDup_Point_1, pDup_Point_2, iCount, E, pResidual);
					Get_Support(pResidual, iCount, fMax_Residual, &oLocal_Support.m_iInlier_Count, &oLocal_Support.m_fResidual_Sum);
					if (oLocal_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oLocal_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oLocal_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
					{
						oBest_Support = oLocal_Support;
						memcpy(Best_Modal, E, 9 * sizeof(_T));
						//bBest_Model_is_Local = 1;
					}
					if (oBest_Support.m_iInlier_Count <= prev_best_num_inliers)
						break;
				}
				dyn_max_num_trials = iCompute_Num_Trials(oBest_Support.m_iInlier_Count, iCount, SAMPLE_COUNT);
			}			
		}
		if (oReport.m_iTrial_Count >= dyn_max_num_trials && oReport.m_iTrial_Count >= 30)
		{
			bAbort = true;
			break;
		}
	}

	oReport.m_oSupport = oBest_Support;
	memcpy(oReport.m_Modal, Best_Modal, 9 * sizeof(_T));

	if (oReport.m_oSupport.m_iInlier_Count > 8 || (iCount==8 && oBest_Support.m_iInlier_Count==8))
		oReport.m_bSuccess = 1;

	for (i = 0; i < iCount; i++)
	{
		if (pResidual[i] <= fMax_Residual)
			oReport.m_pInlier_Mask[i] = 1;
		else
			oReport.m_pInlier_Mask[i] = 0;
	}
	oReport.m_iSample_Count = iCount;
	if (poReport)
		*poReport = oReport;

	//Disp((_T*)oReport.m_Modal, 3, 3);
	//����ͷ�
	if (pMem_Mgr)
		Free(poMem_Mgr, oPtr.m_pBuffer);
	else
	{//�Լ�����Mem_Mgr�Լ������ͷ�
		Free_Mem_Mgr(poMem_Mgr);
		free(poMem_Mgr);
	}
	return;
#undef SAMPLE_COUNT
}
template<typename _T> void Ransac_Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr)
{//���ص�ReportͳһΪdouble����
#define SAMPLE_COUNT 4
	Mem_Mgr* poMem_Mgr;
	short* pSample_Index;
	_T* pResidual, (*pX_Inlier)[2], (*pY_Inlier)[2],
		(*pNorm_Point_1)[2], (*pNorm_Point_2)[2];	//Ϊ���ٶȣ�Norm_Point �ȷ����ڴ�

	_T Best_Modal[3 * 3], X_rand[SAMPLE_COUNT][2], Y_rand[SAMPLE_COUNT][2], H[3 * 3];
	Light_Ptr oPtr;
	int i, j, iTrial,/* bBest_Model_is_Local, */dyn_max_num_trials = 0, bAbort = 0;
	int prev_best_num_inliers;
	Ransac_Support oLocal_Support = {}, oCur_Support = {}, oBest_Support = { 0,(float)1e30 };	//�������
	Ransac_Report oReport = {};

	//RansacҪ��1, ����ָ��һ����������ֵ�����ڴ���ֵ��������������
	_T fMax_Residual = 4 * 4;

	unsigned char* pCur;
	int iSize;

	{//���·����ڴ棬�е�������ֱ���
		iSize = ALIGN_SIZE_128(iCount * sizeof(short) + 
			iCount * 5 * sizeof(_T) +
			iCount * 9 * sizeof(_T)+		//A Vt S //iCount * iCount +
			128 * 4);						//��������128�ֽڶ���
		if (pMem_Mgr)
			poMem_Mgr = pMem_Mgr;
		else
		{
			poMem_Mgr = (Mem_Mgr*)malloc(sizeof(Mem_Mgr));
			//iSize = ALIGN_SIZE_128(iCount * sizeof(short) + iCount * 5 * sizeof(_T) + iCount * iCount + 128 * 4);
			Init_Mem_Mgr(poMem_Mgr, iSize, 1024, 997);
			if (!poMem_Mgr->m_pBuffer)
				return;
		}
		if (pMem_Mgr)
			oReport.m_pInlier_Mask = (unsigned char*)pMalloc(poMem_Mgr, iCount * sizeof(unsigned char));	//oReport.m_pInlier_Mask
		else
			oReport.m_pInlier_Mask = (unsigned char*)malloc(iCount * sizeof(unsigned char));
		oReport.m_iFloat_Size = sizeof(_T);

		Attach_Light_Ptr(oPtr, (unsigned char*)pMalloc(poMem_Mgr, iSize), iSize, -1);
		if (!oPtr.m_pBuffer)
		{
			printf("Fail to allocate memory\n");
			return;
		}
		Malloc(oPtr, iCount * sizeof(short), pCur);		//Sample Index
		pSample_Index = (short*)pCur;
		Malloc(oPtr, iCount * sizeof(_T), pCur);		//Residual
		pResidual = (_T*)pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//Norm_Point_1
		pNorm_Point_1 = pX_Inlier = (_T(*)[2])pCur;
		Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//Norm_Point_2
		pNorm_Point_2 = pY_Inlier = (_T(*)[2])pCur;
		//Malloc(oPtr, iCount * sizeof(unsigned char), pCur);
	}	
	for (i = 0; i < iCount; i++)	//��ʼ���������������������ѡȡ���������ϴ��ҡ����ﱣ֤�����ҵ㲻��
		pSample_Index[i] = i;

	for (oReport.m_iTrial_Count = 0; oReport.m_iTrial_Count < 1381551042; oReport.m_iTrial_Count++)
	{
		if (bAbort)
		{
			oReport.m_iTrial_Count++;
			break;
		}

		//RansacҪ��2�����ȡһ��㹹��һ���ֲڵĽ⡣����������С�������ǳ����ģ�
		//�������е������㼯�Ǵֲڵ�		
		Sample_XY(Point_1, Point_2, pSample_Index, iCount, X_rand, Y_rand, 4);

		//RansacҪ��3������㷨��ÿ�����ݶ��в�ͬ����ⷽ������һ���㣬�ʴ�Ransacֻ��
		//����ṹ�ϵ����壬û��ϸ���ϵ����壬�˴�Ҫ�Լ�д�Լ�����ⷽ��
		Estimate_H(X_rand, Y_rand, 4, H, pNorm_Point_1, pNorm_Point_2,oPtr);
		//Estimate_H(X_rand, Y_rand, SAMPLE_COUNT, H);
		//�˴��ƺ���ŷ�Ͼ���
		Get_Residual_H(Point_1, Point_2, iCount, H, pResidual);
		Get_Support(pResidual, iCount, fMax_Residual, &oCur_Support.m_iInlier_Count, &oCur_Support.m_fResidual_Sum);
		//printf("4 Point Trial:%d Inlier:%d Residual:%f\n", oReport.m_iTrial_Count, oCur_Support.m_iInlier_Count, oCur_Support.m_fResidual_Sum);

		if (oCur_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oCur_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oCur_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
		{
			oBest_Support = oCur_Support;
			//bBest_Model_is_Local = 0;
			memcpy(Best_Modal, H, 9 * sizeof(_T));
			prev_best_num_inliers = oBest_Support.m_iInlier_Count;

			if (oCur_Support.m_iInlier_Count > 4)
			{
				for (iTrial = 0; iTrial < 10; iTrial++)
				{//�˴��Ľṹ�����ˣ���һ��Modal������������ȫ����������Modal��ֱ��������������
				//������������������˸���������˥���Ĳ�����ҲûɶӪ������ȫ�����Լ����Դ�
					//�ٰ�m_iInlier_Count��������һ��E���ƣ���ξͲ����ð˵���
					for (j = i = 0; i < iCount; i++)
					{
						if (pResidual[i] < fMax_Residual)
						{
							pX_Inlier[j][0] = Point_1[i][0];
							pX_Inlier[j][1] = Point_1[i][1];
							pY_Inlier[j][0] = Point_2[i][0];
							pY_Inlier[j][1] = Point_2[i][1];
							j++;
						}
					}
					prev_best_num_inliers = oBest_Support.m_iInlier_Count;
					Estimate_H(pX_Inlier, pY_Inlier,oBest_Support.m_iInlier_Count, H, pNorm_Point_1, pNorm_Point_2, oPtr);
					//Estimate_H(pX_Inlier, pY_Inlier, oBest_Support.m_iInlier_Count, H);
					Get_Residual_H(Point_1, Point_2, iCount, H, pResidual);
					Get_Support(pResidual, iCount, fMax_Residual, &oLocal_Support.m_iInlier_Count, &oLocal_Support.m_fResidual_Sum);
					if (oLocal_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oLocal_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oLocal_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
					{
						oBest_Support = oLocal_Support;
						memcpy(Best_Modal, H, 9 * sizeof(_T));
						//bBest_Model_is_Local = 1;
					}
					if (oBest_Support.m_iInlier_Count <= prev_best_num_inliers)
						break;
					//printf("4 points Trial:%d Full Trial:%d Inlier:%d Residual:%f\n", oReport.m_iTrial_Count,iTrial,
						//oBest_Support.m_iInlier_Count,oBest_Support.m_fResidual_Sum);
				}
			}
			//��ȫ�����Լ����Դ�
			dyn_max_num_trials = iCompute_Num_Trials(oBest_Support.m_iInlier_Count, iCount, 4);
			//printf("dyn_max_num_trials:%d\n", dyn_max_num_trials);
		}
		if (oReport.m_iTrial_Count >= dyn_max_num_trials && oReport.m_iTrial_Count >= 30)
		{
			bAbort = 1;
			break;
		}
	}

	oReport.m_oSupport = oBest_Support;
	memcpy(oReport.m_Modal, Best_Modal, 9 * sizeof(_T));
	
	if (oReport.m_oSupport.m_iInlier_Count > 8)
		oReport.m_bSuccess = 1;

	for (i = 0; i < iCount; i++)
	{
		if (pResidual[i] <= fMax_Residual)
			oReport.m_pInlier_Mask[i] = 1;
		else
			oReport.m_pInlier_Mask[i] = 0;
	}
	oReport.m_iSample_Count = iCount;
	if (poReport)
		*poReport = oReport;

	//����ͷ�
	if (pMem_Mgr)
		Free(poMem_Mgr, oPtr.m_pBuffer);
	else
	{//�Լ�����Mem_Mgr�Լ������ͷ�
		Free_Mem_Mgr(poMem_Mgr);
		free(poMem_Mgr);
	}
#undef SAMPLE_COUNT
}
void Free_Report(Ransac_Report oReport, Mem_Mgr* poMem_Mgr)
{
	if (oReport.m_pInlier_Mask)
	{
		if (poMem_Mgr)
			Free(poMem_Mgr, oReport.m_pInlier_Mask);
		else
			free(oReport.m_pInlier_Mask);
	}
	return;
}

void Disp_Report(Ransac_Report oReport)
{
	printf("%s\n", oReport.m_bSuccess ? "Success" : "Fail");
	printf("Sample Count:%d Trial_Count:%d\n", oReport.m_iSample_Count, oReport.m_iTrial_Count);
	printf("Inlier Count:%d Residual:%f\n", oReport.m_oSupport.m_iInlier_Count, oReport.m_oSupport.m_fResidual_Sum);
	if (oReport.m_iFloat_Size == 4)
		Disp(oReport.m_Modal_f, 3, 3, "Modal");
	else
		Disp(oReport.m_Modal_d, 3, 3, "Modal");
	return;
}

template<typename _T>void Decompose_E(_T E[3 * 3], _T R1[3 * 3], _T R2[3 * 3], _T t1[3], _T t2[3], int bNormalize_t)
{//��һ��E�����зֽ������R ������t
//ע�⣬����ķֽⲻ����ȫ���ն����� E = t^ * R , ���ǽ�t��һ����
	SVD_Info oSVD;
	SVD_Alloc<_T>(3, 3, &oSVD);
	svd_3(E, oSVD, &oSVD.m_bSuccess);
	if (fGet_Determinant((_T*)oSVD.U, 3) < 0)
		Matrix_Multiply((_T*)oSVD.U, 3, 3, (_T)(-1), (_T*)oSVD.U);

	if (fGet_Determinant((_T*)oSVD.Vt, 3) < 0)
		Matrix_Multiply((_T*)oSVD.Vt, 3, 3,(_T)(- 1), (_T*)oSVD.Vt);

	//��û���⣬������δ��ϸ�Ƶ�������ֽ�������ʵ�ֵģ���κ�ǰ���(s0,s0,0)�ֽ�������
	//����ΪCol_Map����
	_T W[3 * 3] = { 0, 1, 0, -1, 0, 0, 0, 0, 1 };

	//R_1 = U * W * V
	Matrix_Multiply((_T*)oSVD.U, 3, 3, (_T*)W, 3, R1);
	Matrix_Multiply(R1, 3, 3, (_T*)oSVD.Vt, 3, R1);

	Matrix_Transpose(W, 3, 3, W);
	Matrix_Multiply((_T*)oSVD.U, 3, 3, (_T*)W, 3, R2);
	Matrix_Multiply(R2, 3, 3, (_T*)oSVD.Vt, 3, R2);

	t1[0] = ((_T*)oSVD.U)[2], t1[1] = ((_T*)oSVD.U)[5], t1[2] = ((_T*)oSVD.U)[8];
	Normalize(t1, 3, t1);
	Matrix_Multiply(t1, 1, 3, (_T)(-1.f), t2);
	if (!bNormalize_t)
	{
		Matrix_Multiply(t1, 1, 3, (_T)((_T*)oSVD.S)[0], t1);
		Matrix_Multiply(t2, 1, 3, (_T)((_T*)oSVD.S)[0], t2);
	}
	Free_SVD(&oSVD);

	//����һ���Ƿ� E= R_1 * t^, ʧ�ܣ�����
	//Test_Decompose_E(E, R_2, t);

	//Disp(R_1, 3, 3, "R_1");
	//Disp(R_2, 3, 3, "R_2");

	////�����������ϵķ���
	//float R_z_t_1[3 * 3],		//Rz(pi/2)		�������ɳ���
	//    R_z_t_2[3 * 3];		    //Rz(-pi/2)
	//float V[] = { 0,0,1,PI / 2.f };
	//Rotation_Vector_2_Matrix_3D(V, R_z_t_1);
	//Disp(R_z_t_1, 3, 3);
	////���Կ����� R_z_t_1={  0, -1, 0,
	////                      1, 0, 0,
	////                      0, 0,0 }

	//V[3] = -PI / 2.f;
	//Rotation_Vector_2_Matrix_3D(V, R_z_t_2);
	//Disp(R_z_t_2, 3, 3);
	////�� R_z_t_2={   0, 1, 0,
	////              -1, 0, 0,
	////               0, 0, 0 }
	////�ɼ�������ת�����һ��

	//Disp(t, 1, 3, "t");
}

void SB_Reconstruct()
{//����Ǹ�ɵ�Ʒ�����������ƭtemplate
	Bundle_Adjust_3D2D_1((double(*)[3])NULL, (double(*)[2])NULL, 0, (double*)NULL, (double*)NULL, NULL);
	Bundle_Adjust_3D2D_1((float(*)[3])NULL, (float(*)[2])NULL, 0, (float*)NULL, (float*)NULL, NULL);

	Image_Pos_2_3D((double(*)[3])NULL, 0, (double*)NULL,(double)0, (double(*)[3])NULL);
	Image_Pos_2_3D((float(*)[3])NULL, 0, (float*)NULL, (float)0, (float(*)[3])NULL);

	RGBD_2_Point_3D({}, NULL, (double(*)[3])NULL, (double)0, (double(*)[3])NULL, NULL);
	RGBD_2_Point_3D({}, NULL, (float(*)[3])NULL, (float)0, (float(*)[3])NULL, NULL);

	Determine_Confg(NULL, (double(*)[2])NULL, (double(*)[2])NULL, 0, (double(**)[2])NULL, (double(**)[2])NULL);
	Determine_Confg(NULL, (float(*)[2])NULL, (float(*)[2])NULL, 0, (float(**)[2])NULL, (float(**)[2])NULL);

	Estimate_Relative_Pose({}, NULL, NULL, (double(*)[2])NULL, (double(*)[2])NULL, 0, NULL);
	Estimate_Relative_Pose({}, NULL, NULL, (float(*)[2])NULL, (float(*)[2])NULL, 0, NULL);

	Gen_Camera_Intrinsic((double*)NULL, 0, 0, 0, 0, 0);
	Gen_Camera_Intrinsic((float*)NULL, 0, 0, 0, 0, 0);

	Test_E((double*)NULL, (double(*)[2])NULL, (double(*)[2])NULL, 0);
	Test_E((float*)NULL, (float(*)[2])NULL, (float(*)[2])NULL, 0);

	Triangulate_Point((double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL);
	Triangulate_Point((float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL);

	Compute_Squared_Sampson_Error((double(*)[2])NULL, (double(*)[2])NULL, 0, (double*)NULL, (double*)NULL);
	Compute_Squared_Sampson_Error((float(*)[2])NULL, (float(*)[2])NULL, 0, (float*)NULL, (float*)NULL);

	Ransac_Estimate_H((double(*)[2])NULL, (double(*)[2])NULL, 0, NULL);
	Ransac_Estimate_H((float(*)[2])NULL, (float(*)[2])NULL, 0, NULL);

	Ransac_Estimate_E((double(*)[2])NULL, (double(*)[2])NULL, 0, 0, 0, 0, NULL);
	Ransac_Estimate_E((float(*)[2])NULL, (float(*)[2])NULL, 0, 0, 0, 0, NULL);

	Ransac_Estimate_F((double(*)[2])NULL, (double(*)[2])NULL, 0, NULL);
	Ransac_Estimate_F((float(*)[2])NULL, (float(*)[2])NULL, 0, NULL);

	Normalize_Point((double(*)[2])NULL, (double(*)[2])NULL, 0, (double(*)[2])NULL, (double(*)[2])NULL, 0, 0, 0);
	Normalize_Point((float(*)[2])NULL, (float(*)[2])NULL, 0, (float(*)[2])NULL, (float(*)[2])NULL, 0, 0, 0);

	Decompose_E((double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL);
	Decompose_E((float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL);

	E_2_R_t((double*)NULL, (double(*)[2])NULL, (double(*)[2])NULL, 0,(double*)NULL,(double*)NULL, (double(*)[3])NULL);
	E_2_R_t((float*)NULL, (float(*)[2])NULL, (float(*)[2])NULL, 0, (float*)NULL, (float*)NULL, (float(*)[3])NULL);
}

template<typename _T>void Triangulate_Point(_T x0[2], _T x1[2], _T KP_0[], _T KP_1[], _T Point_3D[])
{//������������飬���� x = PX�У� ���XΪ�ռ�㣬��ôPӦΪ������������ͶӰ����һ��ƽ����
	//ע�⣬�˴����������KP��������Σ�ƽ��ͶӰ�����������ڲξ��������ξ���������ڲ�
	//x0 = (1/z) * KP * P	z��ȫ��Ӱ��ָ�

	int x;
	_T A[4 * 4] = {};    //�����ϵ���������������һ����С�������⣬ ��Ax=0����С���˽�
	////��һ��
	//A[0] = -1, A[2] = Point_1[0];
	////�ڶ���
	//A[1*4+ 1] = -1, A[1*4+2] = Point_1[1];
	////������
	//for (x = 0; x < 4; x++)
	//    A[2 * 4 + x] = Point_2[0] * P_2[2 * 4 + x] - P_2[0 * 4 + x];
	////��4��
	//for (x = 0; x < 4; x++)
	//    A[3 * 4 + x] = Point_2[1] * P_2[2 * 4 + x] - P_2[1 * 4 + x];

	//Disp(A, 4, 4, "A");
	//�������Լ���һ��A��ʵ��֤�������ַ���ȡ��֮�⣬��𲻴󣬽�һ��
	for (x = 0; x < 4; x++)
	{
		A[0 * 4 + x] = x0[1] * KP_0[2 * 4 + x] - KP_0[1 * 4 + x];
		A[1 * 4 + x] = KP_0[0 * 4 + x] - x0[0] * KP_0[2 * 4 + x];

		A[2 * 4 + x] = x1[1] * KP_1[2 * 4 + x] - KP_1[1 * 4 + x];
		A[3 * 4 + x] = KP_1[0 * 4 + x] - x1[0] * KP_1[2 * 4 + x];
	}

	//Disp(A, 4, 4, "A");

   /* Disp(A, 4, 4, "A");
	Disp(Point_1, 1, 2,"Point_1");
	Disp(Point_2, 1, 2, "Point_2");
	Disp(P_1, 4, 4, "P1");
	Disp(P_2, 4, 4, "P2");*/

	//Ȼ����� Ax����С���˽�
	SVD_Info oSVD;
	SVD_Alloc<_T>(4, 4, &oSVD);
	svd_3(A, oSVD, &oSVD.m_bSuccess);
	//Disp(&((_T*)oSVD.Vt)[3 * 4], 1, 4,"Vt");
	//Test_SVD(A,oSVD);
	Homo_Normalize(&((_T*)oSVD.Vt)[3 * 4], 4, Point_3D);
	Point_3D[3] = 1.f;
	//Disp(New_Point, 1, 4, "New_Point");
	Free_SVD(&oSVD);
	return;
}

template<typename _T>void Check_Cheirality(_T Point_1[][2], _T Point_2[][2], int* piCount, _T R[], _T t[], _T Point_3d[][3])
{//�Ը�����R,t�����Ƿ������ʵ
//ע�⣬�˴���һ��͵���㷨��û�а�����ڲδ���������Ϊ����Խ�������֤���� R t������ĸ������� z>0 
//�ʴ˻ָ�������Poin_3D�����Ͳ���ԭ���ĵ����꣬����ʡ������ڲ�����µĿռ������

	_T P1[4 * 4], P2[4 * 4], Temp_1[4 * 4];
	_T New_Point[4] = {};   //��������ĵ�P����Ӧ����E��Զ����

	//�˴���һ��Scale(����ֵ)ȥ����
	Gen_Homo_Matrix(R, t, P2);
	//Disp(P2, 4, 4, "P2");

	_T fMax_Depth, kMinDepth = 2.2204460492503131e-16;

	//P1ΪI��˵�ù�ȥ���͵��ӵ㣨������ģ������صķ������һ��ƽ�洹ֱ��������
	Gen_I_Matrix(P1, 4, 4);

	//���fMax_Depth;
	Matrix_Transpose(R, 3, 3, Temp_1);
	Matrix_Multiply(Temp_1, 3, 3, t, 1, Temp_1);
	fMax_Depth = 1000.f * fGet_Mod(Temp_1, 3);

	//Disp(P2, 4, 4, "P2");

	_T fDepth_1, fDepth_2, fMod_P2_Col_2;
	int i, j, iCount = *piCount;
	//�ⲽ�Ƿ��б�Ҫ��P2�Ƿ�������
	_T V[4] = { P2[2],P2[6],P2[10],P2[14] };
	fMod_P2_Col_2 = fGet_Mod(V, 4);

	Rotation_Matrix_2_Vector(R, V);
	//Disp(V, 1, 4, "V");
	New_Point[3] = 1;
	for (i = j = 0; i < iCount; i++)
	{
		Triangulate_Point(Point_1[i], Point_2[i], P1, P2, New_Point);
		//Disp((_T*)New_Point, 1, 4);

		//�������, P1�ĵڶ���Ϊ(0,0,1,0), ���ԣ������ô���ӣ�ֱ�Ӹ�ֵ
		fDepth_1 = New_Point[2];
		if (fDepth_1 > kMinDepth && fDepth_1 < fMax_Depth)
		{
			//�˴���P*X=x => P�ĵڶ��е��X����(x,y,z)�е�z
			//�����ٳ�һ����������ģ������Ŀǰֻ��1
			fDepth_2 = fDot(&P2[2 * 4], New_Point, 3) * fMod_P2_Col_2;
			if (fDepth_2 > kMinDepth && fDepth_2 < fMax_Depth)
			{
				if(Point_3d)
					memcpy(Point_3d[j], New_Point, 3 * sizeof(_T));
				j++;
			}
		}
	}
	*piCount = j;
	return;
}

template<typename _T>void E_2_R_t(_T E[3 * 3], _T Norm_Point_1[][2], _T Norm_Point_2[][2], int iCount, _T R[3 * 3], _T t[3], _T Point_3D[][3])
{//��E�лָ�R������ λ�� t, ע���ˣ������������п��У���������ǰ���£�
	//�˴��ù�һ������
	//E= t^ * R ��Ÿ�ԭ��һֱ�ļ�����룬����ת��λ�ơ��������´���
	//���ң�׼ȷ�ı���� E = a * t^ * R, ���� a��E������ֵ��������ֵ����
	_T R1[3 * 3], R2[3 * 3], t1[3], t2[3];
	_T* Comb[4][2] = { {R1,t1},{R2,t1},{R1,t2},{R2,t2} };

	int i;
	Decompose_E(E, R1, R2, t1, t2); //E = a * t^ * R

	//������������һ��
	//Test_Decompose_E(E, R1, t1);

	/*Disp(R1, 3, 3, "R1");
	Disp(R2, 3, 3, "R2");
	Disp(t1, 1, 3, "t1");
	Disp(t2, 1, 3, "t2");*/

	//���4�� (R1,t1), (R2,t1),(R1,t2),(R2,t2),�ֱ����ת�������ĶԴ�
	int Count_1[4], iMax_Count=0,iMax_Index;;
	for (i = 0; i < 4; i++)
	{
		Count_1[i] = iCount;
		Check_Cheirality(Norm_Point_1, Norm_Point_2, &Count_1[i], Comb[i][0], Comb[i][1], Point_3D);
		if (Count_1[i] > iMax_Count)
		{
			iMax_Count = Count_1[i];
			iMax_Index = i;
		}
		if (Count_1[i] == iCount)
			break;  //�ҵ���
	}

	if (iMax_Count)
	{
		memcpy(R, Comb[iMax_Index][0], 3 * 3 * sizeof(_T));
		memcpy(t, Comb[iMax_Index][1], 3 * sizeof(_T));
	}
	return;
}

template<typename _T>void Test_E(_T E[], _T Norm_Point_1[][2], _T Norm_Point_2[][2], int iCount)
{//��������һ�¸�����E ������һ�� R t����Ҫ���� NP2= (x1/x2) * Rt * NP1
//ע�⣬�˴���t�ǹ�һ������
//�������ĳ�����ͨ�����ǻ������ƥ����Ӧ����ά��X�����X���� x��x0��˵�����ֵ��Ȼ��
//ͨ�������ϵչ�������ϵý�������õ����
	_T R[3 * 3], t[4],Rt[4*4],I[4*4];
	_T Temp_1[4 * 4],Point_3D[4], z1, z2;
	int i;

	E_2_R_t(E, Norm_Point_1, Norm_Point_2, iCount, R, t);

	Disp(E, 3, 3, "E");
	
	Gen_Homo_Matrix(R, t, Rt);
	Gen_I_Matrix(I, 4, 4);
	for (i = 0; i < 8; i++)
	{
		//���������ǻ�������ò������z1,z2
		Triangulate_Point(Norm_Point_1[i], Norm_Point_2[i], I, Rt, Point_3D);

		//�������1���������������Ϊ��λ������ô���z���Ǹõ��zֵ
		Matrix_Multiply(I, 4, 4, Point_3D, 1, Temp_1);
		z1 = Temp_1[2];

		//�����������KP���������ȱ��K���ͽ������Rt
		Matrix_Multiply(Rt, 4, 4, Point_3D, 1, Temp_1);
		z2 = Temp_1[2];
		
		//��xΪAͼ�ڹ�һ��ƽ���ϵĵ㣬x'��Bͼ�ڹ�һ��ƽ���϶�Ӧx�ĵ�
		//��ȷ�Ĺ�ϵ�� x' = (z1/z2) * Rt * x, ��ʱ��z1,z2�Ĳ���ز����٣���ΪҪ�ָ����
		//��z1,z2ֻ�����ǻ��Ժ���У��ʴ�Ҫ�����������������𲽻ָ��������
		memcpy(Temp_1, Norm_Point_1[i], 2 * sizeof(_T));
		Matrix_Multiply(Temp_1, 1, 2, z1, Temp_1);	//��һ�λ�������� (x,y,1) * z1
		Temp_1[2] = z1, Temp_1[3] = 1;						//�ڶ��λ�������� (z1*x, z1*y, z1, 1)

		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);		//���� Rt * x
		Matrix_Multiply(Temp_1, 1, 4, 1.f / z2, Temp_1);	//����ٳ��� z2, ��ʱ�ͷ������ǻ���ʽ��
		Disp(Norm_Point_2[i], 1, 2, "NP2");
		Disp(Temp_1, 1, 4, "z1/z2 * Rt * NP1");

		//�ܽ�һ�£�x' = (z1/z2) * Rt * x�� ��������ʽ����������ģ���Ϊx��2d�㣬Rt����ά�任��¿��������
		//���ԣ�Ҫ������һ�����㣬�ؼ��ǽ�x��Ϊ��ά��
		//x= (x,y,1)*z1 ���(x*z1,y*z1,z1) �ٲ�1����� x=��x*z1,y*z1,z1,1) ��Ϳ��Բ���������
		//���x'�Ľ��  x' = (z1/z2) * Rt * x ��Ͷ���
	}
	return;
}

template<typename _T>void Gen_Camera_Intrinsic(_T K[3 * 3], float fFocal, float a, float b, float cx, float cy)
{//�˴�����һ������ڲ�K,Ҫ���׸������ڲε�����ȥ��
//fFocal: �������Ľ��࣬�����Z����ľ�����ء�
//a:	һ������и�����ƽ�棬�������������ʲô��λ���׻������ף�������ʵ
//		��ÿ����λ��Ӧ���ٸ����ء����ֵ����ÿ��λˮƽ�����϶�Ӧ���ٸ�����
//b:	ÿ��λ�ڴ�ֱ�����϶��ö��ٸ����ء�������a=b
//cx:	����һ��ƫ����������Ļ���ꡣcxΪˮƽƫ����
//cy:	��ֱƫ������ ����һ��w*h����Ļ�� (cx,cy)=(w/2,h/2)
//Ȼ����ֻ��K���껹��������һ���ռ������(x,y,z)ֱ���Ƶ������Ӧ����������(u,v,1)����ȱ��Z
//��Ϊ����ԶС�����ͶӰ��ϵ�� ���ԣ����ڿռ��е�һ��(x,y,z)���� (u,v)'=1/z * K * (x,y,z)'
	memset(K, 0, 3 * 3 * sizeof(_T));
	K[0] = a * fFocal;
	K[4] = b * fFocal;
	K[2] = cx;
	K[5] = cy;
	K[8] = 1.f;
	return;
}

template<typename _T> void Determine_Confg(Two_View_Geometry* poGeo, _T Point_1[][2], _T Point_2[][2], int iCount,
	_T(**ppNew_Point_1)[2], _T(**ppNew_Point_2)[2], Mem_Mgr* poMem_Mgr)
{//����㵽�܆���
	float fEF_Ratio = (float)poGeo->m_oReport_E.m_oSupport.m_iInlier_Count / poGeo->m_oReport_F.m_oSupport.m_iInlier_Count,
		fHF_Ratio = (float)poGeo->m_oReport_H.m_oSupport.m_iInlier_Count / poGeo->m_oReport_F.m_oSupport.m_iInlier_Count,
		fHE_Ratio = (float)poGeo->m_oReport_H.m_oSupport.m_iInlier_Count / poGeo->m_oReport_E.m_oSupport.m_iInlier_Count;
	int num_inliers;
	unsigned char* best_inlier_mask = NULL;

	if (poGeo->m_oReport_E.m_bSuccess && fEF_Ratio > 0.95f && poGeo->m_oReport_E.m_oSupport.m_iInlier_Count >= 15)
	{//0.95��15���Ǿ���ֵ
		// Calibrated configuration.
		if (poGeo->m_oReport_E.m_oSupport.m_iInlier_Count >= poGeo->m_oReport_F.m_oSupport.m_iInlier_Count)
		{
			num_inliers = poGeo->m_oReport_E.m_oSupport.m_iInlier_Count;
			best_inlier_mask = poGeo->m_oReport_E.m_pInlier_Mask;
		}
		else
		{
			num_inliers = poGeo->m_oReport_F.m_oSupport.m_iInlier_Count;
			best_inlier_mask = poGeo->m_oReport_F.m_pInlier_Mask;
		}

		if (fHE_Ratio > 0.8f)
		{
			poGeo->m_iConfig = Two_View_Geometry::PLANAR_OR_PANORAMIC;
			if (poGeo->m_oReport_H.m_oSupport.m_iInlier_Count > num_inliers)
			{
				num_inliers = poGeo->m_oReport_H.m_oSupport.m_iInlier_Count;
				best_inlier_mask = poGeo->m_oReport_H.m_pInlier_Mask;
			}
		}
		else
			poGeo->m_iConfig = Two_View_Geometry::CALIBRATED;
	}
	else if (poGeo->m_oReport_F.m_bSuccess && poGeo->m_oReport_F.m_oSupport.m_iInlier_Count >= 15)
	{
		// Uncalibrated configuration.
		num_inliers = poGeo->m_oReport_F.m_oSupport.m_iInlier_Count;
		best_inlier_mask = poGeo->m_oReport_F.m_pInlier_Mask;
		if (fHF_Ratio > 0.8f)
		{
			poGeo->m_iConfig = Two_View_Geometry::PLANAR_OR_PANORAMIC;
			if (poGeo->m_oReport_H.m_oSupport.m_iInlier_Count > num_inliers)
			{
				num_inliers = poGeo->m_oReport_H.m_oSupport.m_iInlier_Count;
				best_inlier_mask = poGeo->m_oReport_H.m_pInlier_Mask;
			}
		}
		else
			poGeo->m_iConfig = Two_View_Geometry::UNCALIBRATED;
	}
	else if (poGeo->m_oReport_H.m_bSuccess && poGeo->m_oReport_H.m_oSupport.m_iInlier_Count >= 15)
	{
		num_inliers = poGeo->m_oReport_H.m_oSupport.m_iInlier_Count;
		best_inlier_mask = poGeo->m_oReport_H.m_pInlier_Mask;
		poGeo->m_iConfig = Two_View_Geometry::PLANAR_OR_PANORAMIC;
	}
	else
		poGeo->m_iConfig = Two_View_Geometry::DEGENERATE;

	_T(*pNew_Point_1)[2] = NULL, (*pNew_Point_2)[2] = NULL;
	int i, j;
	if (best_inlier_mask)
	{//�˴����ҵ�������inlier���ҳ������γ��µ㼯
		if (poMem_Mgr)
			pNew_Point_1 = (_T(*)[2])pMalloc(poMem_Mgr, num_inliers * 4 * sizeof(_T));
		else
			pNew_Point_1 = (_T(*)[2])malloc(num_inliers * 4 * sizeof(_T));
		pNew_Point_2 = pNew_Point_1 + num_inliers;
		for (i = 0, j = 0; i < iCount; i++)
		{
			if (best_inlier_mask[i])
			{
				pNew_Point_1[j][0] = Point_1[i][0];
				pNew_Point_1[j][1] = Point_1[i][1];
				pNew_Point_2[j][0] = Point_2[i][0];
				pNew_Point_2[j][1] = Point_2[i][1];
				j++;
			}
			//printf("%d ", best_inlier_mask[i]);
		}
	}
	*ppNew_Point_1 = pNew_Point_1;
	*ppNew_Point_2 = pNew_Point_2;
	poGeo->num_inliers = num_inliers;
	return;
}
template<typename _T> void Calculate_Triangulation_Angles(_T R[3 * 3], _T t[3], _T Point_3D[][3], int iCount, Two_View_Geometry::Config_Type iConfig, _T* pfAngle)
{
	_T Temp_1[4], Q[4], Center_1[3] = {}, Center_2[3];
	_T fBaseline_Length_Square = fGet_Mod(t, 3);

	//ת��Ϊ4Ԫ��
	Rotation_Matrix_2_Quaternion(R, Q);
	//Disp(Q, 1, 4, "Q");

	//��R = -R'
	Matrix_Transpose(R, 3, 3, R);
	Matrix_Multiply(R, 3, 3, (_T)-1, R);
	//Disp(R, 3, 3, "R");

	Matrix_Multiply(R, 3, 3, t, 1, Center_2);
	//Disp(Center_2, 1, 3,"C2");

	_T* pAngle = (_T*)pMalloc(&oMatrix_Mem, iCount * sizeof(_T));
	_T ray_length_squared1, ray_length_squared2, fDenominator, fNominator, fAngle;

	for (int i = 0; i < iCount; i++)
	{
		//�����˷�����õ������
		ray_length_squared1 = fDot(Point_3D[i], Point_3D[i], 3);
		Vector_Minus(Point_3D[i], Center_2, 3, Temp_1);
		ray_length_squared2 = fDot(Temp_1, Temp_1, 3);

		fDenominator =(_T)(2.f * sqrt(ray_length_squared1 * ray_length_squared2));
		fNominator = ray_length_squared1 + ray_length_squared2 - fBaseline_Length_Square;
		fAngle =(_T)abs(acos(fNominator / fDenominator));
		pAngle[i] = std::min(fAngle, PI - fAngle);
	}

	//Quick_Sort(pAngle, 0, iCount - 1);
	//Disp(pAngle, iCount, 1, "Angle");
	_T fMid = oGet_Nth_Elem(pAngle, iCount, 0, iCount - 1, iCount / 2);
	if (iConfig == Two_View_Geometry::PLANAR_OR_PANORAMIC)
	{
		printf("Not implemented\n");
		return;
	}
	*pfAngle = fMid;
	if (pAngle)
		Free(&oMatrix_Mem, pAngle);
	return;
}
template<typename _T> void Estimate_Relative_Pose(Two_View_Geometry oGeo, float Camera_1[3], float Camera_2[2], _T Point_1[][2], _T Point_2[][2], int iCount, Mem_Mgr* poMem_Mgr)
{// Camera: f, c1,c2,
	if (oGeo.m_iConfig != Two_View_Geometry::CALIBRATED && oGeo.m_iConfig != Two_View_Geometry::UNCALIBRATED && oGeo.m_iConfig != Two_View_Geometry::PLANAR &&
		oGeo.m_iConfig != Two_View_Geometry::PANORAMIC && oGeo.m_iConfig != Two_View_Geometry::PLANAR_OR_PANORAMIC)
		return;
	_T(*pNorm_Point_1)[2], (*pNorm_Point_2)[2];
	pNorm_Point_1 = (_T(*)[2])pMalloc(poMem_Mgr, iCount * 4 * sizeof(_T));
	pNorm_Point_2 = pNorm_Point_1 + iCount;
	Normalize_Point(Point_1, Point_2, iCount, pNorm_Point_1, pNorm_Point_2, Camera_1[0], Camera_1[1], Camera_1[2]);

	_T R[3 * 3], t[3];
	_T(*pPoint_3D)[3] = (_T(*)[3])pMalloc(poMem_Mgr, iCount * 3 * sizeof(_T));
	if (oGeo.m_iConfig == Two_View_Geometry::CALIBRATED || oGeo.m_iConfig == Two_View_Geometry::UNCALIBRATED)
		E_2_R_t((_T*)oGeo.m_oReport_E.m_Modal, pNorm_Point_1, pNorm_Point_2, iCount, R, t, pPoint_3D);
	//Disp((_T*)pPoint_3D, iCount, 3);
	//Disp(oGeo.m_oReport_E.m_Modal_d, 3, 3, "E");

	//��֤δ��
	//E_Test_1(pNorm_Point_1, pNorm_Point_2,iCount, (_T*)oGeo.m_oReport_E.m_Modal, R, t);
	_T fAngle;
	int bSuccess;
	if (iCount)
		Calculate_Triangulation_Angles(R, t, pPoint_3D, iCount, oGeo.m_iConfig, &fAngle);
		
	if (iCount > 100 && t[2] < 0.95 && fAngle > 16.f * 0.0174532925199432954743716805978692718781530857086181640625)
		bSuccess = 1;
	else
		bSuccess = 0;
	//Disp(Q, 1, 4);

	if (pNorm_Point_1)
		Free(poMem_Mgr, pNorm_Point_1);
	if (pPoint_3D)
		Free(poMem_Mgr, pPoint_3D);
	return;
}

template<typename _T> void RGBD_2_Point_3D(Image oImage, unsigned short* pDepth, _T K[][3], _T fDepth_Factor, _T Point_3D[][3], int* piPoint_Count, unsigned char Color[][3])
{//ͨ������ڲκ�����ƽ���ϵ����껹ԭ�ռ��
	int y, x, i, iPoint_Count = 0, iDepth;
	for (y = i = 0; y < oImage.m_iHeight; y++)
	{
		for (x = 0; x < oImage.m_iWidth; x++, i++)
		{
			if (pDepth[i])
			{
				iDepth = (unsigned short)((pDepth[i] >> 8) + (pDepth[i] << 8));
				Point_3D[iPoint_Count][2] = (_T)iDepth / fDepth_Factor;
				Point_3D[iPoint_Count][0] = ((x - K[0][2]) * Point_3D[iPoint_Count][2]) / K[0][0];
				Point_3D[iPoint_Count][1] = ((y - K[1][2]) * Point_3D[iPoint_Count][2]) / K[1][1];
				//�ȿ�u,v������������ƽ���ϵ�����
				//u = X* f * s / Z
				//X = (u / f*s)*Z
				if (Color)
				{
					Color[iPoint_Count][0] = oImage.m_pChannel[0][i];
					Color[iPoint_Count][1] = oImage.m_pChannel[1][i];
					Color[iPoint_Count][2] = oImage.m_pChannel[2][i];
				}
				iPoint_Count++;
			}
		}
	}
	if (piPoint_Count)
		*piPoint_Count = iPoint_Count;
	return;
}
template<typename _T>void Image_Pos_2_3D(_T Image_Pos[][3], int iCount,_T K[], _T fDepth_Factor,_T Pos_3D[][3])
{//Image_Pos: ����ƽ���ϵĵ�����(x,y)�����ͼ�ϵ�d��Ϣ, K ����ڲΣ� fDepth_Factor:���ͼ��������
	//�о��������Ҳ��ʵ�ã��ܶ�ʱ��ֻ���ļ�����Ϣ���ӵ�һ����Ϣ�ָ��ռ��������Ҫ
	int i;
	for (i = 0; i < iCount; i++)
	{
		Pos_3D[i][2] = (_T)Image_Pos[i][2] / fDepth_Factor;
		Pos_3D[i][0] = ((Image_Pos[i][0] - K[2]) * Image_Pos[i][2]) / K[0];
		Pos_3D[i][1] = ((Image_Pos[i][1] - K[5]) * Image_Pos[i][2]) / K[4];
	}
	return;
}

template<typename _T>void Bundle_Adjust_3D2D_1(_T Point_3D_Source_1[][3], _T Point_2D_Source_2[][2],int iCount,_T K[],_T Pose[],int *piResult)
{//�ø�˹ţ�ٷ���BA���ƣ������ʽ��ֻ���� Ksi��Ԫ���ƫ��
	//���������� Point_3D_1���ռ�㼯1��Ҳ������Ϊ���1�۲쵽�Ŀռ�㼯
	//Point_2D_2������ƽ���ϵĵ㼯2
	//K�� ���1�����2ͬһ�ڲ�
	_T e[3], fx = K[0], fy = K[1 * 3 + 1], cx = K[2], cy = K[1 * 3 + 2];
	_T fSum_e, fSum_e_Pre = 1e10, Temp[6 * 6], Delta_Ksi[6];
	_T Pose_Pre[4 * 4], Pose_Estimate[4 * 4], Delta_Pose[4 * 4];
	int i,iResult=1,iIter;
	const _T eps = 1e-10;
	//��ʼ�����£��������е�λ��Ϊ��λ���󣬱�ʾ���ƶ�
	Gen_I_Matrix(Pose_Estimate, 4, 4);

	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		_T  H[6 * 6] = { 0 }, //H=��J'J
			b[6] = { 0 };
		for (i = 0; i < iCount; i++)
		{
			_T Point_3D_1[4], *pPoint_2D_2;
			memcpy(Point_3D_1, Point_3D_Source_1[i], 3 * sizeof(_T));
			Point_3D_1[3] = 1;
			//���ϴε����õ�����λ�˼���㼯1����λ��
			Matrix_Multiply(Pose_Estimate, 4, 4, Point_3D_1, 1, Point_3D_1);
			if ( abs(Point_3D_1[2]) < eps)
				continue;	//��̬���ݲ���
			//������ڲ�KͶӰ������ƽ���ϣ�����������㼯2λ���غ�
			_T Point_2D_1[2] = { fx * Point_3D_1[0] / Point_3D_1[2] + cx, fy * Point_3D_1[1] / Point_3D_1[2] + cy };
			pPoint_2D_2 = Point_2D_Source_2[i];
			e[0] = pPoint_2D_2[0] - Point_2D_1[0];	//��Ӧ��i����ֵ��
			e[1] = pPoint_2D_2[1] - Point_2D_1[1];

			fSum_e += e[0] * e[0] + e[1] * e[1];
			_T  X_Sqr = Point_3D_1[0] * Point_3D_1[0],
				Y_Sqr = Point_3D_1[1] * Point_3D_1[1],
				Z_Sqr = Point_3D_1[2] * Point_3D_1[2];

			_T Jt[6 * 2], 
				J[2 * 6] = { -fx / Point_3D_1[2], 0 , fx * Point_3D_1[0] / Z_Sqr, fx * Point_3D_1[0] * Point_3D_1[1] / Z_Sqr, -fx - fx * X_Sqr / Z_Sqr, fx * Point_3D_1[1] / Point_3D_1[2],
				0, -fy / Point_3D_1[2], fy * Point_3D_1[1] / Z_Sqr, fy + fy * Y_Sqr / Z_Sqr, -fy * Point_3D_1[0] * Point_3D_1[1] / Z_Sqr, -fy * Point_3D_1[0] / Point_3D_1[2] };

			Matrix_Transpose(J, 2, 6, Jt);
			Matrix_Multiply(Jt, 6, 2, J, 6, Temp);
			Matrix_Add(H, Temp, 6, H);  //H += J'J;

			Matrix_Multiply(Jt, 6, 2, e, 1, Temp);
			Vector_Add(b, Temp, 6, b);
		}
		Matrix_Multiply(b, 1, 6, (_T)-1, b);
		
		//�ⷽ�� Hx = b
		Solve_Linear_Gause(H, 6, b, Delta_Ksi, &iResult);

		//�˴���ͣ�������н�������ΪDelta_Ksi��ǰλ�ƺ���ת�����Բ����ô�ͳ��|����|�� 0 ����
		//ֻ��������ɢΪ׼
		if (fSum_e >= fSum_e_Pre || !iResult)
			break;

		//��������ԭΪ��ξ���
		se3_2_SE3(Delta_Ksi, Delta_Pose);

		memcpy(Pose_Pre, Pose_Estimate, 4 * 4 * sizeof(_T));
		Matrix_Multiply(Delta_Pose, 4, 4, Pose_Estimate, 4, Pose_Estimate);
		//Disp(Pose, 4, 4, "Pose");
		fSum_e_Pre = fSum_e;
	}

	*piResult = iResult;
	if (iResult)
		memcpy(Pose, Pose_Pre, 4 * 4 * sizeof(_T));

}