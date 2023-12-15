//重建用的代码 
#pragma once
#include "Reconstruct.h"

#define SQR(Value) (Value)*(Value)

template<typename _T> void Sample_XY(_T Point_1[][2], _T Point_2[][2], short Sample_Index[], int iCount, _T X_rand[][2], _T Y_rand[][2], int iRan_Count = 8)
{//从一堆点集中抽出八个点放在最前面
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
{//将一组点，投影到边长为 2* sqrt(2)的投影平面上,那么所有的点都已经实现某种意义上的归一化
	_T Centroid[2] = { 0 };
	int i;
	//先求个重心
	for (i = 0; i < iCount; i++)
		Centroid[0] += Point[i][0], Centroid[1] += Point[i][1];
	Centroid[0] /= iCount, Centroid[1] /= iCount;

	//每一点到重心有个距离，求8点的距离平方和
	_T rms_mean_dist = 0;
	for (i = 0; i < iCount; i++)
		rms_mean_dist += SQR(Point[i][0] - Centroid[0]) + SQR(Point[i][1] - Centroid[1]);
	rms_mean_dist = (_T)sqrt(rms_mean_dist / iCount);	//此处先除以8再开方才对，别瞎改
	//算完以后，rms_mean_dist就是8点到重心的平均距离

	//如何理解norm_factor？先简化一下，所有的点到重心距离都是rms_mean_dist，
	//可组成一个正方形,边长为2*rms_mean_dist。我们将这个正方形（像素平面）
	//设为2*sqrt(2)大小。 那么rms_mean_dist对应sqrt(2)这么大。然后，norm_factor
	//就是指每一份原距离的像素数量.回顾一下自己做像素平面的方法：
	//a = 1920 / (fMax_x - fMin_x);	
	//b = -1080 / (fMax_y - fMin_y);	有相似之处，都是拿屏幕的大小作为分子
	_T norm_factor = (_T)sqrt(2.0) / rms_mean_dist;
	//这个M矩阵长得很像相机内参，后面的算法会进一步印证
	// fx 0  cx
	//	0 fy cy
	//	0 0  1
	M[0][0] = norm_factor, M[0][1] = 0, M[0][2] = -norm_factor * Centroid[0];
	M[1][0] = 0, M[1][1] = norm_factor, M[1][2] = -norm_factor * Centroid[1];
	M[2][0] = 0, M[2][1] = 0, M[2][2] = 1;
	//Disp((float*)M, 3, 3, "M");

	//规格化
	for (i = 0; i < iCount; i++)
	{
		Norm_Point[i][0] = Point[i][0] * M[0][0] + M[0][2];
		Norm_Point[i][1] = Point[i][1] * M[1][1] + M[1][2];
		//printf("%f %f\n", Norm_Point[i][0], Norm_Point[i][1]);
	}
	return;
}
template<typename _T> void Normalize_Point_1(_T Point[][2], int iCount, _T M[3][3], _T Norm_Point[][2])
{//将点做一次规格化，形成一个相机内参M
	_T Centroid[2] = { 0 };
	int i;
	//先求个重心
	for (i = 0; i < iCount; i++)
		Centroid[0] += Point[i][0], Centroid[1] += Point[i][1];
	Centroid[0] /= iCount, Centroid[1] /= iCount;

	//每一点到重心有个距离，求8点的距离平方和
	_T rms_mean_dist = 0;
	for (i = 0; i < iCount; i++)
		rms_mean_dist += SQR(Point[i][0] - Centroid[0]) + SQR(Point[i][1] - Centroid[1]);
	rms_mean_dist = (_T)sqrt(rms_mean_dist / iCount);	//此处先除以8再开方才对，别瞎改
	//算完以后，rms_mean_dist就是8点到重心的平均距离

	//如何理解norm_factor？先简化一下，所有的点到重心距离都是rms_mean_dist，
	//可组成一个正方形,边长为2*rms_mean_dist。我们将这个正方形（像素平面）
	//设为2*sqrt(2)大小。 那么rms_mean_dist对应sqrt(2)这么大。然后，norm_factor
	//就是指每一份原距离的像素数量.回顾一下自己做像素平面的方法：
	//a = 1920 / (fMax_x - fMin_x);	
	//b = -1080 / (fMax_y - fMin_y);	有相似之处，都是拿屏幕的大小作为分子
	_T norm_factor = (_T)sqrt(2.0) / rms_mean_dist;
	//这个M矩阵长得很像相机内参，后面的算法会进一步印证
	// fx 0  cx
	//	0 fy cy
	//	0 0  1
	M[0][0] = norm_factor, M[0][1] = 0, M[0][2] = -norm_factor * Centroid[0];
	M[1][0] = 0, M[1][1] = norm_factor, M[1][2] = -norm_factor * Centroid[1];
	M[2][0] = 0, M[2][1] = 0, M[2][2] = 1;
	//Disp((float*)M, 3, 3, "M");

	//规格化
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
{//理论上这个函数已经完备，给定的一组点已经能找到一个矩阵F，使得p2'*F*p1=0，已经油最简形式，不需要Norm_Point空间
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

	//从以下可以看到，经过相对中心的规格化后，两者数值进一步接近
	//Disp((_T*)Norm_Point_1, iCount, 2, "Norm_Point_1");
	//Disp((_T*)Norm_Point_2, iCount, 2, "Norm_Point_2");
	//Disp((_T*)Point_2, iCount, 2, "Point_2");
	//构造系数系数矩阵
	//Point_1 x F(3x3) x Point_2' 展开，为行向量
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

	//正式求解
	_T Basic_Solution[7 * 9];
	//int iBasic_Solution_Count, iResult;
	//注意，当Ax=0 中的A为横形，此时可以不用svd求解，就是个齐次方程求基础解系的过程，也许更快更准
	//Solve_Linear_Solution_Construction((float*)A, iCount, 9, B, &iResult, Basic_Solution, &iBasic_Solution_Count, NULL);
	SVD_Info oSVD;
	SVD_Alloc<_T>(iCount, 9, &oSVD);
	//Disp((_T*)A, 8, 9, "A");
	svd_3((_T*)A, oSVD, &oSVD.m_bSuccess);
	//Disp((_T*)oSVD.Vt, 9, 9, "Vt");
	memcpy(Basic_Solution, ((_T(*)[9])oSVD.Vt)[8], 9 * sizeof(_T));
	//第一次SVD已经结束使命
	Free_SVD(&oSVD);

	//由解再组合成一个F矩阵
	_T Temp_1[3 * 3], 
		F_1[3 * 3] = { Basic_Solution[0],Basic_Solution[1],Basic_Solution[2],
		Basic_Solution[3],Basic_Solution[4],Basic_Solution[5],
		Basic_Solution[6],Basic_Solution[7],Basic_Solution[8] };
	//Disp((_T*)E_t, 3, 3, "Et");
	SVD_Alloc<_T>(3, 3, &oSVD);
	svd_3(F_1, oSVD,&oSVD.m_bSuccess);

	//对于F矩阵，奇异值的处理稍微没那么多，仅仅把最后一个奇异值置零
	_T S[3 * 3] = { ((_T*)oSVD.S)[0],0,0,
				0,((_T*)oSVD.S)[1],0,
				0,0,0 };
	Matrix_Multiply((_T*)oSVD.U, 3, 3 ,S, 3, Temp_1);
	Matrix_Multiply(Temp_1, 3, 3, (_T*)oSVD.Vt, 3, F);

	//在算 M2' * F * M1恢复原坐标
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
{//做一个更清爽的接口，Norm_Point_1 ，Norm_Point_2可以不开空间，oPtr可以没有
//理论上这已经是完备接口，给与一组点 x1,x2，求得一个矩阵E， 使得 x2'*E*x1=0
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
	
	//从以下可以看到，经过相对中心的规格化后，两者数值进一步接近
	//Disp((_T*)Norm_Point_1, iCount, 2, "Norm_Point_1");
	//Disp((_T*)Norm_Point_2, iCount, 2, "Norm_Point_2");
	//Disp((_T*)Point_2, iCount, 2, "Point_2");
	//构造系数系数矩阵
	//Point_1 x E(3x3) x Point_2' 展开，为行向量
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
	//正式求解
	_T Basic_Solution[9 * 9];
	
	////求秩,第一次怀疑效果差的原因，可能样本不满秩
	//int iRank;
	//iRank = iGet_Rank((_T*)A, 8, 9);
	//if (iRank != 8)
	//	printf("秩小于8:%d\n", iRank);
	//else
	//	printf("Done");

	
	//int iBasic_Solution_Count, iResult;

	//注意，当Ax=0 中的A为横形，此时可以不用svd求解，就是个齐次方程求基础解系的过程，也许更快更准
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
	//Disp(Basic_Solution, 1, 8, "Solution");	//注意，[8]个没用

	/*_T Temp_3[3];
	for (int i = 0; i < 3; i++)
	{	
		_T Point_back_1[3] = { Norm_Point_1[i][0],Norm_Point_1[i][1],(_T)1};
		_T Point_back_2[3] = { Norm_Point_2[i][0],Norm_Point_2[i][1],(_T)1};

		Matrix_Multiply((_T*)Point_back_2, 1, 3, Basic_Solution, 3, Temp_3);
		Matrix_Multiply(Temp_3, 1, 3, (_T*)Point_back_2, 1,Temp_3);		
	}*/

	//第一次SVD已经结束使命
	Free_SVD(&oSVD);	
	
	//由解再组合成一个E'矩阵，注意，这里搞了点多此一举的转置造成后面费解
	//正确的E = [	e1 e2 e3		而此处是 [	e1 e4 e7	所以这是E'
	//				e3 e4 e5					e2 e5 e8
	//				e6 e7 e8 ]					e3 e6 e9]
	_T E_raw[3*3], Temp_1[3 * 3], Temp_2[3 * 3],
		E_t[3 * 3] = { Basic_Solution[0],Basic_Solution[3],Basic_Solution[6],
		Basic_Solution[1],Basic_Solution[4],Basic_Solution[7],
		Basic_Solution[2],Basic_Solution[5],Basic_Solution[8] };
	//Disp((_T*)E_t, 3, 3, "Et");

	//要推导以下以下E_raw是个什么东西
	// NP_2' * E * NP_1 =0		这是最初的最优化问题，在规格化点上做
	//展开规格化点  (M_2 * P2)' * E * M_1 * P1=0  又根据矩阵乘的转置等于转置后矩阵的乘有
	//=>  P2' * M_2' * E * M_1 * P1=0	可以将中间视为整体E_raw
	//=》 E_raw = M_2' * E * M_1
	// De-normalize to image points.
	Matrix_Transpose((_T*)M_2, 3, 3, Temp_1);
	Matrix_Transpose(E_t, 3, 3, Temp_2);
	Matrix_Multiply(Temp_1, 3, 3, Temp_2, 3, E_raw);
	Matrix_Multiply(E_raw, 3, 3, (_T*)M_1, 3, E_raw);
	
	//Disp(E_raw, 3, 3, "E_raw");
	//第二次SVD, 为E_raw
	SVD_Alloc<_T>(3, 3, &oSVD);
	svd_3(E_raw, oSVD);
	//Disp((_T*)oSVD.S, 1, 3,"S");
	//Disp((_T*)oSVD.Vt, 3, 3, "Vt");

	_T S[3 * 3] = {};
	//然后进行一个骚操作，对角化的三个元素进行调整，暂时不知其理。
	//但是在QR分解中则有一种情况，因为计算误差两个重根存在些许差，此时
	//应该将可疑重根求和再平均，此时误差最小
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
{//给定的数据求解一个Homograph矩阵，满足 min ( ||P1*X-P2||^2)
//理论上该接口已经完备，给与一组点，存在一个矩阵H，s.t. H*p1=p2
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

	//从以下可以看到，经过相对中心的规格化后，两者数值进一步接近
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
{//对H矩阵的误差估计不能用Sampson距离，用的还是欧氏距离，待验证
	_T H_00 = H[0], H_01 = H[1], H_02 = H[2], H_10 = H[3],
		H_11 = H[4], H_12 = H[5], H_20 = H[6], H_21 = H[7], H_22 = H[8];

	for (int i = 0; i < iCount; i++)
	{
		_T s_0 = Point_1[i][0];
		_T s_1 = Point_1[i][1];
		_T d_0 = Point_2[i][0];
		_T d_1 = Point_2[i][1];

		//此处向做了个H * Point_1 的变换
		_T pd_0 = H_00 * s_0 + H_01 * s_1 + H_02;
		_T pd_1 = H_10 * s_0 + H_11 * s_1 + H_12;
		_T pd_2 = H_20 * s_0 + H_21 * s_1 + H_22;

		//此处像投影
		_T inv_pd_2 = 1.f / pd_2;
		_T dd_0 = d_0 - pd_0 * inv_pd_2;
		_T dd_1 = d_1 - pd_1 * inv_pd_2;

		//欧氏距离
		Residual[i] = dd_0 * dd_0 + dd_1 * dd_1;
	}
}
template<typename _T>
void Get_Support(_T Residual[], int iCount, _T fMax_Residual, int* piInlier_Count, float* pfResidual_Sum)
{//根本没什么营养，就是比个误差，
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
{//计算dyn_max_num_trials,动态测试次数
	//当Inlier增大，fInlier_Ratio增大，fInlier_Ratio就是找到匹配点的比例
	float fInlier_Ratio = (float)iInlier_Count / iSample_Count;
	float nom = 1.f - 0.99f;
	//Inlier增大，denom减少。 fInlier_Ration ^ (4|8)，高次函数x^(4|8)看[0,1]区间图像即可
	float denom = 1.f - (float)pow(fInlier_Ratio, kMinNumSamples);
	int iRet;
	if (denom <= 0)
		return 1;
	if (denom == 1.0)	//此时表示Inlier_Ratio=0，找不到任何匹配点
		return (int)(std::numeric_limits<size_t>::max());	//返回个最大值
	
	//printf("nom:%f denom:%f log(nom):%f log(denom):%f\n",nom,denom,log(nom),log(denom));
	//log(nom)= -4.605171, 总为负 denom<1, log（denom) 总为负
	//inlier 增大，denom递减，|log(denom)| 递增，结果递减
	//其实从总体上来看，无非就是匹配的点数越多，继续迭代的次数酒约少。有兴趣可以自己搞个曲线
	iRet = (int)ceil(log(nom) / log(denom) * 3.f);
	return iRet;
}
template<typename _T>void Normalize_Point(_T(*pPoint_1)[2], _T(*pPoint_2)[2], int iCount, _T(*pNorm_Point_1)[2], _T(*pNorm_Point_2)[2], float f, float c1, float c2)
{//用相机内参将屏幕坐标投影到归一化平面上，就是焦距为1的成像平面
	for (int i = 0; i < iCount; i++)
	{	//按照u= a*f*x/z + cx => x= z* (u-cx) /af	//假设a=1
		//然而，源代码用的式子是： (x - c1) / f， 那么z=1.此处存疑
		//猜想假定每个点的z距离是1？
		//OK了，这个过程就是通过相机内参将所有的像素点投影到归一化平面上。
		// 矩阵形式就是 x= K(-1) * p	其中P为像素点位置，x为归一化后位置， K(-1)为 K的逆
		//目前，能想象到的的变换是物体里相机光心为1像素。焦距为768像素，恢复中心点
		//故此物体在在世界的坐标为 (x-c1)/f 坐标 在 -0.5,0.5之间
		pNorm_Point_1[i][0] = (pPoint_1[i][0] - c1) / f;
		pNorm_Point_1[i][1] = (pPoint_1[i][1] - c2) / f;

		pNorm_Point_2[i][0] = (pPoint_2[i][0] - c1) / f;
		pNorm_Point_2[i][1] = (pPoint_2[i][1] - c2) / f;
		//printf("%f %f\n", pNorm_Point_1[i][0], pNorm_Point_1[i][1]);
	}	
	//Disp((float*)pNorm_Point_2, iMatch_Count, 2, "Point_2");
}

template<typename _T> void Compute_Squared_Sampson_Error(_T Point_1[][2], _T Point_2[][2], int iCount, _T E[3 * 3], _T Residual[])
{//计算Sampson距离
	const _T E_00 = E[0 * 3 + 0], E_01 = E[0 * 3 + 1], E_02 = E[0 * 3 + 2],
		E_10 = E[1 * 3 + 0], E_11 = E[1 * 3 + 1], E_12 = E[1 * 3 + 2],
		E_20 = E[2 * 3 + 0], E_21 = E[2 * 3 + 1], E_22 = E[2 * 3 + 2];
	//float fSum = 0;
	for (int i = 0; i < iCount; i++)
	{
		_T x1_0 = Point_1[i][0], x1_1 = Point_1[i][1],
			x2_0 = Point_2[i][0], x2_1 = Point_2[i][1];

		//求 E*x1
		// Ex1 = E * points1[i].homogeneous();
		_T Ex1_0 = E_00 * x1_0 + E_01 * x1_1 + E_02,
			Ex1_1 = E_10 * x1_0 + E_11 * x1_1 + E_12,
			Ex1_2 = E_20 * x1_0 + E_21 * x1_1 + E_22;

		//求 E'*x2
		// Etx2 = E.transpose() * points2[i].homogeneous();
		_T Etx2_0 = E_00 * x2_0 + E_10 * x2_1 + E_20,
			Etx2_1 = E_01 * x2_0 + E_11 * x2_1 + E_21;

		//求 x2'*E*x1 这个是E的定义
		// x2tEx1 = points2[i].homogeneous().transpose() * Ex1;
		_T x2tEx1 = x2_0 * Ex1_0 + x2_1 * Ex1_1 + Ex1_2;

		// Sampson distance 这个距离待理解
		Residual[i] = x2tEx1 * x2tEx1 / (Ex1_0 * Ex1_0 + Ex1_1 * Ex1_1 + Etx2_0 * Etx2_0 + Etx2_1 * Etx2_1);
	}
}
template<typename _T> void Ransac_Estimate_F(_T Point_1[][2], _T Point_2[][2], int iCount, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr)
{//按照算法，算的是 p0*E*x1=0， 其中p0,p1是原位置
#define SAMPLE_COUNT 8	//估计这个模型所需要的最小样本数
	Mem_Mgr* poMem_Mgr;
	short* pSample_Index;
	_T* pResidual, (*pX_Inlier)[2], (*pY_Inlier)[2],
		(*pNorm_Point_1)[2], (*pNorm_Point_2)[2];	//为了速度，Norm_Point 先分配内存

	_T X_rand[SAMPLE_COUNT][2], Y_rand[SAMPLE_COUNT][2], F[3 * 3];
	Light_Ptr oPtr;
	int i, j, iTrial, dyn_max_num_trials = 0, bAbort = 0;
	//所谓Local指的是用粗糙的8点法求出的变换为基础，将所有的点都加入进来，一起迭代多次求得的变换
	Ransac_Support oLocal_Support = {}, oCur_Support = {};	//够大就行
	Ransac_Report oReport = {};

	//所谓 Best 指的是最优解
	_T Best_Modal[3 * 3];
	//int bBest_Model_is_Local;
	Ransac_Support oBest_Support = { 0,(float)1e30 };

	//Ransac要素1, 必须指定一个最大误差阈值。大于此阈值的样本不计入内
	_T fMax_Residual = 4 * 4;
	{//以下分配内存，有点蠢，但又必需
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
		//Report那点空间放在最底
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

	for (i = 0; i < iCount; i++)	//初始化样本索引，后面是随机选取样本，不断打乱。这里保证索引乱点不乱
		pSample_Index[i] = i;

	for (oReport.m_iTrial_Count = 0; oReport.m_iTrial_Count < 1381551042; oReport.m_iTrial_Count++)
	{
		if (bAbort)
		{
			oReport.m_iTrial_Count++;
			break;
		}
		//Ransac要素2，随机取一组点构造一个粗糙的解。这个解对于这小数量点是成立的，
		//但对所有的其他点集是粗糙的		
		Sample_XY(Point_1, Point_2, pSample_Index, iCount, X_rand, Y_rand, SAMPLE_COUNT);
		//Ransac要素3，求解算法。每种数据都有不同的求解方法，不一而足，故此Ransac只有
		//程序结构上的意义，没有细节上的意义，此处要自己写自己的求解方法
		Estimate_F(X_rand, Y_rand, SAMPLE_COUNT, F, pNorm_Point_1, pNorm_Point_2, oPtr);
		//Estimate_F(X_rand, Y_rand, SAMPLE_COUNT, F);
		
		//求Sampson误差
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
					//再把m_iInlier_Count个样本做一次E估计，这次就不是用八点了
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
	//最后释放
	if (pMem_Mgr)
		Free(poMem_Mgr, oPtr.m_pBuffer);
	else
	{//自己开的Mem_Mgr自己负责释放
		Free_Mem_Mgr(poMem_Mgr);
		free(poMem_Mgr);
	}
}
template<typename _T> void Ransac_Estimate_E(_T Point_1[][2], _T Point_2[][2], int iCount, float f,float c1,  float c2, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr)
{//注意，此处应当与求解H矩阵的方法有别，来的Point_1，Point_2已经是Normalized
//按照算法，算的是 x0*E*x1=0, 其中 x0,x1是归一化后的点

#define SAMPLE_COUNT 8	//估计这个模型所需要的最小样本数
	Mem_Mgr* poMem_Mgr;
	short* pSample_Index;
	_T* pResidual, (*pX_Inlier)[2], (*pY_Inlier)[2],
		(*pDup_Point_1)[2], (*pDup_Point_2)[2],		//这是Point_1, Point_2恢复到归一化平面的坐标
		(*pNorm_Point_1)[2], (*pNorm_Point_2)[2];	//为了速度，Norm_Point 先分配内存

	_T Best_Modal[3 * 3], X_rand[SAMPLE_COUNT][2], Y_rand[SAMPLE_COUNT][2], E[3 * 3];
	Light_Ptr oPtr;
	int i, j, iTrial,/* bBest_Model_is_Local,*/ dyn_max_num_trials = 0, bAbort = 0;
	Ransac_Support oLocal_Support = {}, oCur_Support = {}, oBest_Support = { 0,(float)1e30 };	//够大就行
	Ransac_Report oReport = {};

	//Ransac要素1, 必须指定一个最大误差阈值。大于此阈值的样本不计入内
	_T fMax_Residual = 0.005208333333333333f * 0.005208333333333333f;

	{//以下分配内存，有点蠢，但又必需
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
		//Report那点空间放在最底
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
	for (i = 0; i < iCount; i++)	//初始化样本索引，后面是随机选取样本，不断打乱。这里保证索引乱点不乱
		pSample_Index[i] = i;

	//将点恢复到归一化平面
	Normalize_Point(Point_1, Point_2, iCount, pDup_Point_1, pDup_Point_2, f, c1, c2);

	for (oReport.m_iTrial_Count = 0; oReport.m_iTrial_Count < 1381551042; oReport.m_iTrial_Count++)
	{
		if (bAbort)
		{
			oReport.m_iTrial_Count++;
			break;
		}
		//Ransac要素2，随机取一组点构造一个粗糙的解。这个解对于这小数量点是成立的，
		//但对所有的其他点集是粗糙的		
		Sample_XY(pDup_Point_1, pDup_Point_2, pSample_Index, iCount, X_rand, Y_rand, SAMPLE_COUNT);

		//Ransac要素3，求解算法。每种数据都有不同的求解方法，不一而足，故此Ransac只有
		//程序结构上的意义，没有细节上的意义，此处要自己写自己的求解方法
		//Estimate_E(X_rand, Y_rand, SAMPLE_COUNT, E, pNorm_Point_1, pNorm_Point_2, oPtr);
		Estimate_E(X_rand, Y_rand, SAMPLE_COUNT, E);
		//求Sampson误差
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
					//再把m_iInlier_Count个样本做一次E估计，这次就不是用八点了
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
	//最后释放
	if (pMem_Mgr)
		Free(poMem_Mgr, oPtr.m_pBuffer);
	else
	{//自己开的Mem_Mgr自己负责释放
		Free_Mem_Mgr(poMem_Mgr);
		free(poMem_Mgr);
	}
	return;
#undef SAMPLE_COUNT
}
template<typename _T> void Ransac_Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr)
{//返回的Report统一为double类型
#define SAMPLE_COUNT 4
	Mem_Mgr* poMem_Mgr;
	short* pSample_Index;
	_T* pResidual, (*pX_Inlier)[2], (*pY_Inlier)[2],
		(*pNorm_Point_1)[2], (*pNorm_Point_2)[2];	//为了速度，Norm_Point 先分配内存

	_T Best_Modal[3 * 3], X_rand[SAMPLE_COUNT][2], Y_rand[SAMPLE_COUNT][2], H[3 * 3];
	Light_Ptr oPtr;
	int i, j, iTrial,/* bBest_Model_is_Local, */dyn_max_num_trials = 0, bAbort = 0;
	int prev_best_num_inliers;
	Ransac_Support oLocal_Support = {}, oCur_Support = {}, oBest_Support = { 0,(float)1e30 };	//够大就行
	Ransac_Report oReport = {};

	//Ransac要素1, 必须指定一个最大误差阈值。大于此阈值的样本不计入内
	_T fMax_Residual = 4 * 4;

	unsigned char* pCur;
	int iSize;

	{//以下分配内存，有点蠢，但又必需
		iSize = ALIGN_SIZE_128(iCount * sizeof(short) + 
			iCount * 5 * sizeof(_T) +
			iCount * 9 * sizeof(_T)+		//A Vt S //iCount * iCount +
			128 * 4);						//补余数凑128字节对齐
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
	for (i = 0; i < iCount; i++)	//初始化样本索引，后面是随机选取样本，不断打乱。这里保证索引乱点不乱
		pSample_Index[i] = i;

	for (oReport.m_iTrial_Count = 0; oReport.m_iTrial_Count < 1381551042; oReport.m_iTrial_Count++)
	{
		if (bAbort)
		{
			oReport.m_iTrial_Count++;
			break;
		}

		//Ransac要素2，随机取一组点构造一个粗糙的解。这个解对于这小数量点是成立的，
		//但对所有的其他点集是粗糙的		
		Sample_XY(Point_1, Point_2, pSample_Index, iCount, X_rand, Y_rand, 4);

		//Ransac要素3，求解算法。每种数据都有不同的求解方法，不一而足，故此Ransac只有
		//程序结构上的意义，没有细节上的意义，此处要自己写自己的求解方法
		Estimate_H(X_rand, Y_rand, 4, H, pNorm_Point_1, pNorm_Point_2,oPtr);
		//Estimate_H(X_rand, Y_rand, SAMPLE_COUNT, H);
		//此处似乎用欧氏距离
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
				{//此处的结构很明了，从一个Modal出发，不断用全部点调整这个Modal，直到点数不再增加
				//才跳出迭代。后面搞了个迭代次数衰减的参数，也没啥营养，完全可以自己拍脑袋
					//再把m_iInlier_Count个样本做一次E估计，这次就不是用八点了
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
			//完全可以自己拍脑袋
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

	//最后释放
	if (pMem_Mgr)
		Free(poMem_Mgr, oPtr.m_pBuffer);
	else
	{//自己开的Mem_Mgr自己负责释放
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
{//从一个E矩阵中分解出两个R 和两个t
//注意，这里的分解不是完全按照定义来 E = t^ * R , 而是将t归一化了
	SVD_Info oSVD;
	SVD_Alloc<_T>(3, 3, &oSVD);
	svd_3(E, oSVD, &oSVD.m_bSuccess);
	if (fGet_Determinant((_T*)oSVD.U, 3) < 0)
		Matrix_Multiply((_T*)oSVD.U, 3, 3, (_T)(-1), (_T*)oSVD.U);

	if (fGet_Determinant((_T*)oSVD.Vt, 3) < 0)
		Matrix_Multiply((_T*)oSVD.Vt, 3, 3,(_T)(- 1), (_T*)oSVD.Vt);

	//算没问题，但是尚未详细推导，这个分解是怎样实现的，如何和前面的(s0,s0,0)分解结合起来
	//以下为Col_Map代码
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

	//验算一下是否 E= R_1 * t^, 失败，存疑
	//Test_Decompose_E(E, R_2, t);

	//Disp(R_1, 3, 3, "R_1");
	//Disp(R_2, 3, 3, "R_2");

	////尝试用理论上的方法
	//float R_z_t_1[3 * 3],		//Rz(pi/2)		后续做成常量
	//    R_z_t_2[3 * 3];		    //Rz(-pi/2)
	//float V[] = { 0,0,1,PI / 2.f };
	//Rotation_Vector_2_Matrix_3D(V, R_z_t_1);
	//Disp(R_z_t_1, 3, 3);
	////可以看出， R_z_t_1={  0, -1, 0,
	////                      1, 0, 0,
	////                      0, 0,0 }

	//V[3] = -PI / 2.f;
	//Rotation_Vector_2_Matrix_3D(V, R_z_t_2);
	//Disp(R_z_t_2, 3, 3);
	////而 R_z_t_2={   0, 1, 0,
	////              -1, 0, 0,
	////               0, 0, 0 }
	////可见，两个转换结果一样

	//Disp(t, 1, 3, "t");
}

void SB_Reconstruct()
{//这就是个傻逼方法，用来欺骗template
	ICP_SVD((double(*)[3])NULL, (double(*)[3])NULL, 0, (double*)NULL, NULL);
	ICP_SVD((float(*)[3])NULL, (float(*)[3])NULL, 0, (float*)NULL, NULL);

	ICP_Bundle_Adjust((double(*)[3])NULL, (double(*)[3])NULL, 0, (double*)NULL, NULL);
	ICP_Bundle_Adjust((float(*)[3])NULL, (float(*)[3])NULL, 0, (float*)NULL, NULL);

	Get_Deriv_TP_Ksi((double*)NULL, (double*)NULL, (double*)NULL);
	Get_Deriv_TP_Ksi((float*)NULL, (float*)NULL, (float*)NULL);

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
{//这个方程有异议，对于 x = PX中， 如果X为空间点，那么P应为相机参数，最后投影到归一化平面上
	//注意，此处的相机参数KP可以是外参（平行投影），可以是内参矩阵乘以外参矩阵。如果有内参
	//x0 = (1/z) * KP * P	z完全不影响恢复

	int x;
	_T A[4 * 4] = {};    //这个是系数矩阵，最后我们求一个最小二乘问题， 求Ax=0的最小二乘解
	////第一行
	//A[0] = -1, A[2] = Point_1[0];
	////第二行
	//A[1*4+ 1] = -1, A[1*4+2] = Point_1[1];
	////第三行
	//for (x = 0; x < 4; x++)
	//    A[2 * 4 + x] = Point_2[0] * P_2[2 * 4 + x] - P_2[0 * 4 + x];
	////第4行
	//for (x = 0; x < 4; x++)
	//    A[3 * 4 + x] = Point_2[1] * P_2[2 * 4 + x] - P_2[1 * 4 + x];

	//Disp(A, 4, 4, "A");
	//按理论自己搞一下A，实践证明，部分符号取反之外，差别不大，解一样
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

	//然后求解 Ax的最小二乘解
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
{//对给定的R,t检验是否符合现实
//注意，此处是一种偷懒算法，没有把相机内参带进来，因为其初衷仅仅是验证几个 R t组合中哪个能满足 z>0 
//故此恢复出来的Poin_3D根本就不是原来的点坐标，而是省略相机内参情况下的空间点坐标

	_T P1[4 * 4], P2[4 * 4], Temp_1[4 * 4];
	_T New_Point[4] = {};   //计算出来的点P，对应矩阵E的远处点

	//此处把一个Scale(奇异值)去掉了
	Gen_Homo_Matrix(R, t, P2);
	//Disp(P2, 4, 4, "P2");

	_T fMax_Depth, kMinDepth = 2.2204460492503131e-16;

	//P1为I，说得过去，就当视点（相机中心）到像素的方向与归一化平面垂直（正交）
	Gen_I_Matrix(P1, 4, 4);

	//算个fMax_Depth;
	Matrix_Transpose(R, 3, 3, Temp_1);
	Matrix_Multiply(Temp_1, 3, 3, t, 1, Temp_1);
	fMax_Depth = 1000.f * fGet_Mod(Temp_1, 3);

	//Disp(P2, 4, 4, "P2");

	_T fDepth_1, fDepth_2, fMod_P2_Col_2;
	int i, j, iCount = *piCount;
	//这步是否有必要，P2是否正交？
	_T V[4] = { P2[2],P2[6],P2[10],P2[14] };
	fMod_P2_Col_2 = fGet_Mod(V, 4);

	Rotation_Matrix_2_Vector(R, V);
	//Disp(V, 1, 4, "V");
	New_Point[3] = 1;
	for (i = j = 0; i < iCount; i++)
	{
		Triangulate_Point(Point_1[i], Point_2[i], P1, P2, New_Point);
		//Disp((_T*)New_Point, 1, 4);

		//再算深度, P1的第二行为(0,0,1,0), 所以，别搞那么复杂，直接赋值
		fDepth_1 = New_Point[2];
		if (fDepth_1 > kMinDepth && fDepth_1 < fMax_Depth)
		{
			//此处，P*X=x => P的第二行点乘X就是(x,y,z)中的z
			//后面再乘一个列向量的模待考，目前只是1
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
{//从E中恢复R矩阵与 位移 t, 注意了，由于向量可行可列，在列向量前提下，
	//此处用归一化坐标
	//E= t^ * R 这才跟原来一直的计算对齐，先旋转后位移。否则天下大乱
	//并且，准确的表达是 E = a * t^ * R, 其中 a是E的特征值。否则数值不对
	_T R1[3 * 3], R2[3 * 3], t1[3], t2[3];
	_T* Comb[4][2] = { {R1,t1},{R2,t1},{R1,t2},{R2,t2} };

	int i;
	Decompose_E(E, R1, R2, t1, t2); //E = a * t^ * R

	//不服可以验算一下
	//Test_Decompose_E(E, R1, t1);

	/*Disp(R1, 3, 3, "R1");
	Disp(R2, 3, 3, "R2");
	Disp(t1, 1, 3, "t1");
	Disp(t2, 1, 3, "t2");*/

	//组成4对 (R1,t1), (R2,t1),(R1,t2),(R2,t2),分别检验转换后结果的对错
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
			break;  //找到了
	}

	if (iMax_Count)
	{
		memcpy(R, Comb[iMax_Index][0], 3 * 3 * sizeof(_T));
		memcpy(t, Comb[iMax_Index][1], 3 * sizeof(_T));
	}
	return;
}

template<typename _T>void Test_E(_T E[], _T Norm_Point_1[][2], _T Norm_Point_2[][2], int iCount)
{//这里验算一下给定的E 重新算一次 R t，主要验算 NP2= (x1/x2) * Rt * NP1
//注意，此处的t是归一化向量
//这个验算的初衷是通过三角化求出两匹配点对应的三维点X，这个X对于 x与x0来说是相等值，然后
//通过这个关系展开，不断得进行验算得到结果
	_T R[3 * 3], t[4],Rt[4*4],I[4*4];
	_T Temp_1[4 * 4],Point_3D[4], z1, z2;
	int i;

	E_2_R_t(E, Norm_Point_1, Norm_Point_2, iCount, R, t);

	Disp(E, 3, 3, "E");
	
	Gen_Homo_Matrix(R, t, Rt);
	Gen_I_Matrix(I, 4, 4);
	for (i = 0; i < 8; i++)
	{
		//先来个三角化，否则得不到深度z1,z2
		Triangulate_Point(Norm_Point_1[i], Norm_Point_2[i], I, Rt, Point_3D);

		//对于相机1，相机参数可以视为单位矩阵，那么深度z就是该点的z值
		Matrix_Multiply(I, 4, 4, Point_3D, 1, Temp_1);
		z1 = Temp_1[2];

		//对于相机二，KP相机参数中缺个K，就仅用外参Rt
		Matrix_Multiply(Rt, 4, 4, Point_3D, 1, Temp_1);
		z2 = Temp_1[2];
		
		//设x为A图在归一化平面上的点，x'是B图在归一化平面上对应x的点
		//正确的关系是 x' = (z1/z2) * Rt * x, 此时，z1,z2的参与必不可少，因为要恢复齐次
		//而z1,z2只有三角化以后才有，故此要验算这个结果，必须逐步恢复齐次坐标
		memcpy(Temp_1, Norm_Point_1[i], 2 * sizeof(_T));
		Matrix_Multiply(Temp_1, 1, 2, z1, Temp_1);	//第一次化齐次坐标 (x,y,1) * z1
		Temp_1[2] = z1, Temp_1[3] = 1;						//第二次化齐次坐标 (z1*x, z1*y, z1, 1)

		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);		//再算 Rt * x
		Matrix_Multiply(Temp_1, 1, 4, 1.f / z2, Temp_1);	//最后再除以 z2, 此时就符合三角化公式了
		Disp(Norm_Point_2[i], 1, 2, "NP2");
		Disp(Temp_1, 1, 4, "z1/z2 * Rt * NP1");

		//总结一下，x' = (z1/z2) * Rt * x， 但看这条式子是有问题的，因为x是2d点，Rt是三维变换阵，驴唇对马嘴
		//所以，要两者能一起运算，关键是将x变为三维点
		//x= (x,y,1)*z1 变成(x*z1,y*z1,z1) 再补1，最后 x=（x*z1,y*z1,z1,1) 这就可以参与运算了
		//最后x'的结果  x' = (z1/z2) * Rt * x 这就对了
	}
	return;
}

template<typename _T>void Gen_Camera_Intrinsic(_T K[3 * 3], float fFocal, float a, float b, float cx, float cy)
{//此处生成一个相机内参K,要彻底搞明白内参的来龙去脉
//fFocal: 针孔相机的焦距，这个与Z方向的距离相关。
//a:	一个相机有个成像平面，不管这个区域用什么单位，米还是厘米，都会落实
//		到每个单位对应多少个像素。这个值就是每单位水平方向上对应多少个像素
//b:	每单位在垂直方向上对用多少个像素。经常会a=b
//cx:	加上一个偏移量构成屏幕坐标。cx为水平偏移量
//cy:	垂直偏移量。 对于一个w*h的屏幕， (cx,cy)=(w/2,h/2)
//然而，只有K坐标还不足以由一个空间点坐标(x,y,z)直接推导出其对应的像素坐标(u,v,1)，还缺个Z
//因为还有远小近大的投影关系。 所以，对于空间中的一点(x,y,z)，有 (u,v)'=1/z * K * (x,y,z)'
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
{//这里搞到很啰嗦
	float fEF_Ratio = (float)poGeo->m_oReport_E.m_oSupport.m_iInlier_Count / poGeo->m_oReport_F.m_oSupport.m_iInlier_Count,
		fHF_Ratio = (float)poGeo->m_oReport_H.m_oSupport.m_iInlier_Count / poGeo->m_oReport_F.m_oSupport.m_iInlier_Count,
		fHE_Ratio = (float)poGeo->m_oReport_H.m_oSupport.m_iInlier_Count / poGeo->m_oReport_E.m_oSupport.m_iInlier_Count;
	int num_inliers;
	unsigned char* best_inlier_mask = NULL;

	if (poGeo->m_oReport_E.m_bSuccess && fEF_Ratio > 0.95f && poGeo->m_oReport_E.m_oSupport.m_iInlier_Count >= 15)
	{//0.95与15都是经验值
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
	{//此处将找到的最优inlier对找出来，形成新点集
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

	//转换为4元组
	Rotation_Matrix_2_Quaternion(R, Q);
	//Disp(Q, 1, 4, "Q");

	//对R = -R'
	Matrix_Transpose(R, 3, 3, R);
	Matrix_Multiply(R, 3, 3, (_T)-1, R);
	//Disp(R, 3, 3, "R");

	Matrix_Multiply(R, 3, 3, t, 1, Center_2);
	//Disp(Center_2, 1, 3,"C2");

	_T* pAngle = (_T*)pMalloc(&oMatrix_Mem, iCount * sizeof(_T));
	_T ray_length_squared1, ray_length_squared2, fDenominator, fNominator, fAngle;

	for (int i = 0; i < iCount; i++)
	{
		//向量乘方求和用点积了事
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

	//验证未遂
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
{//通过相机内参和像素平面上的坐标还原空间点
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
				//先看u,v的来历，像素平面上的坐标
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
{//Image_Pos: 像素平面上的点坐标(x,y)及深度图上的d信息, K 相机内参； fDepth_Factor:深度图量化因子
	//感觉这个函数也很实用，很多时候只关心几何信息，从第一手信息恢复空间坐标很重要
	int i;
	for (i = 0; i < iCount; i++)
	{
		Pos_3D[i][2] = (_T)Image_Pos[i][2] / fDepth_Factor;
		Pos_3D[i][0] = ((Image_Pos[i][0] - K[2]) * Image_Pos[i][2]) / K[0];
		Pos_3D[i][1] = ((Image_Pos[i][1] - K[5]) * Image_Pos[i][2]) / K[4];
	}
	return;
}

template<typename _T>void Bundle_Adjust_3D2D_1(_T Point_3D_Source_1[][3], _T Point_2D_Source_2[][2], int iCount, _T K[], _T Pose[], int* piResult)
{//用高斯牛顿法搞BA估计，最简形式，只考虑 Ksi六元组的偏导
	//给定条件： Point_3D_1：空间点集1，也可以视为相机1观察到的空间点集
	//Point_2D_2，像素平面上的点集2
	//K： 相机1和相机2同一内参
	_T Pose_Estimate[4 * 4], Delta_Pose[4 * 4], Pose_Pre[4 * 4],
		Delta_Ksi[6];
	_T fSum_e, fSum_e_Pre = 1e10,
		E[2], JJt[6 * 6], H_Inv[6 * 6];
	int i, iResult = 1, iIter;
	const _T eps = (_T)1e-10;
	_T fx = K[0], fy = K[1 * 3 + 1], cx = K[2], cy = K[1 * 3 + 2];
	//初始条件下，迭代格中的位置为单位矩阵，表示无移动
	Gen_I_Matrix(Pose_Estimate, 4, 4);
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		_T  Sigma_H[6 * 6] = { 0 }, //H=∑J'J
			Sigma_JE[6] = { 0 };	//∑JE
		for (i = 0; i < iCount; i++)
		{
			_T Point_3D_1[4], * pPoint_2D_2;
			memcpy(Point_3D_1, Point_3D_Source_1[i], 3 * sizeof(_T));
			Point_3D_1[3] = 1;
			//用上次迭代得到的新位姿计算点集1的新位置
			Matrix_Multiply(Pose_Estimate, 4, 4, Point_3D_1, 1, Point_3D_1);
			if (abs(Point_3D_1[2]) < eps)
				continue;	//病态数据不算
			//用相机内参K投影到像素平面上，最理想是与点集2位置重合
			_T Point_2D_1[2] = { fx * Point_3D_1[0] / Point_3D_1[2] + cx, fy * Point_3D_1[1] / Point_3D_1[2] + cy };
			pPoint_2D_2 = Point_2D_Source_2[i];
			E[0] = pPoint_2D_2[0] - Point_2D_1[0];	//对应点i的数值差
			E[1] = pPoint_2D_2[1] - Point_2D_1[1];

			fSum_e += E[0] * E[0] + E[1] * E[1];

			_T  X_Sqr = Point_3D_1[0] * Point_3D_1[0],
				Y_Sqr = Point_3D_1[1] * Point_3D_1[1],
				Z_Sqr = Point_3D_1[2] * Point_3D_1[2];

			_T JE[6], J[6 * 2], //以下才是真正的Jt，和书上一致。源代码中是跳步走，不利于学习
				Jt[2 * 6] = { fx / Point_3D_1[2], 0 , -fx * Point_3D_1[0] / Z_Sqr, -fx * Point_3D_1[0] * Point_3D_1[1] / Z_Sqr, fx + fx * X_Sqr / Z_Sqr, -fx * Point_3D_1[1] / Point_3D_1[2],
								0, fy / Point_3D_1[2], -fy * Point_3D_1[1] / Z_Sqr, -fy - fy * Y_Sqr / Z_Sqr, fy * Point_3D_1[0] * Point_3D_1[1] / Z_Sqr, fy * Point_3D_1[0] / Point_3D_1[2] };
			Matrix_Multiply(Jt, 2, 6, (_T)-1.f, Jt);      //乘以-1后，这个就是所需的Jt,雅可比是求解的关键

			Matrix_Transpose(Jt, 2, 6, J);
			Matrix_Multiply(J, 6, 2, E, 1, JE);             //JE
			Vector_Add(Sigma_JE, JE, 6, Sigma_JE);

			Matrix_Multiply(J, 6, 2, Jt, 6, JJt);           //JJ'
			Matrix_Add(Sigma_H, JJt, 6, Sigma_H);           //
		}

		Get_Inv_Matrix_Row_Op(Sigma_H, H_Inv, 6, &iResult);		//求H(-1)
		Matrix_Multiply(H_Inv, 6, 6, Sigma_JE, 1, Delta_Ksi);	//H(-1)*JE
		Matrix_Multiply(Delta_Ksi, 1, 6, (_T)-1, Delta_Ksi);
		if (fSum_e_Pre <= fSum_e || !iResult)
			break;

		//接着从ξ恢复T, 将增量还原为齐次矩阵
		se3_2_SE3(Delta_Ksi, Delta_Pose);

		memcpy(Pose_Pre, Pose_Estimate, 4 * 4 * sizeof(_T));
		Matrix_Multiply(Delta_Pose, 4, 4, Pose_Estimate, 4, Pose_Estimate);

		fSum_e_Pre = fSum_e;
	}
	if (iResult)
		*piResult = 1;
	memcpy(Pose, Pose_Pre, 4 * 4 * sizeof(_T));
}

//template<typename _T>void Bundle_Adjust_3D2D_1(_T Point_3D_Source_1[][3], _T Point_2D_Source_2[][2],int iCount,_T K[],_T Pose[],int *piResult)
//{//用高斯牛顿法搞BA估计，最简形式，只考虑 Ksi六元组的偏导
//	//给定条件： Point_3D_1：空间点集1，也可以视为相机1观察到的空间点集
//	//Point_2D_2，像素平面上的点集2
//	//K： 相机1和相机2同一内参
//	_T e[3], fx = K[0], fy = K[1 * 3 + 1], cx = K[2], cy = K[1 * 3 + 2];
//	_T fSum_e, fSum_e_Pre = 1e10, Temp[6 * 6], Delta_Ksi[6];
//	_T Pose_Pre[4 * 4], Pose_Estimate[4 * 4], Delta_Pose[4 * 4];
//	int i,iResult=1,iIter;
//	const _T eps = (_T)1e-10;
//	//初始条件下，迭代格中的位置为单位矩阵，表示无移动
//	Gen_I_Matrix(Pose_Estimate, 4, 4);
//
//	for (iIter = 0;; iIter++)
//	{
//		fSum_e = 0;
//		_T  H[6 * 6] = { 0 }, //H=∑J'J
//			b[6] = { 0 };
//		for (i = 0; i < iCount; i++)
//		{
//			_T Point_3D_1[4], *pPoint_2D_2;
//			memcpy(Point_3D_1, Point_3D_Source_1[i], 3 * sizeof(_T));
//			Point_3D_1[3] = 1;
//			//用上次迭代得到的新位姿计算点集1的新位置
//			Matrix_Multiply(Pose_Estimate, 4, 4, Point_3D_1, 1, Point_3D_1);
//			if ( abs(Point_3D_1[2]) < eps)
//				continue;	//病态数据不算
//			//用相机内参K投影到像素平面上，最理想是与点集2位置重合
//			_T Point_2D_1[2] = { fx * Point_3D_1[0] / Point_3D_1[2] + cx, fy * Point_3D_1[1] / Point_3D_1[2] + cy };
//			pPoint_2D_2 = Point_2D_Source_2[i];
//			e[0] = pPoint_2D_2[0] - Point_2D_1[0];	//对应点i的数值差
//			e[1] = pPoint_2D_2[1] - Point_2D_1[1];
//
//			fSum_e += e[0] * e[0] + e[1] * e[1];
//			_T  X_Sqr = Point_3D_1[0] * Point_3D_1[0],
//				Y_Sqr = Point_3D_1[1] * Point_3D_1[1],
//				Z_Sqr = Point_3D_1[2] * Point_3D_1[2];
//
//			_T Jt[6 * 2], //注意，下面这个才是真正的Jt。 书上那个是跳步，把符号搞里头
//				J[2 * 6] = { -fx / Point_3D_1[2], 0 , fx * Point_3D_1[0] / Z_Sqr, fx * Point_3D_1[0] * Point_3D_1[1] / Z_Sqr, -fx - fx * X_Sqr / Z_Sqr, fx * Point_3D_1[1] / Point_3D_1[2],
//				0, -fy / Point_3D_1[2], fy * Point_3D_1[1] / Z_Sqr, fy + fy * Y_Sqr / Z_Sqr, -fy * Point_3D_1[0] * Point_3D_1[1] / Z_Sqr, -fy * Point_3D_1[0] / Point_3D_1[2] };
//
//			Matrix_Transpose(J, 2, 6, Jt);
//			Matrix_Multiply(Jt, 6, 2, J, 6, Temp);
//			Matrix_Add(H, Temp, 6, H);  //H += J'J;
//
//			Matrix_Multiply(Jt, 6, 2, e, 1, Temp);
//			Vector_Add(b, Temp, 6, b);
//		}
//		Matrix_Multiply(b, 1, 6, (_T)-1, b);
//		
//		//解方程 Hx = b
//		Solve_Linear_Gause(H, 6, b, Delta_Ksi, &iResult);
//
//		//此处的停机条件有讲究，因为Delta_Ksi是前位移后旋转，所以不能用传统的|Δξ|≈ 0 完事
//		//只能用误差不发散为准
//		if (fSum_e >= fSum_e_Pre || !iResult)
//			break;
//
//		//将增量还原为齐次矩阵
//		se3_2_SE3(Delta_Ksi, Delta_Pose);
//
//		memcpy(Pose_Pre, Pose_Estimate, 4 * 4 * sizeof(_T));
//		Matrix_Multiply(Delta_Pose, 4, 4, Pose_Estimate, 4, Pose_Estimate);
//		//Disp(Pose, 4, 4, "Pose");
//		fSum_e_Pre = fSum_e;
//	}
//
//	*piResult = iResult;
//	if (iResult)
//		memcpy(Pose, Pose_Pre, 4 * 4 * sizeof(_T));
//}

template<typename _T>void Get_Deriv_TP_Ksi(_T T[4 * 4], _T P[3], _T Deriv[4 * 6])
{//有必要把扰动模型也做出来。其实就是∂TP/∂ξ。给定一点P及其变换T。给T一点扰动，看变化率。
//此时，ξ就是6维向量变量，但是隐含了。TP为4维齐次坐标。所以，目标为4x6矩阵
	_T _P[4] = { P[0],P[1],P[2],1 };
	_T P1[4];
	_T P1_M[3 * 3];

	Matrix_Multiply(T, 4,4,_P,1,P1);
	//∂TP/∂ξ = ∂P'/∂ξ= I -P'^
	Hat(P1, P1_M);

	//I
	Deriv[0 * 6 + 0] = 1; Deriv[0 * 6 + 1] = 0; Deriv[0 * 6 + 2] = 0;
	Deriv[1 * 6 + 0] = 0; Deriv[1 * 6 + 1] = 1; Deriv[1 * 6 + 2] = 0;
	Deriv[2 * 6 + 0] = 0; Deriv[2 * 6 + 1] = 0; Deriv[2 * 6 + 2] = 1;
	//-P'^
	Deriv[0 * 6 + 3] = -P1_M[0]; Deriv[0 * 6 + 4] = -P1_M[1]; Deriv[0 * 6 + 5] = -P1_M[2];
	Deriv[1 * 6 + 3] = -P1_M[3]; Deriv[1 * 6 + 4] = -P1_M[4]; Deriv[1 * 6 + 5] = -P1_M[5];
	Deriv[2 * 6 + 3] = -P1_M[6]; Deriv[2 * 6 + 4] = -P1_M[7]; Deriv[2 * 6 + 5] = -P1_M[8];

	memset(&Deriv[3 * 6], 0, 6 * sizeof(_T));
	return;
}
template<typename _T>void ICP_Bundle_Adjust(_T P_1[][3], _T P_2[][3], int iCount, _T Pose[],int *piResult)
{//用高斯牛顿法解ICP，紧咬高斯牛顿法形式
	_T Pose_Estimate[4 * 4], Delta_Pose[4 * 4], Pose_Pre[4 * 4],
		Delta_Ksi[6];
	_T fSum_e, fSum_e_Pre = 1e10,
		E[3], JJt[6 * 6], H_Inv[6 * 6];
	_T P_11[4]; //P1'
	int i, iResult, iIter;
	Gen_I_Matrix(Pose_Estimate, 4, 4);
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		_T  Sigma_H[6 * 6] = { 0 }, //H=∑J'J
			Sigma_JE[6] = { 0 };  //∑JE
		for (i = 0; i < iCount; i++)
		{
			memcpy(P_11, P_1[i], 3 * sizeof(_T));
			P_11[3] = 1;
			Matrix_Multiply(Pose_Estimate, 4, 4, P_11, 1, P_11);
			Vector_Minus(P_2[i], P_11, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];

			_T Jt[4 * 6], J[6 * 3], JE[6];    //注意，J和Jt没有本质上的不同，只是行跟列的排列问题
			Get_Deriv_TP_Ksi(Pose_Estimate, P_1[i], Jt);    //这个还不算Jt，因为他是g(x)的Jt,要求 e(x)的jt
			Matrix_Multiply(Jt, 3, 6, (_T)-1.f, Jt);      //乘以-1后，这个就是所需的Jt,雅可比是求解的关键
			//只要链导法结果正确，后面就能算出来

			Matrix_Transpose(Jt, 3, 6, J);
			Matrix_Multiply(J, 6, 3, E, 1, JE);             //JE
			Vector_Add(Sigma_JE, JE, 6, Sigma_JE);

			Matrix_Multiply(J, 6, 3, Jt, 6, JJt);           //JJ'
			Matrix_Add(Sigma_H, JJt, 6, Sigma_H);           //
		}

		Get_Inv_Matrix_Row_Op(Sigma_H, H_Inv, 6, &iResult);		//求H(-1)
		Matrix_Multiply(H_Inv, 6, 6, Sigma_JE, 1, Delta_Ksi);	//H(-1)*JE
		Matrix_Multiply(Delta_Ksi, 1, 6, (_T)-1, Delta_Ksi);	//

		if (fSum_e_Pre <= fSum_e || !iResult)
			break;

		//接着从ξ恢复T, 将增量还原为齐次矩阵
		se3_2_SE3(Delta_Ksi, Delta_Pose);

		memcpy(Pose_Pre, Pose_Estimate, 4 * 4 * sizeof(_T));
		Matrix_Multiply(Delta_Pose, 4, 4, Pose_Estimate, 4, Pose_Estimate);
		//Disp(Pose_Estimate, 4, 4, "Pose");
		//printf("Iter:%d Cost:%f\n", iIter, fSum_e);
		fSum_e_Pre = fSum_e;
	}
	memcpy(Pose, Pose_Pre, 4 * 4 * sizeof(_T));
	return;
}

template<typename _T>void ICP_SVD(_T P_1[][3], _T P_2[][3], int iCount, _T Pose[], int* piResult)
{//用SVD方法解ICP问题，这个速度应该快很多
	_T P_1_Centroid[3] = { 0 },
		P_2_Centroid[3] = { 0 };
	_T w[3 * 3] = { 0 }, q1[3], q2[3], q1q2t[3 * 3];
	int i, iResult = 1;

	for (i = 0; i < iCount; i++)
	{
		Vector_Add(P_1_Centroid, P_1[i], 3, P_1_Centroid);
		Vector_Add(P_2_Centroid, P_2[i], 3, P_2_Centroid);
	}
	Matrix_Multiply(P_1_Centroid, 1, 3, (_T)1.f / iCount, P_1_Centroid);
	Matrix_Multiply(P_2_Centroid, 1, 3, (_T)1.f / iCount, P_2_Centroid);

	for (i = 0; i < iCount; i++)
	{
		Vector_Minus(P_1[i], P_1_Centroid, 3, q1);
		Vector_Minus(P_2[i], P_2_Centroid, 3, q2);
		Matrix_Multiply(q1, 3, 1, q2, 3, q1q2t);
		Matrix_Add(w, q1q2t, 3, w);
	}
	//Disp(w, 3, 3, "w");

	SVD_Info oSVD;
	SVD_Alloc<_T>(3, 3, &oSVD);
	svd_3(w, oSVD);

	_T R[3 * 3];
	Matrix_Multiply((_T*)oSVD.U, 3, 3, (_T*)oSVD.Vt, 3, R);
	Free_SVD(&oSVD);
	//此处要做一个行列式的判断
	if (fGet_Determinant(R, 3) < 0)
		Matrix_Multiply(R, 3, 3, (_T)(- 1.f), R);
	//Disp(R, 3, 3, "R");

	//根据 p - Rp' -t =0 求t, t= p - Rp'
	_T t[3], Rp2[3 * 3];
	Matrix_Multiply(R, 3, 3, P_2_Centroid, 1, Rp2);
	Vector_Minus(P_1_Centroid, Rp2, 3, t);

	Gen_Homo_Matrix(R, t, Pose);
	*piResult = iResult;
}