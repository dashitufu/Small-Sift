//重建用的代码
#include "Reconstruct.h"

#define SQR(Value) (Value)*(Value)

void SB_Reconstruct()
{//这就是个傻逼方法，用来欺骗template
	Ransac_Estimate_H((double(*)[2])NULL, (double(*)[2])NULL, 1, NULL, NULL);
	Ransac_Estimate_H((float(*)[2])NULL, (float(*)[2])NULL, 1, NULL, NULL);
}

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

template<typename _T> void Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, _T Norm_Point_1[][2], _T Norm_Point_2[][2], _T H[3 * 3], Light_Ptr oPtr = { 0 })
{//给定的数据求解一个Homograph矩阵，满足 min ( ||P1*X-P2||^2)
	_T M_1[3][3], M_2[3][3];
	int i, j,iResult;

	//从以下可以看到，经过相对中心的规格化后，两者数值进一步接近
	Normalize_Point_3(Point_1, iCount, M_1, Norm_Point_1);
	Normalize_Point_3(Point_2, iCount, M_2, Norm_Point_2);
	//Disp((_T*)Norm_Point_1, iCount, 2, "Norm_Point_1");
	//Disp((_T*)Norm_Point_2, 4, 2, "Norm_Point_2");

	_T(*A)[9], * S, * Vt;
	unsigned char* pCur;
	Malloc(oPtr, iCount * 9 * sizeof(_T), pCur);
	A = (_T(*)[9])pCur;
	Malloc(oPtr, iCount * sizeof(_T), pCur);
	S = (_T*)pCur;
	Malloc(oPtr, 9 * 9 * sizeof(_T), pCur);
	Vt = (_T*)pCur;

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
	SVD_Allocate( (_T*)(A) , iCount*2, 9, &oSVD);
	svd_3( (_T*)A, oSVD, &iResult);
	//Disp((_T*)oSVD.Vt, oSVD.h_Min_Vt, oSVD.w_Min_Vt, "Vt");
	
	_T H_t[3 * 3];
	memcpy(H_t, &((_T*)oSVD.Vt)[8 * 9], 9 * sizeof(_T));
	Free_SVD(&oSVD);
	_T Temp_1[3 * 3];
	Get_Inv_Matrix_Row_Op((_T*)M_2, (_T*)M_2, 3, &iResult);
	Matrix_Multiply((_T*)M_2, 3, 3, H_t, 3, Temp_1);
	Matrix_Multiply(Temp_1, 3, 3, (_T*)M_1, 3, H);
	return;
}

template<typename _T>
void Get_Residual_H(_T Point_1[][2], _T Point_2[][2], int iCount, _T H[3 * 3], _T Residual[])
{//对H矩阵的误差估计不能用Sampson距离
	_T H_00 = H[0], H_01 = H[1], H_02 = H[2], H_10 = H[3],
		H_11 = H[4], H_12 = H[5], H_20 = H[6], H_21 = H[7], H_22 = H[8];

	for (int i = 0; i < iCount; i++)
	{
		_T s_0 = Point_1[i][0];
		_T s_1 = Point_1[i][1];
		_T d_0 = Point_2[i][0];
		_T d_1 = Point_2[i][1];

		_T pd_0 = H_00 * s_0 + H_01 * s_1 + H_02;
		_T pd_1 = H_10 * s_0 + H_11 * s_1 + H_12;
		_T pd_2 = H_20 * s_0 + H_21 * s_1 + H_22;

		_T inv_pd_2 = 1.f / pd_2;
		_T dd_0 = d_0 - pd_0 * inv_pd_2;
		_T dd_1 = d_1 - pd_1 * inv_pd_2;

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

template<typename _T> void Ransac_Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, Report* poReport, Mem_Mgr* pMem_Mgr)
{//返回的Report统一为double类型
	Mem_Mgr* poMem_Mgr;
	short* pSample_Index;
	_T* pResidual, (*pX_Inlier)[2], (*pY_Inlier)[2],
		(*pNorm_Point_1)[2], (*pNorm_Point_2)[2];	//为了速度，Norm_Point 先分配内存

	_T Best_Modal[3 * 3], X_rand[4][2], Y_rand[4][2], H[3 * 3];
	Light_Ptr oPtr;
	int i, j, iTrial, bBest_Model_is_Local, dyn_max_num_trials = 0, bAbort = 0;
	int prev_best_num_inliers;
	Support oLocal_Support = {}, oCur_Support = {}, oBest_Support = { 0,(float)1e30 };	//够大就行
	Report oReport = {};

	//Ransac要素1, 必须指定一个最大误差阈值。大于此阈值的样本不计入内
	_T fMax_Residual = 4 * 4;

	unsigned char* pCur;
	int iSize;

	iSize = ALIGN_SIZE_128(iCount * sizeof(short) + iCount * 5 * sizeof(_T) + iCount * iCount + iCount * sizeof(unsigned char) + 128 * 4);
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
	Attach_Light_Ptr(oPtr, (unsigned char*)pMalloc(poMem_Mgr, iSize), iSize, -1);
	Malloc(oPtr, iCount * sizeof(short), pCur);		//Sample Index
	pSample_Index = (short*)pCur;
	Malloc(oPtr, iCount * sizeof(_T), pCur);		//Residual
	pResidual = (_T*)pCur;
	Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//Norm_Point_1
	pNorm_Point_1 = pX_Inlier = (_T(*)[2])pCur;
	Malloc(oPtr, iCount * 2 * sizeof(_T), pCur);	//Norm_Point_2
	pNorm_Point_2 = pY_Inlier = (_T(*)[2])pCur;
	Malloc(oPtr, iCount * sizeof(unsigned char), pCur);
	if (pMem_Mgr)
		oReport.m_pInlier_Mask = (unsigned char*)pCur;	//oReport.m_pInlier_Mask
	else
		oReport.m_pInlier_Mask = (unsigned char*)malloc(iCount * sizeof(unsigned char));
	
	for (i = 0; i < iCount; i++)
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
		Estimate_H(X_rand, Y_rand, 4, pNorm_Point_1, pNorm_Point_2, H, oPtr);
		
		Get_Residual_H(Point_1, Point_2, iCount, H, pResidual);
		Get_Support(pResidual, iCount, fMax_Residual, &oCur_Support.m_iInlier_Count, &oCur_Support.m_fResidual_Sum);
		//printf("4 Point Trial:%d Inlier:%d Residual:%f\n", oReport.m_iTrial_Count, oCur_Support.m_iInlier_Count, oCur_Support.m_fResidual_Sum);

		if (oCur_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oCur_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oCur_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
		{
			oBest_Support = oCur_Support;
			bBest_Model_is_Local = 0;
			memcpy(Best_Modal, H, 9 * sizeof(float));
			prev_best_num_inliers = oBest_Support.m_iInlier_Count;

			if (oCur_Support.m_iInlier_Count > 4)
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
					Estimate_H(pX_Inlier, pY_Inlier,oBest_Support.m_iInlier_Count, pNorm_Point_1, pNorm_Point_2, H,oPtr);
					Get_Residual_H(Point_1, Point_2, iCount, H, pResidual);
					Get_Support(pResidual, iCount, fMax_Residual, &oLocal_Support.m_iInlier_Count, &oLocal_Support.m_fResidual_Sum);
					if (oLocal_Support.m_iInlier_Count > oBest_Support.m_iInlier_Count || (oLocal_Support.m_iInlier_Count == oBest_Support.m_iInlier_Count && oLocal_Support.m_fResidual_Sum < oBest_Support.m_fResidual_Sum))
					{
						oBest_Support = oLocal_Support;
						memcpy(Best_Modal, H, 9 * sizeof(_T));
						bBest_Model_is_Local = 1;
					}
					if (oBest_Support.m_iInlier_Count <= prev_best_num_inliers)
						break;
					//printf("4 points Trial:%d Full Trial:%d Inlier:%d Residual:%f\n", oReport.m_iTrial_Count,iTrial,
						//oBest_Support.m_iInlier_Count,oBest_Support.m_fResidual_Sum);
				}
			}
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
}
