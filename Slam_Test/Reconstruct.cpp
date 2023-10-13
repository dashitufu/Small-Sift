//�ؽ��õĴ���
#include "Reconstruct.h"

#define SQR(Value) (Value)*(Value)

void SB_Reconstruct()
{//����Ǹ�ɵ�Ʒ�����������ƭtemplate
	Ransac_Estimate_H((double(*)[2])NULL, (double(*)[2])NULL, 1, NULL, NULL);
	Ransac_Estimate_H((float(*)[2])NULL, (float(*)[2])NULL, 1, NULL, NULL);
}

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

template<typename _T> void Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, _T Norm_Point_1[][2], _T Norm_Point_2[][2], _T H[3 * 3], Light_Ptr oPtr = { 0 })
{//�������������һ��Homograph�������� min ( ||P1*X-P2||^2)
	_T M_1[3][3], M_2[3][3];
	int i, j,iResult;

	//�����¿��Կ���������������ĵĹ�񻯺�������ֵ��һ���ӽ�
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
{//��H����������Ʋ�����Sampson����
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

template<typename _T> void Ransac_Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, Report* poReport, Mem_Mgr* pMem_Mgr)
{//���ص�ReportͳһΪdouble����
	Mem_Mgr* poMem_Mgr;
	short* pSample_Index;
	_T* pResidual, (*pX_Inlier)[2], (*pY_Inlier)[2],
		(*pNorm_Point_1)[2], (*pNorm_Point_2)[2];	//Ϊ���ٶȣ�Norm_Point �ȷ����ڴ�

	_T Best_Modal[3 * 3], X_rand[4][2], Y_rand[4][2], H[3 * 3];
	Light_Ptr oPtr;
	int i, j, iTrial, bBest_Model_is_Local, dyn_max_num_trials = 0, bAbort = 0;
	int prev_best_num_inliers;
	Support oLocal_Support = {}, oCur_Support = {}, oBest_Support = { 0,(float)1e30 };	//�������
	Report oReport = {};

	//RansacҪ��1, ����ָ��һ����������ֵ�����ڴ���ֵ��������������
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

		//RansacҪ��2�����ȡһ��㹹��һ���ֲڵĽ⡣����������С�������ǳ����ģ�
		//�������е������㼯�Ǵֲڵ�		
		Sample_XY(Point_1, Point_2, pSample_Index, iCount, X_rand, Y_rand, 4);
		//RansacҪ��3������㷨��ÿ�����ݶ��в�ͬ����ⷽ������һ���㣬�ʴ�Ransacֻ��
		//����ṹ�ϵ����壬û��ϸ���ϵ����壬�˴�Ҫ�Լ�д�Լ�����ⷽ��
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

	//����ͷ�
	if (pMem_Mgr)
		Free(poMem_Mgr, oPtr.m_pBuffer);
	else
	{//�Լ�����Mem_Mgr�Լ������ͷ�
		Free_Mem_Mgr(poMem_Mgr);
		free(poMem_Mgr);
	}
}
