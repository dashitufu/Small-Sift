#pragma once
#include "sift.h"
#include "Matrix.h"
#include "Image.h"


typedef struct Recon_Image {	//轮到重建阶段一张图的信息
	void* m_poFeature_Image;	//本图对应的Feature匹配图情况
	int m_iCorrespondence;		//与其他图匹配点之和
	int m_iObservation;			//观测点数量？
}Recon_Image;

typedef struct Ransac_Support {
	int m_iInlier_Count;			//搞那么多虚头八脑的概念，指的就是有多少点配上了
	float m_fResidual_Sum;			//所有的距离差加起来
}Ransac_Support;

typedef struct Ransac_Report {	//该结构表示Ransac结果，暂时只做到Ransac估计
	int m_bSuccess;
	int m_iSample_Count;
	int m_iTrial_Count;	//尝试次数
	int m_iFloat_Size;	//4字节或者8字节
	union {		//可容纳两种Modal
		double m_Modal_d[3 * 3];
		float m_Modal_f[3 * 3];
		unsigned char m_Modal[3 * 3 * 8];
	};
	Ransac_Support m_oSupport;
	unsigned char* m_pInlier_Mask;	//凡是Residual落在范围内则为1
}Ransac_Report;

typedef struct Two_View_Geometry {
	enum Config_Type {	//标志暂且照抄
		UNDEFINED = 0,
		// Degenerate configuration (e.g., no overlap or not enough inliers).
		DEGENERATE = 1,
		// Essential matrix.
		CALIBRATED = 2,
		// Fundamental matrix.
		UNCALIBRATED = 3,
		// Homography, planar scene with baseline.
		PLANAR = 4,
		// Homography, pure rotation without baseline.
		PANORAMIC = 5,
		// Homography, planar or panoramic.
		PLANAR_OR_PANORAMIC = 6,
		// Watermark, pure 2D translation in image borders.
		WATERMARK = 7,
		// Multi-model configuration, i.e. the inlier matches result from multiple
		// individual, non-degenerate configurations.
		MULTIPLE = 8,
	};
	Ransac_Report m_oReport_E, m_oReport_F, m_oReport_H;	//三大矩阵用于估计
	Config_Type m_iConfig;									//决定了用哪个矩阵玩
	int num_inliers;
}Two_View_Geometry;

//对于E,F,H三个矩阵的估计，有一个Report，此处要有内存分配
void Free_Report(Ransac_Report oReport, Mem_Mgr* poMem_Mgr = NULL);
void Disp_Report(Ransac_Report oReport);
template<typename _T>void Normalize_Point(_T(*pPoint_1)[2], _T(*pPoint_2)[2], int iCount, _T(*pNorm_Point_1)[2], _T(*pNorm_Point_2)[2], float f, float c1, float c2);

template<typename _T> void Estimate_F(_T Point_1[][2], _T Point_2[][2], int iCount, _T E[3 * 3], _T(*Norm_Point_1)[2] = NULL, _T(*Norm_Point_2)[2] = NULL, Light_Ptr oPtr = { 0 });
template<typename _T> void Estimate_E(_T Point_1[][2], _T Point_2[][2], int iCount, _T E[3 * 3], _T(*Norm_Point_1)[2] = NULL, _T(*Norm_Point_2)[2] = NULL, Light_Ptr oPtr = { 0 });
template<typename _T> void Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, _T H[3 * 3], _T(*Norm_Point_1)[2] = NULL, _T(*Norm_Point_2)[2] = NULL, Light_Ptr oPtr = { 0 });

template<typename _T> void Ransac_Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr = NULL);
template<typename _T> void Ransac_Estimate_E(_T Point_1[][2], _T Point_2[][2], int iCount, float f, float c1, float c2, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr = NULL);
template<typename _T> void Ransac_Estimate_F(_T Point_1[][2], _T Point_2[][2], int iCount, Ransac_Report* poReport, Mem_Mgr* pMem_Mgr = NULL);

template<typename _T>void Decompose_E(_T E[3 * 3], _T R1[3 * 3], _T R2[3 * 3], _T t1[3], _T t2[3], int bNormalize_t = 1);

template<typename _T>void E_2_R_t(_T E[3 * 3], _T Point_1[][2], _T Point_2[][2], int iCount, _T R[3 * 3], _T t[3], _T Point_3D[][3]=NULL);

template<typename _T> void Compute_Squared_Sampson_Error(_T Point_1[][2], _T Point_2[][2], int iCount, _T E[3 * 3], _T Residual[]);

template<typename _T>void Triangulate_Point(_T Point_1[2], _T Point_2[2], _T P_1[], _T P_2[], _T New_Point[]);

template<typename _T>void Gen_Camera_Intrinsic(_T K[3 * 3], float fFocal, float a, float b, float cx, float cy);

template<typename _T>void Test_E(_T E[], _T Norm_Point_1[][2], _T Norm_Point_2[][2], int iCount);	//验算一下E

//以下两个接口不好，以后视情况而定再做良好接口
template<typename _T> void Estimate_Relative_Pose(Two_View_Geometry oGeo, float Camera_1[3], float Camera_2[2], _T Point_1[][2], _T Point_2[][2], int iCount, Mem_Mgr* poMem_Mgr);
template<typename _T> void Determine_Confg(Two_View_Geometry* poGeo, _T Point_1[][2], _T Point_2[][2], int iCount, _T(**ppNew_Point_1)[2], _T(**ppNew_Point_2)[2], Mem_Mgr* poMem_Mgr = NULL);

//从RGBD图恢复空间点
template<typename _T> void RGBD_2_Point_3D(Image oImage, unsigned short* pDepth, _T K[][3], _T fDepth_Factor, _T Point_3D[][3], int* piPoint_Count, unsigned char Color[][3] = 0);
template<typename _T>void Image_Pos_2_3D(_T Image_Pos[][3], int iCount, _T K[], _T fDepth_Factor, _T Pos_3D[][3]);

//Bundle_Adjust，这是估计的关键，估计有很多种变种
template<typename _T>void Bundle_Adjust_3D2D_1(_T Point_3D_1[][3], _T Point_2D_2[][2], int iCount, _T K[], _T Pose[], int* piResult);