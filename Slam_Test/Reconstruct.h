#pragma once
#include "sift.h"
#include "Matrix.h"
#include "Image.h"

typedef struct Plane {
	float a, b, d;
}Plane;

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

template<typename _T> struct Schur_Camera_Data {	//专用于Slam后端估计中的数据记录
	typedef struct One_Camera_Data {    //一个相机及其对应点的数据
		typedef struct Point_Data {
			int m_iPoint_Index;
			_T m_Data_6x3[6 * 3];
			//_T Data_3x3[3 * 3];
		}Point_Data;
		_T m_Data_6x6[6 * 6];    //
		Point_Data* m_pPoint_Data;
		int m_iCur = 0; //当前可用点位置，下一个可用位置
		int m_iPoint_Count;
	}One_Camera_Data;
	_T(*m_pData_3x3)[3 * 3]; //3x3的数据是累加起来的
	union {
		One_Camera_Data* m_pCamera_Data;
		unsigned char* m_pBuffer;
	};
	int m_iCamera_Count;
	int m_iPoint_Count;
	int m_iObservation_Count;
};
template<typename _T> struct Measurement {
	int m_Camera_Index[2];    //观察中的两个位姿
	_T Delta_ksi[7];        //存个ksi，前三项相当于位移，后4项为4元数
};
template<typename _T> struct Pose_Graph_Sigma_H {
	typedef struct Camera_Data {
		_T m_Data_6x6[6 * 6];
		int m_iIndex;   //本数据块在相机组中的索引
	}Camera_Data;
	typedef struct Camera_Line {
		Camera_Data* m_pCamera_Data;     //一个相机在Sigma_H占一行，这是一行数据在块中的开始位置
		int m_iCount;   //每个相机和其他相机产生的联系，属于反自反关系,有<i,j>就不需要<j,i>
	}Camera_Line;

	Camera_Line* m_pLine;
	union {
		Camera_Data* m_pCamera_Data;
		unsigned char* m_pBuffer;
	};
	int m_iCamera_Data_Count;   //比Meaurement Count多 相机个数
	int m_iCamera_Count;        //一共有多少个相机
};

template<typename _T>struct Image_Match_Param {
	_T(*m_pImage_Point_0)[2];
	_T(*m_pImage_Point_1)[2];   //像素平面点对
	_T(*m_pNorm_Point_0)[2];
	_T(*m_pNorm_Point_1)[2];     //归一化平面点对
	_T(*m_pPoint_3D_0)[3];
	_T(*m_pPoint_3D_1)[3];
	Image m_oImage;
};

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
template<typename _T>void Decompose_H(_T H[3 * 3], _T R1[3 * 3], _T R2[3 * 3], _T t1[3], _T t2[3], _T K1[], _T K2[]=NULL);

template<typename _T>void E_2_R_t(_T E[3 * 3], _T Norm_Point_1[][2], _T Norm_Point_2[][2], int iCount, _T R[3 * 3], _T t[3], _T Point_3D[][3]=NULL, int* piCount=NULL);
template<typename _T>void E_2_R_t_Pixel_Pos(_T E[3 * 3], _T Point_1[][2], _T Point_2[][2], _T K[], int iCount, _T R[3 * 3], _T t[3], _T Point_3D[][3]);

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
template<typename _T>void Image_Pos_2_3D(_T Image_Pos[][2], unsigned short Depth[], int iCount, int iWidth, _T K[], _T fDepth_Factor, _T Pos_3D[][3]);

////将屏幕左边转换为归一化平面坐标
template<typename _T>void Image_Pos_2_Norm(_T Image_Pos[][2], int iCount, _T K[], _T(*pNorm_Point)[2]);

//Bundle_Adjust，这是估计的关键，估计有很多种变种
template<typename _T>void Bundle_Adjust_3D2D_1(_T Point_3D_1[][3], _T Point_2D_2[][2], int iCount, _T K[], _T Pose[], int* piResult);
template<typename _T>void ICP_BA_2_Image_1(_T P_1[][3], _T P_2[][3], int iCount, _T Pose[], int* piResult);
template<typename _T>void ICP_BA_2_Image_2(_T P1[][3], _T P2[][3], int iCount, _T Pose[], int* piResult);
template<typename _T>void ICP_SVD(_T P_1[][3], _T P_2[][3], int iCount, _T Pose[], int* piResult);

//此处可能要积累一大堆雅可比
template<typename _T>void Get_Deriv_TP_Ksi(_T T[4 * 4], _T P[3], _T Deriv[4 * 6]);
template<typename _T>void Get_Drive_UV_P(_T fx, _T fy, _T P[3], _T J[2 * 3]);
template<typename _T>void Get_Drive_UV_P(_T K[3 * 3], _T P[3], _T J[2 * 3]);

template<typename _T>void Get_Delta_Pose(_T Pose_1[4 * 4], _T Pose_2[4 * 4], _T Delta_Pose[4 * 4]);

//一组专用于存相机与点关系的函数
template<typename _T>void Init_All_Camera_Data(Schur_Camera_Data<_T>* poData, int Point_Count_Each_Camera[], int iCamera_Count, int iPoint_Count);
template<typename _T>void Free(Schur_Camera_Data<_T>* poData);
template<typename _T>void Distribute_Data(Schur_Camera_Data<_T> oData, _T JJt[9 * 9], int iCamera_ID, int iPoint_ID);
template<typename _T>void Copy_Data_2_Sparse(Schur_Camera_Data<_T> oData, Sparse_Matrix<_T>* poA);

//以下一组函数多用于位姿图
template<typename _T>void TQ_2_Rt(_T TQ[7], _T Rt[4 * 4]);	//7自由度se3转换SE3上的Rt矩阵
template<typename _T>void Init_Pose_Graph(Measurement<_T>* pMeasurement, int iMeasurement_Count, int iCamera_Count, Pose_Graph_Sigma_H<_T>* poPose_Graph);
template<typename _T>void Reset_Pose_Graph(Pose_Graph_Sigma_H<_T> oSigma_H);
template<typename _T>void Get_J_Inv(_T eij[6], _T J_Inv[6 * 6]);
template<typename _T>void Get_Adj(_T Rt[4 * 4], _T Adj[6 * 6]);
template<typename _T>void Distribute_Data(Pose_Graph_Sigma_H<_T> oSigma_H, _T JJt[12 * 12], int iCamera_i, int iCamera_j);
template<typename _T>void Copy_Data_2_Sparse(Pose_Graph_Sigma_H<_T> oPose_Graph, Sparse_Matrix<_T>* poA);


//Schur消元法解Slam线性方程
template<typename _T>void Solve_Linear_Schur(Schur_Camera_Data<_T> oData, _T Sigma_JE[], _T X[], int* pbSuccess = NULL);

template<typename _T> void Optical_Flow_1(Image oImage_1, Image oImage_2, _T KP_1[][2], _T KP_2[][2], int iCount, int* piMatch_Count, int bHas_Initial = 0, int bInverse = false);

template<typename _T>void Disp_Error(_T P1[][3], _T P2[][3], int iCount, _T Pose[4 * 4]);

template<typename _T> void Gen_Cube(_T Cube[][4], float fScale, _T x_Center = 0, _T y_Center = 0, _T z_Center = 0);//生成一个Cube
template<typename _T>void Gen_Cube(_T(**ppCube)[3], int* piCount, float fScale, _T x_Center = 0, _T y_Center = 0, _T z_Center = 0);

template<typename _T>void Gen_Sphere(_T(**ppPoint_3D)[3], int* piCount, _T r = 1.f, int iStep_Count = 40);
template<typename _T>void Gen_Plane(_T(**ppPoint)[3], int* piCount, _T K[3 * 3] = NULL, _T T[4 * 4] = NULL);
template<typename _T>void Gen_Plane_z0(_T(**ppPoint_3D)[3], int* piCount);
template<typename _T>void Get_K_Inv(_T K[3 * 3], _T K_Inv[3 * 3]);	//快速得内参逆矩阵
template<typename _T>void Gen_Pose(_T T[4 * 4], _T v0, _T v1, _T v2, _T theta, _T t0, _T t1, _T t2, int* piResult = NULL);
//Temp Code
template<typename _T> int bTemp_Load_Data(const char* pcFile, _T(**ppT)[7], int* piPoint_Count,
	Measurement<_T>** ppMeasurement, int* piMeasure_Count);
template<typename _T>void Match_2_Image(const char* pcFile_0, const char* pDepth_File_0, const char* pcFile_1, const char* pDepth_File_1,
	_T fDepth_Factor, //深度量化银子
	_T K[],    //内参，可以是NULL
	int* piCount,  //点对数量
	_T(**ppImage_Point_0)[2], _T(**ppImage_Point_1)[2],   //像素平面点对
	_T(**ppNorm_Point_0)[2], _T(**ppNorm_Point_1)[2],     //归一化平面点对
	_T(**ppPoint_3D_0)[3], _T(**ppPoint_3D_1)[3],          //空间点对
	Image* poImage);

template<typename _T>_T fGet_Error(_T Point_3D_0[][3], _T Point_3D_1[][3], _T T[], int iCount);
template<typename _T>void DLT_svd(_T Point_3D_0[][3], _T Point_3D_1[][3], _T Norm_Point_1[][2], int iCount, _T T[]);
template<typename _T>void DLT(_T Point_3D_0[][3], _T Point_3D_1[][3], int iCount, _T T[]);
template<typename _T>_T fGet_Error(_T A[], int m, int n, _T X[]);