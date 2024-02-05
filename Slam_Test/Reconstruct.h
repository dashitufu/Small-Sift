#pragma once
#include "sift.h"
#include "Matrix.h"
#include "Image.h"


typedef struct Recon_Image {	//�ֵ��ؽ��׶�һ��ͼ����Ϣ
	void* m_poFeature_Image;	//��ͼ��Ӧ��Featureƥ��ͼ���
	int m_iCorrespondence;		//������ͼƥ���֮��
	int m_iObservation;			//�۲��������
}Recon_Image;

typedef struct Ransac_Support {
	int m_iInlier_Count;			//����ô����ͷ���Եĸ��ָ�ľ����ж��ٵ�������
	float m_fResidual_Sum;			//���еľ���������
}Ransac_Support;

typedef struct Ransac_Report {	//�ýṹ��ʾRansac�������ʱֻ����Ransac����
	int m_bSuccess;
	int m_iSample_Count;
	int m_iTrial_Count;	//���Դ���
	int m_iFloat_Size;	//4�ֽڻ���8�ֽ�
	union {		//����������Modal
		double m_Modal_d[3 * 3];
		float m_Modal_f[3 * 3];
		unsigned char m_Modal[3 * 3 * 8];
	};
	Ransac_Support m_oSupport;
	unsigned char* m_pInlier_Mask;	//����Residual���ڷ�Χ����Ϊ1
}Ransac_Report;

typedef struct Two_View_Geometry {
	enum Config_Type {	//��־�����ճ�
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
	Ransac_Report m_oReport_E, m_oReport_F, m_oReport_H;	//����������ڹ���
	Config_Type m_iConfig;									//���������ĸ�������
	int num_inliers;
}Two_View_Geometry;

template<typename _T> struct Schur_Camera_Data {	//ר����Slam��˹����е����ݼ�¼
	typedef struct One_Camera_Data {    //һ����������Ӧ�������
		typedef struct Point_Data {
			int m_iPoint_Index;
			_T m_Data_6x3[6 * 3];
			//_T Data_3x3[3 * 3];
		}Point_Data;
		_T m_Data_6x6[6 * 6];    //
		Point_Data* m_pPoint_Data;
		int m_iCur = 0; //��ǰ���õ�λ�ã���һ������λ��
		int m_iPoint_Count;
	}One_Camera_Data;
	_T(*m_pData_3x3)[3 * 3]; //3x3���������ۼ�������
	union {
		One_Camera_Data* m_pCamera_Data;
		unsigned char* m_pBuffer;
	};
	int m_iCamera_Count;
	int m_iPoint_Count;
	int m_iObservation_Count;
};
template<typename _T> struct Measurement {
	int m_Camera_Index[2];    //�۲��е�����λ��
	_T Delta_ksi[7];        //���ksi��ǰ�����൱��λ�ƣ���4��Ϊ4Ԫ��
};
template<typename _T> struct Pose_Graph_Sigma_H {
	typedef struct Camera_Data {
		_T m_Data_6x6[6 * 6];
		int m_iIndex;   //�����ݿ���������е�����
	}Camera_Data;
	typedef struct Camera_Line {
		Camera_Data* m_pCamera_Data;     //һ�������Sigma_Hռһ�У�����һ�������ڿ��еĿ�ʼλ��
		int m_iCount;   //ÿ����������������������ϵ�����ڷ��Է���ϵ,��<i,j>�Ͳ���Ҫ<j,i>
	}Camera_Line;

	Camera_Line* m_pLine;
	union {
		Camera_Data* m_pCamera_Data;
		unsigned char* m_pBuffer;
	};
	int m_iCamera_Data_Count;   //��Meaurement Count�� �������
	int m_iCamera_Count;        //һ���ж��ٸ����
};

//����E,F,H��������Ĺ��ƣ���һ��Report���˴�Ҫ���ڴ����
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

template<typename _T>void Test_E(_T E[], _T Norm_Point_1[][2], _T Norm_Point_2[][2], int iCount);	//����һ��E

//���������ӿڲ��ã��Ժ�����������������ýӿ�
template<typename _T> void Estimate_Relative_Pose(Two_View_Geometry oGeo, float Camera_1[3], float Camera_2[2], _T Point_1[][2], _T Point_2[][2], int iCount, Mem_Mgr* poMem_Mgr);
template<typename _T> void Determine_Confg(Two_View_Geometry* poGeo, _T Point_1[][2], _T Point_2[][2], int iCount, _T(**ppNew_Point_1)[2], _T(**ppNew_Point_2)[2], Mem_Mgr* poMem_Mgr = NULL);

//��RGBDͼ�ָ��ռ��
template<typename _T> void RGBD_2_Point_3D(Image oImage, unsigned short* pDepth, _T K[][3], _T fDepth_Factor, _T Point_3D[][3], int* piPoint_Count, unsigned char Color[][3] = 0);
template<typename _T>void Image_Pos_2_3D(_T Image_Pos[][3], int iCount, _T K[], _T fDepth_Factor, _T Pos_3D[][3]);

//Bundle_Adjust�����ǹ��ƵĹؼ��������кܶ��ֱ���
template<typename _T>void Bundle_Adjust_3D2D_1(_T Point_3D_1[][3], _T Point_2D_2[][2], int iCount, _T K[], _T Pose[], int* piResult);
template<typename _T>void ICP_BA_2_Image_1(_T P_1[][3], _T P_2[][3], int iCount, _T Pose[], int* piResult);
template<typename _T>void ICP_BA_2_Image_2(_T P1[][3], _T P2[][3], int iCount, _T Pose[], int* piResult);
template<typename _T>void ICP_SVD(_T P_1[][3], _T P_2[][3], int iCount, _T Pose[], int* piResult);

//�˴�����Ҫ����һ����ſɱ�
template<typename _T>void Get_Deriv_TP_Ksi(_T T[4 * 4], _T P[3], _T Deriv[4 * 6]);
template<typename _T>void Get_Drive_UV_P(_T fx, _T fy, _T P[3], _T J[2 * 3]);
template<typename _T>void Get_Drive_UV_P(_T K[3 * 3], _T P[3], _T J[2 * 3]);

template<typename _T>void Get_Delta_Pose(_T Pose_1[4 * 4], _T Pose_2[4 * 4], _T Delta_Pose[4 * 4]);

//һ��ר���ڴ��������ϵ�ĺ���
template<typename _T>void Init_All_Camera_Data(Schur_Camera_Data<_T>* poData, int Point_Count_Each_Camera[], int iCamera_Count, int iPoint_Count);
template<typename _T>void Free(Schur_Camera_Data<_T>* poData);
template<typename _T>void Distribute_Data(Schur_Camera_Data<_T> oData, _T JJt[9 * 9], int iCamera_ID, int iPoint_ID);
template<typename _T>void Copy_Data_2_Sparse(Schur_Camera_Data<_T> oData, Sparse_Matrix<_T>* poA);

//����һ�麯��������λ��ͼ
template<typename _T>void TQ_2_Rt(_T TQ[7], _T Rt[4 * 4]);	//7���ɶ�se3ת��SE3�ϵ�Rt����
template<typename _T>void Init_Pose_Graph(Measurement<_T>* pMeasurement, int iMeasurement_Count, int iCamera_Count, Pose_Graph_Sigma_H<_T>* poPose_Graph);
template<typename _T>void Reset_Pose_Graph(Pose_Graph_Sigma_H<_T> oSigma_H);
template<typename _T>void Get_J_Inv(_T eij[6], _T J_Inv[6 * 6]);
template<typename _T>void Get_Adj(_T Rt[4 * 4], _T Adj[6 * 6]);
template<typename _T>void Distribute_Data(Pose_Graph_Sigma_H<_T> oSigma_H, _T JJt[12 * 12], int iCamera_i, int iCamera_j);
template<typename _T>void Copy_Data_2_Sparse(Pose_Graph_Sigma_H<_T> oPose_Graph, Sparse_Matrix<_T>* poA);


//Schur��Ԫ����Slam���Է���
template<typename _T>void Solve_Linear_Schur(Schur_Camera_Data<_T> oData, _T Sigma_JE[], _T X[], int* pbSuccess = NULL);

template<typename _T> void Optical_Flow_1(Image oImage_1, Image oImage_2, _T KP_1[][2], _T KP_2[][2], int iCount, int* piMatch_Count, int bHas_Initial = 0, int bInverse = false);

template<typename _T>void Disp_Error(_T P1[][3], _T P2[][3], int iCount, _T Pose[4 * 4]);

//Temp Code
template<typename _T> int bTemp_Load_Data(const char* pcFile, _T(**ppT)[7], int* piPoint_Count,
	Measurement<_T>** ppMeasurement, int* piMeasure_Count);