#pragma once
#include "sift.h"
#include "Matrix.h"

typedef struct Recon_Image {	//�ֵ��ؽ��׶�һ��ͼ����Ϣ
	void* m_poFeature_Image;	//��ͼ��Ӧ��Featureƥ��ͼ���
	int m_iCorrespondence;		//������ͼƥ���֮��
	int m_iObservation;			//�۲��������
}Recon_Image;

typedef struct Support {
	int m_iInlier_Count;			//����ô����ͷ���Եĸ��ָ�ľ����ж��ٵ�������
	float m_fResidual_Sum;			//���еľ���������
}Support;

typedef struct Report {
	int m_bSuccess;
	int m_iSample_Count;
	int m_iTrial_Count;	//���Դ���
	int m_iFloat_Size;	//4�ֽڻ���8�ֽ�
	union {		//����������Modal
		double m_Modal_d[3 * 3];
		float m_Modal_f[3 * 3];
		unsigned char m_Modal[3 * 3 * 8];
	};
	Support m_oSupport;
	unsigned char* m_pInlier_Mask;	//����Residual���ڷ�Χ����Ϊ1
}Report;

static int iRandom(int iStart, int iEnd)
{//��iStart��iEnd֮�������һ�����֣�����RandomInteger̫ɵ�ƣ�û�б�Ҫ��ʱ���˷���ɵ������
	return iStart + iGet_Random_No() % (iEnd - iStart + 1);
}

template<typename _T> void Ransac_Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, Report* poReport_H, Mem_Mgr* pMem_Mgr);
