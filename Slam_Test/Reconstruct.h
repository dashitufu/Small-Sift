#pragma once
#include "sift.h"
#include "Matrix.h"

typedef struct Recon_Image {	//轮到重建阶段一张图的信息
	void* m_poFeature_Image;	//本图对应的Feature匹配图情况
	int m_iCorrespondence;		//与其他图匹配点之和
	int m_iObservation;			//观测点数量？
}Recon_Image;

typedef struct Support {
	int m_iInlier_Count;			//搞那么多虚头八脑的概念，指的就是有多少点配上了
	float m_fResidual_Sum;			//所有的距离差加起来
}Support;

typedef struct Report {
	int m_bSuccess;
	int m_iSample_Count;
	int m_iTrial_Count;	//尝试次数
	int m_iFloat_Size;	//4字节或者8字节
	union {		//可容纳两种Modal
		double m_Modal_d[3 * 3];
		float m_Modal_f[3 * 3];
		unsigned char m_Modal[3 * 3 * 8];
	};
	Support m_oSupport;
	unsigned char* m_pInlier_Mask;	//凡是Residual落在范围内则为1
}Report;

static int iRandom(int iStart, int iEnd)
{//从iStart到iEnd之间随机出一个数字，由于RandomInteger太傻逼，没有必要把时间浪费在傻逼身上
	return iStart + iGet_Random_No() % (iEnd - iStart + 1);
}

template<typename _T> void Ransac_Estimate_H(_T Point_1[][2], _T Point_2[][2], int iCount, Report* poReport_H, Mem_Mgr* pMem_Mgr);
