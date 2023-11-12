//搞个临时文件，想搞明白Essential Matrix
#include "stdio.h"
#include "Image.h"
#include "Reconstruct.h"

static void E_Test_1()
{//找一组数据验算E矩阵。虽然代码有点散，但咬死了推导过程
#define SAMPLE_COUNT 8
	typedef double _T;

	_T Sample[SAMPLE_COUNT][4] = { {48.00000000, 76.00000000, 300.00000000, 1.00000000},
									{18.00000000, 303.00000000, 300.00000000, 1.00000000},
									{135.00000000, 182.00000000, 300.00000000, 1.00000000},
									{123.00000000, 182.00000000, 300.00000000, 1.00000000},
									{379.00000000, 12.00000000, 300.00000000, 1.00000000},
									{204.00000000, 7.00000000, 300.00000000, 1.00000000},
									{829.00000000, 3.00000000, 300.00000000, 1.00000000},
									{252.00000000, 39.00000000, 300.00000000, 1.00000000} }	
	, K[3 * 3], Pos[4];

	_T Screen_Pos_1[SAMPLE_COUNT][2] = {},Screen_Pos_2[SAMPLE_COUNT][2];
	Image oImage;
	int i;

	Init_Image(&oImage, 1920, 1080, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);

	//for (i = 0; i < 80; i++)
	//{
	//	float fAngle = (2.f * PI * i) / 80.f;
	//	
	//	Sample[i][0] = cos(fAngle) * (i*5+400);
	//	Sample[i][1] = sin(fAngle) * (i*5+400);
	//	Sample[i][2] = 300;
	//	Sample[i][3] = 1;
	//	//printf("%f %f %f\n", fAngle, cos(fAngle), Sample[i][0]);
	//}
//Start:	//本来此处想来个循环寻找一组合适的点
	//Disp((_T*)Sample, 8, 4,"Sample");

	//先搞搞相机内参，先把这个搞利索了
	Gen_Camera_Intrinsic(K,1,100,-100,960,540);
	//Disp(K, 3, 3, "K");
	
	for (i = 0; i < SAMPLE_COUNT; i++)
	{
		Matrix_Multiply(K, 3, 3, Sample[i], 1, Pos);
		Matrix_Multiply(Pos, 1, 2, Pos[2]!=0?(float)(1.f / Pos[2]):0, Screen_Pos_1[i]);
		//Disp(Screen_Pos_1[i], 1, 3);
		Draw_Point(oImage, (int)Screen_Pos_1[i][0], (int)Screen_Pos_1[i][1],1);
		//sprintf(File, "c:\\tmp\\temp\\%d.bmp", i);
		//bSave_Image(File, oImage);
	}
	bSave_Image("c:\\tmp\\temp\\1.bmp", oImage);
	//再构造一个相机外参 Rt, 绕y轴旋转30度, 再平移 30,0
	_T Rotation_Vector[4] = { 0,1,0, PI * 5 / 180 }, t[3] = { -10,0,0 },
		R[3 * 3], Rt[4 * 4];
	_T Temp_1[4*4];

	Rotation_Vector_2_Matrix(Rotation_Vector, R);
	Gen_Homo_Matrix(R, t, Rt);
	Get_Inv_Matrix_Row_Op(Rt, Rt, 4);
	
	//Disp(Rt, 4, 4, "Rt");
	Set_Color(oImage);
	
	for (i = 0; i < SAMPLE_COUNT; i++)
	{
		//先外参， P1 = (1/z) * K*T*P0
		Matrix_Multiply(Rt, 4, 4, Sample[i],1, Pos);
		Matrix_Multiply(K, 3, 3, Pos, 1, Pos);
		Matrix_Multiply(Pos, 1, 2,Pos[2]!=0? (float)(1.f/Pos[2]):0,Screen_Pos_2[i]);
		//Disp(Screen_Pos_2[i], 1, 3);
		Draw_Point(oImage, (int)Screen_Pos_2[i][0], (int)Screen_Pos_2[i][1],1);
		//sprintf(File, "c:\\tmp\\temp\\%d.bmp", i);
		//bSave_Image(File, oImage);
	}
	bSave_Image("c:\\tmp\\temp\\2.bmp", oImage);
	//Disp((_T*)Screen_Pos_2, 8,2);

	//八点有了，看看能否搞个E
	_T Norm_Point_1[8][2], Norm_Point_2[8][2], E[3 * 3];
	_T Residual[8];
	Normalize_Point(Screen_Pos_1, Screen_Pos_2, 8, Norm_Point_1, Norm_Point_2,(float) K[0], (float)K[2], (float)K[5]);
	Estimate_E(Norm_Point_1, Norm_Point_2, 8,E);
	Compute_Squared_Sampson_Error(Norm_Point_1, Norm_Point_2, 8, E, Residual);
	
	Ransac_Report oReport_E;
	Ransac_Estimate_E(Screen_Pos_1, Screen_Pos_2, SAMPLE_COUNT, (float)K[0], (float)K[2], (float)K[5],&oReport_E);
	/*if (oReport_E.m_bSuccess)
	{
		Disp((_T*)Sample, 8, 4, "Sample");
	}*/
	Disp((_T*)Norm_Point_1, 8, 2, "Norm_Point_1");
	Disp((_T*)Norm_Point_2, 8, 2, "Norm_Point_2");

	Disp(E, 3, 3, "E");
	
	//Disp((_T*)Norm_Point_1, 8, 2);	
	E_2_R_t(E, Norm_Point_1, Norm_Point_2,8, R, t);

	//R,t再验算一下
	_T I[4*4],New_Point[8][4];
	Gen_Homo_Matrix(R, t, Rt);
	Gen_I_Matrix(I, 4, 4);
	for (i = 0; i < 8; i++)
	{//以下验算在三角化的范围内成功，根据求解路径可知 相机内参为I。 相机1的外参为I 相机2的外参为Rt
		_T z1, z2;
		Triangulate_Point(Norm_Point_1[i], Norm_Point_2[i], I, Rt, New_Point[i]);
		Disp(New_Point[i], 1, 4,"Point_3D");

		//验算  (1/z) * KP * P1
		Matrix_Multiply(I, 4, 4, New_Point[i], 1, Temp_1);
		z1 = Temp_1[2];
		Matrix_Multiply(Temp_1, 1, 4, (float)(1.f/z1),Temp_1);
		Disp(Norm_Point_1[i], 1, 2, "Norm_Point_1");
		Disp(Temp_1, 1, 3, "相机1的投影");

		//验算 (1/z) * KP * P2
		Matrix_Multiply(Rt, 4, 4, New_Point[i], 1, Temp_1);
		z2 = Temp_1[2];
		Matrix_Multiply(Temp_1, 1, 4, (float)(1.f / z1), Temp_1);
		Disp(Norm_Point_2[i], 1, 2, "Norm_Point_2");
		Disp(Temp_1, 1, 3, "相机2的投影");

		//先看 z * NP1
		Matrix_Multiply(Norm_Point_1[i], 1, 2, (float)z1, Temp_1);
		Disp(Temp_1, 1, 2, "z1 * NP1");

		//再验算 z * Rt(-1) * NP2 = P 
		int iResult;
		_T Rt_1[4 * 4],Temp_2[4];

		Get_Inv_Matrix_Row_Op(Rt, Rt_1, 4, &iResult);
		Matrix_Multiply(Rt, 4, 4, New_Point[i], 1, Temp_1);
		Disp(Temp_1, 1, 4, "Rt * P");
		
		memcpy(Temp_2, Norm_Point_2[i], 2 * sizeof(_T));
		Temp_2[2] = 1.f;
		Matrix_Multiply(Temp_2, 1, 3, (float)z2, Temp_1);
		Disp(Temp_1, 1, 3, "z2 * NP2");

		Temp_1[3] = 1;
		Matrix_Multiply(Rt_1, 4, 4, Temp_1, 1, Temp_1);
		Disp(Temp_1, 1, 4, " z2 * Rt(-1) * NP2");
//
		memcpy(Temp_2, Norm_Point_1[i], 2 * sizeof(_T));
		Temp_2[2] = 1.f;
		Matrix_Multiply(Temp_2, 1, 3, (float)z1, Temp_1);
		Temp_1[3] = 1.f;
		Disp(Temp_1, 1, 4, "z1 * NP1");

		//此处已经有了一个非常非常棒的结果， NP2 的齐次坐标 = z1/z2 * Rt * NP1
		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);
		Matrix_Multiply(Temp_1, 1, 4, (float)(1.f / z2),Temp_1);
		Disp(Temp_1, 1, 4,"z1/z2 * Rt * NP1");

		memcpy(Temp_1, Norm_Point_1[i], 2 * sizeof(_T));
		Temp_1[2] = 1;
		Matrix_Multiply(Temp_1, 1, 3, (float)z1, Temp_1);
		Disp(Temp_1, 1, 4, "z1 * NP1");
		
		//此时，既然已经恢复了深度，那么z1 * NP1 已经是3维坐标，为了与Rt相乘，必须再化为齐次坐标
		Temp_1[3] = 1;
		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);
		Matrix_Multiply(Temp_1, 1, 4, (float)(1.f / z2), Temp_1);
		Disp(Temp_1, 1, 4, "z1/z2 * Rt * NP1");


		//实践证明， 首先 x'= R(x-t) 的运算顺序不适合这个模型，显然这是先位移后旋转。而我们
		//整个相机模型都是先旋转后位移
		//齐次，R是三维，x,x'是二维，也不能直接计算

		//正确的关系是 x' = (z1/z2) * Rt * NP1, 此时，z1,z2的参与必不可少，因为要恢复齐次
		//而z1,z2只有三角化以后才有，故此要验算这个结果，必须逐步恢复齐次坐标
		memcpy(Temp_1, Norm_Point_1[i], 2 * sizeof(_T));
		Matrix_Multiply(Temp_1, 1, 2, (float)z1, Temp_1);	//第一次化齐次坐标 (x,y,1) * z1
		Temp_1[2] = z1, Temp_1[3] = 1;				//第二次化齐次坐标 (z1*x, z1*y, z1, 1)

		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);		//再算 Rt * x
		Matrix_Multiply(Temp_1, 1, 4, (float)(1.f / z2), Temp_1);	//最后再除以 z2, 此时就符合三角化公式了
		Disp(Temp_1, 1, 4, "z1/z2 * Rt * NP1");
	}

	_T V[4];
	Rotation_Matrix_2_Vector(R, V);

	//开始再次搞那堆E的推导，目标是验证 x' = R(x-t)
	_T Norm_Point_Homo_1[8][3], Norm_Point_Homo_2[8][3];
	for (i = 0; i < 8; i++)
	{
		Norm_Point_Homo_1[i][0] = Norm_Point_1[i][0];
		Norm_Point_Homo_1[i][1] = Norm_Point_1[i][1];
		Norm_Point_Homo_1[i][2] = 1;

		Norm_Point_Homo_2[i][0] = Norm_Point_2[i][0];
		Norm_Point_Homo_2[i][1] = Norm_Point_2[i][1];
		Norm_Point_Homo_2[i][2] = 1;
	}

	//先验证 x'*E*x=0，从此处看出，数值的最终结果在0.00x 级别上	
	for (i = 0; i < 8; i++)
	{
		Matrix_Multiply(Norm_Point_Homo_2[i], 1, 3, E, 3, Temp_1);
		Matrix_Multiply(Temp_1, 1, 3, Norm_Point_Homo_1[i], 1, Temp_1);
		printf("x\'*E*x= %f\n", Temp_1[0]);
	}

	//再验证 x'= R(x-t)
	for (i = 0; i < 8; i++)
	{
		Vector_Minus(Norm_Point_Homo_1[i], t, 3,Temp_1);
		Matrix_Multiply(R, 3, 3, Temp_1, 1, Temp_1);
	}
	return;
}

void Camera_Param_Test()
{//相机参数实验
	typedef float _T;

	//设有一点，在世界坐标(0,0,100)
	_T Point_0[4] = { 0,0,100,1 };
	//原相机在(0,0,0)处
	_T Camera_0[4] = { 0, 0, 0, 1 };

	//先看点的移动,先将相机绕y旋转60度, 先左乘R ,再右乘T，即先旋转再位移
	//在z正对屏幕方向为正，绕y轴旋转的情况下，旋转角度为正相当于顺时针
	_T Rotation_Vector[4] = { 0,1,0, -60 * PI / 180 },
		t[3] = { (_T)(Point_0[2] * sqrt(3.f)),0,0 };
	_T R[3 * 3];	
	_T Point_1[4];
	Rotation_Vector_2_Matrix(Rotation_Vector, R);
	Matrix_Multiply(R, 3, 3, Point_0, 1, Point_1);
	//Disp(Point_1, 1, 3, "P0 move to");
	Vector_Add(Point_1, t, 3, Point_1);
	//Disp(Point_1, 1, 3, "Point_0 move to");
		
	//再试一下相机参数，必须画图，从以下实验可以看出，w2c正式相机1的外参
	//用一个特例证明的w2c就是相机外参
	_T c2w[4 * 4], w2c[4 * 4];
	Gen_Homo_Matrix(R, t, c2w);
	Get_Inv_Matrix_Row_Op(c2w, w2c, 4);
	Matrix_Multiply(w2c, 4, 4, Point_0, 1, Point_1);
	//Disp(Point_1, 1, 4, "Point_1");

	//用两个过程看相机的外参
	_T Temp_1[4 * 4] = {}, Temp_2[4 * 4],Temp_3[4*4];
	Gen_Homo_Matrix(R, (_T*)NULL, Temp_1);
	Gen_Homo_Matrix((_T*)NULL, t, Temp_2);	
	Matrix_Multiply(Temp_2, 4, 4, Temp_1, 4, Temp_3);
	//Disp(Temp_3, 4, 4, "Rt");	//可见先R后t才是c2w

	Matrix_Multiply(Temp_1, 4, 4, Temp_2, 4, Temp_3);
	Disp(c2w, 4, 4, "c2w");
	//Disp(Temp_3, 4, 4, "tR");	//先t后R不是c2w

	Get_Inv_Matrix_Row_Op(Temp_3, Temp_3,4);
	Matrix_Multiply(Temp_3, 4, 4, Point_0, 1, Point_1);
	//Disp(Point_1, 1, 4, "Point_1");
	
	Matrix_Multiply(w2c, 4, 4, Point_0, 1, Point_1);
	//Disp(Point_0, 1, 4, "x0");
	//Disp(Point_1, 1, 4, "x1");
	
	{//先试一下最简单的平行投影
		_T x0[2] = { Point_0[0],Point_0[1] },
			x1[2] = { Point_1[0],Point_1[1] };
		_T I[4 * 4],Point_3D[4];
		Gen_I_Matrix(I, 4, 4);
		
		Triangulate_Point(x0, x1, I, w2c, Point_3D);
		//Disp(Point_3D, 1, 3, "Point_3D");	//神奇的一幕发生了，确实能恢复到世界坐标
	}
	
	{//第二个实验，给像素平面的坐标是否能三角化出原坐标
		_T I[4*4],K[3 * 3], KP_0[4 * 4],KP_1[4*4];
		_T x0[4], x1[4], Point_3D[4];

		//此处K是到像素平面
		Gen_Camera_Intrinsic(K, 200, 1, 1, 960, 540);
		Gen_Homo_Matrix(K, (_T*)NULL, KP_0);	//可以用相机内参构造一个齐次矩阵
		Gen_I_Matrix(I, 4, 4);
		Disp(I, 4, 4, "I");
		Matrix_Multiply(KP_0, 4, 4, I, 4, KP_0);
		Disp(KP_0, 4, 4, "KP");

		Matrix_Multiply(KP_0, 4, 4, Point_0,1, x0);
		//记得要乘以 1/Z，否则不是投影
		Matrix_Multiply(x0, 1, 2, 1.f / x0[2],x0);
		Disp(Point_0, 1, 3, "Point_0");
		Disp(x0, 1, 2,"x0");

		Gen_Homo_Matrix(K, (_T*)NULL, KP_1);
		Matrix_Multiply(KP_1, 4, 4, w2c, 4, KP_1);

		Matrix_Multiply(KP_1, 4, 4, Point_0, 1, x1);
		//记得要乘以 1/Z，否则不是投影
		Matrix_Multiply(x1, 1, 4, 1.f / x1[2], x1);
		Disp(x1, 1, 4,"x1");
		//至此， x0,x1都已经是 (1/z)* KP * P 的结果

		Triangulate_Point(x0, x1, KP_0, KP_1, Point_3D);
		Disp(Point_3D, 1, 3, "Point_3D");	//实在太神奇了，到像素屏幕坐标恢复成功
	}
	{//用归一化平面来做相机参数，看看能否三角化
		_T focal = 1, cx = 0.5, cy = 0.5;
		_T K[3 * 3],KP_0[4*4],KP_1[4*4],I[4*4];
		_T x0[4], x1[4], Point_3D[4];
		Gen_Camera_Intrinsic(K, focal, 1, 1, cx, cy);
		Disp(K, 3, 3, "K");

		////看看相机内参对不对，是对的
		//Matrix_Multiply(K, 3, 3, Point_0, 1, x0);
		//Matrix_Multiply(x0, 1, 3, 1.f / x0[2], x0);
		//Disp(x0, 1, 2, "x0");

		Gen_I_Matrix(I, 4, 4);
		Gen_Homo_Matrix(K, (_T*)NULL, KP_0);
		Matrix_Multiply(KP_0, 4, 4, I, 4, KP_0);

		Gen_Homo_Matrix(K, (_T*)NULL, KP_1);
		Matrix_Multiply(KP_1, 4, 4, w2c,4, KP_1);

		Matrix_Multiply(KP_0, 4, 4, Point_0, 1, x0);
		Matrix_Multiply(x0, 1, 4, 1.f / x0[2], x0);
		Disp(x0, 1, 4, "x0");

		Matrix_Multiply(KP_1, 4, 4, Point_0, 1, x1);
		Matrix_Multiply(x1, 1, 4, 1.f / x1[2], x1);
		Disp(x1, 1, 4, "x1");

		Triangulate_Point(x0, x1, KP_0, KP_1, Point_3D);
		Disp(Point_3D, 1, 3, "Point_3D");	//完美！投到归一化平面也行
	}
	return;
}

void E_Test_2()
{//最简E 验算代码，给定一个E, 还有两个归一化匹配点NP1与NP2。验算一把 NP2=(z1/z2) * Rt * NP1
	typedef double _T;
	_T Norm_Point_1[8][2] = {	{0.16000024, -0.25333321},
								{0.06000023, -1.00999989},
								{0.45000024, -0.60666655},
								{0.41000024, -0.60666655},
								{1.26333360, -0.03999987},
								{0.68000025, -0.02333320},
								{2.76333363, -0.00999987},
								{0.84000025, -0.12999987} },
		Norm_Point_2[8][2] = {	{0.10408432, -0.25007111},
								{0.00579728, -1.00564635},
								{0.37978469, -0.58427734},
								{0.34255785, -0.58624562},
								{1.08598052, -0.03606178},
								{0.58908081, -0.02204666},
								{2.17661246, -0.00806469},
								{0.73006282, -0.12123358} },
		E[3 * 3] = { 0.04694487, 0.10917079, 0.56591215,
					-0.18813973, -0.00669381, -0.99870273,
					-0.56528370, 1.01189271, 0.03976162 };

	Test_E(E, Norm_Point_1, Norm_Point_2, 8);
	return;
}

static void Sift_Test_1()
{//Sift实验，给出一张黑白图，求所有的特征点
	float(*pPoint)[2];
	int iCount;
	unsigned long long tStart = iGet_Tick_Count();
	Get_Sift_Feature("c:\\tmp\\Screen_Cut_A.bmp", &pPoint, &iCount, 0);
	printf("%lld\n", iGet_Tick_Count() - tStart);
	free(pPoint);
	return;
}
static void Sift_Test_2()
{//两张图的匹配，返回两组匹配点位置
	float(*pPoint_1)[2] = NULL, (*pPoint_2)[2];
	int iMatch_Count;

	Sift_Match_2_Image("C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\Ajay\\A16.bmp",
		"C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\Ajay\\A17.bmp", &pPoint_1, &pPoint_2, &iMatch_Count);

	for (int i = 0; i < iMatch_Count; i++)
		printf("%f %f %f %f\n", pPoint_1[i][0], pPoint_1[i][1], pPoint_2[i][0], pPoint_2[i][1]);
	free(pPoint_1);
	return;
}
static void Sift_Test_3()
{//整个目录遍历，此处用Mem_Mgr
	Sift_Match_Map oMatch_Map;
	Sift_Simple_Match_Item oMatch;
	Mem_Mgr oMem_Mgr;
	Init_Mem_Mgr(&oMem_Mgr, 128000000, 1024, 997);
	Sift_Match_Path("C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\ET\\bmp", &oMatch_Map, &oMem_Mgr);
	int y, x, iIndex;
	for (y = 0; y < oMatch_Map.m_iImage_Count; y++)
	{
		for (x = y + 1; x < oMatch_Map.m_iImage_Count; x++)
		{
			iIndex = iUpper_Triangle_Cord_2_Index(x, y, oMatch_Map.m_iImage_Count);
			oMatch = oMatch_Map.m_pMatch[iIndex];
			printf("Image Pair:%d %d Match Count:%d\n", oMatch.m_iImage_A, oMatch.m_iImage_B, oMatch.m_iMatch_Count);
			/* for (int k = 0; k < oMatch.m_iMatch_Count; k++)
				 printf("%f %f %f %f\n", oMatch.m_pPoint_1[k][0], oMatch.m_pPoint_1[k][1],
					 oMatch.m_pPoint_2[k][0], oMatch.m_pPoint_2[k][1]);*/
		}
	}
	Free_Mem_Mgr(&oMem_Mgr);
	return;
}
static void Sift_Test_4()
{//整个目录遍历，最简接口
	Sift_Match_Map oMatch_Map;
	Sift_Simple_Match_Item oMatch;

	Sift_Match_Path("C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\Ajay", &oMatch_Map);
	int y, x, iIndex;
	for (y = 0; y < oMatch_Map.m_iImage_Count; y++)
	{
		for (x = y + 1; x < oMatch_Map.m_iImage_Count; x++)
		{
			iIndex = iUpper_Triangle_Cord_2_Index(x, y, oMatch_Map.m_iImage_Count);
			oMatch = oMatch_Map.m_pMatch[iIndex];
			printf("Image Pair:%d %d Match Count:%d\n", oMatch.m_iImage_A, oMatch.m_iImage_B, oMatch.m_iMatch_Count);
			//for (int k = 0; k < oMatch.m_iMatch_Count; k++)
				//printf("%f %f %f %f\n", oMatch.m_pPoint_1[k][0], oMatch.m_pPoint_1[k][1], oMatch.m_pPoint_2[k][0], oMatch.m_pPoint_2[k][1]);
		}
	}
	if(oMatch_Map.m_pBuffer)
		free(oMatch_Map.m_pBuffer);
	return;
}

void Ransac_Test()
{
	typedef double _T;
	_T(*pPoint_1)[2], (*pPoint_2)[2], (*pNew_Point_1)[2], (*pNew_Point_2)[2];
	int iCount;
	float Camera[3] = { 768,320,240 };
	Ransac_Report oReport_H, oReport_E, oReport_F;
	Mem_Mgr oMem_Mgr;
	Init_Mem_Mgr(&oMem_Mgr, 100000000, 1024, 997);
	Temp_Load_Match_Point(&pPoint_1, &pPoint_2, &iCount);
	//Sift_Match_2_Image("C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\ET\\bmp\\et000.bmp",
	  //  "C:\\Users\\Administrator\\Desktop\\colmap-dev\\ComputerVisionDatasets-master\\Datasets\\ET\\bmp\\et003.bmp", &pPoint_1, &pPoint_2, &iCount);

	Ransac_Estimate_H(pPoint_1, pPoint_2, iCount, &oReport_H, &oMem_Mgr);
	Ransac_Estimate_E(pPoint_1, pPoint_2, iCount, Camera[0], Camera[1], Camera[2], &oReport_E, &oMem_Mgr);
	Ransac_Estimate_F(pPoint_1, pPoint_2, iCount, &oReport_F, &oMem_Mgr);
	//Disp_Report(oReport_E);
	
	//根据三份Report决定用哪个矩阵进行位姿估计
	Two_View_Geometry oTwo_View_Geo = { oReport_E, oReport_F,oReport_H };
	Determine_Confg(&oTwo_View_Geo, pPoint_1, pPoint_2, iCount, &pNew_Point_1, &pNew_Point_2);
	Estimate_Relative_Pose(oTwo_View_Geo, Camera, Camera, pNew_Point_1, pNew_Point_2, oTwo_View_Geo.num_inliers, &oMem_Mgr);
	
	Free_Report(oReport_H, &oMem_Mgr);
	Free_Report(oReport_E, &oMem_Mgr);
	Free_Report(oReport_F, &oMem_Mgr);
	free(pPoint_1);
	//如果是读入的点，此处要释放
	free(pPoint_2);
	//Free_Report(oReport_H, &oMem_Mgr);
	Disp_Mem(&oMatrix_Mem, 0);
	Free_Mem_Mgr(&oMem_Mgr);
}

void SVD_Test_1()
{//搞一个2000x9的矩阵
#define _T double
	const int w = 9, h = 2000;
	_T M[w * h];
	/* = { -1.11092001, 1.09082501, -1.00000000, 0.00000000, 0.00000000, 0.00000000, 0.34592309, -0.33966582, 0.31138433,
		 0.44694880, -1.33252455, -1.00000000, 0.00000000, 0.00000000, 0.00000000, -0.07166018, 0.21364628, 0.16033197,
		 1.61034985, 0.32395583, -1.00000000, 0.00000000, 0.00000000, 0.00000000, 3.06228775, 0.61604377, -1.90162886,
		 -0.94637863, -0.08225629, -1.00000000, 0.00000000, 0.00000000, 0.00000000, 1.35323869, 0.11761930, 1.42991256,
		 0.00000000, 0.00000000, 0.00000000, -1.11092001, 1.09082501, -1.00000000, -1.16227131, 1.14124742, -1.04622411,
		 0.00000000, 0.00000000, 0.00000000, 0.44694880, -1.33252455, -1.00000000, -0.45111539, 1.34494674, 1.00932230,
		 0.00000000, 0.00000000, 0.00000000, 1.61034985, 0.32395583, -1.00000000, 0.33483522, 0.06735917, -0.20792701,
		 0.00000000, 0.00000000, 0.00000000, -0.94637863, -0.08225629, -1.00000000, 0.23170076, 0.02013871, 0.24482882 };*/

		 //可以自动赋值，这是个不满秩的矩阵，能很好的测出问题
	for (int i = 0; i < w * h; i++)
		M[i] = i;

	SVD_Info oSVD;

	/*_T* U, * S, * Vt;
	int U_h, U_w, S_w, Vt_h, Vt_w;
	SVD_Allocate(h, w, &U, &S, &Vt, &U_h, &U_w, &S_w, &Vt_h, &Vt_w);*/
	//Disp_Mem(&oMatrix_Mem, 0);
	SVD_Alloc<_T>(h, w, &oSVD);

	//Disp(M, 8, 9, "M");
	int iResult;

	svd_3(M, oSVD, &iResult, 0.000001);
	//Disp((_T*)oSVD.U, oSVD.h_Min_U, oSVD.w_Min_U, "U");
	//Disp((_T*)oSVD.S, 1, oSVD.w_Min_S, "S");
	//Disp((_T*)oSVD.Vt, oSVD.h_Min_Vt, oSVD.w_Min_Vt,"Vt");
	

	//验算SVD结果
	Test_SVD(M, oSVD, &iResult, 0.000001);
	
	//基础行变换，可以看秩
	Elementary_Row_Operation_1(M, h, w, M, &iResult);
	//Disp((_T*)oSVD.U, oSVD.h_Min_U, oSVD.w_Min_U);
	Free_SVD(&oSVD);	//用完记得释放
	
	return;
#undef _T
}

void Gradient_Method_Test_1()
{//多元函数求极值最速下降法（梯度法）求解 min y = 2*x1^2 + x2 ^2 -2x1x2 - 4x1 +4
	//此乃二元二次多项式构成的函数，试用多项式结构看看
	Polynormial oPoly;
	Init_Polynormial(&oPoly, 2, 20);
	//不断加单项式
	Add_Poly_Term(&oPoly, 2, 2, 0);
	Add_Poly_Term(&oPoly, 1, 0, 2);
	Add_Poly_Term(&oPoly, -2, 1, 1);
	Add_Poly_Term(&oPoly, -4, 1, 0);
	Add_Poly_Term(&oPoly, 4, 0, 0);
	//Disp(oPoly);

	//所谓雅可比就是一阶导组成的向量
	Polynormial Jacob[2];	//因为二元函数，所以偏导数有两个：fx,fy
	int i, j;
	//求出各变量的一阶偏导
	for (i = 0; i < oPoly.m_iElem_Count; i++)
	{
		Init_Polynormial(&Jacob[i], 2, 20);
		Get_Derivation(&oPoly, i, &Jacob[i]);
		//Disp(Jacob[i]);
	}

	//再求各个变量的二阶导，即Hess矩阵
	Polynormial Hess[2 * 2];
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 2; j++)
		{
			Init_Polynormial(&Hess[i * 2 + j], 2, 20);
			Get_Derivation(&Jacob[i], j, &Hess[i * 2 + j]);
			//Disp(Hess[i * 2 + j]);
		}
	}

	//梯度法两大要素，1，求一阶导，（fx0,fx1,...,fxn)就是梯度，此处增长最快，也叫Jacob
	//要求min,就是反过来，即 -Jacob为最速下降方向
	//此时，要有个初值(x0,x1,...,xn), 此处没啥经验，用(0,0)作为初值
	// xk_1 = xk - t*xk		此处xk 与 xk_1为向量，形成一个迭代格

	//梯度法要素二，如何确定 t值？不能拍脑袋，一格娘炮两格扯蛋，要恰到好处
	//求 t也是个优化问题， 问 t取何值，使 min( f(xk - t* Delta_xk) ) 其中 Delta_xk 就是Jacob

	//df/dt 是一个数，一个函数值
	//根据泰勒展开，此处有一个通用公式
	// t = (J'J) /(J'* H *J)

	float xk_1[2], xk[2] = { 0,0 };	//初值
	float J[2], H[2 * 2], Temp_1[2], t;
	const float eps = (float)1e-5;
	int iIter;
	for (iIter = 0; iIter < 300; iIter++)
	{
		//代入xk, 求出J向量
		for (i = 0; i < 2; i++)
		{
			//Disp(Jacob[i]);
			J[i] = fGet_Polynormial_Value(Jacob[i], xk);
		}

		for (i = 0; i < 2; i++)
			for (j = 0; j < 2; j++)
				H[i * 2 + j] = fGet_Polynormial_Value(Hess[i * 2 + j], xk);

		Matrix_Multiply(J, 1, 2, J, 1, &t);
		Matrix_Multiply(J, 1, 2, H, 2, Temp_1);
		Matrix_Multiply(Temp_1, 1, 2, J, 1, Temp_1);
		t /= Temp_1[0];

		Matrix_Multiply(J, 1, 2, t, Temp_1);	//t*J
		Vector_Minus(xk, Temp_1, 2, xk_1);		//xk_1 = xk-t*J;
		Disp(xk, 1, 2, "xk");
		Disp(xk_1, 1, 2, "xk_1");

		//if (abs(xk[0]-xk_1[0])< eps && abs(xk[1] -xk_1[1])<eps)
			//break;
		//另一种收敛判断，如果梯度的模<eps
		if (fGet_Mod(J, 2) < eps)
			break;

		memcpy(xk, xk_1, 2 * sizeof(float));	//xk=xk_1
	}

	//释放
	Free_Polynormial(&oPoly);
	for (i = 0; i < 2; i++)
		Free_Polynormial(&Jacob[i]);
	for (i = 0; i < 2 * 2; i++)
		Free_Polynormial(&Hess[i]);
	return;
}
void Optimize_Newton_Test_2()
{//多元函数优化问题，牛顿法。目前还欠个求解的充分条件。否则发散了也不知哪里出了问题
 //此处求解的问题是 min(f(x)= x0^4 - 2*x0^2*x1 + x0^2 + x1^2 - 4x0 + 4
 //感觉此处已经基本上找到套路，但是还有个头疼的问题，初值如何确定？
 //先构造多项式
	Polynormial oPoly;
	const int iElem_Count = 2;  //变量数

	Init_Polynormial(&oPoly, iElem_Count, 20);
	Add_Poly_Term(&oPoly, 1, 4, 0);
	Add_Poly_Term(&oPoly, -2, 2, 1);
	Add_Poly_Term(&oPoly, 1, 2, 0);
	Add_Poly_Term(&oPoly, 1, 0, 2);
	Add_Poly_Term(&oPoly, -4, 1, 0);
	Add_Poly_Term(&oPoly, 4, 0, 0);
	Disp(oPoly);

	//牛顿法要素1，求一阶导Jacob
	Polynormial Jacob[iElem_Count]; //二元函数，有两个偏导数 fx0 fx1
	int i;
	for (i = 0; i < iElem_Count; i++)
	{
		Init_Polynormial(&Jacob[i], iElem_Count, 20);
		Get_Derivation(&oPoly, i, &Jacob[i]);
		Disp(Jacob[i]);
	}

	//牛顿法要素2，求二阶导Hess
	int j;
	Polynormial Hess[iElem_Count * iElem_Count];
	for (i = 0; i < iElem_Count; i++)
	{
		for (int j = 0; j < iElem_Count; j++)
		{
			Init_Polynormial(&Hess[i * iElem_Count + j], iElem_Count, 20);
			Get_Derivation(&Jacob[i], j, &Hess[i * iElem_Count + j]);
			Disp(Hess[i * iElem_Count + j]);
		}
	}

	//要素三，迭代格为 xk_1 = xk - H(-1)(xk) * J(xk), 与一元方程的迭代格何其相似 xk_1 = xk - f'(xk)/f''(xk)
	float H[iElem_Count * iElem_Count], J[iElem_Count], Temp_1[4];
	float xk[iElem_Count] = { 0,0 };
	int iIter, iResult;
	const float eps = (float)1e-5;
	for (iIter = 0; iIter < 300; iIter++)
	{
		for (i = 0; i < iElem_Count; i++)
			for (j = 0; j < iElem_Count; j++)
				H[i * iElem_Count + j] = fGet_Polynormial_Value(Hess[i * iElem_Count + j], xk);
		for (i = 0; i < iElem_Count; i++)
			J[i] = fGet_Polynormial_Value(Jacob[i], xk);

		//Disp(H, 2, 2);
		Get_Inv_Matrix_Row_Op(H, H, 2, &iResult);
		if (!iResult)
		{//不满秩
			printf("err");
			break;
		}
		Matrix_Multiply(H, 2, 2, J, 1, Temp_1);     //H(-1)(xk)* J(xk)
		Vector_Minus(xk, Temp_1, 2, Temp_1);
		if (fGet_Distance(xk, Temp_1, 2) < eps)
			break;
		memcpy(xk, Temp_1, iElem_Count * sizeof(float));
	}

	Free_Polynormial(&oPoly);
	for (i = 0; i < iElem_Count; i++)
		Free_Polynormial(&Jacob[i]);
	for (i = 0; i < iElem_Count * iElem_Count; i++)
		Free_Polynormial(&Hess[i]);
	return;
}
void Optimize_Newton_Test_1()
{//再次玩最优化问题，先搞一次函数最优化，看看否建立对一般形式的f(x)的求解形式
	//设函数为x^5 + x^3 -7x
	//牛顿法要素1，一阶导，二阶导要好求
	//第一步，要求出 f'(x)和 f''(x)的解析式: 
	//          f'(x) = 5x^4 + 3x^2 -7
	//          f''(x)= 20x^3 + 6x

	//牛顿法要素2，要大致确定驻点范围，利用 f'(x)=0 这个线索，正因为有这个条件使得求解x点成为可能
	//对于 f'(x)= 5x^4 + 3x^2 -7, 当 x=0 时 f'(x)=-7,
	//当 x=1时， f'(x) = 1. 又当  0<x<1时，f'(x)连续， 所以必有驻点
	//所以，可以挑[0,1]之间一点作为初点 x0, 就选 x0=0

	//牛顿法要素3, 迭代格为  x(k+1)= xk - f'(xk)/f''(xk) 故此 f''(x0)不能=0
	//x0=0 时， f'(x0)=f'(0)= -7 f'(x0)=f''(0)=0，显然不行，所以要改用x0=1
	//x1=1 时， f'(x0)=5+3-7=1 f''(x0)=20+6= 26, 这个初值可以
	typedef double _T;
#define eps 1e-10

	_T xk = 1.f - 1.f / 26;
	_T f1, f2;
	int iIter;
	for (iIter = 0; iIter < 30; iIter++)
	{
		//f'(x)=5x ^ 4 + 3x ^ 2 - 7
		f1 = 5 * pow(xk, 4) + 3 * xk * xk - 7;
		//f''(x)= 20x^3 + 6x
		f2 = 20 * pow(xk, 3) + 6 * xk;
		xk = xk - f1 / f2;
		if (abs(f1) < eps)
			break;
	}
	//还要检验一下 是否 f''(x)>0
	if (f1 <= 0)
		printf("Fail\n");
	else if (iIter < 30 || abs(f1) < eps)//x^5 + x^3 -7x
		printf("O了\n最优点在:%f, 最优解:%f\n", xk, pow(xk, 5) + pow(xk, 3) - 7 * xk);
	//总结，牛顿法快，但是要求多多，比如一阶二阶导是否好求，一阶导是否容易推出极值点范围
	//然而，这个优化方法并不是元问题，不断追问，它的基本套路事求解 f'(x)=0, 而这个本质上
	//是解方程问题，所以解方程才是优化问题的关键所在
#undef eps
}

void Sec_Method_Test()
{//两点截弦法求 f(x) = x^2 -2 =0
	//先确定x0,x1， x0=1时，f(x0)=-1; x1=1.5时，f(x1)=0.25。故此，初值有了
	float fPre_x = 1e10, x,	//与x轴得交点
		yk, xk = 0,		// 1,
		yk_1, xk_1 = 2;			// 1.5;	//可以视为x(k+1),y(k+1)
	int iIter;
	for (iIter = 0; iIter < 30; iIter++)
	{
		yk = xk * xk - 2;			//yk=f(xk)
		yk_1 = xk_1 * xk_1 - 2;		//yk_1 = f(xk_1)

		//过(xk,yk), (xk_1,yk_1)两点有一条直线，求其交点(x,y)
		//(y-yk)/(x-xk) = (yk_1-yk)/(xk_1-xk) , 倒数也一样成立
		//(x-xk)/(y-yk) = (xk_1-xk)/(yk_1-yk)	=>
		//x-xk = [(xk_1-xk)/(yk_1-yk)] * (y-yk) =>
		//x = xk + [(xk_1-xk)/(yk_1-yk)] * (y-yk)
		//再求这条直线与x轴的交点, 代入y=0 得到x
		x = xk + ((xk_1 - xk) / (yk_1 - yk)) * (0 - yk);

		//判断是否收敛, 若 |f(x)|<eps 则结束迭代 
		if (fPre_x == x)
			break;
		printf("iIter:%d f(x)=%f\n", iIter, x * x - 2);

		//然后淘汰 xk, 剩下xk_1, x作为下一截弦的两端点
		xk = xk_1;
		fPre_x = xk_1 = x;
	}
	printf("iIter:%d f(x)=%f\n", iIter, x * x - 2);
	return;
	//小结：条件：1，估出定义域(a,b)；2，f(x)在 (a,b)上连续
	//不需求导
}

void Non_Linear_Test()
{//一元非线性方程求解， 以 f(x)= x.e^0.5x -1 =0 在 [0,1]中的根
#define eps 0.00001f
	//f(0)= 0-1=-1	f(1)=1* e^0.5x-1>0 所以可以用对分法
	//先来个对半分法
	float x, y;
	float a = 0, b = 1, x_pre = -1;
	int i;
	for (i = 0;; i++)
	{
		x = (a + b) * 0.5f;
		y = x * (float)exp(x * 0.5f) - 1;
		if (x == x_pre)
			break;
		if (y < 0)
			a = x;
		else
			b = x;
		printf("i:%d x=%f\n", i, x);
		x_pre = x;
	}

	//一般迭代法，诸多限制。 要化成 x= phi(x) 且 phi(x)的一阶导<1
	//变形 x= 1/ e^(x/2) 
	x = x_pre = 0;
	for (i = 0;; i++)
	{
		x = 1.f / (float)exp(x / 2.f);
		if (x == x_pre)
			break;
		printf("i:%d x=%f\n", i, x);
		x_pre = x;
	}

	//尝试Aitken法加速
	float z, w;
	x = x_pre = 0;
	for (i = 0; i < 4; i++)
	{
		y = 1.f / (float)exp(x / 2.f);
		z = 1.f / (float)exp(y / 2.f);
		if (x == y || y == z)
			break;
		//这段神奇的代码尚欠推导，此处决定了迭代的加速
		w = z - (z - y) * (z - y) / ((z - y) - (y - x));
		x = w;
		printf("i:%d x=%f\n", i, x);
	}

	//牛顿法， 构造数列 xn_1= xn + f(xn)/f'(xn)， 要求f(x)在[a,b]上可导且不能有 f'(x)=0
	//牛顿法的根基是泰勒展开，这个推导过程要吃透
	x = 1;
	float fx, f1x;	//分别为f(x)与 f'(x)
	for (i = 0;; i++)
	{
		fx = x * (float)exp(x / 2.f) - 1;
		f1x = (float)exp(x / 2.f) + (x / 2.f) * (float)exp(x / 2.f);
		y = x - fx / f1x;
		if (y == x)
			break;
		x = y;
		printf("i:%d %f:\n", i, x);
	}

	float fy;	//以下 fx= f(xn) fy= f(y)
	//这个方法非常有效，只要求f(a)与f(b)异号，但是收敛阶只到1.618，也已经足够快
	x = 0; y = 1;
	for (i = 0;; i++)
	{
		fx = x *(float)exp(x / 2.f) - 1;
		fy = y * (float)exp(y / 2.f) - 1;
		if (fx == fy)
			break;	//此处可以加一个停机条件，否则下面可能出现分母为0
		z = (x * fy - y * fx) / (fy - fx);
		if (z == y)
			break;
		printf("i:%d x:%f\n", i, z);
		x = y;
		y = z;
	}
	return;
#undef eps
}

static void Least_Square_Test_1()
{//先搞个线性实验，拟合一条直线 f(x)= ax+b
	Image oImage;
	Line_1 oLine;
	int x, y, x1, y1, bResult;
	float A[1024][2], B[1024], ab[2], a, b;
	int m;	//一共有多少行数据

	Init_Image(&oImage, 1920, 1080, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);

	//构造一条曲线，从(100, 100)到(300, 400)，搞定后kx+m 相当于 ax+b
	Cal_Line(&oLine, 100, 200, 300, 400);
	for (m = 0, x = 100; x < 400; x += 5)
	{
		y = (int)(oLine.k * x + oLine.m);
		//还要加上(-3,3)之间的随机数
		x1 = x + iGet_Random_No() % 11 - 5;
		y1 = y + iGet_Random_No() % 11 - 5;

		Draw_Point(oImage, x1, y1, 2);
		A[m][0] = (float)x1;
		A[m][1] = 1;
		B[m] = (float)y1;
		m++;	//行数加1
	}
	Solve_Linear_Contradictory((float*)A, m, 2, B, ab, &bResult);
	a = ab[0], b = ab[1];
	x = 100, y = (int)(a * x + b);
	x1 = 400, y1 = (int)(a * (float)x1 + b);
	Mid_Point_Line(oImage, x, y, x1, y1, 255, 0, 0);
	bSave_Image("c:\\tmp\\1.bmp", oImage);
	return;
}
static void Least_Square_Test_2()
{//搞条复杂一点的  y= exp(ax^2 + bx +c)
	//先画一下 y= exp(2x^2+ 3x+4)
	//通过以上例子可以总结，某些曲线拟合关键在于能否通过一定的转换化成线性形式
	float x, y, x1, y1;
	int m, bResult;
	float A[1024][3], B[1024], abc[3], a, b, c;

	Image oImage;

	Init_Image(&oImage, 1920, 1080, Image::IMAGE_TYPE_BMP, 24);
	Set_Color(oImage);

	for (m = 0, x = 0; x < 0.5f; x += 0.01f)
	{
		y = 2.f * x * x + 3.f * x + 4.f;
		y = (float)exp(y);

		x1 = x + (iGet_Random_No() % 100) / 1000.f;
		y1 = y + iGet_Random_No() % 11 - 5;

		A[m][0] = x1 * x1;
		A[m][1] = x1;
		A[m][2] = 1;	//注意：常数项对应的系数恒为1
		B[m] = y1;

		//printf("y:%f\n", y);
		x1 *= 1800;
		if (x1 >= 0 && x1 < oImage.m_iWidth - 1 && y1 >= 0 && y1 <= oImage.m_iHeight - 1)
			Draw_Point(oImage, (int)x1, (int)y1, 2);
		m++;
	}

	//由于此模型为非线性。要用线性的方法来搞，必须变形为： ln(y)= ax^2 + bx +c
	//然后怎么解，还得搞搞
	for (y = 0; y < m; y++)
		B[(int)y] = (float)log(B[(int)y]);
	Solve_Linear_Contradictory((float*)A, m, 3, B, abc, &bResult);

	for (m = 0, x = 0; x < 0.5f; x += 0.01f)
	{
		a = abc[0], b = abc[1], c = abc[2];
		y = 1 * x * x + b * x + c;
		y = (float)exp(y);
		x1 = x * 1800.f;
		if (x1 >= 0 && x1 < oImage.m_iWidth - 1 && y >= 0 && y <= oImage.m_iHeight - 1)
			Draw_Point(oImage, (int)x1, (int)y, 2, 255, 0, 0);
		//printf("%f %f\n",y,B[m]);
		m++;
	}
	bSave_Image("c:\\tmp\\1.bmp", oImage);
	return;
}
static void Gradient_Method_Test_2()
{//搞搞求非线性极值
//先试一下 f(x,y)= x^2 + y^2
	//看看二阶导是否容易求
	// fx= 2*x,		fy=2*y	于是梯度为 -(2x,2y)
	float y, x, z;
	float fx, fy;	//一阶偏导
	float t, delta_x, delta_y;

	x = -800; y = 100;
	z = x * x + y * y;
	while (1)
	{
		fx = 2 * x;
		fy = 2 * y;
		//此时， (fx,fy)指明了前进的方向，但是没有指明前进的步长，步子迈得太大会扯到蛋

		//t就是表征顺着梯度方向走的步长 delta_x = t * fx, delta_y=t*fy
		//然而t 怎么算呢？就是满足 f(x+delta_x,y+delta_x)- f(x,y)达到最大
		//即 f(x+ t*fx, y + t*fy)-f(x,y) 达到最大
		//即可求 df/dt=0, 解此方程即可。所以有解的条件为 对 r的导数方程容易解，否则麻烦
		t = 0.5f;	//居然好解

		delta_x = -t * fx;
		delta_y = -t * fy;
		x += delta_x;
		y += delta_y;

		z = x * x + y * y;
	}
	//这个例子不好，一步到位
}
static void Optimize_Newton_Test_3()
{//牛顿法优化问题， 求解 min f(x1,x2)= x1^3 + x2^3 -3(x1+x2)
	float x1 = 6, x2 = 4;	//这个初值有一定限制，必须在一定范围内？怎么估？
	float fx1, fx2;	//一阶导
	float fx1x1, fx1x2, fx2x1, fx2x2;	//二阶导
	//迭代格式为 xk_1= xk - grad^2 f(xk)^(-1) * grad f(xk)

	while (1)
	{
		//首先求一阶导
	//fx1= 3x1^2-3 fx2=3x2^2-3 -> grad f(x)= 3(x1^2-1, x2^2-1)
		fx1 = 3 * x1 * x1 - 3, fx2 = 3 * x2 * x2 - 3;

		//再求二阶导
		//fx1x1= 3*2*x1=6x1
		fx1x1 = 6 * x1;
		//fx1x2=0	没有x2项
		fx1x2 = 0;
		//fx2x1 = 0;
		fx2x1 = fx1x2;		//求导顺序可换，Hesse矩阵必然堆成
		//fx2x2= 3*2*x2=6x2
		fx2x2 = 6 * x2;

		//直接用向量与矩阵表示
		float xk_1[2], xk[] = { x1,x2 };
		float grad_f[] = { fx1,fx2 };
		float grad2_f[] = { fx1x1,fx1x2,
							fx2x1,fx2x2 };

		//求 xk+1
		float Inv_grad2_f[4], Temp[2];
		int bResult;
		Get_Inv_Matrix_Row_Op(grad2_f, Inv_grad2_f, 2, &bResult);
		Matrix_Multiply(Inv_grad2_f, 2, 2, grad_f, 2, Temp);
		Vector_Minus(xk, Temp, 2, xk_1);

		Vector_Minus(xk_1, xk, 2, Temp);
		if (fGet_Mod(Temp, 2) == 0.f)
			break;

		x1 = xk_1[0], x2 = xk_1[1];	//xk=xk_1
		Disp(xk, 1, 2, "xk");	//可以看出，迅速收敛，牛逼！
	}

	return;
}

void Polynormial_Test()
{//构造多项式例子
	Polynormial oPoly;
	Init_Polynormial(&oPoly, 3, 20);
	//加入各项。如果缺变量，即xi^0=1，即其幂为0
	Add_Poly_Term(&oPoly, 0.5f, 1, 0, 3);
	Add_Poly_Term(&oPoly, 0.5f, 1, 0, 3);
	Add_Poly_Term(&oPoly, 0.5f, 1, 2, 3);
	Disp(oPoly);
	Get_Derivation(&oPoly, 0);	//多项式求导
	Disp(oPoly);
	Free_Polynormial(&oPoly);	//记得释放内存
	return;
}

void Test_Main()
{
	//4个Sift实验，各种接口场合
	Sift_Test_1();
	Sift_Test_2();
	Sift_Test_3();
	Sift_Test_4();
	
	SVD_Test_1();	//作为SVD分解实验还是不错的
	E_Test_2();		//这个例子非常简洁
	Ransac_Test();	//Ransac实验
	
	Camera_Param_Test();	//相机参数实验
}