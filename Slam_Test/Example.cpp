//�����ʱ�ļ����������Essential Matrix
#include "stdio.h"
#include "Image.h"
#include "Reconstruct.h"

static void E_Test_1()
{//��һ����������E������Ȼ�����е�ɢ����ҧ�����Ƶ�����
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
//Start:	//�����˴�������ѭ��Ѱ��һ����ʵĵ�
	//Disp((_T*)Sample, 8, 4,"Sample");

	//�ȸ������ڲΣ��Ȱ������������
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
	//�ٹ���һ�������� Rt, ��y����ת30��, ��ƽ�� 30,0
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
		//����Σ� P1 = (1/z) * K*T*P0
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

	//�˵����ˣ������ܷ���E
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

	//R,t������һ��
	_T I[4*4],New_Point[8][4];
	Gen_Homo_Matrix(R, t, Rt);
	Gen_I_Matrix(I, 4, 4);
	for (i = 0; i < 8; i++)
	{//�������������ǻ��ķ�Χ�ڳɹ����������·����֪ ����ڲ�ΪI�� ���1�����ΪI ���2�����ΪRt
		_T z1, z2;
		Triangulate_Point(Norm_Point_1[i], Norm_Point_2[i], I, Rt, New_Point[i]);
		Disp(New_Point[i], 1, 4,"Point_3D");

		//����  (1/z) * KP * P1
		Matrix_Multiply(I, 4, 4, New_Point[i], 1, Temp_1);
		z1 = Temp_1[2];
		Matrix_Multiply(Temp_1, 1, 4, (float)(1.f/z1),Temp_1);
		Disp(Norm_Point_1[i], 1, 2, "Norm_Point_1");
		Disp(Temp_1, 1, 3, "���1��ͶӰ");

		//���� (1/z) * KP * P2
		Matrix_Multiply(Rt, 4, 4, New_Point[i], 1, Temp_1);
		z2 = Temp_1[2];
		Matrix_Multiply(Temp_1, 1, 4, (float)(1.f / z1), Temp_1);
		Disp(Norm_Point_2[i], 1, 2, "Norm_Point_2");
		Disp(Temp_1, 1, 3, "���2��ͶӰ");

		//�ȿ� z * NP1
		Matrix_Multiply(Norm_Point_1[i], 1, 2, (float)z1, Temp_1);
		Disp(Temp_1, 1, 2, "z1 * NP1");

		//������ z * Rt(-1) * NP2 = P 
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

		//�˴��Ѿ�����һ���ǳ��ǳ����Ľ���� NP2 ��������� = z1/z2 * Rt * NP1
		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);
		Matrix_Multiply(Temp_1, 1, 4, (float)(1.f / z2),Temp_1);
		Disp(Temp_1, 1, 4,"z1/z2 * Rt * NP1");

		memcpy(Temp_1, Norm_Point_1[i], 2 * sizeof(_T));
		Temp_1[2] = 1;
		Matrix_Multiply(Temp_1, 1, 3, (float)z1, Temp_1);
		Disp(Temp_1, 1, 4, "z1 * NP1");
		
		//��ʱ����Ȼ�Ѿ��ָ�����ȣ���ôz1 * NP1 �Ѿ���3ά���꣬Ϊ����Rt��ˣ������ٻ�Ϊ�������
		Temp_1[3] = 1;
		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);
		Matrix_Multiply(Temp_1, 1, 4, (float)(1.f / z2), Temp_1);
		Disp(Temp_1, 1, 4, "z1/z2 * Rt * NP1");


		//ʵ��֤���� ���� x'= R(x-t) ������˳���ʺ����ģ�ͣ���Ȼ������λ�ƺ���ת��������
		//�������ģ�Ͷ�������ת��λ��
		//��Σ�R����ά��x,x'�Ƕ�ά��Ҳ����ֱ�Ӽ���

		//��ȷ�Ĺ�ϵ�� x' = (z1/z2) * Rt * NP1, ��ʱ��z1,z2�Ĳ���ز����٣���ΪҪ�ָ����
		//��z1,z2ֻ�����ǻ��Ժ���У��ʴ�Ҫ�����������������𲽻ָ��������
		memcpy(Temp_1, Norm_Point_1[i], 2 * sizeof(_T));
		Matrix_Multiply(Temp_1, 1, 2, (float)z1, Temp_1);	//��һ�λ�������� (x,y,1) * z1
		Temp_1[2] = z1, Temp_1[3] = 1;				//�ڶ��λ�������� (z1*x, z1*y, z1, 1)

		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);		//���� Rt * x
		Matrix_Multiply(Temp_1, 1, 4, (float)(1.f / z2), Temp_1);	//����ٳ��� z2, ��ʱ�ͷ������ǻ���ʽ��
		Disp(Temp_1, 1, 4, "z1/z2 * Rt * NP1");
	}

	_T V[4];
	Rotation_Matrix_2_Vector(R, V);

	//��ʼ�ٴθ��Ƕ�E���Ƶ���Ŀ������֤ x' = R(x-t)
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

	//����֤ x'*E*x=0���Ӵ˴���������ֵ�����ս����0.00x ������	
	for (i = 0; i < 8; i++)
	{
		Matrix_Multiply(Norm_Point_Homo_2[i], 1, 3, E, 3, Temp_1);
		Matrix_Multiply(Temp_1, 1, 3, Norm_Point_Homo_1[i], 1, Temp_1);
		printf("x\'*E*x= %f\n", Temp_1[0]);
	}

	//����֤ x'= R(x-t)
	for (i = 0; i < 8; i++)
	{
		Vector_Minus(Norm_Point_Homo_1[i], t, 3,Temp_1);
		Matrix_Multiply(R, 3, 3, Temp_1, 1, Temp_1);
	}
	return;
}

void Camera_Param_Test()
{//�������ʵ��
	typedef float _T;

	//����һ�㣬����������(0,0,100)
	_T Point_0[4] = { 0,0,100,1 };
	//ԭ�����(0,0,0)��
	_T Camera_0[4] = { 0, 0, 0, 1 };

	//�ȿ�����ƶ�,�Ƚ������y��ת60��, �����R ,���ҳ�T��������ת��λ��
	//��z������Ļ����Ϊ������y����ת������£���ת�Ƕ�Ϊ���൱��˳ʱ��
	_T Rotation_Vector[4] = { 0,1,0, -60 * PI / 180 },
		t[3] = { (_T)(Point_0[2] * sqrt(3.f)),0,0 };
	_T R[3 * 3];	
	_T Point_1[4];
	Rotation_Vector_2_Matrix(Rotation_Vector, R);
	Matrix_Multiply(R, 3, 3, Point_0, 1, Point_1);
	//Disp(Point_1, 1, 3, "P0 move to");
	Vector_Add(Point_1, t, 3, Point_1);
	//Disp(Point_1, 1, 3, "Point_0 move to");
		
	//����һ��������������뻭ͼ��������ʵ����Կ�����w2c��ʽ���1�����
	//��һ������֤����w2c����������
	_T c2w[4 * 4], w2c[4 * 4];
	Gen_Homo_Matrix(R, t, c2w);
	Get_Inv_Matrix_Row_Op(c2w, w2c, 4);
	Matrix_Multiply(w2c, 4, 4, Point_0, 1, Point_1);
	//Disp(Point_1, 1, 4, "Point_1");

	//���������̿���������
	_T Temp_1[4 * 4] = {}, Temp_2[4 * 4],Temp_3[4*4];
	Gen_Homo_Matrix(R, (_T*)NULL, Temp_1);
	Gen_Homo_Matrix((_T*)NULL, t, Temp_2);	
	Matrix_Multiply(Temp_2, 4, 4, Temp_1, 4, Temp_3);
	//Disp(Temp_3, 4, 4, "Rt");	//�ɼ���R��t����c2w

	Matrix_Multiply(Temp_1, 4, 4, Temp_2, 4, Temp_3);
	Disp(c2w, 4, 4, "c2w");
	//Disp(Temp_3, 4, 4, "tR");	//��t��R����c2w

	Get_Inv_Matrix_Row_Op(Temp_3, Temp_3,4);
	Matrix_Multiply(Temp_3, 4, 4, Point_0, 1, Point_1);
	//Disp(Point_1, 1, 4, "Point_1");
	
	Matrix_Multiply(w2c, 4, 4, Point_0, 1, Point_1);
	//Disp(Point_0, 1, 4, "x0");
	//Disp(Point_1, 1, 4, "x1");
	
	{//����һ����򵥵�ƽ��ͶӰ
		_T x0[2] = { Point_0[0],Point_0[1] },
			x1[2] = { Point_1[0],Point_1[1] };
		_T I[4 * 4],Point_3D[4];
		Gen_I_Matrix(I, 4, 4);
		
		Triangulate_Point(x0, x1, I, w2c, Point_3D);
		//Disp(Point_3D, 1, 3, "Point_3D");	//�����һĻ�����ˣ�ȷʵ�ָܻ�����������
	}
	
	{//�ڶ���ʵ�飬������ƽ��������Ƿ������ǻ���ԭ����
		_T I[4*4],K[3 * 3], KP_0[4 * 4],KP_1[4*4];
		_T x0[4], x1[4], Point_3D[4];

		//�˴�K�ǵ�����ƽ��
		Gen_Camera_Intrinsic(K, 200, 1, 1, 960, 540);
		Gen_Homo_Matrix(K, (_T*)NULL, KP_0);	//����������ڲι���һ����ξ���
		Gen_I_Matrix(I, 4, 4);
		Disp(I, 4, 4, "I");
		Matrix_Multiply(KP_0, 4, 4, I, 4, KP_0);
		Disp(KP_0, 4, 4, "KP");

		Matrix_Multiply(KP_0, 4, 4, Point_0,1, x0);
		//�ǵ�Ҫ���� 1/Z��������ͶӰ
		Matrix_Multiply(x0, 1, 2, 1.f / x0[2],x0);
		Disp(Point_0, 1, 3, "Point_0");
		Disp(x0, 1, 2,"x0");

		Gen_Homo_Matrix(K, (_T*)NULL, KP_1);
		Matrix_Multiply(KP_1, 4, 4, w2c, 4, KP_1);

		Matrix_Multiply(KP_1, 4, 4, Point_0, 1, x1);
		//�ǵ�Ҫ���� 1/Z��������ͶӰ
		Matrix_Multiply(x1, 1, 4, 1.f / x1[2], x1);
		Disp(x1, 1, 4,"x1");
		//���ˣ� x0,x1���Ѿ��� (1/z)* KP * P �Ľ��

		Triangulate_Point(x0, x1, KP_0, KP_1, Point_3D);
		Disp(Point_3D, 1, 3, "Point_3D");	//ʵ��̫�����ˣ���������Ļ����ָ��ɹ�
	}
	{//�ù�һ��ƽ��������������������ܷ����ǻ�
		_T focal = 1, cx = 0.5, cy = 0.5;
		_T K[3 * 3],KP_0[4*4],KP_1[4*4],I[4*4];
		_T x0[4], x1[4], Point_3D[4];
		Gen_Camera_Intrinsic(K, focal, 1, 1, cx, cy);
		Disp(K, 3, 3, "K");

		////��������ڲζԲ��ԣ��ǶԵ�
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
		Disp(Point_3D, 1, 3, "Point_3D");	//������Ͷ����һ��ƽ��Ҳ��
	}
	return;
}

void E_Test_2()
{//���E ������룬����һ��E, ����������һ��ƥ���NP1��NP2������һ�� NP2=(z1/z2) * Rt * NP1
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
{//Siftʵ�飬����һ�źڰ�ͼ�������е�������
	float(*pPoint)[2];
	int iCount;
	unsigned long long tStart = iGet_Tick_Count();
	Get_Sift_Feature("c:\\tmp\\Screen_Cut_A.bmp", &pPoint, &iCount, 0);
	printf("%lld\n", iGet_Tick_Count() - tStart);
	free(pPoint);
	return;
}
static void Sift_Test_2()
{//����ͼ��ƥ�䣬��������ƥ���λ��
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
{//����Ŀ¼�������˴���Mem_Mgr
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
{//����Ŀ¼���������ӿ�
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
	
	//��������Report�������ĸ��������λ�˹���
	Two_View_Geometry oTwo_View_Geo = { oReport_E, oReport_F,oReport_H };
	Determine_Confg(&oTwo_View_Geo, pPoint_1, pPoint_2, iCount, &pNew_Point_1, &pNew_Point_2);
	Estimate_Relative_Pose(oTwo_View_Geo, Camera, Camera, pNew_Point_1, pNew_Point_2, oTwo_View_Geo.num_inliers, &oMem_Mgr);
	
	Free_Report(oReport_H, &oMem_Mgr);
	Free_Report(oReport_E, &oMem_Mgr);
	Free_Report(oReport_F, &oMem_Mgr);
	free(pPoint_1);
	//����Ƕ���ĵ㣬�˴�Ҫ�ͷ�
	free(pPoint_2);
	//Free_Report(oReport_H, &oMem_Mgr);
	Disp_Mem(&oMatrix_Mem, 0);
	Free_Mem_Mgr(&oMem_Mgr);
}

void SVD_Test_1()
{//��һ��2000x9�ľ���
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

		 //�����Զ���ֵ�����Ǹ������ȵľ����ܺܺõĲ������
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
	

	//����SVD���
	Test_SVD(M, oSVD, &iResult, 0.000001);
	
	//�����б任�����Կ���
	Elementary_Row_Operation_1(M, h, w, M, &iResult);
	//Disp((_T*)oSVD.U, oSVD.h_Min_U, oSVD.w_Min_U);
	Free_SVD(&oSVD);	//����ǵ��ͷ�
	
	return;
#undef _T
}

void Gradient_Method_Test_1()
{//��Ԫ������ֵ�����½������ݶȷ������ min y = 2*x1^2 + x2 ^2 -2x1x2 - 4x1 +4
	//���˶�Ԫ���ζ���ʽ���ɵĺ��������ö���ʽ�ṹ����
	Polynormial oPoly;
	Init_Polynormial(&oPoly, 2, 20);
	//���ϼӵ���ʽ
	Add_Poly_Term(&oPoly, 2, 2, 0);
	Add_Poly_Term(&oPoly, 1, 0, 2);
	Add_Poly_Term(&oPoly, -2, 1, 1);
	Add_Poly_Term(&oPoly, -4, 1, 0);
	Add_Poly_Term(&oPoly, 4, 0, 0);
	//Disp(oPoly);

	//��ν�ſɱȾ���һ�׵���ɵ�����
	Polynormial Jacob[2];	//��Ϊ��Ԫ����������ƫ������������fx,fy
	int i, j;
	//�����������һ��ƫ��
	for (i = 0; i < oPoly.m_iElem_Count; i++)
	{
		Init_Polynormial(&Jacob[i], 2, 20);
		Get_Derivation(&oPoly, i, &Jacob[i]);
		//Disp(Jacob[i]);
	}

	//������������Ķ��׵�����Hess����
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

	//�ݶȷ�����Ҫ�أ�1����һ�׵�����fx0,fx1,...,fxn)�����ݶȣ��˴�������죬Ҳ��Jacob
	//Ҫ��min,���Ƿ��������� -JacobΪ�����½�����
	//��ʱ��Ҫ�и���ֵ(x0,x1,...,xn), �˴�ûɶ���飬��(0,0)��Ϊ��ֵ
	// xk_1 = xk - t*xk		�˴�xk �� xk_1Ϊ�������γ�һ��������

	//�ݶȷ�Ҫ�ض������ȷ�� tֵ���������Դ���һ���������񳶵���Ҫǡ���ô�
	//�� tҲ�Ǹ��Ż����⣬ �� tȡ��ֵ��ʹ min( f(xk - t* Delta_xk) ) ���� Delta_xk ����Jacob

	//df/dt ��һ������һ������ֵ
	//����̩��չ�����˴���һ��ͨ�ù�ʽ
	// t = (J'J) /(J'* H *J)

	float xk_1[2], xk[2] = { 0,0 };	//��ֵ
	float J[2], H[2 * 2], Temp_1[2], t;
	const float eps = (float)1e-5;
	int iIter;
	for (iIter = 0; iIter < 300; iIter++)
	{
		//����xk, ���J����
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
		//��һ�������жϣ�����ݶȵ�ģ<eps
		if (fGet_Mod(J, 2) < eps)
			break;

		memcpy(xk, xk_1, 2 * sizeof(float));	//xk=xk_1
	}

	//�ͷ�
	Free_Polynormial(&oPoly);
	for (i = 0; i < 2; i++)
		Free_Polynormial(&Jacob[i]);
	for (i = 0; i < 2 * 2; i++)
		Free_Polynormial(&Hess[i]);
	return;
}
void Optimize_Newton_Test_2()
{//��Ԫ�����Ż����⣬ţ�ٷ���Ŀǰ��Ƿ�����ĳ������������ɢ��Ҳ��֪�����������
 //�˴����������� min(f(x)= x0^4 - 2*x0^2*x1 + x0^2 + x1^2 - 4x0 + 4
 //�о��˴��Ѿ��������ҵ���·�����ǻ��и�ͷ�۵����⣬��ֵ���ȷ����
 //�ȹ������ʽ
	Polynormial oPoly;
	const int iElem_Count = 2;  //������

	Init_Polynormial(&oPoly, iElem_Count, 20);
	Add_Poly_Term(&oPoly, 1, 4, 0);
	Add_Poly_Term(&oPoly, -2, 2, 1);
	Add_Poly_Term(&oPoly, 1, 2, 0);
	Add_Poly_Term(&oPoly, 1, 0, 2);
	Add_Poly_Term(&oPoly, -4, 1, 0);
	Add_Poly_Term(&oPoly, 4, 0, 0);
	Disp(oPoly);

	//ţ�ٷ�Ҫ��1����һ�׵�Jacob
	Polynormial Jacob[iElem_Count]; //��Ԫ������������ƫ���� fx0 fx1
	int i;
	for (i = 0; i < iElem_Count; i++)
	{
		Init_Polynormial(&Jacob[i], iElem_Count, 20);
		Get_Derivation(&oPoly, i, &Jacob[i]);
		Disp(Jacob[i]);
	}

	//ţ�ٷ�Ҫ��2������׵�Hess
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

	//Ҫ������������Ϊ xk_1 = xk - H(-1)(xk) * J(xk), ��һԪ���̵ĵ������������ xk_1 = xk - f'(xk)/f''(xk)
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
		{//������
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
{//�ٴ������Ż����⣬�ȸ�һ�κ������Ż�������������һ����ʽ��f(x)�������ʽ
	//�躯��Ϊx^5 + x^3 -7x
	//ţ�ٷ�Ҫ��1��һ�׵������׵�Ҫ����
	//��һ����Ҫ��� f'(x)�� f''(x)�Ľ���ʽ: 
	//          f'(x) = 5x^4 + 3x^2 -7
	//          f''(x)= 20x^3 + 6x

	//ţ�ٷ�Ҫ��2��Ҫ����ȷ��פ�㷶Χ������ f'(x)=0 �������������Ϊ���������ʹ�����x���Ϊ����
	//���� f'(x)= 5x^4 + 3x^2 -7, �� x=0 ʱ f'(x)=-7,
	//�� x=1ʱ�� f'(x) = 1. �ֵ�  0<x<1ʱ��f'(x)������ ���Ա���פ��
	//���ԣ�������[0,1]֮��һ����Ϊ���� x0, ��ѡ x0=0

	//ţ�ٷ�Ҫ��3, ������Ϊ  x(k+1)= xk - f'(xk)/f''(xk) �ʴ� f''(x0)����=0
	//x0=0 ʱ�� f'(x0)=f'(0)= -7 f'(x0)=f''(0)=0����Ȼ���У�����Ҫ����x0=1
	//x1=1 ʱ�� f'(x0)=5+3-7=1 f''(x0)=20+6= 26, �����ֵ����
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
	//��Ҫ����һ�� �Ƿ� f''(x)>0
	if (f1 <= 0)
		printf("Fail\n");
	else if (iIter < 30 || abs(f1) < eps)//x^5 + x^3 -7x
		printf("O��\n���ŵ���:%f, ���Ž�:%f\n", xk, pow(xk, 5) + pow(xk, 3) - 7 * xk);
	//�ܽᣬţ�ٷ��죬����Ҫ���࣬����һ�׶��׵��Ƿ����һ�׵��Ƿ������Ƴ���ֵ�㷶Χ
	//Ȼ��������Ż�����������Ԫ���⣬����׷�ʣ����Ļ�����·����� f'(x)=0, �����������
	//�ǽⷽ�����⣬���Խⷽ�̲����Ż�����Ĺؼ�����
#undef eps
}

void Sec_Method_Test()
{//������ҷ��� f(x) = x^2 -2 =0
	//��ȷ��x0,x1�� x0=1ʱ��f(x0)=-1; x1=1.5ʱ��f(x1)=0.25���ʴˣ���ֵ����
	float fPre_x = 1e10, x,	//��x��ý���
		yk, xk = 0,		// 1,
		yk_1, xk_1 = 2;			// 1.5;	//������Ϊx(k+1),y(k+1)
	int iIter;
	for (iIter = 0; iIter < 30; iIter++)
	{
		yk = xk * xk - 2;			//yk=f(xk)
		yk_1 = xk_1 * xk_1 - 2;		//yk_1 = f(xk_1)

		//��(xk,yk), (xk_1,yk_1)������һ��ֱ�ߣ����佻��(x,y)
		//(y-yk)/(x-xk) = (yk_1-yk)/(xk_1-xk) , ����Ҳһ������
		//(x-xk)/(y-yk) = (xk_1-xk)/(yk_1-yk)	=>
		//x-xk = [(xk_1-xk)/(yk_1-yk)] * (y-yk) =>
		//x = xk + [(xk_1-xk)/(yk_1-yk)] * (y-yk)
		//��������ֱ����x��Ľ���, ����y=0 �õ�x
		x = xk + ((xk_1 - xk) / (yk_1 - yk)) * (0 - yk);

		//�ж��Ƿ�����, �� |f(x)|<eps ��������� 
		if (fPre_x == x)
			break;
		printf("iIter:%d f(x)=%f\n", iIter, x * x - 2);

		//Ȼ����̭ xk, ʣ��xk_1, x��Ϊ��һ���ҵ����˵�
		xk = xk_1;
		fPre_x = xk_1 = x;
	}
	printf("iIter:%d f(x)=%f\n", iIter, x * x - 2);
	return;
	//С�᣺������1������������(a,b)��2��f(x)�� (a,b)������
	//������
}

void Non_Linear_Test()
{//һԪ�����Է�����⣬ �� f(x)= x.e^0.5x -1 =0 �� [0,1]�еĸ�
#define eps 0.00001f
	//f(0)= 0-1=-1	f(1)=1* e^0.5x-1>0 ���Կ����öԷַ�
	//�������԰�ַ�
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

	//һ���������������ơ� Ҫ���� x= phi(x) �� phi(x)��һ�׵�<1
	//���� x= 1/ e^(x/2) 
	x = x_pre = 0;
	for (i = 0;; i++)
	{
		x = 1.f / (float)exp(x / 2.f);
		if (x == x_pre)
			break;
		printf("i:%d x=%f\n", i, x);
		x_pre = x;
	}

	//����Aitken������
	float z, w;
	x = x_pre = 0;
	for (i = 0; i < 4; i++)
	{
		y = 1.f / (float)exp(x / 2.f);
		z = 1.f / (float)exp(y / 2.f);
		if (x == y || y == z)
			break;
		//�������Ĵ�����Ƿ�Ƶ����˴������˵����ļ���
		w = z - (z - y) * (z - y) / ((z - y) - (y - x));
		x = w;
		printf("i:%d x=%f\n", i, x);
	}

	//ţ�ٷ��� �������� xn_1= xn + f(xn)/f'(xn)�� Ҫ��f(x)��[a,b]�Ͽɵ��Ҳ����� f'(x)=0
	//ţ�ٷ��ĸ�����̩��չ��������Ƶ�����Ҫ��͸
	x = 1;
	float fx, f1x;	//�ֱ�Ϊf(x)�� f'(x)
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

	float fy;	//���� fx= f(xn) fy= f(y)
	//��������ǳ���Ч��ֻҪ��f(a)��f(b)��ţ�����������ֻ��1.618��Ҳ�Ѿ��㹻��
	x = 0; y = 1;
	for (i = 0;; i++)
	{
		fx = x *(float)exp(x / 2.f) - 1;
		fy = y * (float)exp(y / 2.f) - 1;
		if (fx == fy)
			break;	//�˴����Լ�һ��ͣ������������������ܳ��ַ�ĸΪ0
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
{//�ȸ������ʵ�飬���һ��ֱ�� f(x)= ax+b
	Image oImage;
	Line_1 oLine;
	int x, y, x1, y1, bResult;
	float A[1024][2], B[1024], ab[2], a, b;
	int m;	//һ���ж���������

	Init_Image(&oImage, 1920, 1080, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);

	//����һ�����ߣ���(100, 100)��(300, 400)���㶨��kx+m �൱�� ax+b
	Cal_Line(&oLine, 100, 200, 300, 400);
	for (m = 0, x = 100; x < 400; x += 5)
	{
		y = (int)(oLine.k * x + oLine.m);
		//��Ҫ����(-3,3)֮��������
		x1 = x + iGet_Random_No() % 11 - 5;
		y1 = y + iGet_Random_No() % 11 - 5;

		Draw_Point(oImage, x1, y1, 2);
		A[m][0] = (float)x1;
		A[m][1] = 1;
		B[m] = (float)y1;
		m++;	//������1
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
{//��������һ���  y= exp(ax^2 + bx +c)
	//�Ȼ�һ�� y= exp(2x^2+ 3x+4)
	//ͨ���������ӿ����ܽᣬĳЩ������Ϲؼ������ܷ�ͨ��һ����ת������������ʽ
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
		A[m][2] = 1;	//ע�⣺�������Ӧ��ϵ����Ϊ1
		B[m] = y1;

		//printf("y:%f\n", y);
		x1 *= 1800;
		if (x1 >= 0 && x1 < oImage.m_iWidth - 1 && y1 >= 0 && y1 <= oImage.m_iHeight - 1)
			Draw_Point(oImage, (int)x1, (int)y1, 2);
		m++;
	}

	//���ڴ�ģ��Ϊ�����ԡ�Ҫ�����Եķ������㣬�������Ϊ�� ln(y)= ax^2 + bx +c
	//Ȼ����ô�⣬���ø��
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
{//���������Լ�ֵ
//����һ�� f(x,y)= x^2 + y^2
	//�������׵��Ƿ�������
	// fx= 2*x,		fy=2*y	�����ݶ�Ϊ -(2x,2y)
	float y, x, z;
	float fx, fy;	//һ��ƫ��
	float t, delta_x, delta_y;

	x = -800; y = 100;
	z = x * x + y * y;
	while (1)
	{
		fx = 2 * x;
		fy = 2 * y;
		//��ʱ�� (fx,fy)ָ����ǰ���ķ��򣬵���û��ָ��ǰ���Ĳ�������������̫��ᳶ����

		//t���Ǳ���˳���ݶȷ����ߵĲ��� delta_x = t * fx, delta_y=t*fy
		//Ȼ��t ��ô���أ��������� f(x+delta_x,y+delta_x)- f(x,y)�ﵽ���
		//�� f(x+ t*fx, y + t*fy)-f(x,y) �ﵽ���
		//������ df/dt=0, ��˷��̼��ɡ������н������Ϊ �� r�ĵ����������׽⣬�����鷳
		t = 0.5f;	//��Ȼ�ý�

		delta_x = -t * fx;
		delta_y = -t * fy;
		x += delta_x;
		y += delta_y;

		z = x * x + y * y;
	}
	//������Ӳ��ã�һ����λ
}
static void Optimize_Newton_Test_3()
{//ţ�ٷ��Ż����⣬ ��� min f(x1,x2)= x1^3 + x2^3 -3(x1+x2)
	float x1 = 6, x2 = 4;	//�����ֵ��һ�����ƣ�������һ����Χ�ڣ���ô����
	float fx1, fx2;	//һ�׵�
	float fx1x1, fx1x2, fx2x1, fx2x2;	//���׵�
	//������ʽΪ xk_1= xk - grad^2 f(xk)^(-1) * grad f(xk)

	while (1)
	{
		//������һ�׵�
	//fx1= 3x1^2-3 fx2=3x2^2-3 -> grad f(x)= 3(x1^2-1, x2^2-1)
		fx1 = 3 * x1 * x1 - 3, fx2 = 3 * x2 * x2 - 3;

		//������׵�
		//fx1x1= 3*2*x1=6x1
		fx1x1 = 6 * x1;
		//fx1x2=0	û��x2��
		fx1x2 = 0;
		//fx2x1 = 0;
		fx2x1 = fx1x2;		//��˳��ɻ���Hesse�����Ȼ�ѳ�
		//fx2x2= 3*2*x2=6x2
		fx2x2 = 6 * x2;

		//ֱ��������������ʾ
		float xk_1[2], xk[] = { x1,x2 };
		float grad_f[] = { fx1,fx2 };
		float grad2_f[] = { fx1x1,fx1x2,
							fx2x1,fx2x2 };

		//�� xk+1
		float Inv_grad2_f[4], Temp[2];
		int bResult;
		Get_Inv_Matrix_Row_Op(grad2_f, Inv_grad2_f, 2, &bResult);
		Matrix_Multiply(Inv_grad2_f, 2, 2, grad_f, 2, Temp);
		Vector_Minus(xk, Temp, 2, xk_1);

		Vector_Minus(xk_1, xk, 2, Temp);
		if (fGet_Mod(Temp, 2) == 0.f)
			break;

		x1 = xk_1[0], x2 = xk_1[1];	//xk=xk_1
		Disp(xk, 1, 2, "xk");	//���Կ�����Ѹ��������ţ�ƣ�
	}

	return;
}

void Polynormial_Test()
{//�������ʽ����
	Polynormial oPoly;
	Init_Polynormial(&oPoly, 3, 20);
	//���������ȱ��������xi^0=1��������Ϊ0
	Add_Poly_Term(&oPoly, 0.5f, 1, 0, 3);
	Add_Poly_Term(&oPoly, 0.5f, 1, 0, 3);
	Add_Poly_Term(&oPoly, 0.5f, 1, 2, 3);
	Disp(oPoly);
	Get_Derivation(&oPoly, 0);	//����ʽ��
	Disp(oPoly);
	Free_Polynormial(&oPoly);	//�ǵ��ͷ��ڴ�
	return;
}

void Test_Main()
{
	//4��Siftʵ�飬���ֽӿڳ���
	Sift_Test_1();
	Sift_Test_2();
	Sift_Test_3();
	Sift_Test_4();
	
	SVD_Test_1();	//��ΪSVD�ֽ�ʵ�黹�ǲ����
	E_Test_2();		//������ӷǳ����
	Ransac_Test();	//Ransacʵ��
	
	Camera_Param_Test();	//�������ʵ��
}