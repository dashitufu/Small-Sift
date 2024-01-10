//搞个临时文件，想搞明白Essential Matrix
#include "stdio.h"
#include "Image.h"
#include "Reconstruct.h"

void SB_Sample()
{
	Temp_Load_File(NULL, (double(**)[3])NULL, (double(**)[3])NULL, NULL);
	Temp_Load_File(NULL, (float(**)[3])NULL, (float(**)[3])NULL, NULL);

	Temp_Load_File_1(NULL, (double(**)[3])NULL, (double(**)[3])NULL, NULL);
	Temp_Load_File_1(NULL, (float(**)[3])NULL, (float(**)[3])NULL, NULL);
}
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
		Matrix_Multiply(Pos, 1, 2, Pos[2]!=0?1.f / Pos[2]:0, Screen_Pos_1[i]);
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
		Matrix_Multiply(Pos, 1, 2,Pos[2]!=0? 1.f/Pos[2]:0,Screen_Pos_2[i]);
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
		Matrix_Multiply(Temp_1, 1, 4, 1.f/z1,Temp_1);
		Disp(Norm_Point_1[i], 1, 2, "Norm_Point_1");
		Disp(Temp_1, 1, 3, "相机1的投影");

		//验算 (1/z) * KP * P2
		Matrix_Multiply(Rt, 4, 4, New_Point[i], 1, Temp_1);
		z2 = Temp_1[2];
		Matrix_Multiply(Temp_1, 1, 4, 1.f / z1, Temp_1);
		Disp(Norm_Point_2[i], 1, 2, "Norm_Point_2");
		Disp(Temp_1, 1, 3, "相机2的投影");

		//先看 z * NP1
		Matrix_Multiply(Norm_Point_1[i], 1, 2, z1, Temp_1);
		Disp(Temp_1, 1, 2, "z1 * NP1");

		//再验算 z * Rt(-1) * NP2 = P 
		int iResult;
		_T Rt_1[4 * 4],Temp_2[4];

		Get_Inv_Matrix_Row_Op(Rt, Rt_1, 4, &iResult);
		Matrix_Multiply(Rt, 4, 4, New_Point[i], 1, Temp_1);
		Disp(Temp_1, 1, 4, "Rt * P");
		
		memcpy(Temp_2, Norm_Point_2[i], 2 * sizeof(_T));
		Temp_2[2] = 1.f;
		Matrix_Multiply(Temp_2, 1, 3, z2, Temp_1);
		Disp(Temp_1, 1, 3, "z2 * NP2");

		Temp_1[3] = 1;
		Matrix_Multiply(Rt_1, 4, 4, Temp_1, 1, Temp_1);
		Disp(Temp_1, 1, 4, " z2 * Rt(-1) * NP2");
//
		memcpy(Temp_2, Norm_Point_1[i], 2 * sizeof(_T));
		Temp_2[2] = 1.f;
		Matrix_Multiply(Temp_2, 1, 3, z1, Temp_1);
		Temp_1[3] = 1.f;
		Disp(Temp_1, 1, 4, "z1 * NP1");

		//此处已经有了一个非常非常棒的结果， NP2 的齐次坐标 = z1/z2 * Rt * NP1
		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);
		Matrix_Multiply(Temp_1, 1, 4, 1.f / z2,Temp_1);
		Disp(Temp_1, 1, 4,"z1/z2 * Rt * NP1");

		memcpy(Temp_1, Norm_Point_1[i], 2 * sizeof(_T));
		Temp_1[2] = 1;
		Matrix_Multiply(Temp_1, 1, 3, z1, Temp_1);
		Disp(Temp_1, 1, 4, "z1 * NP1");
		
		//此时，既然已经恢复了深度，那么z1 * NP1 已经是3维坐标，为了与Rt相乘，必须再化为齐次坐标
		Temp_1[3] = 1;
		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);
		Matrix_Multiply(Temp_1, 1, 4, 1.f / z2, Temp_1);
		Disp(Temp_1, 1, 4, "z1/z2 * Rt * NP1");


		//实践证明， 首先 x'= R(x-t) 的运算顺序不适合这个模型，显然这是先位移后旋转。而我们
		//整个相机模型都是先旋转后位移
		//齐次，R是三维，x,x'是二维，也不能直接计算

		//正确的关系是 x' = (z1/z2) * Rt * NP1, 此时，z1,z2的参与必不可少，因为要恢复齐次
		//而z1,z2只有三角化以后才有，故此要验算这个结果，必须逐步恢复齐次坐标
		memcpy(Temp_1, Norm_Point_1[i], 2 * sizeof(_T));
		Matrix_Multiply(Temp_1, 1, 2, z1, Temp_1);	//第一次化齐次坐标 (x,y,1) * z1
		Temp_1[2] = z1, Temp_1[3] = 1;				//第二次化齐次坐标 (z1*x, z1*y, z1, 1)

		Matrix_Multiply(Rt, 4, 4, Temp_1, 1, Temp_1);		//再算 Rt * x
		Matrix_Multiply(Temp_1, 1, 4, 1.f / z2, Temp_1);	//最后再除以 z2, 此时就符合三角化公式了
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
void Camera_Extrinsic_Test_2()
{
	float P_0[4] = { 0,0,1000,1 } , P_1[4];
	{//第一个实验，不考虑相机，将点先绕原点转30度，再向右移动100
		//左手系
		float Rotation_Vector[4] = { 0,1,0,PI / 6.f }, t[3] = { 100,0,0 };
		float R[3 * 3], T[4 * 4],P_1[4];
		Rotation_Vector_2_Matrix(Rotation_Vector, R);
		Gen_Homo_Matrix(R, t, T);
		
		//再算 P1=T*P0
		Matrix_Multiply(T, 4, 4, P_0, 1, P_1);
		Disp(P_1, 1, 4, "P1");
	}

	{//第二个实验，以相机得观点看待问题，先将相机与第一个实验反方向旋转30度，然后还是
		//按照世界坐标向左移动100，看其位置
		float Rotation_Vector[4] = { 0,1,0,-PI / 6.f }, t[3] = { -100,0,0};
		float R[3 * 3], T[4 * 4], P_1[4],w2c[4*4];
		Rotation_Vector_2_Matrix(Rotation_Vector, R);
		Gen_Homo_Matrix(R, t, T);
				
		Get_Inv_Matrix_Row_Op(T, w2c, 4);
		Matrix_Multiply(w2c, 4, 4, P_0, 1, P_1);
		Disp(P_1, 1, 4, "P_1");
		//可见，实验二与实验一的结果不一样，只能按照各自的观点进行计算
	}

	{//第二个实验，用李代数		
		float Ksi[6];
		float Rotation_Vector[4] = { 0,1,0,-PI / 6.f }, t[3] = { -100,0,0 };
		float c2w[4 * 4],w2c[4*4];

		//必须用这个函数根据t,φ求i出ξ
		Gen_Ksi_by_Rotation_Vector_t(Rotation_Vector, t, Ksi, 4);

		se3_2_SE3(Ksi, c2w);	//此方法也没啥营养，只是将ksi转换为c2w

		Matrix_Multiply(c2w, 4, 4, P_0, 1, P_1);
		Get_Inv_Matrix_Row_Op(c2w, w2c, 4);
		Matrix_Multiply(w2c, 4, 4, P_0, 1, P_1);
		Disp(P_1, 1, 4, "P_1");
		//可见，实验三与实验二等价，特别注意ρ和t的区别，不能直接用t
	}

	return;
}
void Camera_Extrinsic_Test_1()
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
		
	//再试一下相机参数，必须画图，从以下实验可以看出，w2c正是相机1的外参
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
void Camera_Intrinsic_Test()
{//相机内参实验，假定位针孔相机，假定焦距与物体坐标用同一距离单位，成正像
	{//第一个实验，搞明白相机的焦距变化对成像平面上的坐标（u,v)的影响
		float X = 100, Y = 200, Z = 1000;    //点在1000之外
		float f = 10;      //假定焦距为10
		float u, v;
		//x/x' = z/f 求 x' => x'=x*f/z
		u = X * f / Z;
		v = Y * f / Z;
		//z' =z*f/z=f
		printf("投影平面上的坐标为：u:%f v:%f z:%f\n", u, v, f);

		//换个焦距，看看成像平面上的坐标变化
		f = 20;             //假定焦距为20
		u = X * f / Z;
		v = Y * f / Z;
		printf("投影平面上的坐标为：u:%f v:%f z:%f\n", u, v, f);
		//结论，焦距越大，投影坐标越大

		//还有个要点。对于(u,v,f)= (x,y,z) * (f/z) 中，x,y,z可以取任意正值，世界无限大
		//(u,v)作为值域也可以取任意值，即成像平面可以无限大
	}

	{//第二个实验，不同的焦距投影到归一化平面上的坐标
		float X = 100, Y = 200, Z = 1000;    //点在1000之外
		float f = 10;      //假定焦距为10
		float u, v;

		//第一次变化，到成像平面
		u = X * f / Z;
		v = Y * f / Z;
		//第二次变化，再投影到归一化平面，即投影到光心距离为1的平面上
		u /= f;
		v /= f;
		printf("focal:%f u:%f v:%f\n", f, u, v);

		//改变焦距，看看归一化平面上的投影变化
		f = 20;      //假定焦距为10
		//第一次变化，到成像平面
		u = X * f / Z;
		v = Y * f / Z;
		//第二次变化，再投影到归一化平面，即投影到光心距离为1的平面上
		u /= f;
		v /= f;
		printf("focal:%f u:%f v:%f\n", f, u, v);
		//结论1，可见，无论焦距多少，投影到归一化平面上的坐标不变


		//假定一步到位，相机的焦距为1
		f = 1;      //假定焦距为10
		u = X * f / Z;
		v = Y * f / Z;
		printf("focal:%f u:%f v:%f\n", f, u, v);
		//结论2，相机在归一化平面上的投影可以归结为不必考虑焦距，即焦距可以省略或者
		//焦距假定为1
	}

	{//第三个实验，到像素平面的投影。可以理解为成像平面与像素平面之间存在一个缩放。
		//我的理解是成像平面每单位对应多少像素
		float X = 100, Y = 200, Z = 1000;    //点在1000之外
		float f = 10;      //假定焦距为10
		float u, v;
		float s = 20;            //成像平面每单位s个像素

		u = X * f / Z;
		v = Y * f / Z;
		u *= s;
		v *= s;
		printf("像素平面上u:%f v:%f\n", u, v);

		f = 20;      //假定焦距为10
		u = X * f / Z;
		v = Y * f / Z;
		u *= s;
		v *= s;
		printf("像素平面上u:%f v:%f\n", u, v);
		//结论，从成像平面到像素平面的投影，会随焦距的变化而变化
	}

	{//第4个实验，归一化坐标与像素平面坐标的关系
		float X = 100, Y = 200, Z = 1000;    //点在1000之外
		float f = 20;      //假定焦距为10
		float u, v;
		float s = 20;            //成像平面每单位s个像素

		u = X * f / Z;
		v = Y * f / Z;
		printf("归一化坐标：u:%f v:%f\n", u / f, v / f);
		u *= s;
		v *= s;
		printf("像素平面上u:%f v:%f\n", u, v);
		printf("从像素平面直接推归一化坐标：u:%f v:%f\n", u / (f * s), v / (f * s));
		//此时，忽略位移，f*x就是最简相机内参
		//结论，通过相机内参可以一步换算为归一化坐标
	}

	{//第5个实验，已知像素平面上的坐标，还原点的空间位置
		float u = 40, v = 80;
		float Z = 1000;
		float fxy = 400;    //相机参数， fx=fy=400

		float u1, v1;   //归一化平面坐标
		u1 = u / fxy;
		v1 = v / fxy;
		printf("归一化平面坐标：u:%f v:%f\n", u1, v1);
		printf("空间坐标：X:%f Y:%f Z:%f\n", u1 * Z, v1 * Z, Z);
		//此处已经提供了相机内参还原空间坐标的线索
	}
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

	//for (int i = 0; i < iMatch_Count; i++)
		//printf("%f %f %f %f\n", pPoint_1[i][0], pPoint_1[i][1], pPoint_2[i][0], pPoint_2[i][1]);
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

void Least_Square_Test_3()
{//搞个二维曲面拟合实验， z= a (x/w)^2 + b(y/h)^2 搞一组添加了随机噪声的样本，用这些样本来做实验
//至此，由样本拟合参数的牛顿法OK了，但是梯度法还没行
	const int w = 10, h = 10;
	const int iSample_Count = w * h;
	const float eps = (float)1e-6;
	float* pCur_Point, (*pPoint_3D)[3] = (float(*)[3])malloc(iSample_Count * 3 * sizeof(float));
	float a0 = 4, b0 = 5,
		xa, xb;  //待求参数

	//造样本集
	int y, x;
	for (y = 0; y < h; y++)
	{
		for (x = 0; x < w; x++)
		{
			pCur_Point = pPoint_3D[y * w + x];
			pCur_Point[0] = (float)x;
			pCur_Point[1] = (float)y;
			pCur_Point[2] = a0 * (float)pow((float)x / w, 2) + b0 * (float)pow((float)y / h, 2);
			//printf("x:%f y:%f z:%f\n", pCur_Point[0], pCur_Point[1], pCur_Point[2]);
			pCur_Point[2] += (float)pow(-1, y * w + x) * iGet_Random_No_cv(0, 10) / 50.f;
			//printf("x:%f y:%f z1:%f\n", pCur_Point[0], pCur_Point[1], pCur_Point[2]);
		}
	}
	//存个盘看图像
	//bSave_PLY("c:\\tmp\\1.ply", pPoint_3D, iSample_Count, 1);

	//目标是求 min(yi - fi(x))^2 这个是最小二乘的一般形式，yi好办，每个样本的函数值，问题是， xk在此处对应的是什么？
	//显然我们要求的是a,b，所以， f(x)不再视为关于向量x的函数，而是视为 关于a,b的函数，所以
	//此处，我们讲a,b改为 xa,xb，这样可能好看些。故此， xk就是(xa,xb)在迭代过程中的一个取值，
	//目标是建立迭代格 x=(xa,xb)= (J'J)(-1) * J'*e + xk
	//先对fi(x)在xk处进行一阶泰勒展开 = fi(xk) + J(xk)(x-xk) 二元为 fi(x,y)= fi(xk,yk) + f'xk(xk,yk)*(x-xk) + f'yk(xk,yk)(y-yk)
	//于是，原来求解问题变成 min Sigma[ (yi-fi(x))^2 ] , x=(xa,xb)
	//泰勒展开 = min Sigma [ yi- fi(xk) - J(xk)(x-xk)]^2 , 等于只将yi-fi(x)展开
	//将前面两项合并为一项 ei = yi - fi(xk) 则上式=
	// min Sigma [ (ei - J(xk)(x-xk))^2] , 驻点在一阶导等于0处
	//即求解 ei - J(xk)(x-xk) = 0 矛盾方程组，求x, 此时，将J视为fi(x)关于x的一阶偏导矩阵，
	// J (x-xk)=ei => J'J (x-xk) = J'ei 
	// (J'J)(-1)(J'J) (x-xk) = (J'J)(-1) * J' ei
	// x-xk=  (J'J)(-1) * J' ei
	//x = (J'J)(-1) * J' ei + xk

	xa = 1, xb = 1;               //xk初值
	float J[iSample_Count][2];  //Jacob
	float Jt[2][iSample_Count]; //J'
	float e[iSample_Count];
	float Temp[iSample_Count * 2];  //搞个足够大的临时空间
	int i, iResult;
	while (1)
	{
		for (i = 0; i < iSample_Count; i++)
		{
			pCur_Point = pPoint_3D[i];
			//逐个求一阶偏导 f(a,b)= a (x/w)^2 + b(y/h)^2
			//fi'a (对a求偏导)  = (x/w)^2
			J[i][0] = (float)pow(pCur_Point[0] / w, 2);
			//fi'b (对b求偏导) = (y/h)^2
			J[i][1] = (float)pow(pCur_Point[1] / h, 2);

			//ei = yi - fi(xk) 要特别留意此处，fi(xk)作为一个整体，要加括号，否则发散
			e[i] = pCur_Point[2] - (xa * (float)pow(pCur_Point[0] / w, 2) + xb * (float)pow(pCur_Point[1] / h, 2));
		}

		Matrix_Transpose((float*)J, iSample_Count, 2, (float*)Jt);          //= J' 2*n
		Matrix_Multiply((float*)Jt, 2, iSample_Count, (float*)J, 2, Temp);  //=J'J 2x2
		Get_Inv_Matrix_Row_Op(Temp, Temp, 2, &iResult);                     //(J'J)(-1) 逆矩阵 2x2
		Matrix_Multiply(Temp, 2, 2, (float*)Jt, iSample_Count, Temp);        //=(J'J)(-1)*J' 2xn
		Matrix_Multiply(Temp, 2, iSample_Count, e, 1, Temp);                //=(J'J)(-1)*J'*e 2x1

		xa = Temp[0] + xa;
		xb = Temp[1] + xb;
		if (abs(Temp[0]) < eps && abs(Temp[1]) < eps)
			break;
		printf("%f %f\n", Temp[0], Temp[1]);
	}
	return;
}
void Least_Square_Test_4()
{//重做一组最小二乘法拟合曲面 z=3*x^2 + 4*y^2, 增加一点噪声，一共拟合2个参数
//第一个实验，一阶梯度法， 迭代格为 x(k+1) = xk + Δx 而 Δx= -tJ
	//第一步，先搞样本
	float xyzs[100][4]; //x, y, f(x), sample
	const float a = 3.f, b = 4.f;
	int y, x, i;

	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (float)x;
			xyzs[i][1] = (float)y;
			xyzs[i][2] = a * x * x + b * y * y;
			xyzs[i][3] = xyzs[i][2] + ((iGet_Random_No() % 100) - 50.f) / 100.f;
		}
	}

	float C[2] = { 1,1 }, Temp[2 * 2];
	float t, fPart_1, fPart_2; //-2(yi - f(xi))
	int iIter = 0;

	//F(x) = [yi - f(xi)]^2，从参数的观点看，F(x)= [yi - fi(C)]^2
	for (iIter = 0;; iIter++)
	{//注意，此处求解的2可以省略
		float J[2] = { 0 }, H[2][2] = { 0 };
		for (i = 0; i < 100; i++)
		{   //F(x) = ∑ [yi - (c1 * x ^ 2 + c2 * y ^ 2)]^2
			//先求J向量 ∂F/∂cj = -2∑[yi - fi(C)]*∂fi/∂cj
			fPart_1 = -2 * (xyzs[i][3] - (C[0] * xyzs[i][0] * xyzs[i][0] + C[1] * xyzs[i][1] * xyzs[i][1]));
			J[0] += fPart_1 * xyzs[i][0] * xyzs[i][0];  //∂fi/∂c=x^2
			J[1] += fPart_1 * xyzs[i][1] * xyzs[i][1];  //∂fi/∂c=y^2

			//∂F/∂c1 = -2∑[yi - fi(C)]* x^2
			//∂^2F/∂c1c1= -2∑ x^2 * (-∂fi/∂c0)= 2∑x^2* ∂fi/∂c1 = 2∑x^2* x^2 = 2∑x^4
			H[0][0] += 2.f * (float)pow(xyzs[i][0], 4);
			//∂^2F/∂c1c2=2∑x^2* ∂fi/∂c2= 2∑x^2*y^2;
			H[0][1] += 2 * xyzs[i][0] * xyzs[i][0] * xyzs[i][1] * xyzs[i][1];
			//∂F/∂c2=-2∑[yi - fi(C)]*y^2
			//∂F/∂c2c1=-2∑y^2*(-∂fi/∂c1) = -2∑y^2*(-x^2)=-2∑y^2*x^2;
			H[1][0] += 2 * xyzs[i][1] * xyzs[i][1] * xyzs[i][0] * xyzs[i][0];
			//∂F/∂c2c2= -2∑y^2*(-∂fi/∂c2)=2∑y^2 * y^2
			H[1][1] += 2.f * (float)pow(xyzs[i][1], 4);
		}

		////再求个t= (J'J) /(J'* H *J)
		//t = fDot(J, J, 2);
		//Matrix_Multiply(J, 1, 2, (float*)H, 2, Temp);
		//Matrix_Multiply(Temp, 1, 2, J, 1, Temp);
		//t /= Temp[0];

		//以上方法要求二阶导，感觉不是一阶梯度法，以下用另外一种方式求解步长t
		//t 满足 F(C + ΔC)达到最小 F(x) =∑[yi - fi(C-tJ)]^2 
		//dF/dt = 2∑[yi - fi(C-tJ)]*df/dt= 2∑[yi - ((c1-J1t) *x^2 + (c2-J2t)*y^2))]*df/dt
		//f =(c1-J1t) *x^2 + (c2-J2t)*y^2 = (c1*x^2 + c2*y2^2) - (J1*x^2+J2*y^2)*t
		//df/dt = -(J1*x^2+J2*y^2)
		//dF/dt = -2∑[yi - ((c1*x^2 + c2*y2^2) - (J1*x^2+J2*y^2)*t)]*(J1*x^2+J2*y^2) =0
		// dF/dt= -2∑ (yi - ((c1*x^2 + c2*y2^2))*(J1*x^2+J2*y^2) - (J1*x^2+J2*y^2)*(J1*x^2+J2*y^2)t=0
		//t= ∑ (yi - ((c1*x^2 + c2*y2^2))*(J1*x^2+J2*y^2)/∑(J1*x^2+J2*y^2)*(J1*x^2+J2*y^2)

		fPart_1 = fPart_2 = 0;
		for (i = 0; i < 100; i++)
		{
			float fTemp = J[0] * xyzs[i][0] * xyzs[i][0] + J[1] * xyzs[i][1] * xyzs[i][1];;
			fPart_1 += fTemp * (C[0] * xyzs[i][0] * xyzs[i][0] + C[1] * xyzs[i][1] * xyzs[i][1] - xyzs[i][3]);
			fPart_2 += fTemp * fTemp;
		}
		t = fPart_1 / fPart_2;
		//以上两种方法的收敛速度一样，都是6次，故此可以看作同一级别的计算。所以，以上
		//计算方法有可能是同一算法，要展开才知道

		//ΔC = -tJ
		Matrix_Multiply(J, 1, 2, -t, Temp);
		//C(k + 1) = Ck + ΔC
		Vector_Add(C, Temp, 2, C);

		if (fGet_Mod(Temp, 2) < 0.00001f)
			break;
	}

	Disp(Temp, 1, 2);
	return;
}
void Least_Square_Test_5()
{//二阶梯度法，看看能否收敛快点
	//第一步，先搞样本
	float xyzs[100][4]; //x, y, f(x), sample
	const float a = 3.f, b = 4.f;
	int y, x, i;

	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (float)x;
			xyzs[i][1] = (float)y;
			xyzs[i][2] = a * x * x + b * y * y;
			xyzs[i][3] = xyzs[i][2] + ((iGet_Random_No() % 100) - 50.f) / 100.f;
		}
	}

	float C[2] = { 1,1 }, Temp[2 * 2];
	float fPart_1; //-2(yi - f(xi))
	int iIter = 0, iResult;

	//F(x) = [yi - f(xi)]^2，从参数的观点看，F(x)= [yi - fi(C)]^2
	for (iIter = 0;; iIter++)
	{//注意，此处求解的2可以省略
		float J[2] = { 0 }, H[2][2] = { 0 };
		for (i = 0; i < 100; i++)
		{   //F(x) = ∑ [yi - (c1 * x ^ 2 + c2 * y ^ 2)]^2
			//先求J向量 ∂F/∂cj = -2∑[yi - fi(C)]*∂fi/∂cj
			fPart_1 = -2 * (xyzs[i][3] - (C[0] * xyzs[i][0] * xyzs[i][0] + C[1] * xyzs[i][1] * xyzs[i][1]));
			J[0] += fPart_1 * xyzs[i][0] * xyzs[i][0];  //∂fi/∂c=x^2
			J[1] += fPart_1 * xyzs[i][1] * xyzs[i][1];  //∂fi/∂c=y^2

			//∂F/∂c1 = -2∑[yi - fi(C)]* x^2
			//∂^2F/∂c1c1= -2∑ x^2 * (-∂fi/∂c0)= 2∑x^2* ∂fi/∂c1 = 2∑x^2* x^2 = 2∑x^4
			H[0][0] += 2.f * (float)pow(xyzs[i][0], 4);
			//∂^2F/∂c1c2=2∑x^2* ∂fi/∂c2= 2∑x^2*y^2;
			H[0][1] += 2 * xyzs[i][0] * xyzs[i][0] * xyzs[i][1] * xyzs[i][1];
			//∂F/∂c2=-2∑[yi - fi(C)]*y^2
			//∂F/∂c2c1=-2∑y^2*(-∂fi/∂c1) = -2∑y^2*(-x^2)=-2∑y^2*x^2;
			H[1][0] += 2 * xyzs[i][1] * xyzs[i][1] * xyzs[i][0] * xyzs[i][0];
			//∂F/∂c2c2= -2∑y^2*(-∂fi/∂c2)=2∑y^2 * y^2
			H[1][1] += 2.f * (float)pow(xyzs[i][1], 4);
		}

		//最主要的差别在于ΔC = -H(-1) * J
		Get_Inv_Matrix_Row_Op((float*)H, (float*)H, 2, &iResult);
		if (!iResult)
			printf("Fail");
		Matrix_Multiply((float*)H, 2, 2, J, 1, Temp);
		Matrix_Multiply(Temp, 1, 2, -1.f, Temp);

		//C(k + 1) = Ck + ΔC
		Vector_Add(C, Temp, 2, C);
		if (fGet_Mod(Temp, 2) < 0.00001f)
			break;
	}
	//结论：收敛快
	Disp(Temp, 1, 2);
	return;
}

void Least_Square_Test_6()
{//第三种方法，拟合曲面 z=3*x^2 + 4*y^2，希望不必求解二阶导，所谓高斯牛顿法
//此处已经不是求F(x)的负梯度再迈一个步长这种观念了，而是直接干ΔC，走一个ΔC，使得下降最快
	float xyzs[100][4]; //x, y, f(x), sample
	const float a = 3.f, b = 4.f;
	int y, x, i;

	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (float)x;
			xyzs[i][1] = (float)y;
			xyzs[i][2] = a * x * x + b * y * y;
			xyzs[i][3] = xyzs[i][2] + ((iGet_Random_No() % 100) - 50.f) / 100.f;
		}
	}

	float C[2] = { 1,1 }, Delta_C[2];
	const float eps = 0.000001f;
	int iIter = 0, iResult;
	for (iIter = 0;; iIter++)
	{//注意，此处求解的2可以省略
		float J[2] = { 0 }, H[2][2] = { 0 }, H_Inv[2][2],
			Temp[2 * 2];

		float f, Jf[2] = { 0 };

		for (i = 0; i < 100; i++)
		{   // f(C)= ∑yi - gi(C) =∑yi -(c1*x^2 + c2*y^2)   , 求 f(C)的梯度
			//∂f/∂c1 = -∑x^2
			J[0] = -xyzs[i][0] * xyzs[i][0];
			//∂f/∂c2 = -∑y^2
			J[1] = -xyzs[i][1] * xyzs[i][1];

			f = xyzs[i][3] - (C[0] * xyzs[i][0] * xyzs[i][0] + C[1] * xyzs[i][1] * xyzs[i][1]);

			Jf[0] += f * J[0];  //求∑Jf
			Jf[1] += f * J[1];

			Matrix_Multiply(J, 2, 1, J, 2, Temp);
			Matrix_Add((float*)H, Temp, 2, (float*)H);  //∑H = ∑JJ'
		}

		//求解Jf = JJ'ΔC，解出ΔC
		Get_Inv_Matrix_Row_Op((float*)H, (float*)H_Inv, 2, &iResult);

		//ΔC = H(-1) * Jf
		Matrix_Multiply((float*)H_Inv, 2, 2, Jf, 1, Delta_C);
		Matrix_Multiply(Delta_C, 1, 2, -1.f, Delta_C);

		if (fGet_Mod(Delta_C, 2) < eps)
			break;
		Vector_Add(Delta_C, C, 2, C);
	}
	printf("Iteration times:%d\n", iIter);
	Disp(C, 1, 2, "Solution C");
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

void RGBD_Test()
{//此处为一个RGBD图根据相机参数及 深度量化因子恢复空间点坐标的例子
	//必须要有两大参数：1，相机内参；2，深度因子
	typedef double _T;
	//已知相机二所观察到的点
	_T(*pPoint_3D)[3];
	_T K[3 * 3] = { 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 };    //已知条件相机内参
	unsigned char* pDepth_1;
	int i, iSize, iPoint_Count;
	Image oImage_1;
	bLoad_Raw_Data("D:\\Sample\\Depth\\Slam_Test\\1.depth", &pDepth_1, &iSize);
	bLoad_Image("D:\\Sample\\Depth\\Slam_Test\\1.bmp", &oImage_1);
	pPoint_3D = (_T(*)[3])malloc(iSize * 3 * sizeof(_T));

	//此处做了一个恢复函数，不看细节犹可
	unsigned char(*pColor)[3] = (unsigned char(*)[3])malloc(iSize * 3);
	RGBD_2_Point_3D(oImage_1, (unsigned short*)pDepth_1, (_T(*)[3])K, (_T)5000.f, pPoint_3D, &iPoint_Count, pColor);
	//bSave_PLY("c:\\tmp\\2.ply", pPoint_3D, iPoint_Count,pColor);

	//此处正经开始做恢复的演示
	//恢复要素1，深度量化因子，从量化为16位的深度图中恢复真实的深度，米为单位
	_T fDepth_Factor = 5000; //这个数字怀疑和相机参数一样，来自上游。不知道可能不行
	iSize = oImage_1.m_iWidth * oImage_1.m_iHeight;
	int y, x;

	//首先简单取数，把现有的x,y,z取出
	for (iPoint_Count = y = i = 0; y < oImage_1.m_iHeight; y++)
	{
		for (x = 0; x < oImage_1.m_iWidth; x++, i++)
		{
			if (*(unsigned short*)&pDepth_1[i << 1])
			{
				pPoint_3D[iPoint_Count][0] = x;
				pPoint_3D[iPoint_Count][1] = y;
				//取个深度，顺序有点变态
				pPoint_3D[iPoint_Count][2] = (unsigned short)((pDepth_1[i << 1] << 8) + pDepth_1[(i << 1) + 1]);
				iPoint_Count++;
			}
		}
	}

	for (i = 0; i < iPoint_Count; i++)
	{
		//第一步，先恢复真实深度。除以深度因子方可
		pPoint_3D[i][2] /= fDepth_Factor;
		//第二步，恢复真实x,y
		//已知屏幕坐标u,y 首先得其归一化坐标
		//x = (u-tx)/fx y=(v-ty)/fy 然后再乘上深度z即为真实的点坐标
		pPoint_3D[i][0] = (pPoint_3D[i][0] - K[2]) * pPoint_3D[i][2] / K[0];
		pPoint_3D[i][1] = (pPoint_3D[i][1] - K[5]) * pPoint_3D[i][2] / K[4];
	}
	free(pPoint_3D);
	free(pColor);
	free(pDepth_1);
	Free_Image(&oImage_1);
	return;
}

template<typename _T>void Temp_Load_File_1(const char* pcFile, _T(**ppPoint_3D_1)[3], _T(**ppPoint_3D_2)[3], int* piCount)
{
	_T(*pPoint_3D_1)[3], (*pPoint_3D_2)[3];
	int i, iCount = (int)(iGet_File_Length((char*)pcFile) / (6 * sizeof(float)));
	FILE* pFile = fopen(pcFile, "rb");
	if (!pFile)
	{
		printf("Failt to load file\n");
		return;
	}
	pPoint_3D_1 = (_T(*)[3])malloc(iCount * 3 * sizeof(_T) * 2);
	pPoint_3D_2 = pPoint_3D_1 + iCount;
	for (i = 0; i < iCount; i++)
	{
		float Data[6];
		fread(Data, 1, 6 * sizeof(float), pFile);
		pPoint_3D_1[i][0] = Data[0];
		pPoint_3D_1[i][1] = Data[1];
		pPoint_3D_1[i][2] = Data[2];
		pPoint_3D_2[i][0] = Data[3];
		pPoint_3D_2[i][1] = Data[4];
		pPoint_3D_2[i][2] = Data[5];
	}
	fclose(pFile);
	if (ppPoint_3D_1)
		*ppPoint_3D_1 = pPoint_3D_1;
	if (ppPoint_3D_2)
		*ppPoint_3D_2 = pPoint_3D_2;
	if (piCount)
		*piCount = iCount;
	return;
}

template<typename _T>void Temp_Load_File(const char* pcFile, _T(**ppPoint_3D_1)[3], _T(**ppPoint_3D_2)[3], int* piCount)
{
	_T(*pPoint_3D_1)[3], (*pPoint_3D_2)[3];
	int i, iCount = (int)(iGet_File_Length((char*)pcFile) / (5 * sizeof(float)));
	FILE* pFile = fopen(pcFile, "rb");
	if (!pFile)
	{
		printf("Failt to load file\n");
		return;
	}
	pPoint_3D_1 = (_T(*)[3])malloc(iCount * 3 * sizeof(_T) * 2);
	pPoint_3D_2 = pPoint_3D_1 + iCount;
	for (i = 0; i < iCount; i++)
	{
		float Data[5];
		fread(Data, 1, 5 * sizeof(float), pFile);
		pPoint_3D_1[i][0] = Data[0];
		pPoint_3D_1[i][1] = Data[1];
		pPoint_3D_1[i][2] = Data[2];
		pPoint_3D_2[i][0] = Data[3];
		pPoint_3D_2[i][1] = Data[4];
		pPoint_3D_2[i][2] = 0;
	}
	fclose(pFile);
	if (ppPoint_3D_1)
		*ppPoint_3D_1 = pPoint_3D_1;
	if (ppPoint_3D_2)
		*ppPoint_3D_2 = pPoint_3D_2;
	if (piCount)
		*piCount = iCount;
	return;
}



void BA_Test_2()
{//尝试自建点云替代样本，检验位姿估计的有效性
	typedef float _T;
	_T xyzs[100][4]; //x, y, f(x), sample
	const _T a = 3.f, b = 4.f;
	int y, x, i;

	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (_T)x;
			xyzs[i][1] = (_T)y;
			xyzs[i][2] = a * x * x + b * y * y + 1000;	//此处加1000刻意避开z<0的问题
			//xyzs[i][3] = xyzs[i][2];	// +((iGet_Random_No() % 100) - 50.f) / 100.f;
		}
	}

	_T Ksi[6], Pose_Org[4 * 4],
		K[3 * 3] = { 520.9f,  0.f,    325.1f,     //内参必须有
					0.f,      521.f,  249.7f,
					0.f,      0.f,      1.f };
	_T Rotation_Vector[4] = { 0,1,0,-PI / 6.f }, t[3] = { -100,0,0 };

	//c2w
	Gen_Ksi_by_Rotation_Vector_t(Rotation_Vector, t, Ksi);
	se3_2_SE3(Ksi, Pose_Org);
	Get_Inv_Matrix_Row_Op(Pose_Org, Pose_Org, 4);
	Disp(Pose_Org, 4, 4, "Pose_Org");

	//生成一组点，
	_T Point_3D_1[101][3], Point_2D_2[101][2];
	_T Point_3D_2[4];
	for (i = 0; i < 100; i++)
	{//注意，做数据必须合理，比如z>0，否则会出现病态数据造成估计失败
		memcpy(Point_3D_1[i], xyzs[i], 3 * sizeof(_T));
		memcpy(Point_3D_2, Point_3D_1[i], 3 * sizeof(_T));
		Point_3D_2[3] = 1;
		Matrix_Multiply(Pose_Org, 4, 4, Point_3D_2, 1, Point_3D_2);

		////加点噪声，留心观察对位姿的影响，把一下这句注释则可得到精确解
		//for(int j=0;j<3;j++)
		//	Point_3D_2[j] += ((iGet_Random_No() % 100) - 50.f) / 100.f;

		Matrix_Multiply(K, 3, 3, Point_3D_2, 1, Point_3D_2);
		Point_3D_2[0] /= Point_3D_2[2], Point_3D_2[1] /= Point_3D_2[2], Point_3D_2[2] = 1;
		memcpy(Point_2D_2[i], Point_3D_2, 2 * sizeof(_T));
	}

	_T Pose[4 * 4];
	int iResult;
	Bundle_Adjust_3D2D_1(Point_3D_1, Point_2D_2, 100, K, Pose, &iResult);
	Disp(Pose, 4, 4, "胜利来的如此突然");
	return;
}

void BA_Test_1()
{//设有RGBD数据图1， RGBD数据图2，求相机2的位姿Rt
	typedef double _T;
	//第一步，装入图1，图2的信息
	_T(*pPoint_3D_1)[3], (*pPoint_3D_2)[3];
	_T(*pPoint_2D_2)[2];
	_T K[3 * 3] = { 520.9f,  0.f,    325.1f,     //内参必须有
					0.f,      521.f,  249.7f,
					0.f,      0.f,      1.f },
		fDepth_Factor = 5000.f;                 //深度图量化参数

	int i, iCount, iResult;
	Temp_Load_File("sample\\7.8.2.bin", &pPoint_3D_1, &pPoint_3D_2, &iCount);
	pPoint_2D_2 = (_T(*)[2])malloc(iCount * 2 * sizeof(_T));
	for (i = 0; i < iCount; i++)
		memcpy(pPoint_2D_2[i], pPoint_3D_2[i], 2 * sizeof(_T));
	_T Pose[4 * 4];

	Bundle_Adjust_3D2D_1(pPoint_3D_1, pPoint_2D_2, iCount,K, Pose, &iResult);
	Disp(Pose, 4, 4, "Pose");
	return;
}

void ICP_Test_1()
{//BA解 ICP，假定已经通过某种方法找到匹配点对
	typedef float _T;
	_T(*P_1)[3], (*P_2)[3];
	int iCount;

	Temp_Load_File_1("Sample\\7.10.bin", &P_2, &P_1, &iCount);
	_T Pose[4 * 4];
	int iResult;
	ICP_BA_2_Image_1((_T(*)[3])P_1, (_T(*)[3])P_2, iCount, Pose, &iResult);
	Disp(Pose, 4, 4, "秃然胜利");
	free(P_2);
	return;
}

void ICP_Test_2()
{
	typedef double _T;
	_T(*P_1)[3], P_1_Centroid[3] = { 0 },
		(*P_2)[3], P_2_Centroid[3] = { 0 };
	_T Pose[4 * 4];
	int iCount, iResult;
	Temp_Load_File_1("Sample\\7.10.bin", &P_2, &P_1, &iCount);
	ICP_SVD(P_1, P_2, iCount, Pose, &iResult);
	Disp(Pose, 4, 4, "再下一城");
	free(P_2);
	return;
}

template<typename _T>void Temp_Load_Data(const char* pcFile, _T** ppBuffer)
{
	int i, iCount = (int)iGet_File_Length((char*)pcFile) / 4;
	FILE* pFile = fopen(pcFile, "rb");
	_T* pBuffer = (_T*)malloc(iCount * sizeof(_T));
	for (i = 0; i < iCount; i++)
	{
		float fData;
		fread(&fData, 1, sizeof(float), pFile);
		pBuffer[i] = fData;
	}
	*ppBuffer = pBuffer;
	return;
}

void Optical_Flow_Test_1()
{//目测不靠谱，这玩意不结合 ransac就是一坨屎
	typedef double _T;
	_T(*kp1)[2], (*kp2)[2];
	int iCount, iMatch_Count;
	iCount = (int)iGet_File_Length((char*)"sample\\8.3.bin") / (4 * 2);
	Temp_Load_Data("sample\\8.3.bin", (_T**)&kp1);
	//Temp_Load_Data("sample\\8.3_2.bin", (_T**)&kp2);
	kp2 = (_T(*)[2])malloc(iCount * 2 * sizeof(_T));
	Image oImage_1, oImage_2;
	bLoad_Image("D:\\Software\\3rdparty\\slambook2\\ch8\\LK1.bmp", &oImage_1);
	bLoad_Image("D:\\Software\\3rdparty\\slambook2\\ch8\\LK2.bmp", &oImage_2);

	Optical_Flow_1(oImage_1, oImage_2, kp1, kp2, iCount, &iMatch_Count);
	for (int i = 0; i < iCount; i++)
	{
		Draw_Point(oImage_1, (int)kp1[i][0], (int)kp1[i][1]);
		Draw_Point(oImage_2, (int)kp2[i][0], (int)kp2[i][1]);
	}
	bSave_Image("c:\\tmp\\1.bmp", oImage_1);
	bSave_Image("c:\\tmp\\2.bmp", oImage_2);

	free(kp1);
	free(kp2);
	Free_Image(&oImage_1);
	Free_Image(&oImage_2);
}

void ICP_Test_3()
{//还是用BA的方法解ICP问题，这次自建数据，不借助外部数据
	typedef float _T;
	_T xyzs[100][4]; //x, y, f(x), sample
	const _T a = 3.f, b = 4.f;
	int y, x, i, iResult;

	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (_T)x;
			xyzs[i][1] = (_T)y;
			xyzs[i][2] = a * x * x + b * y * y + 1000;	//此处加1000刻意避开z<0的问题
		}
	}

	_T Ksi[6], Pose_Org[4 * 4];
	_T Rotation_Vector[4] = { 0,1,0,-PI / 6.f }, t[3] = { -100,0,0 };

	//c2w
	Gen_Ksi_by_Rotation_Vector_t(Rotation_Vector, t, Ksi);
	se3_2_SE3(Ksi, Pose_Org);

	_T P1[101][3], P11[4],
		P2[101][3];
	for (i = 0; i < 100; i++)
	{//注意，做数据必须合理，比如z>0，否则会出现病态数据造成估计失败
		memcpy(P1[i], xyzs[i], 3 * sizeof(_T));
		memcpy(P11, P1[i], 3 * sizeof(_T));
		P11[3] = 1;
		Matrix_Multiply(Pose_Org, 4, 4, P11, 1, P2[i]);

		//此处加点噪声
		for (int j = 0; j < 3; j++)
			P2[i][j] += (iGet_Random_No() % 100) / 100.f;
	}

	{//方法1，简单BA，只调整位姿不调整点
		_T Pose[4 * 4];
		ICP_BA_2_Image_1(P1, P2, 100, Pose, &iResult);
		Disp(Pose_Org, 4, 4, "Pose_Org");
		Disp(Pose, 4, 4, "Estimated");
		Disp_Error(P1, P2, 100, Pose);
	}

	{//方法2，简单svd
		_T Pose[4 * 4];
		ICP_SVD(P1, P2, 100, Pose, &iResult);
		Disp(Pose_Org, 4, 4, "Pose_Org");
		Disp(Pose, 4, 4, "Estimated");
		Disp_Error(P1, P2, 100, Pose);
	}

	{//调整原点集位置，一种破坏性的方法
		_T Pose[4 * 4];
		ICP_BA_2_Image_2(P1, P2, 100, Pose, &iResult);
		Disp_Error(P1, P2, 100, Pose);
	}
}

void ICP_Test_4()
{//试一下三个点集的ICP，该实验只是估计位姿，不调整点集
	typedef float _T;
	_T xyzs[100][4]; //x, y, f(x), sample
	const _T a = 3.f, b = 4.f;
	int y, x, i, iResult;

	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (_T)x;
			xyzs[i][1] = (_T)y;
			xyzs[i][2] = a * x * x + b * y * y + 1000;	//此处加1000刻意避开z<0的问题
		}
	}
	//造两个相机位姿给点集2，点集3
	_T Pose_Org[2][4 * 4];
	_T Rotation_Vector[2][4] = { { 0,1,0,-PI / 6.f },
									{0,0,1,-PI / 3.f } };
	_T t[2][3] = { { -100,0,0 },
					{ -200,0,0 } };

	//_T Q[4],V[4],R[3*3];
	//Rotation_Vector_2_Matrix(Rotation_Vector[0], R);
	//Rotation_Matrix_2_Vector(R, V);
	//Disp(R, 3, 3, "R");
	//Disp(V, 1, 4, "V");

	Gen_Homo_Matrix_1(Rotation_Vector[0], t[0], Pose_Org[0]);
	Gen_Homo_Matrix_1(Rotation_Vector[1], t[1], Pose_Org[1]);

	//Disp(Pose_Org[0], 4, 4, "Pose_Org_1");
	//Disp(Pose_Org[1], 4, 4, "Pose_Org_2");

	//造三个点集
	_T P1[101][3], P2[101][3], P3[101][3], P11[4];
	for (i = 0; i < 100; i++)
	{//注意，做数据必须合理，比如z>0，否则会出现病态数据造成估计失败
		memcpy(P1[i], xyzs[i], 3 * sizeof(_T));
		memcpy(P11, P1[i], 3 * sizeof(_T));
		P11[3] = 1;
		Matrix_Multiply(Pose_Org[0], 4, 4, P11, 1, P2[i]);
		//此处加点噪声
		for (int j = 0; j < 3; j++)
			P2[i][j] += (iGet_Random_No() % 100) / 100.f;

		Matrix_Multiply(Pose_Org[1], 4, 4, P11, 1, P3[i]);
		//此处加点噪声
		for (int j = 0; j < 3; j++)
			P3[i][j] += (iGet_Random_No() % 100) / 100.f;
	}

	/*_T Delta_1[4 * 4];
	Get_Delta_Pose(Pose_Org[0], Pose_Org[1], Delta_1);
	Disp_Error(P1, P2, 100, Pose_Org[0]);
	Disp_Error(P1, P3, 100, Pose_Org[1]);
	Disp_Error(P2, P3, 100, Delta_1);*/


	//T12: 相机2相对相机1的位姿  T13: 相机3相对于相机1的位姿，这两个是待求位姿
	//T23: 相机3相对于相机2的位姿，这个是中间位姿，用于求Δx
	_T T12[4 * 4], T13[4 * 4], T23[4 * 4],
		Delta_Pose[2][4 * 4], Pose_Pre[2][4 * 4],
		Jct[4 * 6], Jt[3 * 12], J[12 * 3], JEt[12];
	_T  Sigma_H[12 * 12], fSum_e, fSum_e_Pre = 1e10,
		E[3], H_Inv[12 * 12];

	_T Sigma_JEt[12], H[12 * 12], X[12];
	_T P_Temp[4]; //P1'
	int j, k, iIter;
	Gen_I_Matrix(T12, 4, 4);
	Gen_I_Matrix(T13, 4, 4);

	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		Get_Delta_Pose(T12, T13, T23);
		/*if (iIter == 1)
		{
			Disp(T12, 4, 4, "T12");
			Disp(T13, 4, 4, "T13");
			Disp(T23, 4, 4, "T23");
		}*/
		memset(Sigma_H, 0, 12 * 12 * sizeof(_T));
		memset(Sigma_JEt, 0, 12 * sizeof(_T));

		for (i = 0; i < 100; i++)
		{
			//先搞搞P1,P2匹配
			memcpy(P_Temp, P1[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T12, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P2[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
			Get_Deriv_TP_Ksi(T12, P1[i], Jct);    //∂TP/∂ξ
			memset(Jt, 0, 3 * 12 * sizeof(_T));
			//∂T12P1/∂ 放在前6列
			for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P 
				for (k = 0; k < 6; k++)
					Jt[j * 12 + k] = Jct[j * 6 + k];
			Matrix_Multiply(Jt, 3, 12, (_T)-1.f, Jt);    //J'已经到位
			Matrix_Transpose(Jt, 3, 12, J);
			Matrix_Multiply(J, 12, 3, E, 1, JEt);    //JE'到位
			Matrix_Multiply(J, 12, 3, Jt, 12, H);
			Matrix_Add(Sigma_H, H, 12, Sigma_H);         //∑H, JJ'到位
			Vector_Add(Sigma_JEt, JEt, 12, Sigma_JEt);   //∑JE'

			//再搞P1,P3匹配
			memcpy(P_Temp, P1[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T13, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P3[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
			Get_Deriv_TP_Ksi(T13, P1[i], Jct);    //∂TP/∂ξ
			memset(Jt, 0, 3 * 12 * sizeof(_T));
			//∂T13P1/∂ 放在后6列
			for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P 
				for (k = 0; k < 6; k++)
					Jt[j * 12 + 6 + k] = Jct[j * 6 + k];
			Matrix_Multiply(Jt, 3, 12, (_T)-1.f, Jt);    //J'已经到位
			Matrix_Transpose(Jt, 3, 12, J);
			Matrix_Multiply(J, 12, 3, E, 1, JEt);    //JE'到位
			Matrix_Multiply(J, 12, 3, Jt, 12, H);
			Matrix_Add(Sigma_H, H, 12, Sigma_H);         //∑H, JJ'到位
			Vector_Add(Sigma_JEt, JEt, 12, Sigma_JEt);   //∑JE'

			//搞T23，假如还是调整T13，窃以为这一步很重要，否则解决不了累积误差。
			//加了这个调整以后，能形成相机路径闭环
			memcpy(P_Temp, P2[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T23, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P3[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];

			Get_Deriv_TP_Ksi(T13, P1[i], Jct);    //∂TP/∂ξ
			memset(Jt, 0, 3 * 12 * sizeof(_T));
			//∂T13P1/∂ 放在后6列
			for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P 
				for (k = 0; k < 6; k++)
					Jt[j * 12 + 6 + k] = Jct[j * 6 + k];
			Matrix_Multiply(Jt, 3, 12, (_T)-1.f, Jt);    //J'已经到位
			Matrix_Transpose(Jt, 3, 12, J);
			Matrix_Multiply(J, 12, 3, E, 1, JEt);    //JE'到位
			Matrix_Multiply(J, 12, 3, Jt, 12, H);
			Matrix_Add(Sigma_H, H, 12, Sigma_H);         //∑H, JJ'到位
			Vector_Add(Sigma_JEt, JEt, 12, Sigma_JEt);   //∑JE'

			////以下这步注释掉完全没有影响，收敛还快
			////假如连T12一起也调整了。目前看到这个可有可无，加了对T12的调整以后，收敛慢了，数据基本一致
			////然而这个实验相机太少，无法评价这一步是否需要。
			//Get_Deriv_TP_Ksi(T12, P1[i], Jct);    //∂TP/∂ξ
			//memset(Jt, 0, 3 * 12 * sizeof(_T));
			////∂T12P1/∂ 放在前6列
			//for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P 
			//    for (k = 0; k < 6; k++)
			//        Jt[j * 12 + k] = Jct[j * 6 + k];
			//Matrix_Multiply(Jt, 3, 12, (_T)-1.f, Jt);    //J'已经到位
			//Matrix_Transpose(Jt, 3, 12, J);
			//Matrix_Multiply(J, 12, 3, E, 1, JEt);    //JE'到位
			//Matrix_Multiply(J, 12, 3, Jt, 12, H);
			//Matrix_Add(Sigma_H, H, 12, Sigma_H);         //∑H, JJ'到位
			//Vector_Add(Sigma_JEt, JEt, 12, Sigma_JEt);   //∑JE'
		}

		Get_Inv_Matrix_Row_Op_2(Sigma_H, H_Inv, 12, &iResult);
		if (!iResult)
		{//此处对∑H的调整是关键，否则很有可能不满秩，注意，此处调整量应该是
			// H + λI，只不过H 没有呈现比较大的数值，故此λ=1已经够用
			_T I[12 * 12];
			Gen_I_Matrix(I, 12, 12);
			Matrix_Add(Sigma_H, I, 12, Sigma_H);
			Get_Inv_Matrix_Row_Op_2(Sigma_H, H_Inv, 12, &iResult);
		}

		//Δx= -(∑H)(-1) * ∑JE'	 (E'为3x1)
		Matrix_Multiply(H_Inv, 12, 12, Sigma_JEt, 1, X);
		Matrix_Multiply(X, 1, 12, (_T)-1, X);

		if ((fSum_e_Pre <= fSum_e && iIter > 0) || !iResult)
			break;

		//先更新一下Pose_Estimate
		se3_2_SE3(X, Delta_Pose[0]);
		se3_2_SE3(X + 6, Delta_Pose[1]);

		memcpy(Pose_Pre[0], T12, 4 * 4 * sizeof(_T));
		memcpy(Pose_Pre[1], T13, 4 * 4 * sizeof(_T));
		Matrix_Multiply(Delta_Pose[0], 4, 4, T12, 4, T12);
		Matrix_Multiply(Delta_Pose[1], 4, 4, T13, 4, T13);

		printf("Iter:%d Cost:%f\n", iIter, fSum_e);
		fSum_e_Pre = fSum_e;
	}

	Get_Delta_Pose(Pose_Pre[0], Pose_Pre[1], T23);
	Disp_Error(P1, P2, 100, Pose_Pre[0]);
	Disp_Error(P1, P3, 100, Pose_Pre[1]);
	Disp_Error(P2, P3, 100, T23);
	return;
}

void ICP_Test_5()
{//最简闭环实验，三个点集的ICP，该实验要做两大尝试，第一，对于闭环的匹配点集尝试一种一般的位姿估计，
//第二，连点集一起调整
	typedef float _T;
	_T xyzs[100][4]; //x, y, f(x), sample
	const _T a = 3.f, b = 4.f;
	int y, x, i, iResult;

	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (_T)x;
			xyzs[i][1] = (_T)y;
			xyzs[i][2] = a * x * x + b * y * y + 1000;	//此处加1000刻意避开z<0的问题
		}
	}

	//造两个相机位姿给点集2，点集3
	_T Pose_Org[2][4 * 4];
	_T Rotation_Vector[2][4] = { { 0,1,0,-PI / 6.f },
									{0,1,0,-PI / 3.f } };
	_T t[2][3] = { { -100,0,0 },
					{ -200,0,0 } };

	Gen_Homo_Matrix_1(Rotation_Vector[0], t[0], Pose_Org[0]);
	Gen_Homo_Matrix_1(Rotation_Vector[1], t[1], Pose_Org[1]);

	//造三个点集
	_T P1[101][3], P2[101][3], P3[101][3], P11[4];
	for (i = 0; i < 100; i++)
	{//注意，做数据必须合理，比如z>0，否则会出现病态数据造成估计失败
		memcpy(P1[i], xyzs[i], 3 * sizeof(_T));
		memcpy(P11, P1[i], 3 * sizeof(_T));
		P11[3] = 1;
		Matrix_Multiply(Pose_Org[0], 4, 4, P11, 1, P2[i]);
		//此处加点噪声
		for (int j = 0; j < 3; j++)
			P2[i][j] += (iGet_Random_No() % 100) / 100.f;

		Matrix_Multiply(Pose_Org[1], 4, 4, P11, 1, P3[i]);
		//此处加点噪声
		for (int j = 0; j < 3; j++)
			P3[i][j] += (iGet_Random_No() % 100) / 100.f;
	}

	//T12: 相机2相对相机1的位姿  T13: 相机3相对于相机1的位姿，这两个是待求位姿
	//T23: 相机3相对于相机2的位姿，这个是中间位姿，用于求Δx
	_T T12[4 * 4], T13[4 * 4], T23[4 * 4], R12[3 * 3], R23[3 * 3],
		Delta_Pose[2][4 * 4], Pose_Pre[2][4 * 4],
		Jct[4 * 6];
	const int w = 18;
	_T Jt[3 * w], J[w * 3], JEt[w];
	_T  Sigma_H[w * w], fSum_e, fSum_e_Pre = 1e10,
		E[3], H_Inv[w * w];

	_T Sigma_JEt[w], H[w * w], X[w];
	_T P_Temp[4]; //P1'
	int j, k, iIter;
	Gen_I_Matrix(T12, 4, 4);
	Gen_I_Matrix(T23, 4, 4);
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		memset(Sigma_H, 0, w * w * sizeof(_T));
		memset(Sigma_JEt, 0, w * sizeof(_T));
		//根据T12,T23推出T13，T13= T23 * T12
		Matrix_Multiply(T23, 4, 4, T12, 4, T13);
		Get_R_t(T12, R12);
		Get_R_t(T23, R23);
		for (i = 0; i < 100; i++)
		{
			//先搞搞P1,P2匹配
			memcpy(P_Temp, P1[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T12, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P2[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
			Get_Deriv_TP_Ksi(T12, P1[i], Jct);    //∂TP/∂ξ
			memset(Jt, 0, 3 * w * sizeof(_T));
			for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P
			{//∂T12P1/∂ξ 放在前6列
				for (k = 0; k < 6; k++)
					Jt[j * w + k] = Jct[j * 6 + k];
				//将∂P2/∂P1 放在12-15列
				for (k = 0; k < 3; k++)
					Jt[j * w + 12 + k] = R12[j * 3 + k];
			}
			Matrix_Multiply(Jt, 3, w, (_T)-1.f, Jt);    //J'已经到位
			Matrix_Transpose(Jt, 3, w, J);
			Matrix_Multiply(J, w, 3, E, 1, JEt);    //JE'到位
			Matrix_Multiply(J, w, 3, Jt, w, H);
			Matrix_Add(Sigma_H, H, w, Sigma_H);         //∑H, JJ'到位
			Vector_Add(Sigma_JEt, JEt, w, Sigma_JEt);   //∑JE'

			//再搞P2,P3匹配
			memcpy(P_Temp, P2[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T23, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P3[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
			Get_Deriv_TP_Ksi(T23, P2[i], Jct);    //∂TP/∂ξ
			memset(Jt, 0, 3 * w * sizeof(_T));
			for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P
			{//∂T23P2/∂ξ 放在6-11列
				for (k = 0; k < 6; k++)
					Jt[j * w + 6 + k] = Jct[j * 6 + k];
				for (k = 0; k < 3; k++)
					Jt[j * w + 15 + k] = R23[j * 3 + k];
			}
			Matrix_Multiply(Jt, 3, w, (_T)-1.f, Jt);    //J'已经到位
			Matrix_Transpose(Jt, 3, w, J);
			Matrix_Multiply(J, w, 3, E, 1, JEt);    //JE'到位
			Matrix_Multiply(J, w, 3, Jt, w, H);
			Matrix_Add(Sigma_H, H, w, Sigma_H);         //∑H, JJ'到位
			Vector_Add(Sigma_JEt, JEt, w, Sigma_JEt);   //∑JE'

			//第三步，闭环一步，用P1,P3的误差来修正T23
			memcpy(P_Temp, P1[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T13, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P3[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
			Get_Deriv_TP_Ksi(T23, P2[i], Jct);    //∂TP/∂ξ
			memset(Jt, 0, 3 * w * sizeof(_T));
			for (j = 0; j < 3; j++)         //将∂TP/∂ξ与 ∂P'/∂P
			{//∂T23P2/∂ξ 放在6-11列
				for (k = 0; k < 6; k++)
					Jt[j * w + 6 + k] = Jct[j * 6 + k];
				//此处也不能少，最后一步闭环不但影响位姿，还要调整点集
				for (k = 0; k < 3; k++)
					Jt[j * w + 15 + k] = R23[j * 3 + k];
			}
			Matrix_Multiply(Jt, 3, w, (_T)-1.f, Jt);    //J'已经到位
			Matrix_Transpose(Jt, 3, w, J);
			Matrix_Multiply(J, w, 3, E, 1, JEt);    //JE'到位
			Matrix_Multiply(J, w, 3, Jt, w, H);
			Matrix_Add(Sigma_H, H, w, Sigma_H);         //∑H, JJ'到位
			Vector_Add(Sigma_JEt, JEt, w, Sigma_JEt);   //∑JE'
		}

		iResult = 0;
		if (!iResult)
		{//此处对∑H的调整是关键，否则很有可能不满秩，注意，此处调整量应该是
			// H + λI，只不过H 没有呈现比较大的数值，故此λ=1已经够用
			_T I[w * w];
			Gen_I_Matrix(I, w, w);
			Matrix_Add(Sigma_H, I, w, Sigma_H);
			Get_Inv_Matrix_Row_Op_2(Sigma_H, H_Inv, w, &iResult);
		}
		//Δx= -(∑H)(-1) * ∑JE'	 (E'为3x1)
		Matrix_Multiply(H_Inv, w, w, Sigma_JEt, 1, X);
		Matrix_Multiply(X, 1, w, (_T)-1, X);
		//if (iIter == 1)
			//Disp(X, 1, w, "X");
		if ((fSum_e_Pre <= fSum_e && iIter > 0) || !iResult)
			break;

		//先更新一下Pose_Estimate
		se3_2_SE3(X, Delta_Pose[0]);
		se3_2_SE3(X + 6, Delta_Pose[1]);

		memcpy(Pose_Pre[0], T12, 4 * 4 * sizeof(_T));
		memcpy(Pose_Pre[1], T23, 4 * 4 * sizeof(_T));
		Matrix_Multiply(Delta_Pose[0], 4, 4, T12, 4, T12);
		Matrix_Multiply(Delta_Pose[1], 4, 4, T23, 4, T23);

		//继续调整P1,P2
		for (i = 0; i < 100; i++)
		{
			Vector_Add(P1[i], &X[12], 3, P1[i]);
			Vector_Add(P2[i], &X[15], 3, P2[i]);
		}
		printf("Iter:%d Cost:%f\n", iIter, fSum_e);
		fSum_e_Pre = fSum_e;
	}

	memcpy(T12, Pose_Pre[0], 4 * 4 * sizeof(_T));
	memcpy(T23, Pose_Pre[1], 4 * 4 * sizeof(_T));
	Matrix_Multiply(T23, 4, 4, T12, 4, T13);

	printf("成功闭环！%f\n",fSum_e_Pre);
	/*Disp_Error(P1, P2, 100, T12);
	Disp_Error(P1, P3, 100, T13);
	Disp_Error(P2, P3, 100, T23);
	Disp(P1[0], 1, 3, "P1");
	Disp(P2[0], 1, 3, "P2");*/
	return;
}
void Transform_Example_2D()
{//用正路方法，二维下的变换

	{//第一个实验，点(0,100)绕原点逆时针旋转30度
		float R[2 * 2];	//齐次变换矩阵，先生成这个矩阵，跟着理论走
		float P1[2], P0[2] = { 100,0 };
		Gen_Rotation_Matrix_2D(R, PI / 6.f);
		Disp(R, 2, 2, "R");
		Matrix_Multiply(R, 2, 2, P0, 1, P1);
		Disp(P1, 2, 1, "旋转后");
	}

	{//第二个实验，位移必须用齐次坐标，就是第三维为1，点(0,0,1)位移(10,10)
		float P1[3], P0[3] = { 0,0,1 };
		float T[3 * 3] = { 0,0, 10,
							0,0, 10,
							0,0, 1 };
		Matrix_Multiply(T, 3, 3, P0, 1, P1);
		Disp(P1, 3, 1, "位移后");
	}
	{//第三个实验，生成一个旋转加位移齐次矩阵
		float R[2 * 2], t[2] = { 10,10 };	//齐次变换矩阵，先生成这个矩阵，跟着理论走
		float P1[3], P0[3] = { 100,0,1 };
		float T[3 * 3];
		Gen_Rotation_Matrix_2D(R, PI / 6.f);
		Gen_Homo_Matrix_2D(R, t, T);
		Disp(T, 3, 3, "齐次变换矩阵");
		Matrix_Multiply(T, 3, 3, P0, 1, P1);
		Disp(P1, 3, 1, "变换后坐标");

		//此处解释T矩阵的动作费解
		Matrix_Multiply(R, 2, 2, P0, 1, P1);
		Vector_Add(P1, t, 2, P1);
		Disp(P1, 2, 1, "P1");	//可见，先旋转后位移。反之不然。因为矩阵乘法不可交换
	}
}
void Sparse_Matrix_Test()
{//稀疏矩阵小试牛刀，试一下三个点集的ICP，该实验只是估计位姿，不调整点集
	typedef float _T;
	_T xyzs[100][4]; //x, y, f(x), sample
	const _T a = 3.f, b = 4.f;
	int y, x, i, iResult;
	for (y = 0; y < 10; y++)
	{
		for (x = 0; x < 10; x++)
		{
			i = y * 10 + x;
			xyzs[i][0] = (_T)x;
			xyzs[i][1] = (_T)y;
			xyzs[i][2] = a * x * x + b * y * y + 1000;	//此处加1000刻意避开z<0的问题
		}
	}

	//造两个相机位姿给点集2，点集3
	_T Pose_Org[2][4 * 4];
	_T Rotation_Vector[2][4] = { { 0,1,0,-PI / 6.f },
									{0,0,1,-PI / 3.f } };
	_T t[2][3] = { { -100,0,0 },
					{ -200,0,0 } };

	Gen_Homo_Matrix_1(Rotation_Vector[0], t[0], Pose_Org[0]);
	Gen_Homo_Matrix_1(Rotation_Vector[1], t[1], Pose_Org[1]);

	//Disp(Pose_Org[0], 4, 4, "Pose_Org_1");
	//Disp(Pose_Org[1], 4, 4, "Pose_Org_2");

	//造三个点集
	_T P1[101][3], P2[101][3], P3[101][3], P11[4];
	for (i = 0; i < 100; i++)
	{//注意，做数据必须合理，比如z>0，否则会出现病态数据造成估计失败
		memcpy(P1[i], xyzs[i], 3 * sizeof(_T));
		memcpy(P11, P1[i], 3 * sizeof(_T));
		P11[3] = 1;
		Matrix_Multiply(Pose_Org[0], 4, 4, P11, 1, P2[i]);
		//此处加点噪声
		for (int j = 0; j < 3; j++)
			P2[i][j] += (iGet_Random_No() % 100) / 100.f;

		Matrix_Multiply(Pose_Org[1], 4, 4, P11, 1, P3[i]);
		//此处加点噪声
		for (int j = 0; j < 3; j++)
			P3[i][j] += (iGet_Random_No() % 100) / 100.f;
	}

	//T12: 相机2相对相机1的位姿  T13: 相机3相对于相机1的位姿，这两个是待求位姿
	//T23: 相机3相对于相机2的位姿，这个是中间位姿，用于求Δx
	_T T12[4 * 4], T13[4 * 4], T23[4 * 4],
		Delta_Pose[2][4 * 4], Pose_Pre[2][4 * 4],
		Jct[4 * 6];
	_T fSum_e, fSum_e_Pre = 1e10,
		E[3];

	_T Sigma_JEt[12], X[12];
	_T P_Temp[4]; //P1'
	Sparse_Matrix<_T> oSigma_JEt, oSigma_H;
	Sparse_Matrix<_T> oJ, oJt, oE, oJEt, oH, oH_Inv;
	int j, k, iIter;
	Gen_I_Matrix(T12, 4, 4);
	Gen_I_Matrix(T13, 4, 4);
	//Init_Sparse_Matrix(&oSigma_JEt, 12,1,12);
	Init_Sparse_Matrix(&oSigma_H, 12 * 12, 12, 12);
	Init_Sparse_Matrix(&oJt, 3 * 12, 12, 3);
	Init_Sparse_Matrix(&oJ, 12 * 3, 3, 12);
	Init_Sparse_Matrix(&oE, 3 * 1, 1, 3);
	Init_Sparse_Matrix(&oJEt, 12 * 1, 1, 12);
	Init_Sparse_Matrix(&oSigma_JEt, 12, 1, 12);
	Init_Sparse_Matrix(&oH, 12 * 12, 12, 12);
	Gen_I_Matrix(T12, 4, 4);
	Gen_I_Matrix(T13, 4, 4);

	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		Get_Delta_Pose(T12, T13, T23);
		Reset_Sparse_Matrix(&oSigma_H);
		Reset_Sparse_Matrix(&oSigma_JEt);

		for (i = 0; i < 100; i++)
		{
			//先搞搞P1,P2匹配
			memcpy(P_Temp, P1[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T12, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P2[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
			Get_Deriv_TP_Ksi(T12, P1[i], Jct);    //?TP/?ξ
			Reset_Sparse_Matrix(&oJt);
			Reset_Sparse_Matrix(&oE);
			//?T12P1/? 放在前6列
			for (j = 0; j < 3; j++)         //将?TP/?ξ与 ?P'/?P 
			{
				for (k = 0; k < 6; k++)
					Set_Value(&oJt, k, j, Jct[j * 6 + k]);
				Set_Value(&oE, 0, j, E[j]);
			}
			Matrix_Multiply(oJt, (_T)-1.f);				//J'已经到位
			Matrix_Transpose_1(oJt, &oJ);
			Matrix_Multiply(oJ, oE, &oJEt);             //JE
			//printf("i:%d %f\n",i, fGet_Value(&oJEt, 0, 4));
			Matrix_Multiply(oJ, oJt, &oH);				//H=JJt			
			Matrix_Add(oSigma_H, oH, &oSigma_H);		//
			Matrix_Add(oSigma_JEt, oJEt, &oSigma_JEt);

			//再搞P1, P3匹配
			memcpy(P_Temp, P1[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T13, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P3[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
			Get_Deriv_TP_Ksi(T13, P1[i], Jct);    //∂TP/∂ξ
			Reset_Sparse_Matrix(&oJt);
			Reset_Sparse_Matrix(&oE);
			//?T12P1/? 放在前6列
			for (j = 0; j < 3; j++)         //将?TP/?ξ与 ?P'/?P 
			{
				for (k = 0; k < 6; k++)
					Set_Value(&oJt, 6 + k, j, Jct[j * 6 + k]);
				Set_Value(&oE, 0, j, E[j]);
			}
			Matrix_Multiply(oJt, (_T)-1.f);				//J'已经到位
			Matrix_Transpose_1(oJt, &oJ);
			Matrix_Multiply(oJ, oE, &oJEt);             //JE
			Matrix_Multiply(oJ, oJt, &oH);				//H=JJt
			Matrix_Add(oSigma_H, oH, &oSigma_H);		//
			Matrix_Add(oSigma_JEt, oJEt, &oSigma_JEt);

			//搞T23，假如还是调整T13，窃以为这一步很重要，否则解决不了累积误差。
			//加了这个调整以后，能形成相机路径闭环
			memcpy(P_Temp, P2[i], 3 * sizeof(_T));
			P_Temp[3] = 1;
			Matrix_Multiply(T23, 4, 4, P_Temp, 1, P_Temp);
			Vector_Minus(P3[i], P_Temp, 3, E);
			fSum_e += E[0] * E[0] + E[1] * E[1] + E[2] * E[2];

			Get_Deriv_TP_Ksi(T13, P1[i], Jct);    //∂TP/∂ξ
			Reset_Sparse_Matrix(&oJt);
			Reset_Sparse_Matrix(&oE);
			//∂T13P1/∂ 放在后6列
			for (j = 0; j < 3; j++)         //将?TP/?ξ与 ?P'/?P 
			{
				for (k = 0; k < 6; k++)
					Set_Value(&oJt, 6 + k, j, Jct[j * 6 + k]);
				Set_Value(&oE, 0, j, E[j]);
			}
			Matrix_Multiply(oJt, (_T)-1.f);				//J'已经到位
			Matrix_Transpose_1(oJt, &oJ);
			Matrix_Multiply(oJ, oE, &oJEt);             //JE
			Matrix_Multiply(oJ, oJt, &oH);				//H=JJt
			Matrix_Add(oSigma_H, oH, &oSigma_H);		//
			Matrix_Add(oSigma_JEt, oJEt, &oSigma_JEt);
		}

		Sparse_2_Dense(oSigma_JEt, Sigma_JEt);
		Solve_Linear_Gause(oSigma_H, Sigma_JEt, X, &iResult);
		if (!iResult)
		{
			Add_I_Matrix(&oSigma_H);
			Solve_Linear_Gause(oSigma_H, Sigma_JEt, X, &iResult);
		}
		Matrix_Multiply(X, 1, 12, (_T)-1, X);
		if ((fSum_e_Pre <= fSum_e && iIter > 0) || !iResult)
			break;

		//先更新一下Pose_Estimate
		se3_2_SE3(X, Delta_Pose[0]);
		se3_2_SE3(X + 6, Delta_Pose[1]);

		memcpy(Pose_Pre[0], T12, 4 * 4 * sizeof(_T));
		memcpy(Pose_Pre[1], T13, 4 * 4 * sizeof(_T));

		Matrix_Multiply(Delta_Pose[0], 4, 4, T12, 4, T12);
		Matrix_Multiply(Delta_Pose[1], 4, 4, T13, 4, T13);

		printf("Iter:%d Cost:%f\n", iIter, fSum_e);
		fSum_e_Pre = fSum_e;
	}

	Get_Delta_Pose(Pose_Pre[0], Pose_Pre[1], T23);
	Disp_Error(P1, P2, 100, Pose_Pre[0]);
	Disp_Error(P1, P3, 100, Pose_Pre[1]);
	Disp_Error(P2, P3, 100, T23);

	Free_Sparse_Matrix(&oSigma_JEt);
	Free_Sparse_Matrix(&oH);
	Free_Sparse_Matrix(&oSigma_H);
	Free_Sparse_Matrix(&oJt);
	Free_Sparse_Matrix(&oJ);
	Free_Sparse_Matrix(&oE);
	Free_Sparse_Matrix(&oJEt);
	Disp_Mem(&oMatrix_Mem,0);
	return;
}

void Sphere_Test_2()
{//4基站定位实验，高斯牛顿法寻找4个球的交点最优解
	typedef float _T;
	_T Sphere_Center[4][3] = { { 200,300,400 },
								{ 300,400,410 } ,
								{ 350,300,420 } ,
								{ 200,400,430 } };

	_T d[4] = { 100,110,120,130 };	//可视为基站到物体距离
	int i;
	Image oImage;
	Init_Image(&oImage, 1000, 1000, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for (i = 0; i < 4; i++)
		Draw_Arc(oImage, (int)d[i], (int)Sphere_Center[i][0], (int)Sphere_Center[i][1]);

	_T P[3] = { 0,0,0 }, Pre_P[3];	//最后的解
	int iIter, iResult;

	_T Jt[1 * 3], J[3 * 1], JJt[3 * 3], Je[3 * 1], H_Inv[3 * 3], Delta_X[3],
		fSum_e, e, fSum_e_Pre = 1e10;
	_T Sigma_H[3 * 3], Sigma_Je[3 * 1];
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		//置零
		memset(Sigma_H, 0, 3 * 3 * sizeof(_T));
		memset(Sigma_Je, 0, 3 * sizeof(_T));

		//到4张皮的距离和最小感觉最好，因为这才是题意
		for (i = 0; i < 4; i++)
		{	//e = [(x-a)^2 + (x-b)^2 + (z-c)^2]^1/2
			_T fDist_Sqr = (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) +
				(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]) +
				(P[2] - Sphere_Center[i][2]) * (P[2] - Sphere_Center[i][2]);
			e = d[i] - (_T)sqrt(fDist_Sqr);
			fSum_e += abs(e);	// e* e;	//这里就必须用e的平方了，或者用绝对值，因为有正有负。要琢磨一下误差的度量对结果的影响
			//?e/?x = - [(x-a)^2 + (x-b)^2 + (z-a)^2] *(x-a)
			Jt[0] = -(_T)pow(fDist_Sqr, -0.5f) * (P[0] - Sphere_Center[i][0]);
			Jt[1] = -(_T)pow(fDist_Sqr, -0.5f) * (P[1] - Sphere_Center[i][1]);
			Jt[2] = -(_T)pow(fDist_Sqr, -0.5f) * (P[2] - Sphere_Center[i][2]);

			memcpy(J, Jt, 3 * sizeof(_T));
			Matrix_Multiply(J, 3, 1, Jt, 3, JJt);
			Matrix_Multiply(J, 3, 1, e, Je);
			//	//累加
			Matrix_Add(Sigma_H, JJt, 3, Sigma_H);
			Vector_Add(Sigma_Je, Je, 3, Sigma_Je);
		}

		////试一下,到圆心距离和最小
		//for (i = 0; i < 4; i++)
		//{	//e = [(x-a)^2 + (x-b)^2 + (z-c)^2]^1/2
		//	float fDist_Sqr = (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) +
		//		(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]) +
		//		(P[2] - Sphere_Center[i][2]) * (P[2] - Sphere_Center[i][2]);
		//	if (fDist_Sqr == 0)
		//		continue;
		//	e = sqrt(fDist_Sqr);
		//	fSum_e += e;	//此处到底用e还是e的平方？用e表示到点的距离之和，用e*e表示距离的平方和，微妙差别

		//	//?e/?x = [(x-a)^2 + (x-b)^2 + (z-c)^2] *(x-a) 
		//	Jt[0] = pow(fDist_Sqr, -0.5) * (P[0] - Sphere_Center[i][0]);
		//	Jt[1] = pow(fDist_Sqr, -0.5) * (P[1] - Sphere_Center[i][1]);
		//	Jt[2] = pow(fDist_Sqr, -0.5) * (P[2] - Sphere_Center[i][2]);

		//	memcpy(J, Jt, 3 * sizeof(_T));
		//	Matrix_Multiply(J, 3, 1, Jt, 3, JJt);
		//	Matrix_Multiply(J, 3, 1, e, Je);
		//	//	//累加
		//	Matrix_Add(Sigma_H, JJt, 3, Sigma_H);
		//	Vector_Add(Sigma_Je, Je, 3, Sigma_Je);
		//}

		//后面全一样，毫无差别毫无营养，搞利索以后改成解方程，
		//会比求个逆可能快一点
		Sigma_H[0] += 1, Sigma_H[4] += 1, Sigma_H[8] += 1;
		Get_Inv_Matrix_Row_Op(Sigma_H, H_Inv, 3, &iResult);
		Matrix_Multiply(H_Inv, 3, 3, Sigma_Je, 1, Delta_X);
		Matrix_Multiply(Delta_X, 3, 1, (_T)-1.f, Delta_X);

		if ((fSum_e_Pre <= fSum_e && iIter > 0) || !iResult)
			break;
		printf("Iter:%d Error:%f\n", iIter, fSum_e);

		Pre_P[0] = (P[0] += Delta_X[0]);
		Pre_P[1] = (P[1] += Delta_X[1]);
		Pre_P[2] = (P[2] += Delta_X[2]);
		fSum_e_Pre = fSum_e;
	}

	//Disp(Pre_P, 1, 3, "Point");
	Draw_Point(oImage, (int)Pre_P[0], (int)Pre_P[1], 2);
	bSave_Image("c:\\tmp\\1.bmp", oImage);
	Free_Image(&oImage);
}

void Sphere_Test_1()
{//4基站定位实验，高斯牛顿法寻找4个圆的交点最优解
	typedef float _T;
	_T Sphere_Center[4][3] = { { 200,300,400 },
								{ 300,400,410 } ,
								{ 350,300,420 } ,
								{ 200,400,430 } };

	_T R[4] = { 100,110,120,130 };
	Image oImage;
	int i;
	Init_Image(&oImage, 1000, 1000, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for (i = 0; i < 4; i++)
		Draw_Arc(oImage, (int)R[i], (int)Sphere_Center[i][0], (int)Sphere_Center[i][1]);

	_T P[3] = { 0,0,0 }, Pre_P[3];	//最后的解
	int iIter, iResult;

	_T Jt[1 * 2], J[2 * 1], JJt[2 * 2], Je[2 * 1], H_Inv[2 * 2], Delta_X[2],
		fSum_e, e, fSum_e_Pre = 1e10;
	_T Sigma_H[2 * 2], Sigma_Je[2 * 1];
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		memset(Sigma_H, 0, 2 * 2 * sizeof(_T));
		memset(Sigma_Je, 0, 2 * sizeof(_T));

		//这个条件不好
		//for (i = 0; i < 2; i++)
		//{
		//	e = R[i] * R[i] - (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) -
		//		(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]);
		//	fSum_e += e * e;
		//	//?e/?x= (-2x, -2y)	,第一步顺着这个梯度走，能走到目的地
		//	Jt[0] = -2 * (P[0]-Sphere_Center[i][0]), Jt[1] = -2 * (P[1]-Sphere_Center[i][1]);
		//	//Disp(Jt, 1, 2, "Jt");
		//	memcpy(J, Jt, 2 * sizeof(_T));
		//	//第二步，要求Δx
		//	Matrix_Multiply(J, 2, 1, Jt, 2, JJt);
		//	//Disp(JJt, 2, 2, "JJt");
		//	Matrix_Multiply(J, 2, 1, e, Je);

		//	//累加
		//	Matrix_Add(Sigma_H, JJt, 2, Sigma_H);
		//	Vector_Add(Sigma_Je, Je, 2, Sigma_Je);
		//}

		////以下为到4个圆边距离和最近
		//for (i = 0; i < 4; i++)
		//{//e(x) = Ri - [(x-a)^2 + (x-b)^2]^1/2
		//	float fDist_Sqr = (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) +
		//		(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]);
		//	e= R[i] - sqrt(fDist_Sqr);
		//	fSum_e += e * e;
		//	//?e/?x = - [(x-a)^2 + (x-b)^2] *(x-a)
		//	Jt[0] = -pow(fDist_Sqr,-0.5) * (P[0] - Sphere_Center[i][0]);
		//	Jt[1] = -pow(fDist_Sqr,-0.5) * (P[1] - Sphere_Center[i][1]);

		//	memcpy(J, Jt, 2 * sizeof(_T));
		//	Matrix_Multiply(J, 2, 1, Jt, 2, JJt);
		//	Matrix_Multiply(J, 2, 1, e, Je);
		//	//	//累加
		//	Matrix_Add(Sigma_H, JJt, 2, Sigma_H);
		//	Vector_Add(Sigma_Je, Je, 2, Sigma_Je);
		//}

		//试一下,到圆心距离和最小
		for (i = 0; i < 4; i++)
		{
			_T fDist_Sqr = (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) +
				(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]);
			if (fDist_Sqr == 0)
				continue;
			e = (_T)sqrt(fDist_Sqr);
			fSum_e += e;
			//?e/?x = [(x-a)^2 + (x-b)^2] *(x-a)
			Jt[0] = (_T)pow(fDist_Sqr, -0.5f) * (P[0] - Sphere_Center[i][0]);
			Jt[1] = (_T)pow(fDist_Sqr, -0.5f) * (P[1] - Sphere_Center[i][1]);

			memcpy(J, Jt, 2 * sizeof(_T));
			Matrix_Multiply(J, 2, 1, Jt, 2, JJt);
			Matrix_Multiply(J, 2, 1, e, Je);
			//	//累加
			Matrix_Add(Sigma_H, JJt, 2, Sigma_H);
			Vector_Add(Sigma_Je, Je, 2, Sigma_Je);
		}

		Sigma_H[0] += 1, Sigma_H[3] += 1;
		Get_Inv_Matrix_Row_Op(Sigma_H, H_Inv, 2, &iResult);
		Matrix_Multiply(H_Inv, 2, 2, Sigma_Je, 1, Delta_X);
		Matrix_Multiply(Delta_X, 2, 1, (_T)-1.f, Delta_X);

		if ((fSum_e_Pre <= fSum_e && iIter > 0) || !iResult)
			break;
		printf("Iter:%d Error:%f\n", iIter, fSum_e);
		//Draw_Point(oImage, P[0], P[1],2);

		Pre_P[0] = (P[0] += Delta_X[0]);
		Pre_P[1] = (P[1] += Delta_X[1]);
		fSum_e_Pre = fSum_e;
	}
	Draw_Point(oImage, (int)Pre_P[0], (int)Pre_P[1], 2);
	Disp(Pre_P, 1, 3, "Point");
	bSave_Image("c:\\tmp\\1.bmp", oImage);
	Free_Image(&oImage);
	return;
}

void Test_Main()
{
	//BA_Test_1();
	//BA_Test_2();
	//Transform_Example_2D();	//二维下的旋转，位移变换

	//Sparse_Matrix_Test();	//稀疏矩阵实验，等于ICP_Test_4
	//ICP_Test_1();	//两图ICP BA方法
	//ICP_Test_2();	//两图ICP SVD方法
	//ICP_Test_3();	//两图ICP 实验，自己造数据
	//ICP_Test_4();	//闭环实验，仅估计位姿
	//ICP_Test_5();	//最简三图闭环实验

	//Camera_Extrinsic_Test_2();
	//Least_Square_Test_4();	//一阶梯度法
	//Least_Square_Test_5();	//二阶梯度法
	//Least_Square_Test_6();	//高斯牛顿法

	//E_Test_2();	//对E矩阵进行验算实验

	////4个Sift实验，各种接口场合
	//Sift_Test_1();
	//Sift_Test_2();
	//Sift_Test_3();
	//Sift_Test_4();

	//SVD_Test_1();		//作为SVD分解实验还是不错的
	//E_Test_2();		//这个例子非常简洁
	//Ransac_Test();	//Ransac实验

	//Camera_Param_Test();	//相机参数实验

	//Sphere_Test_1();	//4基站定位实验，二维方法
	//Sphere_Test_2();	//4基站定位实验，三维方法
}