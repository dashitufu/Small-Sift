// Slam_Test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#pragma once
#include <iostream>
#include "Common.h"
#include "Image.h"
#include "sift.h"
#include "Matrix.h"
#include "Reconstruct.h"

extern void Test_Main();

extern "C"
{
#include "Buddy_System.h"
}
//template<typename _T>void Temp_Load_Data(_T (**ppPoint_2D)[2], _T (**ppPoint_3D)[3], int* piCount)
//{
//    int iCount = (int)(iGet_File_Length((char*)"c:\\tmp\\1.bin") / (5 * sizeof(float)));
//    FILE* pFile = fopen("c:\\tmp\\1.bin", "rb");
//    _T(*pPoint_2D)[2], (*pPoint_3D)[3];
//    if (!pFile)
//    {
//        printf("Fail to open file\n");
//        return;
//    }
//    pPoint_2D = (_T(*)[2])malloc(iCount * 2 * sizeof(_T));
//    pPoint_3D = (_T(*)[3])malloc(iCount * 3 * sizeof(_T));
//    for (int i = 0; i < iCount; i++)
//    {
//        float Data[5];
//        fread(Data, 1, 5 * sizeof(float), pFile);
//        pPoint_2D[i][0] = Data[0];
//        pPoint_2D[i][1] = Data[1];
//        pPoint_3D[i][0] = Data[2];
//        pPoint_3D[i][1] = Data[3];
//        pPoint_3D[i][2] = Data[4];
//    }
//    fclose(pFile);
//    *ppPoint_2D = pPoint_2D;
//    *ppPoint_3D = pPoint_3D;
//    *piCount = iCount;
//    return;
//}

typedef struct Point_2D {
    unsigned int m_iCamera_Index;
    unsigned int m_iPoint_Index;
    float m_Pos[2];
}Point_2D;

void Temp_Load_File_2(int* piCameta_Count, int* piPoint_Count, int* piObservation_Count,
    Point_2D** ppPoint_2D, float(**ppPoint_3D)[3], float(**ppCamera)[3*3])
{//这次装入problem-16-22106-pre.txt。搞个好点的数据结构指出匹配关系

    int i, iCamera_Count, iPoint_Count, iObservation_Count;
    float(*pPoint_3D)[3], (*pCamera)[3*3];
    Point_2D* pPoint_2D;
    FILE* pFile = fopen("Sample\\problem-16-22106-pre.txt", "rb");
    fscanf(pFile, "%d %d %d\n", &iCamera_Count, &iPoint_Count, &iObservation_Count);
    pPoint_2D = (Point_2D*)malloc(iObservation_Count * 2 * sizeof(Point_2D));
    pPoint_3D = (float(*)[3])malloc(iPoint_Count * 3 * sizeof(float));
    pCamera = (float(*)[3*3])malloc(iCamera_Count * 16 * sizeof(float));

    for (i = 0; i < iObservation_Count; i++)
        fscanf(pFile, "%d %d %f %f", &pPoint_2D[i].m_iCamera_Index, &pPoint_2D[i].m_iPoint_Index, &pPoint_2D[i].m_Pos[0], &pPoint_2D[i].m_Pos[1]);
    for (i = 0; i < iCamera_Count; i++)
        for (int j = 0; j < 9; j++)
            fscanf(pFile, "%f ", &pCamera[i][j]);
    for (i = 0; i < iPoint_Count; i++)
        fscanf(pFile, "%f %f %f ", &pPoint_3D[i][0], &pPoint_3D[i][1], &pPoint_3D[i][2]);
    fclose(pFile);
    *piCameta_Count = iCamera_Count;
    *piPoint_Count = iPoint_Count;
    *piObservation_Count = iObservation_Count;
    *ppPoint_2D = pPoint_2D;
    *ppPoint_3D = pPoint_3D;
    *ppCamera = pCamera;
    return;
}
void BA_Test_3()
{
    int iCamera_Count, iPoint_Count, iObservation_Count;
    float(*pPoint_3D)[3], 
        (*pCamera)[3*3];    //只是个内参
    Point_2D* pPoint_2D;
    Temp_Load_File_2(&iCamera_Count, &iPoint_Count, &iObservation_Count, &pPoint_2D, &pPoint_3D,&pCamera);
}
#define Build_Link_Col(oC) \
{ \
	/*从最下一行向上建立Col Link*/ \
	int iItem_Index; \
	Sparse_Matrix<_T>::Item* poItem; \
	for (int i = oC.m_iRow_Count - 1; i >= 0; i--) \
	{ \
		if (oC.m_pRow[i]) \
		{ \
			iItem_Index = oC.m_pRow[i]; \
			poItem = &oC.m_pBuffer[iItem_Index]; \
			while (1) \
			{ \
				if (oC.m_pCol[poItem->x])/*此column下一行有东西*/ \
					poItem->m_iCol_Next = oC.m_pCol[poItem->x]; \
				oC.m_pCol[poItem->x] = iItem_Index; \
				if (poItem->m_iRow_Next) \
				{ \
					iItem_Index = poItem->m_iRow_Next; \
					poItem = &oC.m_pBuffer[poItem->m_iRow_Next]; \
				}else \
					break; \
			} \
		} \
	} \
}

template<typename _T>void Matrix_Add(Sparse_Matrix<_T> oA, Sparse_Matrix<_T> oB, Sparse_Matrix<_T>* poC)
{//此处必须做点假定，太随意很难做完备， C= A+B, 1， C假定已经足够空间。不够就失败退出
	//2，A必须和B有相同的长宽
	Sparse_Matrix<_T> oC;
	Sparse_Matrix<_T>::Item oNew_Item;

	if (oA.m_iRow_Count != oB.m_iRow_Count || oA.m_iCol_Count != oB.m_iCol_Count)
	{
		printf("Mis matched size in Matrix_Add\n");
		return;
	}
	Init_Sparse_Matrix(&oC, oA.m_iCur_Item + oB.m_iCur_Item, oA.m_iCol_Count, oA.m_iRow_Count);
	oC.m_iCur_Item = 1;
	int y, x;
	Sparse_Matrix<_T>::Item oB_Cur,oA_Cur;
	for (y = 0; y < oC.m_iRow_Count; y++)
	{
		if (oC.m_iCur_Item >= oC.m_iMax_Item_Count && (oA.m_pRow[y] || oB.m_pRow[y]))
		{
			printf("Exceed Item count in Matrix_Add\n");
			return;
		}
		if (oA.m_pRow[y] && oB.m_pRow[y])
		{
			oA_Cur = oA.m_pBuffer[oA.m_pRow[y]];
			oB_Cur = oB.m_pBuffer[oB.m_pRow[y]];
			oC.m_pRow[y]= oC.m_iCur_Item;
			while (1)
			{
				if (oA_Cur.x == oB_Cur.x)
				{//可以加
					oNew_Item = { oA_Cur.x,oA_Cur.y,oA_Cur.m_fValue+oB_Cur.m_fValue, oC.m_iCur_Item + 1 };
					if (oA_Cur.m_iRow_Next)
						oA_Cur = oA.m_pBuffer[oA_Cur.m_iRow_Next];
					else
						oA_Cur.x = 0xFFFFFFFF;
					if (oB_Cur.m_iRow_Next)
						oB_Cur = oB.m_pBuffer[oB_Cur.m_iRow_Next];
					else
						oB_Cur.x = 0xFFFFFFFF;
				}else if(oA_Cur.x < oB_Cur.x)
				{//写入A元素
					oNew_Item = { oA_Cur.x,oA_Cur.y,oA_Cur.m_fValue, oC.m_iCur_Item + 1 };
					if (oA_Cur.m_iRow_Next)
						oA_Cur = oA.m_pBuffer[oA_Cur.m_iRow_Next];
					else
						oA_Cur.x = 0xFFFFFFFF;
				}else if (oB_Cur.x < oA_Cur.x)
				{
					oNew_Item = { oB_Cur.x,oB_Cur.y,oB_Cur.m_fValue, oC.m_iCur_Item + 1 };
					if (oB_Cur.m_iRow_Next)
						oB_Cur = oB.m_pBuffer[oB_Cur.m_iRow_Next];
					else
						oB_Cur.x = 0xFFFFFFFF;
				}
				oC.m_pBuffer[oC.m_iCur_Item++] = oNew_Item;
				if (oA_Cur.x == 0xFFFFFFFF && oB_Cur.x == 0xFFFFFFFF)
				{
					oC.m_pBuffer[oC.m_iCur_Item - 1].m_iRow_Next = 0;
					break;
				}					
			}
		}else if (!oA.m_pRow[y] && oB.m_pRow[y])
		{//特殊情况，B整行加入到C
			oB_Cur = oB.m_pBuffer[oB.m_pRow[y]];
			oC.m_pRow[y] = oC.m_iCur_Item;
			while(1) 
			{
				if (oB_Cur.m_iRow_Next)
				{//还有下一个元素
					oNew_Item = { oB_Cur.x,oB_Cur.y,oB_Cur.m_fValue, oC.m_iCur_Item + 1 };
					oC.m_pBuffer[oC.m_iCur_Item++] = oNew_Item;
					oB_Cur = oB.m_pBuffer[oB_Cur.m_iRow_Next];
				}else
				{
					oNew_Item = { oB_Cur.x,oB_Cur.y,oB_Cur.m_fValue, 0 };
					oC.m_pBuffer[oC.m_iCur_Item++] = oNew_Item;
					break;
				}
			}
		}else if (!oB.m_pRow[y] && oA.m_pRow[y])
		{//A整行加入C
			oA_Cur = oA.m_pBuffer[oA.m_pRow[y]];
			oC.m_pRow[y] = oC.m_iCur_Item;
			while (1) {
				if (oA_Cur.m_iRow_Next)
				{//还有下一个元素
					oNew_Item = { oA_Cur.x,oA_Cur.y,oA_Cur.m_fValue, oC.m_iCur_Item + 1 };
					oC.m_pBuffer[oC.m_iCur_Item++] = oNew_Item;
					oA_Cur = oA.m_pBuffer[oA_Cur.m_iRow_Next];
				}else
				{
					oNew_Item = { oA_Cur.x,oA_Cur.y,oA_Cur.m_fValue, 0 };
					oC.m_pBuffer[oC.m_iCur_Item++] = oNew_Item;
					break;
				}
			}
		}
	}	
	Build_Link_Col(oC);
	if (poC->m_iMax_Item_Count)
		Free_Sparse_Matrix(poC);
	*poC = oC;
	return;
}
template<typename _T> void Get_Inv_Matrix_Row_Op(Sparse_Matrix<_T> oM, Sparse_Matrix<_T> *poInv, int* pbSuccess = NULL)
{//稀疏矩阵求逆，约定: poInv由此处分配内存，Caller负责最终释放
	Sparse_Matrix<_T> oAux;
	Init_Sparse_Matrix(&oAux, oM.m_iMax_Item_Count * 2, oM.m_iCol_Count * 2, oM.m_iRow_Count);
	
	//初始化值
	int y, x;
	Sparse_Matrix<_T>::Item oOrg_Item,oNew_Item;
	for (y = 0; y < oM.m_iRow_Count; y++)
	{
		if (oM.m_pRow[y])
		{//此行有值
			oAux.m_pRow[y] = oAux.m_iCur_Item;
			oOrg_Item = oM.m_pBuffer[oM.m_pRow[y]];
			while (1)
			{
				oAux.m_pBuffer[oAux.m_iCur_Item++] = { oOrg_Item.x,oOrg_Item.y,oOrg_Item.m_fValue,oAux.m_iCur_Item + 1 };
				if (oOrg_Item.m_iRow_Next)
					oOrg_Item = oM.m_pBuffer[oOrg_Item.m_iRow_Next];
				else
					break;
			}
			oOrg_Item = oM.m_pBuffer[oM.m_pRow[y]];
			while (1)
			{
				if (oOrg_Item.m_iRow_Next)
				{
					oAux.m_pBuffer[oAux.m_iCur_Item++] = { oOrg_Item.x+oM.m_iRow_Count,oOrg_Item.y,oOrg_Item.m_fValue,oAux.m_iCur_Item + 1 };
					oOrg_Item = oM.m_pBuffer[oOrg_Item.m_iRow_Next];
				}else
				{
					oAux.m_pBuffer[oAux.m_iCur_Item++] = { oOrg_Item.x,oOrg_Item.y,0 };
					break;
				}
			}
		}
	}
	Build_Link_Col(oAux);
	Disp(oAux);
	const _T eps = (_T)1e-10;
	//此处有点遗憾，没有用列主元法，因为不是解方程，而是将前半部分化成单位矩阵形式，所以还没想出一个很好的办法
	for (y = 0; y < oAux.m_iRow_Count; y++)
	{

	}
	return;
}
int bGet_Bit(unsigned char* pBuffer, int iBit_Pos)
{
	return pBuffer[iBit_Pos >> 3] & (1 << (iBit_Pos & 0x7));
}
void Set_Bit(unsigned char* pBuffer, int iBit_Pos)
{
	pBuffer[iBit_Pos >> 3] |= (1 << (iBit_Pos & 0x7));
}
template<typename _T>void Copy_Sparse_Matrix(Sparse_Matrix<_T> oA, Sparse_Matrix<_T>* poB)
{
	Sparse_Matrix<_T> oB = *poB;
	Sparse_Matrix<_T>::Item oOrg, * poNew;
	int y, x;
	for (y = 0; y < oA.m_iRow_Count; y++)
	{
		if (!oA.m_pRow[y])
			continue;
		oOrg = oA.m_pBuffer[oA.m_pRow[y]];
		while (1)
		{
			if (oOrg.m_fValue != 0)
			{
				if (!oB.m_pRow[y])
					oB.m_pRow[y] = oB.m_iCur_Item;
				oB.m_pBuffer[oB.m_iCur_Item++] = { oOrg.x, oOrg.y,oOrg.m_fValue,oB.m_iCur_Item + 1 };
			}
			if (oOrg.m_iRow_Next)
				oOrg = oA.m_pBuffer[oOrg.m_iRow_Next];
			else
				break;
		}
		if (oB.m_pRow[y])
			oB.m_pBuffer[oB.m_iCur_Item - 1].m_iRow_Next = 0;
	}
	Build_Link_Col(oB);
	*poB = oB;
}
template<typename _T>void Re_Arrange_Sparse_Matrix(Sparse_Matrix<_T> *poA)
{//重新整理一次Sparse_Matrix,删去0元素
	Sparse_Matrix<_T> oA = *poA, oB;
	Init_Sparse_Matrix(&oB, oA.m_iMax_Item_Count, oA.m_iCol_Count, oA.m_iRow_Count);
	Copy_Sparse_Matrix(oA, &oB);
	Free(&oMatrix_Mem,&oA);
	*poA = oB;
	return;
}
template<typename _T>void Solve_Linear_Gause(Sparse_Matrix<_T> oA, _T B[], _T X[], int* pbSuccess=NULL)
{//只有稀疏矩阵A是稀疏矩阵，其余用向量，简化
	typedef struct Reverse_Item {
		unsigned int m_iQ_Index:31;	//该行已经在Q里面的那个Index;
		unsigned int m_bDone:1;		//改行是否已经做完列主元的处理
	}Reverse_Item;
	int y, y1, x, x1, i, iRow_Size;
	int iMax, iTemp, iPos, * pQ,  bSuccess = 1;
	Reverse_Item* pQ_Reverse;
	//要做一个Q的Reverse look up table，否则不行，单一个pQ_Done不行
	_T fMax, * pfMax_Row, fValue;
	//这个标志数组用以标志某一行是否已经做完列主元的处理，稀疏矩阵能否引入列主元法靠他了
	//unsigned char* pQ_Done = (unsigned char*)pMalloc(&oMatrix_Mem, ((oA.m_iRow_Count + 7) >> 3));
	pQ = (int*)pMalloc(&oMatrix_Mem, oA.m_iRow_Count * sizeof(int));
	pQ_Reverse = (Reverse_Item*)pMalloc(&oMatrix_Mem, oA.m_iRow_Count * sizeof(Reverse_Item));

	Sparse_Matrix<_T> oA1;
	Sparse_Matrix<_T>::Item oItem, *poItem;
	Init_Sparse_Matrix(&oA1, oA.m_iMax_Item_Count + oA.m_iRow_Count, oA.m_iCol_Count + 1, oA.m_iRow_Count);
	if (/*!pQ_Done ||*/ !pQ || !oA1.m_pBuffer)
	{
		printf("Fail to allocate in Solve_Linear_Gause\n");
		goto END;
	}

	//将oA,B 抄到oA1中
	for (y = 0; y < oA1.m_iRow_Count; y++)
	{
		if (oA.m_pRow[y])
		{//有内容
			oItem = oA.m_pBuffer[oA.m_pRow[y]];
			oA1.m_pRow[y] = oA1.m_iCur_Item;
			while (1)
			{
				if (oItem.m_iRow_Next)
				{//还有
					oA1.m_pBuffer[oA1.m_iCur_Item++] = { oItem.x,oItem.y,oItem.m_fValue,oA1.m_iCur_Item + 1 };
					oItem = oA.m_pBuffer[oItem.m_iRow_Next];
				}else
				{
					oA1.m_pBuffer[oA1.m_iCur_Item++] = { oItem.x,oItem.y,oItem.m_fValue,0 };
					break;
				}
			}
		}

		//再把B[y]加进去
		if (B[y] != 0)
		{
			if (oA.m_pRow[y]) //上面加了行，加到行尾
				oA1.m_pBuffer[oA1.m_iCur_Item - 1].m_iRow_Next = oA1.m_iCur_Item;
			else //没有就自己加一行
				oA1.m_pRow[y] = oA1.m_iCur_Item;
			oA1.m_pBuffer[oA1.m_iCur_Item++] = { (unsigned int)oA.m_iRow_Count,(unsigned int)y,B[y] };
		}
		pQ[y] = y;	//每次主元所在的行
		pQ_Reverse[y] = { (unsigned int)y,0 };
	}
	Build_Link_Col(oA1);
	const _T eps = 1e-10;
	int iRank = 0;
	Sparse_Matrix<_T>::Item* pMax_Row, *poMax_Row_Item=NULL;
	for (x = 0; x < oA1.m_iCol_Count-1; x++)
	{
		if (!oA1.m_pCol[x])
		{
			printf("不满秩\n");
			bSuccess = 0;
			goto END;
		}
		oItem = oA1.m_pBuffer[oA1.m_pCol[x]];
		int iBit;
		fMax = 0;
		
		while(1)
		{
			//检测此行是否已经做完列主元判断，做完不算
			//iBit= bGet_Bit(pQ_Done, oItem.y);
			iBit = pQ_Reverse[oItem.y].m_bDone;
			if (!iBit)
			{
				if (abs(oItem.m_fValue) > abs(fMax))
				{
					fMax = oItem.m_fValue;
					iMax = oItem.y;
				}
			}
			if (oItem.m_iCol_Next)
				oItem = oA1.m_pBuffer[oItem.m_iCol_Next];
			else
				break;
		}
		if (abs(fMax) < eps)
		{
			printf("不满秩\n");
			bSuccess = 0;
			goto END;
		}
		
		//此时，iMax表示真实行，不是pQ索引行，所以要转换为pQ所在行
		iMax = pQ_Reverse[iMax].m_iQ_Index;
		y = x;	//注意，y只是pQ的位置，已经不是主元所在行
		//将最大元SWAP到Q的当前位置上		
		iTemp = pQ[y];
		pQ[y] = pQ[iMax];
		pQ[iMax] = iTemp;
		//Disp(pQ, 1, 3, "Q");
		//对Reverse Lookup table也调整
		pQ_Reverse[pQ[y]].m_iQ_Index = y;
		pQ_Reverse[pQ[iMax]].m_iQ_Index = iMax;
		iRank++;
		
		//对iMax所在的行进行系数计算，新系数/=A[y][y]
		poItem = &oA1.m_pBuffer[oA1.m_pRow[pQ[y]]];
		x1 = y;	//表示搞到哪一列, 以下不需要进行各种边界判断，如若访问错误，结构有错，错在前面
		while (poItem->x != x1)
			poItem = &oA1.m_pBuffer[poItem->m_iRow_Next];
		poItem->m_fValue = 1.f;
		pMax_Row = poItem= poItem->m_iRow_Next ? &oA1.m_pBuffer[poItem->m_iRow_Next] : NULL;
		while (poItem)
		{
			poItem->m_fValue /= fMax;
			poItem = poItem->m_iRow_Next ? &oA1.m_pBuffer[poItem->m_iRow_Next] : NULL;
		}
		//if (y == 1)
			//Disp(pQ,3,1, "Q");
		
		//对后面所有行代入
		for (i = y + 1; i < oA1.m_iRow_Count; i++)
		{//i表示第i行
			y1 = pQ[i];
			//对y1行的所有主元列以后进行修改
			poItem = &oA1.m_pBuffer[oA1.m_pRow[y1]];
			//移动到主元列
			while (poItem->x < x)
				poItem = poItem->m_iRow_Next?&oA1.m_pBuffer[poItem->m_iRow_Next]:NULL;
			if (!poItem || poItem->x != x)
				continue;
			//已经移动到主元列或之后
			fValue = poItem->m_fValue;
			if (fValue == 0)
				continue;
			poItem->m_fValue = 0.f;
			poItem = poItem->m_iRow_Next ? &oA1.m_pBuffer[poItem->m_iRow_Next] : NULL;
			poMax_Row_Item = pMax_Row;
			while (poMax_Row_Item)
			{
				//该行需要调整
				if (!poItem)
				{//将poItem所在列的对应行位置上调整，加元素
					Set_Value(&oA1, poMax_Row_Item->x, y1, -fValue * poMax_Row_Item->m_fValue);
					poMax_Row_Item = poMax_Row_Item->m_iRow_Next ? &oA1.m_pBuffer[poMax_Row_Item->m_iRow_Next] : NULL;
				}
				else if (poItem->x == poMax_Row_Item->x)
				{//更新，poItem 与 poMax_Row_Item都向右移动
					poItem->m_fValue -= fValue * poMax_Row_Item->m_fValue;
					poItem = poItem->m_iRow_Next ? &oA1.m_pBuffer[poItem->m_iRow_Next] : NULL;
					poMax_Row_Item = poMax_Row_Item->m_iRow_Next ? &oA1.m_pBuffer[poMax_Row_Item->m_iRow_Next] : NULL;
				}
				else if (poItem->x < poMax_Row_Item->x)
				{//同样将poItem所在列的对应行位置上调整，加元素，poItem向右移动
					Set_Value(&oA1, poMax_Row_Item->x, y1, -fValue * poMax_Row_Item->m_fValue);
					poItem = poItem->m_iRow_Next ? &oA1.m_pBuffer[poItem->m_iRow_Next] : NULL;
				}
				else if (poItem->x > poMax_Row_Item->x)
				{//同样将poItem所在列的对应行位置上调整，加元素，poMax_Row_Item向右移动
					Set_Value(&oA1, poMax_Row_Item->x, y1, -fValue * poMax_Row_Item->m_fValue);
					poMax_Row_Item = poMax_Row_Item->m_iRow_Next ? &oA1.m_pBuffer[poMax_Row_Item->m_iRow_Next] : NULL;
				}
			}
			//if (y == 1)
				//Disp(oA1, "A1");
		}
		//Set_Bit(pQ_Done, iMax);
		pQ_Reverse[pQ[y]].m_bDone = 1;
		//Disp(oA1, "A1");
	}
	
	//此处可以做一次整理，删除oA1中所有的非0fei'fei
	//Disp(oA1,"A1");
	Re_Arrange_Sparse_Matrix(&oA1);
	//Disp(oA1);
	Sparse_Matrix<_T>::Item* poB_Item;

	//回代，从Q[iOrder - 1]开始回代，从最下一行向上回代
	for (y = oA1.m_iRow_Count - 1; y >= 0; y--)
	{
		poItem =poB_Item= &oA1.m_pBuffer[oA1.m_pRow[pQ[y]]];
		while (poB_Item->m_iRow_Next)
			poB_Item = &oA1.m_pBuffer[poB_Item->m_iRow_Next];
		fValue = poB_Item->x == oA1.m_iCol_Count-1 ? poB_Item->m_fValue : 0;
		X[poItem->x] = fValue;
		if (fValue == 0)
			continue;
		//该行向上消元，定位到主元所在列
		poItem = &oA1.m_pBuffer[oA1.m_pCol[y]];
		while (1)
		{
			if (poItem->y != pQ[y])
			{
				//横着过去寻找B列
				if (poItem->m_iRow_Next)
				{
					poB_Item = &oA1.m_pBuffer[poItem->m_iRow_Next];
					while (poB_Item->m_iRow_Next)
						poB_Item = &oA1.m_pBuffer[poB_Item->m_iRow_Next];
					if (poB_Item->x == oA1.m_iCol_Count - 1)
						poB_Item->m_fValue += -poItem->m_fValue * fValue;
					else//加一个新Item
						Set_Value(&oA1, poItem->x, poItem->y, -poItem->m_fValue * fValue);
				}else
					Set_Value(&oA1, poItem->x, poItem->y, -poItem->m_fValue * fValue);
				poItem->m_fValue = 0;
			}
			if (poItem->m_iCol_Next)
				poItem = &oA1.m_pBuffer[poItem->m_iCol_Next];
			else
				break;
		}
		//Disp(oA1,"A1");
	}
	//Disp(oA1,"A1");
	//Disp(X, 1, oA1.m_iRow_Count, "X");
END:
	if (pQ)
		Free(&oMatrix_Mem, pQ);
	if (pQ_Reverse)
		Free(&oMatrix_Mem, pQ_Reverse);
	Free_Sparse_Matrix(&oA1);
	//if (pQ_Done)
		//Free(&oMatrix_Mem, pQ_Done);
	*pbSuccess = bSuccess;
	return;
}
template<typename _T>void Reset_Sparse_Matrix(Sparse_Matrix<_T> *poA)
{
	poA->m_iCur_Item = 1;
}
template<typename _T>void Dense_2_Sparse(_T A[], int m, int n, Sparse_Matrix<_T> *poA)
{//尽可能快将一般矩阵转换为稀疏矩阵格式，假定oA来之前已经初始化好空间
	int y, x, bLine_Add, iPos = 0;
	Sparse_Matrix<_T> oA = *poA;
	Reset_Sparse_Matrix(&oA);	//最简重置
	for (y = 0; y < m; y++)
	{
		for (bLine_Add = x = 0; x < n; x++,iPos++)
		{
			if (A[iPos] != 0)
			{
				if (!bLine_Add)
				{//尚未加行首
					oA.m_pRow[y] = oA.m_iCur_Item;
					bLine_Add = 1;
				}
				oA.m_pBuffer[oA.m_iCur_Item++] = {(unsigned int)x,(unsigned int)y,A[iPos],oA.m_iCur_Item+1};
			}			
		}
		if (bLine_Add)
			oA.m_pBuffer[oA.m_iCur_Item - 1].m_iRow_Next = 0;
	}
	Build_Link_Col(oA);
	Disp(oA);		
}
template<typename _T>void Sparse_2_Dense(Sparse_Matrix<_T> oA, _T B[])
{//将稀疏矩阵转换为稠密，常用于向量
	int i;
	Sparse_Matrix<_T>::Item oItem;
	if (oA.m_iRow_Count == 1)
	{//行向量
		if (oA.m_pRow[0])
		{
			oItem = oA.m_pBuffer[oA.m_pRow[0]];
			for (i = 0; i < oA.m_iCol_Count; i++)
			{
				if (oItem.x == i)
				{
					B[i] = oItem.m_fValue;
					if (oItem.m_iRow_Next)
						oItem = oA.m_pBuffer[oItem.m_iRow_Next];
					else
					{
						i++;
						break;
					}
				}else
					B[i] = 0;
			}
			memset(&B[i], 0, (oA.m_iCol_Count - i) * sizeof(_T));
		}
		else
			memset(B,0, oA.m_iCol_Count * sizeof(_T));
		
	}else if (oA.m_iCol_Count == 1)
	{//列向量
		if (oA.m_pCol[0])
		{
			oItem = oA.m_pBuffer[oA.m_pCol[0]];
			for (i = 0; i < oA.m_iRow_Count; i++)
			{
				if (oItem.y == i)
				{
					B[i] = oItem.m_fValue;
					if (oItem.m_iCol_Next)
						oItem = oA.m_pBuffer[oItem.m_iCol_Next];
					else
					{
						i++;
						break;
					}						
				}else
					B[i] = 0;				
			}
			memset(&B[i], 0, (oA.m_iRow_Count - i) * sizeof(_T));
		}else
			memset(B, 0, oA.m_iRow_Count * sizeof(_T));
	}else
		printf("Not implemented\n");
	return;
}
void Sparse_Matrix_Test()
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

	//T12: 相机2相对相机1的位姿  T13: 相机3相对于相机1的位姿，这两个是待求位姿
	//T23: 相机3相对于相机2的位姿，这个是中间位姿，用于求Δx
	_T T12[4 * 4], T13[4 * 4], T23[4 * 4],
		Delta_Pose[2][4 * 4], Pose_Pre[2][4 * 4],
		Jct[4 * 6], Jt[3 * 12], J[12 * 3], JEt[12];
	_T  Sigma_H[12 * 12], fSum_e, fSum_e_Pre = 1e10,
		E[3], H_Inv[12 * 12];

	_T Sigma_JEt[12], H[12 * 12], X[12];
	_T P_Temp[4]; //P1'
	Sparse_Matrix<_T> oSigma_JEt, oSigma_H;
	Sparse_Matrix<_T> oJ, oJt, oE, oJEt, oH, oH_Inv;
	int j, k, iIter;
	Gen_I_Matrix(T12, 4, 4);
	Gen_I_Matrix(T13, 4, 4);
	//Init_Sparse_Matrix(&oSigma_JEt, 12,1,12);
	Init_Sparse_Matrix(&oSigma_H, 12 * 12, 12,12);
	Init_Sparse_Matrix(&oJt, 3 * 12, 12,3);
	Init_Sparse_Matrix(&oJ, 12 * 3, 3, 12);
	Init_Sparse_Matrix(&oE, 3 * 1, 1,3);
	Init_Sparse_Matrix(&oJEt, 12 * 1, 1, 12);
	Init_Sparse_Matrix(&oH, 12 * 12, 12, 12);
	Disp_Mem(&oMatrix_Mem,0);
	for (iIter = 0;; iIter++)
	{
		fSum_e = 0;
		Get_Delta_Pose(T12, T13, T23);
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
			{
				for (k = 0; k < 6; k++)
					Set_Value(&oJt, k, j, Jct[j * 6 + k]);
				Set_Value(&oE, 0, j, E[j]);
			}
			Matrix_Multiply(oJt, (_T)-1.f);				//J'已经到位
			Matrix_Transpose_1(oJt,&oJ);
			Matrix_Multiply(oJ, oE, &oJEt);             //JE
			Matrix_Multiply(oJ, oJt, &oH);				//H=JJt
			Matrix_Add(oSigma_H, oH, &oSigma_H);		//
		}
		//Disp(oSigma_H,"Sigma_H");
		Sparse_2_Dense(oJEt, JEt);
		Solve_Linear_Gause(oSigma_H, JEt,X);

		printf("here");
	}

	Get_Delta_Pose(Pose_Pre[0], Pose_Pre[1], T23);
	Disp_Error(P1, P2, 100, Pose_Pre[0]);
	Disp_Error(P1, P3, 100, Pose_Pre[1]);
	Disp_Error(P2, P3, 100, T23);
	Free_Sparse_Matrix(&oSigma_JEt);
	Free_Sparse_Matrix(&oH);
	return;
}

//template<typename _T> void Get_Inv_Matrix_Row_Op_1(_T* pM, _T* pInv, int iOrder, int* pbSuccess = NULL)
//{//试一下列主元法，看看行不行
//	typedef struct Q_Item {
//		unsigned char m_iRow_Index;	//当前列对应的列主元所在行索引
//		unsigned char m_iCol_Index;	//			列主元对应的列索引，x索引
//	}Q_Item;
//
//	int y, x, x_1, i, bRet = 1, iRank = 0, iPos, iMax, iRow_Size = iOrder * 2;
//	Q_Item* Q, iTemp;
//	_T fValue, fMax, * pAux = NULL;
//	union {
//		_T* pfMax_Row;
//		_T* pfBottom_Row;
//		_T* pfCur_Row;
//	};
//	Q = (Q_Item*)pMalloc(&oMatrix_Mem, iOrder * sizeof(Q_Item));
//	pAux = (_T*)pMalloc(&oMatrix_Mem, iOrder * iOrder * 2 * sizeof(_T));
//	if (!Q || !pAux)
//	{
//		printf("Fail to malloc in Get_Inv_Matrix_Row_Op_1\n");
//		bRet = 0;
//		goto END;
//	}
//	//初始化值
//	for (y = 0; y < iOrder; y++)
//	{
//		iPos = y * iRow_Size;
//		for (x = 0; x < iOrder; x++)
//			pAux[iPos + x] = pM[y * iOrder + x];
//		iPos += iOrder;
//		for (x = 0; x < iRow_Size; x++)
//			pAux[iPos + x] = y == x ? 1.f : 0.f;
//	}
//	Disp(pAux, 3, 6, "Aux");
//	iPos = 0;
//	for (y = 0; y < iOrder; y++)
//		Q[y] = { (unsigned char)y };	//每次主元所在的行
//	for (x_1 = 0, y = 0; y < iOrder; y++)
//	{//这个方法y与x独立推进，各不相干
//		//if (y == 6)
//			//printf("here");
//		iMax = y;
//		fMax = pAux[Q[iMax].m_iRow_Index * iRow_Size + x_1];
//		for (i = y + 1; i < iOrder; i++)
//		{
//			if (abs(pAux[iPos = Q[i].m_iRow_Index * iRow_Size + x_1]) > abs(fMax))
//			{
//				fMax = pAux[iPos];
//				iMax = i;
//			}
//		}
//
//		if (abs(fMax) < ZERO_APPROCIATE)
//		{//列主元为0，显然不满秩，该方程没有唯一解
//			//Disp(A_1, m, n,"\n");
//			bRet = 0;
//			goto END;
//		}
//
//		//将最大元SWAP到Q的当前位置上
//		iTemp = Q[y];
//		Q[y] = Q[iMax];
//		Q[iMax] = iTemp;
//		Q[y].m_iCol_Index = x_1;
//		iRank++;
//
//		//Disp(A_1, m, n, "\n");
//		pfMax_Row = &pAux[Q[y].m_iRow_Index * iRow_Size];
//		pfMax_Row[x_1] = 1.f;
//		for (x = x_1 + 1; x < iRow_Size; x++)
//			pfMax_Row[x] /= fMax;
//		//Disp(A_1, m, n, "\n");
//
//		//对后面所有行代入
//		for (i = y + 1; i < iOrder; i++)
//			//for (i = 0; i < m; i++)
//		{//i表示第i行
//			iPos = Q[i].m_iRow_Index * iRow_Size;
//			if (((fValue = pAux[iPos + x_1]) != 0) && i != y)
//			{//对于对应元不为0才有算的意义
//				for (x = x_1; x < iRow_Size; x++)
//					pAux[iPos + x] -= fValue * pfMax_Row[x];
//				pAux[iPos + x_1] = 0;	//此处也不是必须的，置零只是好看
//			}
//			//Disp(Ai, iOrder, iOrder + 1, "\n");
//		}
//		//Disp(A_1, m, n, "\n");
//		x_1++;
//	}
//	Disp(pAux, iOrder, iRow_Size, "Aux");
//	for (y = 0; y < iOrder; y++)
//	{
//		iPos = Q[y].m_iRow_Index * iRow_Size + iOrder;
//		memcpy(&pInv[y * iOrder], &pAux[iPos], iOrder * sizeof(_T));
//	}
//	//Disp(pInv, 3, 3, "Inv");
//	
//END:
//	if (Q)
//		Free(&oMatrix_Mem, Q);
//	if (pAux)
//		Free(&oMatrix_Mem, pAux);
//	if (pbSuccess)
//		*pbSuccess = bRet;
//	return;
//}

void Gauss_Test()
{//稀疏矩阵解线性方程实验
	typedef float _T;
	//{ 2,3,1,1,-1,2,1,2,-1 }, B[3] = { 6,-1,5 };
	//{ 2,3,-1,3,-2,3,1,3,-2 }, B[3] = { 4,7,-1 };
	//{ 1,1,-1,2,3,1,1,-2,-1}, B[3] = {5,10,20};
	//{ 1,2,3,4,5,6,7,8,9 }, B[3] = { 10,11,12 };
	_T X[3],A[9] = { 1,2,0,2,3,4,5,6,7 },B[3] = { 1,2,3 };
	int iResult;
	Sparse_Matrix<_T> oA;
	//Elementary_Row_Operation_1(A,3, 3,A, &iResult);
	Solve_Linear_Gause(A, 3, B, X, &iResult);
	Disp(X, 3, 1, "X");
	Init_Sparse_Matrix(&oA, 9, 3, 3);
	Dense_2_Sparse(A, 3, 3, &oA);
	Solve_Linear_Gause(oA, B, X, &iResult);
	Disp(X, 3, 1, "X");
	
	/*Sparse_Matrix<_T>oX,oB1;
	Init_Sparse_Matrix(&oB1, 3, 1, 3);
	Init_Sparse_Matrix(&oX, 3, 1, 3);
	Dense_2_Sparse(X, 3, 1, &oX);
	Matrix_Multiply(oA, oX, &oB1);
	Disp(oB1, "B1");*/
	return;
}
void Sphere_Test_1()
{//先看看二元情况
	typedef float _T;
	_T Sphere_Center[4][3] = {	{ 200,300,400 },
								{ 300,400,410 } ,
								{ 350,300,420 } ,
								{ 200,400,430 } };

	_T R[4] = { 100,110,120,130 };
	Image oImage;
	int i;
	Init_Image(&oImage, 1000, 1000, Image::IMAGE_TYPE_BMP, 8);
	Set_Color(oImage);
	for(i=0;i<4;i++)
		Draw_Arc(oImage, R[i], Sphere_Center[i][0], Sphere_Center[i][1]);
	
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
		//	//∂e/∂x= (-2x, -2y)	,第一步顺着这个梯度走，能走到目的地
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
		//	//∂e/∂x = - [(x-a)^2 + (x-b)^2] *(x-a)
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
			float fDist_Sqr = (P[0] - Sphere_Center[i][0]) * (P[0] - Sphere_Center[i][0]) +
				(P[1] - Sphere_Center[i][1]) * (P[1] - Sphere_Center[i][1]);
			if (fDist_Sqr == 0)
				continue;
			e = sqrt(fDist_Sqr);
			fSum_e += e;
			//∂e/∂x = - [(x-a)^2 + (x-b)^2] *(x-a)
			Jt[0] = pow(fDist_Sqr, -0.5) * (P[0] - Sphere_Center[i][0]);
			Jt[1] = pow(fDist_Sqr, -0.5) * (P[1] - Sphere_Center[i][1]);

			memcpy(J, Jt, 2 * sizeof(_T));
			Matrix_Multiply(J, 2, 1, Jt, 2, JJt);
			Matrix_Multiply(J, 2, 1, e, Je);
			//	//累加
			Matrix_Add(Sigma_H, JJt, 2, Sigma_H);
			Vector_Add(Sigma_Je, Je, 2, Sigma_Je);
		}
		
		Sigma_H[0] += 1, Sigma_H[3] += 1;
				
		Get_Inv_Matrix_Row_Op(Sigma_H, H_Inv,2,&iResult);
		Matrix_Multiply(H_Inv, 2, 2, Sigma_Je, 1, Delta_X);
		Matrix_Multiply(Delta_X, 2, 1,(_T)- 1.f, Delta_X);
		
		if ((fSum_e_Pre <= fSum_e && iIter > 0) || !iResult)
			break;
		printf("Iter:%d Error:%f\n", iIter, fSum_e);
		//Draw_Point(oImage, P[0], P[1],2);

		Pre_P[0] = (P[0] += Delta_X[0]);
		Pre_P[1] = (P[1] += Delta_X[1]);		
		fSum_e_Pre = fSum_e;
	}
	Draw_Point(oImage, Pre_P[0], Pre_P[1], 2);
	bSave_Image("c:\\tmp\\1.bmp", oImage);
	Free_Image(&oImage);
	return;
}
void Test_1()
{
	float A[4 * 2] = { 160001,240000,
						240000,360001,
						360001,480000,
						480000,640001 };
	float B[4] = { 48000000,
					72000000,
					142740000,
					190320000 };
	float X[4];
	int iResult;
	Solve_Linear_Contradictory(A, 4, 2, B, X, &iResult);
	return;
}

int main()
{
    Init_Env();
	Test_Main();
	
    Free_Env();
#ifdef WIN32
    _CrtDumpMemoryLeaks();
#endif
}
