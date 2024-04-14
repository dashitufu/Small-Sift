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

void Test_1()
{
    float A[] = { 1,	1,	1,
                1	,2 ,-3 ,
    -6 ,-3	,2 }, B[] = { 0,0,1 },X[3];
    int iResult;
    Solve_Linear_Gause(A, 3,B, X, &iResult);
    Disp(X, 3, 1, "X");
    
    return;
}
int main()
{
    Init_Env();
    //Test_1();
	Test_Main();
    Free_Env();
#ifdef WIN32
    _CrtDumpMemoryLeaks();
#endif
}
