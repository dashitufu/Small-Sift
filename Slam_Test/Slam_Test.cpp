// Slam_Test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#pragma once
#include <iostream>
#include "Common.h"
#include "Image.h"
//#include "sift.h"
#include "Matrix.h"
#include "Reconstruct.h"

extern void Test_Main();
extern "C"
{
#include "Buddy_System.h"
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
