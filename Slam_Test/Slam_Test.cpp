// Slam_Test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>
#include "Common.h"
#include "Image.h"
#include "sift.h"

extern "C"
{
#include "Buddy_System.h"
}

void Sift_Example();

int main()
{
    Sift_Example();
#ifdef WIN32
    _CrtDumpMemoryLeaks();
#endif
}
