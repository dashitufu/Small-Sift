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
void SB_Slam_Test()
{
}
    
template<typename _T>int bTemp_Load_Data(char* pcFile, _T (**ppCamera)[4*4], int *piCamera_Count,_T **ppDepth)
{
    _T(*pCamera)[4 * 4]=NULL,*pDepth;
    int bRet = 1;
    char str[16];

    //char Value[256];
    FILE* pFile = fopen(pcFile, "rb");
    int iResult, iTemp, iCamera_Count = 0;
    int i;
    if (!pFile)
    {
        bRet = 0;
        goto END;
    }
    pCamera = (_T  (*)[4 * 4])pMalloc(&oMatrix_Mem, 1000 * 16 * sizeof(_T));
    if (!pCamera)
    {
        bRet = 0;
        goto END;
    }    
    union {
        _T Temp[7];
        double Temp_1[7];
        float Temp_2[7];
    };
        
    while (1)
    {        
        iResult = fscanf(pFile,"scene_%03d.png ", &iTemp);
        if (iResult<=0)
            break;
        
        char str[16];
        if (typeid(_T) == typeid(double))
            strcpy(str, "%lf ");
        else
            strcpy(str, "%f ");
        iResult = fscanf(pFile, str, &Temp[0]);
        iResult = fscanf(pFile, str, &Temp[1]);
        iResult = fscanf(pFile, str, &Temp[2]);
        iResult = fscanf(pFile, str, &Temp[4]);
        iResult = fscanf(pFile, str, &Temp[5]);
        iResult = fscanf(pFile, str, &Temp[6]);
        iResult = fscanf(pFile, str, &Temp[3]);

        TQ_2_Rt(Temp, pCamera[iCamera_Count++]);
        //Disp(pCamera[0], 4, 4);
    }
    fclose(pFile);

    Shrink(&oMatrix_Mem, pCamera, iCamera_Count * 16 * sizeof(_T));
    pDepth = (_T*)pMalloc(&oMatrix_Mem, 640 * 480 * sizeof(_T));
    if (typeid(_T) == typeid(double))
        strcpy(str, "%lf ");
    else
        strcpy(str, "%f ");

    pFile = fopen("D:\\Software\\3rdparty\\slambook2\\ch12\\test_data\\depthmaps\\scene_000.depth","rb");
    i = 0;
    for (int y = 0; y < 480; y++)
        for (int x = 0; x < 640; x++, i++)
        {
            _T fValue;
            fscanf(pFile, str, &fValue);
            pDepth[i] = fValue / 100.f;
        }
            

    *ppCamera = pCamera;
    *piCamera_Count = iCamera_Count;
    *ppDepth = pDepth;
END:

    return bRet;
}
const float fx = 481.2f;       // 相机内参
const float fy = -480.0f;
const float cx = 319.5f;
const float cy = 239.5f;

#define Project(x,y,Point) \
{ Point[0] = ((x) - cx) / fx;Point[1] = ((y) - cy) / fy; Point[2] = 1; }
#define Project_Inv(Point,x,y) \
{ \
    x = Point[0] * fx / Point[2] + cx; \
    y = Point[1] * fy / Point[2] + cy; \
}

#define Interpolate(oImage, x, y,fValue)    \
{   \
    int iPos = ((int)y) * oImage.m_iWidth + (int)x; \
    unsigned char* pd = &oImage.m_pChannel[0][iPos]; \
    _T xx = x - floor(x); \
    _T yy = y - floor(y); \
    fValue= _T(((1 - xx) * (1 - yy) * _T(pd[0]) + \
        xx * (1 - yy) * _T(pd[1]) + \
        (1 - xx) * yy * _T(pd[oImage.m_iWidth]) + \
        xx * yy * _T(pd[oImage.m_iWidth + 1])) / 255.f); \
}

template<typename _T>_T NCC(Image oRef, Image oCur, int x1, int y1, _T x2, _T y2)
{//未拆完，怀疑就是个块比较
    const int ncc_window_size = 3;
    const int ncc_area = (2 * ncc_window_size + 1) * (2 * ncc_window_size + 1);
    _T /*values_ref[ncc_area],*/ values_curr[ncc_area];
    int i,x, y, iPos;
    _T /*value_ref,*/ value_curr,/*mean_ref = 0,*/ mean_curr = 0;
    unsigned char values_ref[ncc_area],value_ref;
    union {
        unsigned int mean_ref_int = 0;
        _T mean_ref;
    };
     for (i=0,y = -ncc_window_size; y <= ncc_window_size; y++)
    {
        iPos = ((int)(y + y1)) * oRef.m_iWidth - ncc_window_size + (int)x1;
        for (x = -ncc_window_size; x <= ncc_window_size; x++,i++,iPos++)
        {
            //value_curr = fInterpolate(oCur, x2 + x, y2 + y);
            Interpolate(oCur, x2 + x, y2 + y, value_curr);

            value_ref = oRef.m_pChannel[0][iPos];
            mean_ref_int += value_ref;
            mean_curr += value_curr;

            values_ref[i] = value_ref;
            values_curr[i] = value_curr;
        }
    }

    mean_ref = (_T) mean_ref_int / (ncc_area*255.f) ;
    mean_curr /= ncc_area;

    _T numerator = 0, demoniator1 = 0, demoniator2 = 0;
    for (i = 0; i < ncc_area; i++)
    {
        _T diff_1 = values_ref[i] * (1.f/255.f) - mean_ref, diff_2 = values_curr[i] - mean_curr;
        _T n = diff_1 * diff_2;
        numerator += n;
        demoniator1 += diff_1 * diff_1;
        demoniator2 += diff_2 * diff_2;
    }
    return _T(numerator / sqrt(demoniator1 * demoniator2 + 1e-10));   // 防止分母出现零
}

template<typename _T>int epipolarSearch(Image oRef, Image oCur,
    int x,int y,_T fDepth, _T fDepth_Cov, _T Delta_T[4 * 4],_T pt_curr[2],_T epipolar_direction[2])
{
    _T f_ref[3],P_ref[4], px_mean_curr[4];
    Project(x, y, f_ref);
    Normalize(f_ref, 3, f_ref); //不知什么意思
    Matrix_Multiply(f_ref, 3, 1, fDepth, P_ref);
    //Disp(P_ref, 4, 1, "P_ref");
    P_ref[3] = 1;

    Matrix_Multiply(Delta_T,4,4, P_ref,1, px_mean_curr);
    Project_Inv(px_mean_curr, px_mean_curr[0], px_mean_curr[1]);
    //Disp(px_mean_curr, 2, 1, "px_mean_curr");
    _T d_min = fDepth - 3 * fDepth_Cov, d_max = fDepth + 3 * fDepth_Cov;
    if (d_min < 0.1f) 
        d_min = 0.1f;
    _T px_min_curr[4], px_max_curr[4];
    Matrix_Multiply(f_ref, 3, 1, d_min, px_min_curr);
    px_min_curr[3] = 1;
    Matrix_Multiply(Delta_T, 4, 4, px_min_curr, 1, px_min_curr);
    Project_Inv(px_min_curr, px_min_curr[0], px_min_curr[1]);

    Matrix_Multiply(f_ref, 3, 1, d_max, px_max_curr);
    px_max_curr[3] = 1;
    Matrix_Multiply(Delta_T, 4, 4, px_max_curr, 1, px_max_curr);
    Project_Inv(px_max_curr, px_max_curr[0], px_max_curr[1]);

    _T epipolar_line[2], half_length;
    Vector_Minus(px_max_curr, px_min_curr, 2, epipolar_line);
    Normalize(epipolar_line, 2, epipolar_direction);
    half_length= 0.5f*fGet_Mod(epipolar_line, 2);
    if (half_length > 100) 
        half_length = 100;
    //Disp(epipolar_line, 2, 1, "epipolar_line");

    _T best_ncc = -1.f, best_px_curr[2], px_curr[2],Temp[2],l;
    int i;
    for (i=0,l = -half_length; l <= half_length; l += 0.7f,i++)
    {
        Matrix_Multiply(epipolar_direction, 2, 1, l, Temp);
        Vector_Add(px_mean_curr,Temp,2, px_curr);
        //判断Temp点是否还在图的内框内
        if (!(px_curr[0] >= 20 && px_curr[0] < oRef.m_iWidth - 20 &&
            px_curr[1] >= 20 && px_curr[1] < oRef.m_iHeight - 20))
            continue;
        _T ncc = 0;
        ncc = NCC(oRef, oCur, x, y, px_curr[0], px_curr[1]);

        if (ncc > best_ncc) 
        {
            best_ncc = ncc;
            best_px_curr[0] = px_curr[0];
            best_px_curr[1] = px_curr[1];
        }
    }
    if (best_ncc < 0.85f)      // 只相信 NCC 很高的匹配
        return 0;
    pt_curr[0] = best_px_curr[0];
    pt_curr[1] = best_px_curr[1];
    return 1;
}
template<typename _T>int updateDepthFilter(_T x1, _T y1, _T x2, _T y2, _T Delta_T[4 * 4], _T epipolar_direction[2], _T Depth[], _T Depth_Cov[], int iWidth, int iHeight)
{
    _T T_Inv[4 * 4], f_ref[4], f_curr[4];
    int iResult;
    Get_Inv_Matrix_Row_Op_2(Delta_T, T_Inv, 4, &iResult);
    Project(x1, y1,f_ref);
    Normalize(f_ref, 3, f_ref);

    Project(x2, y2, f_curr);
    Normalize(f_curr, 3, f_curr);

    _T R[3 * 3], t[3], f2[3], b[2];
    union {
        _T A[2 * 2];
        _T A_Inv[2 * 2];
    };
    
    Get_R_t(T_Inv,R, t);
    Matrix_Multiply(R, 3, 3, f_curr, 1, f2);
    b[0] = fDot(t, f_ref, 3), b[1] = fDot(t, f2, 3);
    A[0*2+0] = fDot(f_ref, f_ref, 3);
    A[0*2+1] = -fDot(f_ref, f2,3);
    A[1 * 2 + 0] = -A[0 * 2 + 1];
    A[1 * 2 + 1] = -fDot(f2, f2, 3);
    
    _T ans[2], xm[3], xn[3], p_esti[3], depth_estimation, p[3],a[3];
    Get_Inv_Matrix_Row_Op_2(A, A_Inv, 2, &iResult);
    Matrix_Multiply(A_Inv, 2, 2, b, 1, ans);
    Matrix_Multiply(f_ref, 3, 1, ans[0], xm);
    
    Matrix_Multiply(f2, 3, 1, ans[1], xn);
    Vector_Add(xn, t, 3, xn);

    Vector_Add(xm, xn, 3, p_esti);
    Matrix_Multiply(p_esti, 3, 1, (_T)0.5f, p_esti);

    depth_estimation = fGet_Mod(p_esti, 3);
    Matrix_Multiply(f_ref, 3, 1, depth_estimation,p);
    Vector_Minus(p, t, 3, a);

    _T t_norm, a_norm, alpha, beta;
    t_norm = fGet_Mod(t, 3);
    a_norm = fGet_Mod(a, 3);
    alpha = acos(fDot(f_ref,t,3) / t_norm);
    beta = acos(-fDot(a,t,3) / (a_norm * t_norm));

    _T f_curr_prime[3];
    Project(x2+epipolar_direction[0], y2+ epipolar_direction[1], f_curr_prime);
    Normalize(f_curr_prime, 3, f_curr_prime);

    _T beta_prime, gamma, p_prime, d_cov, d_cov2;
    beta_prime = acos( -fDot(f_curr_prime,t,3) / t_norm);
    gamma = PI - alpha - beta_prime;
    p_prime = t_norm * sin(beta_prime) / sin(gamma);
    d_cov = p_prime - depth_estimation;
    d_cov2 = d_cov * d_cov;

    _T mu, sigma2, mu_fuse, sigma_fuse2;
    int iPos = ((int)y1) * iWidth + (int)x1;
    mu = Depth[iPos];
    sigma2 = Depth_Cov[iPos];
    mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2);
    sigma_fuse2 = (sigma2 * d_cov2) / (sigma2 + d_cov2);
    Depth[iPos] = mu_fuse;
    Depth_Cov[iPos] = sigma_fuse2;

    return 0;
}
template<typename _T>void Update(Image oRef, Image oCur,_T Depth[],_T Depth_Cov[], _T Delta_T[4 * 4])
{
    int x,y;
    _T pt_cur[2], epipolar_direction[2];
    for (x = 20; x < oRef.m_iWidth - 20; x++)
    {
        for (y = 20; y < oRef.m_iHeight - 20; y++)
        {
            if (!epipolarSearch(oRef, oCur, x, y, Depth[y * oRef.m_iWidth + x], sqrt(Depth_Cov[y * oRef.m_iWidth + x]), Delta_T, pt_cur,epipolar_direction))
                continue;

            updateDepthFilter((_T)x, (_T)y, pt_cur[0], pt_cur[1], Delta_T, epipolar_direction, Depth, Depth_Cov, oRef.m_iWidth, oRef.m_iHeight);
            //printf("here");
        }
    }
}

template<typename _T>void evaludateDepth(_T depth_truth[], _T depth_estimate[],int iWidth,int iHeight)
{
    int y, x,iPos, cnt_depth_data = 0;
    _T ave_depth_error = 0, ave_depth_error_sq = 0, error;
    for (y = 20; y <=iHeight-20; y++)
    {
        for (x = 20; x <=iWidth-20; x++)
        {
            iPos = y * iWidth + x;
            error = depth_truth[iPos] - depth_estimate[iPos];
            ave_depth_error += error;
            ave_depth_error_sq += error * error;
            cnt_depth_data++;
        }
    }
    ave_depth_error /= cnt_depth_data;
    ave_depth_error_sq /= cnt_depth_data;

    cout << "Average squared error = " << ave_depth_error_sq << ", average error: " << ave_depth_error << endl;
}

static void Test_1()
{//尝试搞歌单目稠密重建
    typedef double _T;
    
    int i, iResult,iCamera_Count;
    Image oImage_Ref,oImage_Cur;
    char File[256];
    _T (*pCamera)[4 * 4], *pRef_Depth;
    if (!bTemp_Load_Data((char*)"D:\\Software\\3rdparty\\slambook2\\ch12\\test_data\\first_200_frames_traj_over_table_input_sequence.txt", &pCamera, &iCamera_Count,&pRef_Depth))
        return;
    _T *pT_Ref, *pTi, Delta_T[4 * 4],Temp[4*4];
    _T* pDepth, * pDepth_Cov;
    pT_Ref = pCamera[0];
    bLoad_Image("c:\\tmp\\temp\\000.bmp",&oImage_Ref);
    pDepth = (_T*)pMalloc(&oMatrix_Mem, oImage_Ref.m_iWidth * oImage_Ref.m_iHeight * sizeof(_T));
    pDepth_Cov = (_T*)pMalloc(&oMatrix_Mem, oImage_Ref.m_iWidth * oImage_Ref.m_iHeight * sizeof(_T));
    for (i = 0; i < oImage_Ref.m_iWidth * oImage_Ref.m_iHeight; i++)
        pDepth[i] = pDepth_Cov[i] = 3.f;

    for (i = 1; i < 10; i++)
    {
        pTi = pCamera[i];
        //ΔT = Ti(-1) * Tref
        Get_Inv_Matrix_Row_Op_2(pTi, Temp, 4, &iResult);
        Matrix_Multiply(Temp,4,4, pT_Ref,4,Delta_T);
                
        //Disp(pT_Ref, 4, 4, "Ref");
        sprintf(File, "c:\\tmp\\temp\\%03d.bmp", i);
        if (!bLoad_Image(File, &oImage_Cur))
            continue;
        //Disp(Delta_T, 4, 4);
        Update<_T>(oImage_Ref, oImage_Cur,pDepth,pDepth_Cov,Delta_T);
        evaludateDepth(pRef_Depth, pDepth,oImage_Ref.m_iWidth, oImage_Ref.m_iHeight);

        Free_Image(&oImage_Cur);
        //printf("%d\n", i);
    }
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
