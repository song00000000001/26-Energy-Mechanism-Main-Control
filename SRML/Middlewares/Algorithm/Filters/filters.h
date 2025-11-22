/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    filter.h
 * @author  buff buffdemail@163.com
 * @brief   Filter set in general signal process and analysis.
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#ifndef _FILTER_H
#define _FILTER_H

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus

#include "srml_std_lib.h"
#include <algorithm>
/* Exported function declarations --------------------------------------------*/
/* LowPassFilter */
class LowPassFilter
{
public:
  /**
    @brief trust (0,1)
   */
  LowPassFilter(float trust = 1) :
    Trust(trust)
  {
    now_num = 0;
    last_out = 0;
  }
  ~LowPassFilter(){};
  float Trust;
  void operator<<(const float&);
  void operator>>(float&);
  float f(float num);

protected:
  void in(float num);
  float out();

private:
  float now_num;
  float last_out;
};

/* MedianFilter	*/
template <int Length>
class MedianFilter
{
  /**
    @brief 滤波宽度(1,100)
   */
public:
  MedianFilter()
  {
    static_assert((Length > 0) && (Length < 101), "MedianFilter Length [1,100]");
    flag = Length;
    where_num = 0;
  }
  ~MedianFilter(){};
  void operator>>(float& num)
  {
    num = out();
  }
  void operator<<(const float& num)
  {
    in(num);
  }
  float f(float num)
  {
    in(num);
    return (out());
  }

protected:
  void in(float num)
  {
    now_num = num;
    /* flag=Length然后递减保证宽度内都是有效波值 */
    flag > 0 ? flag-- : 0;
    buffer_num[where_num++] = num;
    where_num %= Length;
  }

  float out()
  {
    if (flag > 0)
      return now_num;
    else
    {
      /* 准备排序 */
      memcpy(sort_num, buffer_num, sizeof(sort_num));
      std::sort(sort_num, sort_num + Length);
      return sort_num[int(Length / 2)];
    }
  }

private:
  float buffer_num[Length];
  float sort_num[Length];
  float now_num;
  int flag, where_num;
};

/* MeanFilter */
template <int Length>
class MeanFilter
{
public:
  /**
    @brief 滤波宽度(1,100)
   */
  MeanFilter()
  {
    static_assert((Length > 0) && (Length < 101), "MedianFilter Length [1,100]");
    for (int x = 0; x < Length; x++) buffer_num[x] = 0;
    flag = Length;
    where_num = 0;
    sum = 0;
  }
  ~MeanFilter(){};
  void operator>>(float& num)
  {
    num = out();
  }
  void operator<<(const float& num)
  {
    in(num);
  }
  float f(float num)
  {
    in(num);
    return (out());
  }

protected:
  void in(float num)
  {
    now_num = num;
    sum -= buffer_num[where_num]; /*<! sum减去旧值 */
    sum += num;                   /*<! sum加上新值 */
    buffer_num[where_num++] = num;
    flag > 0 ? flag-- : 0; /*<!flag=Length然后递减保证宽度内都是有效波值 */
    where_num %= Length;
  }

  float out()
  {
    if (flag > 0)
      return now_num;
    else
      return (sum / Length);
  }

private:
  float buffer_num[Length];
  float now_num;
  float sum; /*<! 宽度和数字和 */
  int flag, where_num;
};

namespace SRML_Filter
{
    /**
     * @brief 任意阶数滤波器基类
     * 
     * @tparam order 滤波器阶数
     */
    template <const uint8_t order>
    class filter_base
    {
    protected:
        float a[order + 1];
        float b[order + 1];
        float x[order + 1] = {};
        float y[order + 1] = {};

    public:
        filter_base(/* args */)
        {
            static_assert((order > 0) && (order < 101), "Filter Order [1,100]");
        };

        /**
         * @brief 滤波函数
         *
         * @param x0 输入信号
         * @return float 输出信号
         */
        float f(float x0)
        {
            x[0] = x0;
            y[0] = b[0] * x[0];

            for (int i = 1; i < (order + 1); i++)
            {
                y[0] += b[i] * x[i];
                y[0] -= a[i] * y[i];
            }

            for (int i = order; i > 0; i--)
            {
                x[i] = x[i - 1];
                y[i] = y[i - 1];
            }

            return y[0];
        }

        /**
         * @brief Reset the filter state
         *
         */
        void reset()
        {
            memset(x, 0, sizeof(x));
            memset(y, 0, sizeof(y));
        }
    };

    inline void cal1stOrderLPFCoeffs(float cutoff_freq, float sampling_freq, float _zeta, float a[2], float b[2])
    {
        float wd = tanf(PI * cutoff_freq / (sampling_freq));
        float temp = wd + 1;

        b[1] = b[0] = wd / temp;

        a[0] = 1;
        a[1] = (wd - 1) / temp;
    }

    inline void cal2ndOrderLPFCoeffs(float cutoff_freq, float sampling_freq, float _zeta, float a[3], float b[3])
    {
        float wd = tanf(PI * cutoff_freq / (sampling_freq));
        float temp = 2 * _zeta * wd;
        float wd2 = wd * wd;

        b[2] = b[0] = wd2 / (wd2 + temp + 1.f);
        b[1] = 2.f * b[0];

        a[0] = 1;
        a[1] = 2.f * (wd2 - 1.f) / (wd2 + temp + 1.f);
        a[2] = (wd2 - temp + 1.f) / (wd2 + temp + 1.f);
    }

    inline void cal3rdOrderLPFCoeffs(float cutoff_freq, float sampling_freq, float _zeta, float* a, float* b)
    {
        // 计算一阶低通的系数
        float a1[2], b1[2];
        cal1stOrderLPFCoeffs(cutoff_freq, sampling_freq, _zeta, a1, b1);
        // 计算二阶低通的系数
        float a2[3], b2[3];
        cal2ndOrderLPFCoeffs(cutoff_freq, sampling_freq, _zeta, a2, b2);
        // 计算三阶低通的系数
        b[0] = b1[0] * b2[0];
        b[1] = b1[1] * b2[0] + b1[0] * b2[1];
        b[2] = b1[1] * b2[1] + b1[0] * b2[2];
        b[3] = b1[1] * b2[2];

        a[0] = 1;
        a[1] = a1[1] + a2[1];
        a[2] = a1[1] * a2[1] + a2[2];
        a[3] = a1[1] * a2[2];
    }

    inline void cal2ndOrderHPFCoeffs(float cutoff_freq, float sampling_freq, float _zeta, float a[3], float b[3])
    {
        float wd = tanf(PI * cutoff_freq / (sampling_freq));
        float temp = 2 * _zeta * wd;
        float wd2 = wd * wd;

        b[2] = b[0] = (1) / (wd2 + temp + 1.f);
        b[1] = -2.f * b[0];

        a[0] = 1;
        a[1] = 2.f * (wd2 - 1.f) / (wd2 + temp + 1.f);
        a[2] = (wd2 - temp + 1.f) / (wd2 + temp + 1.f);
    }

    inline void cal2ndOrderBSFCoeffs(float stop_freq, float sampling_freq, float depth, float width, float a[3], float b[3])
    {
        float wd = tanf(PI * stop_freq / (sampling_freq));
        float wd2 = wd * wd;
        float wc = 2 * PI * stop_freq;
        float B = 2 * PI * width;
        float zeta1 = sqrt((1 - sqrt((B * B) / (wc * wc) + 1)) / (4 * depth * depth - 2));
        float zeta2 = depth * zeta1;

        float temp = wd2 + 2 * zeta1 * wd + 1.f;
        a[0] = 1;
        a[1] = (2 * wd * wd - 2) / temp;
        a[2] = (1 - 2 * zeta1 * wd + wd2) / temp;

        b[0] = (1 + 2 * zeta2 * wd + wd2) / temp;
        b[1] = a[1];
        b[2] = (1 - 2 * zeta2 * wd + wd2) / temp;
    }
}

#endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
