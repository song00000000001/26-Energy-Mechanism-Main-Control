/**
 * @file statistics.h
 * @author qiquan.cui (860241578@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-09-24
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once

#if !defined(_STATISTICS_H_)
#define _STATISTICS_H_

#include "srml_config.h"

#if USE_SRML_STATISTICS

#ifdef __cplusplus

#include "cstdint"

typedef enum
{
    Ready,
    NotReady
} StatisticsReady;

template <uint16_t Length>
class Statistics
{
public:
    void DataRefresh(void);

    void DataInput(double input);

    bool isDataReady()
    {
        return data_ready;
    }

    StatisticsReady GetAverage(float* _average)
    {
        if (data_ready == 1)
        {
            *_average = average;
            return Ready;
        }

        *_average = 0;
        return NotReady;
    }

    StatisticsReady GetMeanSquare(float* _mean_square)
    {
        if (data_ready == 1)
        {
            *_mean_square = mean_square;
            return Ready;
        }

        *_mean_square = 0;
        return NotReady;
    }

    StatisticsReady GetVariance(float* _variance)
    {
        if (data_ready == 1)
        {
            *_variance = variance;
            return Ready;
        }

        *_variance = 0;
        return NotReady;
    }

private:
public:
    uint16_t cur_input_index = 0;
    bool data_ready = 0;

    float data_series[Length] = {};

    float data_sum = 0;    // 求和
    float average = 0;     // 均值
    double square_sum = 0; // 平方和
    float mean_square = 0; // 均方值
    float variance = 0;    // 方差
};

template <uint16_t Length>
void Statistics<Length>::DataRefresh(void)
{
    // cur_input_index = 0;
    // data_ready = 0;

    // data_sum = 0;
    // average = 0;
    // square_sum = 0;
    // mean_square = 0;
    // variance = 0;
    *this = Statistics<Length>();
}

template <uint16_t Length>
void Statistics<Length>::DataInput(double input)
{
    // 限制cur_input_index范围
    cur_input_index %= Length;

    // 如果数组已满，则需要将之前最旧的一个数据删除
    if (data_ready == 1)
    {
        double old_data = data_series[cur_input_index];

        data_sum -= old_data;
        square_sum -= old_data * old_data;
    }

    data_series[cur_input_index] = input;
    data_sum += input;
    square_sum += input * input;

    // 需要操作的数据序号
    cur_input_index++;

    // 数据就绪状态更新
    if (data_ready != 1)
    {
        if (cur_input_index >= Length)
        {
            data_ready = 1;
        }
    }

    // 如果数据已就绪，则可以输出
    if (data_ready == 1)
    {
        average = data_sum / Length;
        mean_square = square_sum / Length;
        variance = mean_square - average * average;
    }

    // 限制cur_input_index范围
    cur_input_index %= Length;
}

#endif

#endif // USE_SRML_STATISTICS

#endif // _STATISTICS_H_