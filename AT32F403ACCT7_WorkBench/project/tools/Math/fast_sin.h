//
// Created by czt on 2023/1/30.
//
// 快速三角函数库
// 使用 Remez 算法拟合的 5 阶多项式近似，比标准 math.h 的 sin/cos 快数倍，
// 精度约 1e-8，满足 FOC 控制需求。
//
// 参考：https://github.com/samhocevar/lolremez/wiki/Tutorial-4-of-5:-fixing-lower-order-parameters
//

#ifndef FAST_SIN_H
#define FAST_SIN_H

#define M_PI (3.1415926f)

/**
 * @brief 多项式 f1(x)：用于 sin 的泰勒展开校正
 * 拟合范围 [1e-50, pi²]，最大误差 ~1.5e-9
 * 服务于 fast_sin: sin(x) = x + x³ * f1(x²)
 */
static inline float f1(float x)
{
    float u = 1.3528548e-10f;
    u = u * x + -2.4703144e-08f;
    u = u * x + 2.7532926e-06f;
    u = u * x + -0.00019840381f;
    u = u * x + 0.0083333179f;
    return u * x + -0.16666666f;
}

/**
 * @brief 多项式 f2(x)：用于 cos 的泰勒展开校正
 * 拟合范围 [1e-50, pi²]，最大误差 ~1.2e-8
 * 服务于 fast_cos: cos(x) = 1 + x² * f2(x²)
 */
static inline float f2(float x)
{
    float u = 1.7290616e-09f;
    u = u * x + -2.7093486e-07f;
    u = u * x + 2.4771643e-05f;
    u = u * x + -0.0013887906f;
    u = u * x + 0.041666519f;
    return u * x + -0.49999991f;
}

/**
 * @brief 快速正弦函数
 * 使用多项式近似，无分支预测惩罚。
 */
static inline float fast_sin(float x)
{
    int si = (int)(x * 0.31830988f);  // 1/pi
    x = x - (float)si * M_PI;
    if (si & 1) {
        x = x > 0.0f ? x - M_PI : x + M_PI;
    }
    return x + x * x * x * f1(x * x);
}

/**
 * @brief 快速余弦函数
 */
static inline float fast_cos(float x)
{
    int si = (int)(x * 0.31830988f);
    x = x - (float)si * M_PI;
    if (si & 1) {
        x = x > 0.0f ? x - M_PI : x + M_PI;
    }
    return 1.0f + x * x * f2(x * x);
}

/**
 * @brief 同时计算正弦和余弦（比分别调用快一倍）
 */
static inline void fast_sin_cos(float x, float *sin_x, float *cos_x)
{
    int si = (int)(x * 0.31830988f);
    x = x - (float)si * M_PI;
    if (si & 1) {
        x = x > 0.0f ? x - M_PI : x + M_PI;
    }
    *sin_x = x + x * x * x * f1(x * x);
    *cos_x = 1.0f + x * x * f2(x * x);
}

#endif /* FAST_SIN_H */
