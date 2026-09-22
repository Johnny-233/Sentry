/**
 ******************************************************************************
 * @file    matrix.cpp/h
 * @brief   矩阵/向量计算库（基于 CMSIS-DSP 加速）
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 * Modifications Copyright (c) 2026 by DT46
 ******************************************************************************
 * 
 * @attention
 * 本文件基于 Team JiaoLong-SJTU 的开源版本进行了全面重构与优化。
 * 原始版权归 Team JiaoLong-SJTU 所有，修改部分归修改者所有。
 * 感谢 上海交通大学 JiaoLong 团队的原版贡献者们。
 * 
 * --- 主要修改记录 (Modification Log) ---
 * 2026-08-27 : 重构 norm() 计算方式（从矩阵乘法 O(n^3) 改为点积 O(n^2)）；
 *              新增 dot() 接口，基于 arm_dot_prod_f32；
 *              修复 eye() 单位矩阵构造中的指针解引用错误；
 *              优化 zeros/ones/eye 工厂方法，消除冗余拷贝；
 *              添加 static_assert 编译期维度检查，增强类型安全；
 *              全局适配 CMSIS-DSP 规范，提升 SIMD 执行效率；
 *              删除矩阵乘法多余的友元，并加入了const；
 *              更详细的注释；
 *              一些细节改动以提高代码清晰度和维护性。
 *
 ******************************************************************************
 */

extern "C" {
#include "arm_math.h"
}

#pragma once

// 模板矩阵类：行数 _rows，列数 _cols（编译期常量）
template <uint16_t _rows, uint16_t _cols>
class Matrixf
{
    public:
    // ---------- 构造函数 ----------

    /**
     * @brief 无参构造：数据初始化为 0，并初始化 CMSIS-DSP 矩阵实例
     * @note  使用 constexpr 修饰，但在 C++20 之前调用 C 库函数并不真正编译期计算，此处仅为标记
     */
    Matrixf(void){ 
        static_assert(_rows > 0 && _cols > 0, "Matrix dimensions must be positive.");
        arm_mat_init_f32(&arm_mat_, _rows, _cols, data_); 
    }

    /**
     * @brief 从一维数组构造（数据按行主序存储）
     * @param data 长度为 _rows*_cols 的 float 数组
     */
    Matrixf(float data[_rows * _cols]) : Matrixf() {
        memcpy(data_, data, sizeof(float) * _rows * _cols);
    }

    /**
     * @brief 拷贝构造函数
     * @param mat 同类型的矩阵
     */
    Matrixf(const Matrixf<_rows, _cols> &mat) : Matrixf() {
        memcpy(data_, mat.data_, _rows * _cols * sizeof(float));
    }

    /**
     * @brief 析构函数（无特殊资源释放，所有数据在栈上）
     */
    ~Matrixf(void) = default;

    // ---------- 属性查询 ----------
    uint32_t rows(void) const { return _rows; }   // 行数
    uint32_t cols(void) const { return _cols; }   // 列数

    // ---------- 元素访问 ----------
    /**
     * @brief 重载下标运算符，返回第 row 行的指针（行主序）
     * @param row 行索引（从 0 开始）
     */
    float *operator[](const int &row) { return &data_[row * _cols]; }
    const float* operator[](const int &row) const { return &data_[row * _cols]; }

    // ---------- 赋值运算符 ----------
    /**
     * @brief 拷贝赋值
     * @param mat 右侧矩阵
     */
    Matrixf<_rows, _cols> &operator=(const Matrixf<_rows, _cols>& mat) {
        memcpy(data_, mat.data_, _rows * _cols * sizeof(float));
        return *this;
    }

    // ---------- 复合赋值运算符（原地运算） ----------
    /**
     * @brief 矩阵加法（自身 += 其他矩阵）
     * @note  使用 CMSIS-DSP 的 arm_mat_add_f32
     * @warning 输入和输出指针相同，CMSIS 标准实现支持原地操作，但某些编译优化可能产生别名问题，建议谨慎。
     */
    Matrixf<_rows, _cols> &operator+=(const Matrixf<_rows, _cols>& mat) {
        arm_mat_add_f32(&arm_mat_, &mat.arm_mat_, &arm_mat_);
        return *this;
    }

    /**
     * @brief 矩阵减法（自身 -= 其他矩阵）
     */
    Matrixf<_rows, _cols> &operator-=(const Matrixf<_rows, _cols>& mat) {
        arm_mat_sub_f32(&arm_mat_, &mat.arm_mat_, &arm_mat_);
        return *this;
    }

    /**
     * @brief 标量乘法（自身 *= 标量）
     */
    Matrixf<_rows, _cols> &operator*=(const float &val) {
        arm_mat_scale_f32(&arm_mat_, val, &arm_mat_);
        return *this;
    }

    /**
     * @brief 标量除法（自身 /= 标量）
     */
    Matrixf<_rows, _cols> &operator/=(const float &val) {
        arm_mat_scale_f32(&arm_mat_, 1.f / val, &arm_mat_);
        return *this;
    }

    // ---------- 二元运算符（返回新矩阵） ----------
    /**
     * @brief 矩阵加法（返回新矩阵）
     */
    Matrixf<_rows, _cols> operator+(const Matrixf<_rows, _cols> &mat) const {
        Matrixf<_rows, _cols> res;
        arm_mat_add_f32(&arm_mat_, &mat.arm_mat_, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 矩阵减法（返回新矩阵）
     */
    Matrixf<_rows, _cols> operator-(const Matrixf<_rows, _cols> &mat) const {
        Matrixf<_rows, _cols> res;
        arm_mat_sub_f32(&arm_mat_, &mat.arm_mat_, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 右标量乘（矩阵 * 标量）
     */
    Matrixf<_rows, _cols> operator*(const float &val) const {
        Matrixf<_rows, _cols> res;
        arm_mat_scale_f32(&arm_mat_, val, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 左标量乘（标量 * 矩阵），友元函数
     */
    friend Matrixf<_rows, _cols> operator*(const float &val, const Matrixf<_rows, _cols> &mat) {
        Matrixf<_rows, _cols> res;
        arm_mat_scale_f32(&mat.arm_mat_, val, &res.arm_mat_);
        return res;
    }

    
    /**
     * @brief 标量除法（矩阵 / 标量）
     */
    Matrixf<_rows, _cols> operator/(const float &val) const {
        Matrixf<_rows, _cols> res;
        arm_mat_scale_f32(&arm_mat_, 1.f / val, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 矩阵乘法（要求左列数 == 右行数）
     * @tparam cols2 右侧矩阵的列数
     * @note  返回值类型为 Matrixf<_rows, cols2>
     */
    template <uint16_t cols2>
    Matrixf<_rows, cols2> operator*(const Matrixf<_cols, cols2> &mat2) const {
        Matrixf<_rows, cols2> res;
        arm_mat_mult_f32(&arm_mat_, &mat2.arm_mat_, &res.arm_mat_);
        return res;
    }

    // ---------- 比较运算符 ----------
    /**
     * @brief 判断两个矩阵是否完全相等（逐元素比较，使用 `!=`，易受浮点误差影响）
     */
    bool operator==(const Matrixf<_rows, _cols> &mat) const {
        for (uint32_t i = 0; i < _rows * _cols; i++) {
            if (data_[i] != mat.data_[i])
                return false;
        }
        return true;
    }

    // ---------- 子矩阵操作 ----------
    /**
     * @brief 提取子矩阵（块）
     * @tparam rows   子矩阵行数
     * @tparam cols   子矩阵列数
     * @param start_row  起始行
     * @param start_col  起始列
     * @return 新的子矩阵
     */
    template <int rows, int cols>
    Matrixf<rows, cols> block(const int &start_row, const int &start_col) const {
        Matrixf<rows, cols> res;
        for (uint16_t row = start_row; row < start_row + rows; row++) {
            // 拷贝行数据
            memcpy(res[0] + (row - start_row) * cols, 
                   data_ + row * _cols + start_col, 
                   cols * sizeof(float));
        }
        return res;
    }

    /**
     * @brief 获取指定行（返回 1×_cols 矩阵）
     */
    Matrixf<1, _cols> row(const int &row) const { return block<1, _cols>(row, 0); }

    /**
     * @brief 获取指定列（返回 _rows×1 矩阵）
     */
    Matrixf<_rows, 1> col(const int &col) const { return block<_rows, 1>(0, col); }

    // ---------- 矩阵运算 ----------
    /**
     * @brief 转置
     */
    Matrixf<_cols, _rows> trans(void) const {
        Matrixf<_cols, _rows> res;
        arm_mat_trans_f32(&arm_mat_, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 迹（对角线元素之和）
     */
    float trace(void) const {
        float res = 0;
        for (uint16_t i = 0; i < min_dim(); i++) {
            res += (*this)[i][i];
        }
        return res;
    }

    
    /**
    * @brief 计算 Frobenius 内积（逐元素相乘后求和）
    * @param other 维度相同的矩阵
    * @return 内积标量值
    * @note  对于行向量或列向量，等价于标准点积
    */
    float dot(const Matrixf<_rows, _cols> &other) const {
        float32_t result = 0.0f;
        // 利用 CMSIS-DSP 计算两个数组的点积
        arm_dot_prod_f32(data_, other.data_, _rows * _cols, &result);
        return result;
    }

    /**
    * @brief 矩阵的欧几里得范数（Frobenius 范数）
    */
    float norm(void) const { 
        return sqrtf(dot(*this));  // 标准库
        // 或使用 CMSIS 的开方：float32_t r; arm_sqrt_f32(dot(*this), &r); return r;
    }

    /**
     * @brief 求逆（仅方阵）
     * @return 逆矩阵，若奇异则返回零矩阵
     */
    Matrixf<_cols, _rows> inv(void) const {
        // 编译期断言，非方阵无法求逆
        static_assert(_rows == _cols, "Matrix must be square to compute inverse");

        Matrixf<_cols, _rows> res;
        arm_status status = arm_mat_inverse_f32(&arm_mat_, &res.arm_mat_);

        if (status == ARM_MATH_SINGULAR)
            return Matrixf<_cols, _rows>::zeros();

        return res;
    }

    // ---------- 静态工厂方法 ----------
    /**
     * @brief 返回一个全零矩阵
     */
    static Matrixf<_rows, _cols> zeros(void) {
        return Matrixf<_rows, _cols>();
    }

    /**
     * @brief 返回一个全一矩阵
     */
    static Matrixf<_rows, _cols> ones(void) {
        Matrixf<_rows, _cols> mat;
        for (uint32_t i = 0; i < _rows * _cols; i++) {
            mat.data_[i] = 1;
        }
        return mat;
    }

    /**
     * @brief 返回单位矩阵（主对角线为 1，其余为 0）
     * @note  若行列不等，则只填充 min(_rows,_cols) 个 1，其余为 0（非方阵情况下也称为“广义单位阵”）
     */
    static Matrixf<_rows, _cols> eye(void) {
        Matrixf<_rows, _cols> mat;
        for (uint16_t i = 0; i < min_dim(); i++) {
            mat[i][i] = 1;
        }
        return mat;
    }

    /**
     * @brief 由列向量构造对角矩阵
     * @param vec 长度为 _rows 的列向量（若 _rows != _cols，只取前 min(_rows,_cols) 个）
     */
    static Matrixf<_rows, _cols> diag(Matrixf<_rows, 1> vec) {
        Matrixf<_rows, _cols> res = Matrixf<_rows, _cols>::zeros();
        for (uint16_t i = 0; i < min_dim(); i++) {
            res[i][i] = vec[i][0];
        }
        return res;
    }

    public:
    arm_matrix_instance_f32 arm_mat_;   // CMSIS-DSP 矩阵实例（包含行数、列数、数据指针）

    private:
    static constexpr uint16_t min_dim() { return (_rows < _cols) ? _rows : _cols; }

    protected:
    float data_[_rows * _cols] = {0};         // 矩阵数据（栈上分配）
};