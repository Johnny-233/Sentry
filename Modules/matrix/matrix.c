#include "matrix.h"


/**
 * @brief 将姿态(弧度制)变换到齐次变换矩阵
 * @note  输入的为XYZ顺次欧拉角，成功输出1，失败输出0;
 * @param Xpt : 前三个元素为坐标，后三个元素为三轴欧拉角
 * @param Tpt ：齐次变换矩阵
 * @param numX ：姿态数组的元素个数
 * @param numT : 齐次变换矩阵的元素个数
 */
int pos2tranXYZ(float *Xpt, float *Tpt, int numX , int numT)
{
  //判断元素个数是否合法
  if((numX != 6)||(numT != 16)) return 0;
  // pos to homogeneous transformation matrix
  // first row
  Tpt[0 * 4 + 0] = cos(Xpt[4])*cos(Xpt[5]);
  Tpt[0 * 4 + 1] = -cos(Xpt[4])*sin(Xpt[5]);
  Tpt[0 * 4 + 2] = sin(Xpt[4]);
  Tpt[0 * 4 + 3] = Xpt[0];
  // second row
  Tpt[1 * 4 + 0] = sin(Xpt[3]) * sin(Xpt[4]) * cos(Xpt[5]) + cos(Xpt[3]) * sin(Xpt[5]);
  Tpt[1 * 4 + 1] = -sin(Xpt[3]) * sin(Xpt[4]) * sin(Xpt[5]) + cos(Xpt[3]) * cos(Xpt[5]);
  Tpt[1 * 4 + 2] = -sin(Xpt[3]) * cos(Xpt[4]);
  Tpt[1 * 4 + 3] = Xpt[1];
  // third row
  Tpt[2 * 4 + 0] = -cos(Xpt[3])*sin(Xpt[4])*cos(Xpt[5]) + sin(Xpt[3])*sin(Xpt[5]);
  Tpt[2 * 4 + 1] = cos(Xpt[3])*sin(Xpt[4])*sin(Xpt[5]) + sin(Xpt[3])*cos(Xpt[5]);
  Tpt[2 * 4 + 2] = cos(Xpt[3])*cos(Xpt[4]);
  Tpt[2 * 4 + 3] = Xpt[2];
  // forth row
  Tpt[3 * 4 + 0] = 0.0;
  Tpt[3 * 4 + 1] = 0.0;
  Tpt[3 * 4 + 2] = 0.0;
  Tpt[3 * 4 + 3] = 1.0;
  return 1;
}


/**
 * @brief 求齐次变换矩阵的逆矩阵
 * @note  成功输出1，失败输出0;
 * @param Titi ：待求逆的齐次变换矩阵
 * @param Titf : 齐次变换矩阵的逆矩阵
 * @param numI : 待求逆的齐次变换矩阵的元素个数
 * @param numF ：齐次变换矩阵的逆矩阵的元素个数
 */
int invtran(float *Titi, float *Titf, int numI ,int numF)
{
  if((numI != 16)||(numF != 16)) return 0;
  // finding the inverse of the homogeneous transformation matrix
  // first row
  Titf[0 * 4 + 0] = Titi[0 * 4 + 0];
  Titf[0 * 4 + 1] = Titi[1 * 4 + 0];
  Titf[0 * 4 + 2] = Titi[2 * 4 + 0];
  Titf[0 * 4 + 3] = -Titi[0 * 4 + 0] * Titi[0 * 4 + 3] - Titi[1 * 4 + 0] * Titi[1 * 4 + 3] - Titi[2 * 4 + 0] * Titi[2 * 4 + 3];
  // second row
  Titf[1 * 4 + 0] = Titi[0 * 4 + 1];
  Titf[1 * 4 + 1] = Titi[1 * 4 + 1];
  Titf[1 * 4 + 2] = Titi[2 * 4 + 1];
  Titf[1 * 4 + 3] = -Titi[0 * 4 + 1] * Titi[0 * 4 + 3] - Titi[1 * 4 + 1] * Titi[1 * 4 + 3] - Titi[2 * 4 + 1] * Titi[2 * 4 + 3];
  // third row
  Titf[2 * 4 + 0] = Titi[0 * 4 + 2];
  Titf[2 * 4 + 1] = Titi[1 * 4 + 2];
  Titf[2 * 4 + 2] = Titi[2 * 4 + 2];
  Titf[2 * 4 + 3] = -Titi[0 * 4 + 2] * Titi[0 * 4 + 3] - Titi[1 * 4 + 2] * Titi[1 * 4 + 3] - Titi[2 * 4 + 2] * Titi[2 * 4 + 3];
  // forth row
  Titf[3 * 4 + 0] = 0.0;
  Titf[3 * 4 + 1] = 0.0;
  Titf[3 * 4 + 2] = 0.0;
  Titf[3 * 4 + 3] = 1.0;
  return 1;
}


/**
 * @brief 输入一个m*p的矩阵A，输入一个p*n的矩阵B，并将两个矩阵相乘得到C
 * @note  矩阵的乘法，成功输出1，失败输出0;
 * @param A : m*p的输入矩阵
 * @param B ：p*n的输入矩阵
 * @param m ：矩阵A的行数
 * @param p : 矩阵A的列数 = 矩阵B的行数
 * @param n ：矩阵B的列数
 * @param C ：m*n的输出矩阵
 * @param numA : 矩阵A的元素个数
 * @param numB : 矩阵B的元素个数
 * @param numC : 矩阵C的元素个数
 */
int MatrixMultiply(float *A, float *B, int m, int p, int n, float *C,int numA ,int numB,int numC)
{
  //判断元素是否合法
  if((numA != m*p)||(numB != p*n)||(numC != m*n)) return 0;
  int i, j, k;
  for (i = 0; i < m; i++)
    for (j = 0; j < n; j++)
    {
      C[n * i + j] = 0;
      for (k = 0; k < p; k++)
        C[n * i + j] = C[n * i + j] + A[p * i + k] * B[n * k + j];
    }
  return 1;
}

/**
 * @brief 输入一个m*n的矩阵A，将该矩阵进行转置并输出给矩阵C
 * @note  矩阵的转置,成功输出1，失败输出0
 * @param A : m*n的输入矩阵
 * @param m ：矩阵的行数
 * @param n ：矩阵的列数
 * @param C ：n*m的输出矩阵
 * @param numA : 矩阵A的元素个数
 * @param numB : 矩阵B的元素个数
 */
int MatrixTranspose(float *A, int m, int n, float *C,int numA ,int numB)
{
  //判断元素个数是否合法
  if((numA != m*n)||(numB != m*n)) return 0;
  int i, j;
  for (i = 0; i < m; i++)
    for (j = 0; j < n; j++)
      C[m * j + i] = A[n * i + j];
  return 1;
}

/**
 * @brief 输入一个m*n的矩阵A，并将该矩阵复制给B;
 * @note  矩阵的复制，成功输出1，失败输出0;
 * @param A : m*n的输入矩阵
 * @param B ：m*n的输出矩阵
 * @param m ：矩阵的行数
 * @param n ：矩阵的列数
 * @param numA : 矩阵A的元素个数
 * @param numB : 矩阵B的元素个数
 */
int MatrixCopy(float *A, int n, int m, float *B,int numA, int numB)
{
  //判断元素是否合法
  if((numA != m*n)||(numB != m*n)) return 0;
  int i, j;
  for (i = 0; i < m; i++)
    for (j = 0; j < n; j++)
    {
      B[n * i + j] = A[n * i + j];
    }
  return 1;
}