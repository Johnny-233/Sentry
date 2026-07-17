#ifndef MATRIX_H
#define MATRIX_H

#include <math.h>
#include <main.h>


int pos2tranXYZ(float *Xpt, float *Tpt, int numX , int numT);
int invtran(float *Titi, float *Titf, int numI ,int numF);
int MatrixMultiply(float *A, float *B, int m, int p, int n, float *C,int numA ,int numB,int numC);
int MatrixTranspose(float *A, int m, int n, float *C,int numA ,int numB);
int MatrixCopy(float *A, int n, int m, float *B,int numA, int numB);

//void InverseK(float* plan_data->arm_pos, float* Jik);
//void MatrixPrint(float* A , int num);
//void ForwardK(float* Jfk, float* Xfk);

#endif
