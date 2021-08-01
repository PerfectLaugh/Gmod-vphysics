#ifndef MISCMATH_H
#define MISCMATH_H

#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>

#define SAFE_DIVIDE(a, b) ((b) != 0 ? (a)/(b) : 0)

double AngDragIntegral(double invInertia, double l, double w, double h);
void BtMatrix_vimult(const btMatrix3x3 &matrix, const btVector3 &in, btVector3 &out);

#endif