#ifndef CALC_H
#define CALC_H

#include <Eigen/Dense>

#include "matrices.h"

using namespace Eigen;

Matrix3d Theta_correl(Vector3d Theta)
{
    return span_matrix(Theta);
}

#endif // CALC_H
