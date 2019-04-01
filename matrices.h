#ifndef MATRICES_H
#define MATRICES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "constants.h"
#include "random.h"

using namespace std;
using namespace Eigen;

Vector3d rand_vec_uniform(double halfwidth)
{
    double p = halfwidth;
    double m = -1 * halfwidth;
    Vector3d vec;
    vec << rand_double(m,p), rand_double(m,p), rand_double(m,p);
    return vec;
}

Vector3d rand_vec_normal(double sigma)
{
    Vector3d vec;
    vec << rand_normal(0.0, sigma), rand_normal(0.0, sigma), rand_normal(0.0, sigma);
    return vec;
}

Vector3d rand_vec(double sigma)
{
    return rand_vec_uniform(sigma);
}

Matrix3d rand_matrix(double sigma)  // Body frame
{
    Vector3d vec = rand_vec(sigma);
    double angle = vec.norm();
    vec.normalize();
    Matrix3d R;
    R = AngleAxisd(angle, vec);
    return R;
}

Vector3d rot_vec_from_matrix(Matrix3d R)
{
    Vector3d Theta;
    if (R.isIdentity(PRECISION))
    {
        Theta << 0, 0, 0;
    }
    else
    {
        double c = (R.trace() - 1)/2;
        double s = sqrt(1 - pow(c,2));
        double theta = acos(c);
        Vector3d vec;
        vec << R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1);
        Theta = theta/(2 * s) * vec;
    }
    return Theta;
}

Matrix3d rot_matrix_from_vector(Vector3d rot_vector)
{
    double norm = rot_vector.norm();
    rot_vector = rot_vector.normalized();
    Matrix3d R;
    R = AngleAxisd(norm, rot_vector);
    return R;
}

Matrix3d fix_triad(Matrix3d old_triad)
{
    Vector3d e_1 = old_triad.col(0);
    Vector3d e_2 = old_triad.col(1);
    Vector3d e_3 = old_triad.col(2);

    e_1 = e_2.cross(e_3);
    e_2 = e_3.cross(e_1);
    e_1.normalize();
    e_2.normalize();
    e_3.normalize();

    Matrix3d new_triad;

    new_triad.col(0) = e_1;
    new_triad.col(1) = e_2;
    new_triad.col(2) = e_3;

    return new_triad;
}

string string_from_vector(Vector3d vec)
{
    string out = "";
    for (int index = 0; index < 3; ++index)
    {
        out += "\t" + to_string(vec[index]);
    }
    return out;
}

string string_from_matrix(Matrix3d mat)
{
    string out = "[";

    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            if (!(row == 0 && col == 0))
            {
                out += ",";
            }
            out += to_string(mat(row,col));
        }
    }

    out += "]";

    return out;
}

#endif // MATRICES_H
