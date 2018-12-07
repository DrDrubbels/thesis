#ifndef MATRICES_H
#define MATRICES_H

#include <Eigen/Dense>

using namespace std;

using namespace Eigen;

Matrix3d cross_matrix(Vector3d axis)
{
    Matrix3d cross;
    cross << 0, -1 * axis[2], axis[1],
             axis[2], 0, -1 * axis[0],
             -1 * axis[1], axis[0], 0;
    return cross;
};

Matrix3d M(Vector3d Theta)
{
    return -1 * cross_matrix(Theta);
}

Matrix3d span_matrix(Vector3d v){
    Matrix3d span;
    span << v[0]*v[0], v[0]*v[1], v[0]*v[2],
            v[1]*v[0], v[1]*v[1], v[1]*v[2],
            v[2]*v[0], v[2]*v[1], v[2]*v[2];
    return span;
};

Matrix3d rotation_matrix_2(Vector3d axis, double angle)
{
    double s = sin(angle);
    double c = cos(angle);
    Matrix3d I = Matrix3d::Identity();
    return c * I + s * cross_matrix(axis) + (1-c) * span_matrix(axis);
};

Matrix3d rotation_matrix(Vector3d rotation_vector)
{
    double angle = rotation_vector.norm();
    Vector3d axis = rotation_vector.normalized();
    return rotation_matrix_2(axis, angle);
}

Vector3d rotation_vector_from_matrix(Matrix3d R)
{
    double c = (R.trace() - 1)/2;
    double s = sqrt(1 - pow(c,2));
    double theta = acos(c);
    Vector3d vec;
    vec << R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1);
    Vector3d Theta = theta/(2 * s) * vec;
    return Theta;
}

Vector3d relative_position(Vector3d abs_pos, Vector3d anchor)
{
    Vector3d rel_pos = abs_pos - anchor;
    return rel_pos;
};

Vector3d absolute_position(Vector3d rel_pos, Vector3d anchor)
{
    Vector3d abs_pos = rel_pos + anchor;
    return abs_pos;
};

Vector3d rotate_point_by_matrix(Vector3d abs_pos, Vector3d anchor, Matrix3d matrix)
{
    Vector3d rel_pos = relative_position(abs_pos, anchor);
    Vector3d rotated_rel_pos = matrix * rel_pos;
    Vector3d rotated_abs_pos = absolute_position(rotated_rel_pos, anchor);
    return rotated_abs_pos;
};

Vector3d rotate_vector_by_matrix(Vector3d current_vector, Matrix3d matrix)
{
    return matrix * current_vector;
}

Matrix3d stiffness_matrix(double A_1, double A_2, double C, double G)
{
    Matrix3d stiffness;
    stiffness << A_1, 0, 0,
                 0, A_2, G,
                 0, G, C;
    return stiffness;
}

string matrix_to_string(Matrix3d matrix)
{
    string out = "[";
    for (int row_index = 0; row_index < 3; ++row_index)
    {
        for (int col_index = 0; col_index < 3; ++col_index)
        {
            if (row_index != 0 || col_index != 0)
            {
                out += ", ";
            }
            out += to_string(matrix(row_index, col_index));
        }
    }
    out += "]";

    return out;
}

#endif // MATRICES_H


