#ifndef CHAIN_OPERATIONS_H
#define CHAIN_OPERATIONS_H

#include <Eigen/Dense>

#include "matrices.h"

using namespace Eigen;

struct joint
{
    Vector3d position;  // used for all
    Matrix3d triad;     // used for all
    Vector3d tangent;   // used for all except last one (but still calculate it because why not)
    Matrix3d R;         // used for all except first one (rotation matrix of triad when compared to the previous triad) (set to unity in first one)
    double E_stiffness; // used for all except first one (calculated based on R) (automatically becomes 0 in first one)
};

Vector3d tangent(Matrix3d triad, double a)
{
    Vector3d tangent = a * triad.col(2);
    return tangent;
}

void set_triad(joint chain[], int index, Matrix3d triad, double a)
{
    Vector3d tang = tangent(triad, a);
    chain[index].triad = triad;
    chain[index].tangent = tang;
}

Matrix3d reset_triad(Matrix3d triad) // Needs expansion to do the cross-product thing
{
    double D = triad.determinant();
    return pow(D,-1/3) * triad;
}

void reset_chain(joint chain[], int chain_length, double a)
{
    // Rescale triads & recalculate tangents:
    for (int index = 0; index < chain_length; ++index)
    {
        Matrix3d triad = reset_triad(chain[index].triad);
        set_triad(chain, index, triad, a); // Sets triads AND tangents
    }

    // Recalculate rotation matrices:
    chain[0].R = Matrix3d::Identity();
    for (int index = 1; index < chain_length; ++index)
    {
        chain[index].R = chain[index].triad * chain[index - 1].triad.inverse();
    }

    chain[0].position << 0, 0, 0;

    // Reconstruct positions based on tangents:
    for (int index = 1; index < chain_length; ++index)
    {
        joint previous_joint = chain[index - 1];
        Vector3d position_reset = previous_joint.position + previous_joint.tangent;
        chain[index].position = position_reset;
    }
}

Matrix3d starting_triad(int index, double a, double theta_0)
{
    double theta = index * theta_0;
    Vector3d Theta;
    Theta << 0, 0, theta;
    Matrix3d triad = rotation_matrix(Theta);
    return triad;
}

void initialize_triads_chain_straight(joint chain[], int chain_length, double a, double theta_0)
{
    for (int index = 0; index < chain_length; ++index)
    {
        //chain[index].position << 0, 0, a*index;
        chain[index].triad = starting_triad(index, a, theta_0);
        //chain[index].tangent = tangent(chain[index].triad);
    }
}

void initialize_chain(joint chain[], int chain_length, double a, double theta_0)
{
    cout << "Initializing chain" << endl;
    initialize_triads_chain_straight(chain, chain_length, a, theta_0);
    reset_chain(chain, chain_length, a);
}

void apply_pivot(joint chain[], int chain_length, int pivot_point, Matrix3d rotation_matrix, double a, Matrix3d new_R, double new_E_stiffness)
{
    Vector3d anchor = chain[pivot_point].position;

    for (int index = pivot_point + 1; index < chain_length; ++index)
    {
        // joint this_joint = chain[index];
        Vector3d unrotated_pos = chain[index].position;
        Vector3d rotated_pos = rotate_point_by_matrix(unrotated_pos, anchor, rotation_matrix);
        chain[index].position = rotated_pos;
        chain[index].triad = rotation_matrix * chain[index].triad;
        chain[index].tangent = tangent(chain[index].triad, a);
    }

    chain[pivot_point].R = new_R;
    chain[pivot_point].E_stiffness = new_E_stiffness;
}

Matrix3d correl_matrix(joint chain[], int chain_length)
{
    Matrix3d correl = Matrix3d::Zero();

    for (int index = 0; index < chain_length; ++index)
    {
        Vector3d Theta = rotation_vector_from_matrix(chain[index].R);
        Matrix3d correl_term = Theta * Theta.transpose();
        if (index == 10)
        {
            // cout << endl << correl_term << endl << endl;
        }
        correl += correl_term;
    }

    correl = correl / chain_length;

    return correl;
}



#endif // CHAIN_OPERATIONS_H

