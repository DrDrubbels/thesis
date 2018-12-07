#ifndef ENERGY_H
#define ENERGY_H

#include <Eigen/Dense>

#include "chain_operations.h"
#include "matrices.h"

using namespace Eigen;

double extension_energy_length(double length, double F)
{
    return -1 * F * length;
}

double extension_energy_chain(joint chain[], int chain_length, double F)
{
    double z_pos = chain[chain_length - 1].position[2];
    return extension_energy_length(z_pos, F);
}

double joint_energy(Vector3d Theta, Matrix3d stiffness, double a, double beta, double theta_0)
{
    Vector3d Theta_col = Theta;
    // cout << "Theta column (pre-subtraction):" << Theta_col << endl << endl;
    Theta_col[2] -= theta_0;
    RowVector3d Theta_row = Theta_col.transpose();
    // cout << "Theta row:" << Theta_row << endl << endl;
    double product = Theta_row * stiffness * Theta;
    return product / (2 * a * beta);
}

double chain_energy(joint chain[], int chain_length, Matrix3d stiffness, double a, double beta, double theta_0)
{
    double energy = 0.0;
    for (int index = 1; index < chain_length; ++index)
    {
        Vector3d Theta = rotation_vector_from_matrix(chain[index].R);
        energy += joint_energy(Theta, stiffness, a, beta, theta_0);
    }
    return energy;
}

#endif // ENERGY_H
