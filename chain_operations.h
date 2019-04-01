#ifndef CHAIN_OPERATIONS_H
#define CHAIN_OPERATIONS_H

#include <Eigen/Dense>
#include <fstream>

#include "benergy.h"
#include "joint.h"
#include "matrices.h"

using namespace std;
using namespace Eigen;

void set_benergy_open(Joint chain[], int chain_length, double a, double beta, double force, Vector3d Theta_0, Matrix3d stiffness)
{
    chain[0].benergy = joint_extension_benergy(chain[0], a, beta, force);

    for (int index = 1; index < chain_length - 1; ++index)
    {
        chain[index].benergy = joint_extension_benergy(chain[index], a, beta, force) + joint_elastic_benergy(chain[index], stiffness, a, Theta_0);
    }

    chain[chain_length - 1].benergy = joint_elastic_benergy(chain[chain_length - 1], stiffness, a, Theta_0);
}

void set_benergy_closed(Joint chain[], int chain_length, double a, double beta, Vector3d Theta_0, Matrix3d stiffness)
{
    for (int index = 0; index < chain_length; ++index)
    {
        chain[index].benergy = joint_elastic_benergy(chain[index], stiffness, a, Theta_0);
    }
}

void set_benergy(Joint chain[], int chain_length, double a, double beta, double force, Vector3d Theta_0, Matrix3d stiffness, bool closed)
{
    if (closed)
    {
        set_benergy_closed(chain, chain_length, a, beta, Theta_0, stiffness);
    }
    else
    {
        set_benergy_open(chain, chain_length, a, beta, force, Theta_0, stiffness);
    }
}

double chain_benergy(Joint chain[], int chain_length)
{
    double benergy = 0;
    for (int index = 0; index < chain_length; ++index)
    {
        benergy += chain[index].benergy;
    }
    return benergy;
}

void initialize_chain_straight(Joint chain[], int chain_length, double a, Vector3d Theta_0)
{
    Matrix3d R = rot_matrix_from_vector(Theta_0);
    chain[0].triad = Matrix3d::Identity();
    chain[0].Theta = Vector3d::Zero();
    for (int index = 1; index < chain_length; ++index)
    {
        chain[index].triad = chain[index - 1].triad * R;
        chain[index].Theta = Theta_0;
    }
}

void chain_copy(Joint origin[], Joint target[], int chain_length)
{
    for (int index = 0; index < chain_length; ++index)
    {
        target[index].benergy = origin[index].benergy;
        target[index].Theta = origin[index].Theta;
        target[index].triad = origin[index].triad;
    }
}

Matrix3d relative_R(Joint chain[], int chain_length, int index, Matrix3d rot_mat) //relative_R if chain[index].triad has been rotated by rot_mat
{
    if (index > 0)
    {
        return chain[index - 1].triad.transpose() * rot_mat * chain[index].triad;
    }
    else
    {
        return chain[chain_length - 1].triad.transpose() * rot_mat * chain[index].triad;
    }
}

Vector3d calc_Theta_open(Joint chain[], int chain_length, int index, Vector3d Theta_0)
{
    if (index == 0)
    {
        return Theta_0;
    }
    else
    {
        Matrix3d rot_mat = relative_R(chain, chain_length, index, Matrix3d::Identity());
        Vector3d Theta = rot_vec_from_matrix(rot_mat);
        return Theta;
    }
}

Vector3d calc_Theta_closed(Joint chain[], int chain_length, int index, Vector3d Theta_0)
{
    Matrix3d rot_mat = relative_R(chain, chain_length, index, Matrix3d::Identity());
    Vector3d Theta = rot_vec_from_matrix(rot_mat);
    return Theta;
}

void set_Thetas(Joint chain[], int chain_length, Vector3d Theta_0, bool closed)
{
    for (int index = 0; index < chain_length; ++index)
    {
        Vector3d Theta;
        if (closed)
        {
            Theta = calc_Theta_closed(chain, chain_length, index, Theta_0);
        }
        else
        {
            Theta = calc_Theta_open(chain, chain_length, index, Theta_0);
        }
        chain[index].Theta = Theta;
    }
}

void rotate_chain(Joint chain[], int chain_length, int low_index, int high_index, Matrix3d rot_mat)
{
    for (int index = low_index; index < high_index; ++index)
    {
        // chain[index].triad = chain[index].triad * rot_mat;
        chain[index].triad = rot_mat * chain[index].triad;
    }
}

void rotate_chain_closed(Joint chain[], int chain_length, int low_index, int high_index, bool in_between, Matrix3d rot_mat)
{
    if (in_between)
    {
        rotate_chain(chain, chain_length, low_index, high_index, rot_mat);
    }
    else
    {
        rotate_chain(chain, chain_length, high_index, chain_length, rot_mat);
        rotate_chain(chain, chain_length, 0, low_index, rot_mat);
    }
}

void dump_chain(Joint chain[], int chain_length, string filename)
{
    ofstream myfile;
    myfile.open(filename);
    for (int index = 0; index < chain_length; ++index)
    {
        myfile << chain[index];
    }
    myfile.close();
}

void load_chain(Joint chain[], int chain_length, string filename)
{
    ifstream myfile;
    myfile.open(filename);
    for (int index = 0; index < chain_length; ++index)
    {
        myfile >> chain[index];
    }
    myfile.close();
}

Matrix3d chain_correlation(Joint chain[], int chain_length, Vector3d Theta_0)
{
    Matrix3d correl = Matrix3d::Zero();

    for (int index = 0; index < chain_length; ++index)
    {
        Vector3d Theta_reduced = chain[index].Theta - Theta_0;
        correl += Theta_reduced * Theta_reduced.transpose();
    }

    correl /= chain_length;

    return correl;
}

#endif // CHAIN_OPERATIONS_H
