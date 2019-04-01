#ifndef MOVES_H
#define MOVES_H

#include <Eigen/Dense>

#include "chain_operations.h"
#include "random.h"

using namespace std;
using namespace Eigen;

void pivot_move(Joint chain[], int chain_length, double sigma, Vector3d Theta_0)
{
    int pivot_point = rand_int(0, chain_length);
    bool direction = roll(0.5);

    Vector3d rot_vec = rand_vec(sigma);
    Matrix3d rot_mat = rot_matrix_from_vector(rot_vec);
    // rot_mat = chain[pivot_point].triad * rot_mat * chain[pivot_point].triad.transpose();

    int low_index;
    int high_index;

    if (direction)
    {
        low_index = pivot_point;
        high_index = chain_length;
    }
    else
    {
        low_index = 0;
        high_index = pivot_point;
    }

    rotate_chain(chain, chain_length, low_index, high_index, rot_mat);

    Vector3d new_Theta = calc_Theta_open(chain, chain_length, pivot_point, Theta_0);
    chain[pivot_point].Theta = new_Theta;
}

void crank_shaft_move(Joint chain[], int chain_length, double sigma, Vector3d Theta_0)
{
    index_pair indices = rand_pair(0, chain_length, true);
    int low_index = min(indices.index_1, indices.index_2);
    int high_index = max(indices.index_1, indices.index_2);

    // cout << low_index << "; " << high_index << endl;

    Vector3d *positions;
    positions = chain_position(chain, chain_length, 1.0);
    Vector3d axis = *(positions + high_index) - *(positions + low_index);
    axis.normalize();

    double rot_angle = rand_double(-1 * sigma, sigma);
    axis = rot_angle*axis;
    Matrix3d rot_mat = rot_matrix_from_vector(axis);

    rotate_chain(chain, chain_length, low_index, high_index, rot_mat);

    Vector3d new_low_Theta = calc_Theta_open(chain, chain_length, low_index, Theta_0);
    Vector3d new_high_Theta = calc_Theta_open(chain, chain_length, high_index, Theta_0);
    chain[low_index].Theta = new_low_Theta;
    chain[high_index].Theta = new_high_Theta;
}

void crank_shaft_circular_move(Joint chain[], int chain_length, int low_index, int high_index, bool in_between, double angle, Vector3d Theta_0)
{
    Vector3d *positions;
    positions = chain_position(chain, chain_length, 1.0);
    Vector3d axis = *(positions + high_index) - *(positions + low_index);
    axis.normalize();

    axis = angle*axis;
    Matrix3d rot_mat = rot_matrix_from_vector(axis);

    if (in_between)
    {
        rotate_chain(chain, chain_length, low_index, high_index, rot_mat);
    }
    else
    {
        rotate_chain(chain, chain_length, high_index, chain_length, rot_mat);
        rotate_chain(chain, chain_length, 0, low_index, rot_mat);
    }

    Vector3d new_low_Theta = calc_Theta_closed(chain, chain_length, low_index, Theta_0);
    Vector3d new_high_Theta = calc_Theta_closed(chain, chain_length, high_index, Theta_0);
    chain[low_index].Theta = new_low_Theta;
    chain[high_index].Theta = new_high_Theta;
}

void crank_shaft_circular_move_random(bool test_acceptance, Joint chain[], int chain_length, double sigma, Vector3d Theta_0)
{
    index_pair indices = rand_pair(0, chain_length, true);
    int low_index = min(indices.index_1, indices.index_2);
    int high_index = max(indices.index_1, indices.index_2);

    bool in_between; // true if we're going to rotate the triads in between our indices; false if we're going to rotate all other ones
    if (high_index - low_index <= chain_length/2)
    {
        in_between = true;
    }
    else
    {
        in_between = false;
    }

    double angle = rand_double(-1 * sigma, sigma);

    crank_shaft_circular_move(chain, chain_length, low_index, high_index, in_between, angle, Theta_0);
}

#endif // MOVES_H
