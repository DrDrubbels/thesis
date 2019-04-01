#ifndef MONTE_CARLO_H
#define MONTE_CARLO_H

#include "chain_operations.h"
#include "joint.h"
#include "matrices.h"
#include "random.h"
#include "volume.h"
#include "utility.h"

using namespace Eigen;

bool benergy_accept(double benergy_diff)
{
    // cout << "benergy_diff = " << benergy_diff << endl;
    if (benergy_diff <= 0)
    {
        return true;
    }
    else
    {
        // double probability = pow(EULER, -1 * benergy_diff);
        double probability = exp(-1 * benergy_diff);
        // cout << "acceptance probability = " << probability << endl;
        if (roll(probability))
        {
            return true;
        }
        else
        {
            if (ACCEPT_ALL)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }
}

bool collision_accept(vector<int> neighbour_lists[], Joint chain[], int chain_length, int exclusion, double a, double radius)
{
    Vector3d *positions;
    positions = chain_position(chain, chain_length, a);

    for (int index_major = 0; index_major < chain_length; ++index_major)
    {
        Vector3d position_major = *(positions + index_major);
        vector<int> neighbours = neighbour_lists[index_major];
        for (int index_list = 0; index_list < neighbours.size(); ++index_list)
        {
            int index_minor = neighbours.at(index_list);
            if (index_minor > index_major + exclusion)
            {
                Vector3d position_minor = *(positions + index_minor);
                Vector3d position_relative = position_minor - position_major;
                double relative_distance = position_relative.norm();

                // cout << "Checked distance between " << index_major << " and " << index_minor << endl;

                if (relative_distance < radius)
                {
                    // cout << "Collision reject" << endl;
                    return false;
                }
            }
        }
    }

    return true;
}

struct pivot_attempt
{
    bool benergy_accept;
    double new_pivot_benergy;
    double benergy_diff;
    Matrix3d M; //rotation matrix
    Vector3d new_Theta;
};

struct crank_shaft_circular_attempt
{
    bool benergy_accept;
    double new_low_benergy;
    double new_high_benergy;
    double benergy_diff;
    Matrix3d M; //rotation matrix
    Vector3d new_low_Theta;
    Vector3d new_high_Theta;
};

pivot_attempt benergy_try_pivot(Vector3d positions[], Joint chain[], int chain_length, int pivot_point, bool direction, Vector3d rot_vector, double a, Vector3d Theta_0, Matrix3d stiffness)
{

}

crank_shaft_circular_attempt benergy_try_crank_shaft_circular(Vector3d positions[], Joint chain[], int chain_length, crank_shaft_circular_index_pair crank_indices, double angle, double a, Vector3d Theta_0, Matrix3d stiffness)
{
    int low_index = crank_indices.low_index;
    int high_index = crank_indices.high_index;

    Vector3d low_pos = positions[low_index];
    Vector3d high_pos = positions[high_index];
    Vector3d axis = (high_pos - low_pos).normalized();
    Matrix3d M;
    M = AngleAxisd(angle, axis);
    Matrix3d N = M.inverse();

    Vector3d old_low_Theta = chain[low_index].Theta;
    // Matrix3d old_low_R = rot_matrix_from_vector(old_low_Theta);
    Vector3d old_high_Theta = chain[high_index].Theta;
    // Matrix3d old_high_R = rot_matrix_from_vector(old_high_Theta);

    Matrix3d new_low_R;
    Matrix3d new_high_R;

    if (crank_indices.in_between)
    {
        //Rotate everything from low_index (inclusive) to high_index (exclusive)
        new_low_R = relative_R(chain, chain_length, low_index, M);
        new_high_R = relative_R(chain, chain_length, high_index, N);
    }
    else
    {
        new_low_R = relative_R(chain, chain_length, low_index, N);
        new_high_R = relative_R(chain, chain_length, high_index, M);
    }

    Vector3d new_low_Theta = rot_vec_from_matrix(new_low_R);
    Vector3d new_high_Theta = rot_vec_from_matrix(new_high_R);

    double old_benergy = Theta_elastic_benergy(old_low_Theta, Theta_0, stiffness, a) + Theta_elastic_benergy(old_high_Theta, Theta_0, stiffness, a);
    double new_low_benergy = Theta_elastic_benergy(new_low_Theta, Theta_0, stiffness, a);
    double new_high_benergy = Theta_elastic_benergy(new_high_Theta, Theta_0, stiffness, a);
    double new_benergy = new_low_benergy + new_high_benergy;
    double benergy_diff = new_benergy - old_benergy;

    /*
    cout << "Function - Old low Theta: " << xyz_line(old_low_Theta);
    cout << "Function - Old high Theta: " << xyz_line(old_high_Theta);
    cout << "Function - New low Theta: " << xyz_line(new_low_Theta);
    cout << "Function - New high Theta: " << xyz_line(new_high_Theta);
    */

    crank_shaft_circular_attempt attempt;
    attempt.benergy_accept = benergy_accept(benergy_diff);
    attempt.new_low_benergy = new_low_benergy;
    attempt.new_high_benergy = new_high_benergy;
    attempt.benergy_diff = benergy_diff;
    attempt.M = M;
    attempt.new_low_Theta = new_low_Theta;
    attempt.new_high_Theta = new_high_Theta;

    return attempt;
}

#endif // MONTE_CARLO_H
