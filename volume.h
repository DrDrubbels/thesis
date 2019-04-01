#ifndef VOLUME_H
#define VOLUME_H

#include <Eigen/Dense>

#include "chain_operations.h"
#include "constants.h"
#include "joint.h"

using namespace std;
using namespace Eigen;

void add_neighbour(vector<int> neighbour_lists[], int index_1, int index_2)
{
    neighbour_lists[index_1].push_back(index_2);
    neighbour_lists[index_2].push_back(index_1);
}

void clear_neighbour_lists(vector<int> neighbour_lists[], int chain_length)
{
    for (int index = 0; index < chain_length; ++index)
    {
        neighbour_lists[index].resize(0);
    }
}

void calculate_neighbour_lists(vector<int> neighbour_lists[], Joint chain[], int chain_length, double a, double cutoff, int exclusion)
{
    clear_neighbour_lists(neighbour_lists, chain_length);

    Vector3d *positions;
    positions = chain_position(chain, chain_length, a);

    for (int index_major = 0; index_major < chain_length - 1; ++index_major)
    {
        for (int index_minor = index_major + exclusion + 1; index_minor < chain_length; ++index_minor)
        {
            Vector3d relative_position = *(positions + index_major) - *(positions + index_minor);
            double distance = relative_position.norm();

            if (distance <= cutoff)
            {
                add_neighbour(neighbour_lists, index_major, index_minor);
            }
        }
    }
}

#endif // VOLUME_H
