#ifndef LINKING_H
#define LINKING_H

#include "chain_operations.h"
#include "quantities.h"

#define WRITHE_OPT writhe_1

using namespace std;
using namespace Eigen;

double overtwist(Joint chain[], int chain_length, Vector3d Theta_0)
{
    double twist = 0;
    for (int index = 0; index < chain_length; ++index)
    {
        twist += chain[index].Theta[2] - Theta_0[2];
    }
    return twist;
}

double func_writhe_1(Joint chain[], int chain_length, double a)
{
    double writhe_terms = 0;
    Vector3d *positions;
    positions = chain_position(chain, chain_length, a);
    Vector3d *tangents;
    tangents = chain_tangents(chain, chain_length);

    for (int index_1 = 0; index_1 < chain_length - 1; ++index_1)
    {
        Vector3d position_1 = *(positions + index_1);
        Vector3d tangent_1 = *(tangents + index_1);
        for (int index_2 = index_1 + 1; index_2 < chain_length; ++index_2)
        {
            Vector3d position_2 = *(positions + index_2);
            Vector3d tangent_2 = *(tangents + index_2);

            Vector3d relative_position = position_2 - position_1;
            double relative_distance = relative_position.norm();

            writhe_terms += (tangent_2.cross(tangent_1)).dot(relative_position * pow(relative_distance, -3));
        }
    }

    return 2*pow(a,2)/(4 * PI) * writhe_terms;
}

double Vologodskii_F()
{

}

double func_writhe_2(Joint chain[], int chain_length, double a)
{

}

double writhe(Joint chain[], int chain_length, double a)
{
    double writhe_1 = func_writhe_1(chain, chain_length, a);
    return WRITHE_OPT;
}

double linking_number(Joint chain[], int chain_length, double a, Vector3d Theta_0)
{
    double twist_contribution = overtwist(chain, chain_length, Theta_0) / (2 * PI);
    double writhe_contribution = writhe(chain, chain_length, a);
    return twist_contribution + writhe_contribution;
}

bool linksafe(double target_lk, double current_lk)
{
    double discrepancy = fabs(current_lk - target_lk);
    bool safeness;
    // cout << "Discrepancy = " << discrepancy << " (|" << current_lk << " - " << target_lk << "|)" << endl;
    if (discrepancy < LK_TOLERANCE)
    {
        safeness = true;
    }
    else
    {
        safeness = false;
    }

    return safeness;
}



#endif // LINKING_H
