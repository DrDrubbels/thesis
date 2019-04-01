#ifndef QUANTITIES_H
#define QUANTITIES_H

#include "chain_operations.h"

using namespace std;
using namespace Eigen;

void set_chain_position(Vector3d positions[], Joint chain[], int chain_length, double a)
{
    Vector3d position = Vector3d::Zero();

    for (int index = 0; index < chain_length; ++index)
    {
        positions[index] = position;
        position += a * chain[index].triad.col(2);
    }
}

Vector3d * partial_chain_position(Joint chain[], int low_index, int high_index, double a)
{
    static Vector3d positions[MAX_CHAIN_LENGTH];

    Vector3d position = Vector3d::Zero();

    for (int index = low_index; index < high_index; ++index)
    {
        positions[index] = position;
        position += a * chain[index].triad.col(2);
    }

    return positions;
}

Vector3d * chain_position(Joint chain[], int chain_length, double a)
{
    return partial_chain_position(chain, 0, chain_length, a);
}

Vector3d * partial_chain_Thetas(Joint chain[], int low_index, int high_index)
{
    static Vector3d Thetas[MAX_CHAIN_LENGTH];

    for (int index = low_index; index < high_index; ++index)
    {
        Thetas[index] = chain[index].Theta;
    }

    return Thetas;
}

Vector3d * chain_Thetas(Joint chain[], int chain_length)
{
    return partial_chain_Thetas(chain, 0, chain_length);
}

Vector3d * partial_chain_tangents(Joint chain[], int low_index, int high_index)
{
    static Vector3d tangents[MAX_CHAIN_LENGTH];

    for (int index = low_index; index < high_index; ++index)
    {
        tangents[index] = chain[index].triad.col(2);
    }

    return tangents;
}

Vector3d * chain_tangents(Joint chain[], int chain_length)
{
    return partial_chain_tangents(chain, 0, chain_length);
}

string xyz_line(Vector3d position)
{
    return "A\t" + to_string(position[0]) + "\t" + to_string(position[1]) + "\t" + to_string(position[2]) + "\n";
}

string write_chain_pos(Joint chain[], int chain_length, double a)
{
    string xyz = "";
    xyz += to_string(chain_length) + "\n\n";

    Vector3d *positions;
    positions = chain_position(chain, chain_length, a);

    for (int index = 0; index < chain_length; ++index)
    {
        xyz += xyz_line( *(positions + index) );
        // position += a * chain[index].triad.col(2);
    }
    return xyz;
}

string write_chain_tan(Joint chain[], int chain_length)
{
    string tan = "";
    tan += to_string(chain_length) + "\n\n";

    Vector3d *tangents;
    tangents = chain_tangents(chain, chain_length);

    for (int index = 0; index < chain_length; ++index)
    {
        tan += xyz_line( *(tangents + index));
    }
    return tan;
}

string write_chain_theta(Joint chain[], int chain_length)
{
    string Theta = "";
    Theta += to_string(chain_length) + "\n\n";

    Vector3d *Thetas;
    Thetas = chain_Thetas(chain, chain_length);

    for (int index = 0; index < chain_length; ++index)
    {
        Theta += xyz_line( *(Thetas + index));
    }
    return Theta;
}



#endif // QUANTITIES_H
