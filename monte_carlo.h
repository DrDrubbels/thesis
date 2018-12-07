
#ifndef MONTE_CARLO_H
#define MONTE_CARLO_H

#include "random.h"
#include "constants.h"
#include "chain_operations.h"

bool energy_accept(double E_diff, double beta)
{
    // cout << "E = " << E_init << "; possible E = " << E_final << endl;
    if (E_diff <= 0)
    {
        // cout << "Accept because " << E_final << " < " << E_init << endl;
        return true;
    }
    else{
        double chance = pow(EULER, -1 * beta * E_diff);
        if (roll(chance))
        {
            // cout << "Accept on dice roll" << endl;
            return true;
        }
        else
        {
            // cout << "Reject" << endl;
            return false;
        }
    }
}

bool collision_accept(joint chain[], int chain_length, double a)
{
    return true;
}

#endif // MONTE_CARLO_H
