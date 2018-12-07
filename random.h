
#ifndef RANDOM_H
#define RANDOM_H

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

double rand_normal(double average, double sigma)
{
    static bool first_call = true;
    if (first_call)
    {
        first_call = false;
        srand(1);
    }
    static default_random_engine generator;
    static normal_distribution<double> distribution(0.0,1.0);
    return average + sigma * distribution(generator);
}

Vector3d rand_vector(double sigma)
{
    Vector3d vec;
    vec << rand_normal(0.0, sigma), rand_normal(0.0, sigma), rand_normal(0.0, sigma);
    return vec;
}

int rand_int(int min_val, int max_val) //min_val inclusive; max_val exclusive
{
    static bool first_call = true;
    if (first_call)
    {
        first_call = false;
        srand(1);
    }
    return rand() % (max_val - min_val) + min_val;
}

double rand_double(double min_val, double max_val)
{
    static bool first_call = true;
    if (first_call)
    {
        first_call = false;
        srand(1);
    }
    return (double)rand()/RAND_MAX * (max_val - min_val) + min_val;
}

bool roll(double probability)
{
    double roll_outcome = rand_double(0.0,1.0);
    if (roll_outcome < probability)
    {
        // cout << "Rolled " << roll_outcome << " < " << probability << endl;
        return true;
    }else
    {
        // cout << "Rolled " << roll_outcome << " > " << probability << endl;
        return false;
    }
}

/*
index_pair rand_pair(int num_points)
{
    int n1;
    int n2;
    do
    {
        n1 = rand_int();
        n2 = rand_int();
    }while(accept_pair(n1,n2,num_points));
};
*/

#endif // RANDOM_H

