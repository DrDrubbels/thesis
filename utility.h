#ifndef UTILITY_H
#define UTILITY_H

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

struct run_args
{
    double a = 0.34;    // nm
    double T = 310.15;  // K
    double F = 1.00;    // pN
    double A_1 = 50.0;  // nm
    double A_2 = 50.0;  // nm
    double C = 100.0;   // nm
    double G = 0.0;     // nm
    Vector3d Theta_0 = Vector3d::Zero();
    bool circular = false;

    double sigma = 0.10;
    int steps = 1.0 * pow(10,1);

    bool new_chain = true;
};

struct xyz_args
{
    double frequency = 1.0 * pow(10,-3);
    string xyz_filename;
    string tan_filename;
    string theta_filename;
};

struct correl_args
{
    double frequency = 1.0 * pow(10, -3);
    string filename;
};

struct neighbour_list_args
{
    double recalc_frequency = 1.0 * pow(10, -3);
    double radius = 2.0;    // nm
    double cutoff = 3.0;    // unitless; radius is multiplied by this to give the actual cutoff distance
    double check_lk_frequency = 1.0 * pow(10, -3);
    double adaptive_threshold_factor = 0.5;
};

struct progress_args
{
    double frequency;
};

struct fix_args
{
    double fix_triads_frequency;
    double fix_Thetas_frequency;
    double fix_benergy_frequency;
};

struct report
{
    double accept_rate;
    int steps_worked;
};

struct check_pair
{
    int start_point;
    int end_point;
    int check_threshold;
};

enum movemode
{
    PIVOT,
    CRANK_SHAFT,
    CRANK_SHAFT_CIRCULAR,
};

struct chain_type
{
    movemode mode;
    bool closed;
};

bool output_now(int threshold, int step)
{
    if (step % threshold < PRECISION)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void print_vec(vector<int> v)
{
    int k = v.size() - 1;
    for (int i = 0; i < k; ++i)
    {
        cout << v[i] << ", ";
    }
    cout << v[k] << endl;
    // cout << "Size: " << v.size() << endl;
    // cout << "Capacity: " << v.capacity() << endl;
}

void print_lists(vector<int> veclist[], int llength)
{
    for (int i = 0; i < llength; ++i)
    {
        print_vec(veclist[i]);
    }
}

#endif // UTILITY_H
