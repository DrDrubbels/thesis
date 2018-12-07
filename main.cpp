#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <math.h>
#include <random>
#include <cstdlib>
#include <ctime>
#include <string>
#include <windows.h>

#include "calc.h"
#include "chain_operations.h"
#include "constants.h"
#include "energy.h"
#include "files.h"
#include "matrices.h"
#include "monte_carlo.h"
#include "print_chain.h"
#include "random.h"



const double a_def = 0.34;          // nm
const double T_def = 310.15;        // K
const double F_def = 0.0;           // pN
const int chain_length_def = 201;

const double A_1_def = 50;          // nm
const double A_2_def = 50;          // nm
const double C_def = 100;           // nm
const double G_def = 0;             // nm
// const double theta_0_def = 0;
const double theta_0_def = 0.628319;

bool new_chain_def = true;

const string filename_def = "test.dat";

const double sigma_def = 0.125;

const double report_frac_def = 1.0 * pow(10,-3);
const int steps_def = 1.0 * pow(10,4);

using namespace std;
using namespace Eigen;

/*
    Units:
    length/distance - nm (10^-9 m)
    force           - pN (10^-12 N)
    energy          - zJ (10^-21 J = 10^-9 m * 10^-12 N)
    temperature     - K
*/

struct run_args
{
    double a = a_def;
    double T = T_def;
    double F = F_def;
    int chain_length = chain_length_def;

    double A_1 = A_1_def;
    double A_2 = A_2_def;
    double C = C_def;
    double G = G_def;
    double theta_0 = theta_0_def;

    bool new_chain = new_chain_def;
    // joint chain[chain_length_def];

    string filename = filename_def;
    double sigma = sigma_def;
    double report_frac = report_frac_def;
    int steps = steps_def;
};

struct report
{
    bool report_accept_rate = true;
    bool report_extension = true;
    bool report_correl = true;
};

string run(run_args args, joint chain[])
{
    // Import arguments

    double a = args.a;
    double T = args.T;
    double F = args.F;
    int chain_length = args.chain_length;

    double A_1 = args.A_1;
    double A_2 = args.A_2;
    double C = args.C;
    double G = args.G;
    double theta_0 = args.theta_0;

    // joint chain[args.chain_length];
    // chain = args.chain;
    if (args.new_chain)
    {
        initialize_chain(chain, chain_length, a, theta_0);
    }

    string filename = args.filename;
    double sigma = args.sigma;
    double report_frac = args.report_frac;
    int steps = args.steps;

    // Create derived quantities (beta, stiffness matrix...)

    double beta = 1/(KB * T);           // zJ^-1

    double L = (chain_length - 1) * a;   // nm

    Matrix3d stiffness = stiffness_matrix(A_1, A_2, C, G);

    // Create output variables

    int report_threshold = round(pow(report_frac, -1));
    int accepts = 0;
    double accept_rate = 0.0;

    string out_meta = "";
    string out_extensions = "";
    string out_correl = "";
    string out = "";

    double E_extension = extension_energy_chain(chain, chain_length, F);

    // Print force

    cout << "F = " << F << endl;

    // Pivot move

    for (int step = 0; step < steps; ++step)
    {
        if (step % report_threshold == 0)
        {
            cout << "Doing step #" << step << " (" << (double)step / steps << ")" << endl;
        }

        int pivot_point = rand_int(0, chain_length);
        /*
            If pivot_point = 0, no change in stiffness energy (entire chain rotates as a whole) (only extension)
            If pivot_point = chain_length - 1, no change in extension energy (only stiffness)
        */

        double E_extension_diff;
        double E_stiffness_diff;
        double E_diff;

        Matrix3d current_R = chain[pivot_point].R;
        Vector3d rot_vec = rand_vector(sigma);
        Matrix3d rot_mat = rotation_matrix(rot_vec);

        // cout << "Step #" << step << "; want to rotate by" << endl << rot_vec << endl << " at " << pivot_point << endl;

        Matrix3d poss_R;
        double poss_E_stiffness;

        if (pivot_point != 0)
        {
            double current_E_stiffness = chain[pivot_point].E_stiffness;

            poss_R = rot_mat * current_R;
            Vector3d poss_Theta = rotation_vector_from_matrix(poss_R);
            poss_E_stiffness = joint_energy(poss_Theta, stiffness, a, beta, theta_0);

            E_stiffness_diff = poss_E_stiffness - current_E_stiffness;
        }
        else    // so if pivot_point == 0
        {
            poss_R = Matrix3d::Identity();
            E_stiffness_diff = 0;
        }

        if (pivot_point != chain_length - 1)
        {
            Vector3d current_end_pos = chain[chain_length - 1].position;
            Vector3d anchor = chain[pivot_point].position;
            Vector3d new_end_pos = rotate_point_by_matrix(current_end_pos, anchor, rot_mat);
            double poss_z = new_end_pos[2];
            double poss_E_extension = extension_energy_length(poss_z, F);

            E_extension_diff = poss_E_extension - E_extension;
        }
        else    // so if pivot_point == chain_length - 1
        {
            E_extension_diff = 0;
        }

        E_diff = E_stiffness_diff + E_extension_diff;

        bool do_move = false;

        if (collision_accept(chain, chain_length, a))
        {
            if (energy_accept(E_diff, beta))
            {
                do_move = true;
            }
        }

        if (do_move)
        {
            apply_pivot(chain, chain_length, pivot_point, rot_mat, a, poss_R, poss_E_stiffness);
            ++accepts;
            E_extension += E_extension_diff;
            // cout << "Accept #" << step << endl;
        }
        else
        {
            // cout << "Reject #" << step << endl;
        }

        if (step % report_threshold == 0)
        {
            if (step != 0)
            {
                out_extensions += ", ";
                out_correl += ", ";
            }

            double z_L = chain[chain_length - 1].position[2] / L;
            Matrix3d correl = correl_matrix(chain, chain_length);

            out_extensions += to_string(z_L);
            out_correl += matrix_to_string(correl);
        }
    }

    //append_file(filename, "[" + out_extensions + "]");

    // print_chain_pos(chain, chain_length);
    // print_chain_triad(chain, chain_length);

    accept_rate = (double)accepts/steps;
    cout << "Accept rate = " << accept_rate << endl;

    out_meta = "[" + to_string(beta * F) + ", " + to_string(accept_rate) + ", " + to_string(steps) + ", " + to_string(report_frac) + "]";
    out_extensions = "[" + out_extensions + "]";
    out_correl = "[" + out_correl + "]";
    out = "[" + out_meta;
    out += ", " + out_extensions;
    // out += ", " + out_correl;
    out += "]";

    return out;
}

int main()
{
    string filename = next_filename(0);
    create_file(filename);

    string output;

    run_args args;
    args.F = 5;
    args.chain_length = 201;
    joint chain[args.chain_length];
    args.sigma = 0.15;
    args.steps = 1.0 * pow(10,5);
    args.report_frac = 1.0 * pow(10,-3);
    args.new_chain = true;
    output = run(args, chain);

    append_file(filename, output);

    cout << filename << endl;

    /*

    // New run

    filename = next_filename(0);
    create_file(filename);

    run_args new_args;
    new_args.F = 10;
    new_args.chain_length = 201;
    // args.chain = chain;
    new_args.sigma = 0.1;
    new_args.steps = 1000;
    new_args.report_frac = 0.1;
    new_args.new_chain = false;

    output = run(new_args, chain);

    append_file(filename, output);

    cout << filename << endl;

    */

    return 0;
}
