#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <math.h>
#include <random>
#include <cstdlib>
#include <ctime>
#include <string>
#include <windows.h>

#include "benergy.h"
#include "chain_operations.h"
#include "constants.h"
#include "files.h"
#include "joint.h"
#include "linking.h"
#include "matrices.h"
#include "monte_carlo.h"
#include "moves.h"
#include "quantities.h"
#include "random.h"
#include "utility.h"
#include "volume.h"

using namespace std;
using namespace Eigen;

report simulate(run_args run, chain_type type, fix_args fix, xyz_args xyz, correl_args correl, neighbour_list_args neighbour_list, progress_args progress, Joint chain[], int chain_length)
{
    double a = run.a;
    double T = run.T;
    double beta = 1 / (KB * T);
    double F = run.F;
    Matrix3d stiffness;
    stiffness << run.A_1, 0, 0,
                 0, run.A_2, run.G,
                 0, run.G, run.C;
    Vector3d Theta_0 = run.Theta_0;

    double sigma = run.sigma;
    int steps = run.steps;

    if (run.new_chain)
    {
        initialize_chain_straight(chain, chain_length, a, Theta_0);
    }

    bool closed = type.closed;
    movemode mode = type.mode;

    // set_Thetas(chain, chain_length, Theta_0);
    set_Thetas(chain, chain_length, Theta_0, closed);

    Joint poss_chain[chain_length];
    Joint backup_chain[chain_length];
    Vector3d positions[chain_length];

    //Vector3d positions_a[chain_length];
    //Vector3d positions_b[chain_length];
    double move_distances[chain_length];

    bool has_moved[chain_length]; // For Enrico-style parametrized collision detection

    int fix_triads_threshold = pow(fix.fix_triads_frequency, -1);
    int fix_Thetas_threshold = pow(fix.fix_Thetas_frequency, -1);
    int fix_benergy_threshold = pow(fix.fix_benergy_frequency, -1);

    int xyz_threshold = pow(xyz.frequency, -1);
    string xyz_filename = xyz.xyz_filename;
    create_file(xyz_filename);
    cout << "Storing xyz data in " << xyz_filename;
    string tan_filename = xyz.tan_filename;
    create_file(tan_filename);
    cout << ", tan data in " << tan_filename;
    string theta_filename = xyz.theta_filename;
    create_file(theta_filename);
    cout << ", and theta data in " << theta_filename << endl;

    int correl_threshold = pow(correl.frequency, -1);
    string correl_filename = correl.filename;
    create_file(correl_filename);
    cout << "Storing correlation data in " << correl_filename << endl;

    string unsafe_xyz;
    create_file("unsafe.xyz");
    string unsafe_tan;
    create_file("unsafe.tan");
    string unsafe_theta;
    create_file("unsafe.theta");
    string unsafe_corr;
    create_file("unsafe.corr");

    vector<int> neighbour_lists[chain_length];
    for (int index = 0; index < chain_length; ++index)
    {
        neighbour_lists[index].reserve(MAX_CHAIN_LENGTH);
    }
    int neighbour_list_threshold = pow(neighbour_list.recalc_frequency, -1);
    // int exclusion = neighbour_list.cutoff;
    // double cutoff = neighbour_list.cutoff * a;
    int check_lk_threshold = pow(neighbour_list.check_lk_frequency, -1);
    double adaptive_threshold_factor = neighbour_list.adaptive_threshold_factor;

    vector<check_pair> lk_check_pairs;
    check_pair main_pair;
    main_pair.start_point = 0;
    main_pair.end_point = steps;
    main_pair.check_threshold = check_lk_threshold;
    lk_check_pairs.push_back(main_pair);

    int exclusion = 21;
    double cutoff = 20*a;
    double radius = neighbour_list.radius;
    calculate_neighbour_lists(neighbour_lists, chain, chain_length, a, 10*cutoff*a, 10);

    int progress_threshold = pow(progress.frequency, -1);

    double accepts = 0.0;
    double unsafe_accepts = 0.0;

    double calc_overtwist = overtwist(chain, chain_length, Theta_0)/(2*PI);
    double calc_writhe = writhe(chain, chain_length, a);

    cout << "Overtwist: " << calc_overtwist << endl;
    cout << "writhe: " << calc_writhe << endl;
    double target_lk = calc_overtwist + calc_writhe;
    cout << "Calculated linking number as " << target_lk << endl;

    {
    /* ============================================================================================================

    string test_filename = "logs\\test_6.xyz";

    create_file(test_filename);
    append_file(test_filename, write_chain_pos(chain, chain_length, a));

    set_Thetas(chain, chain_length, Theta_0, closed);
    set_chain_position(positions, chain, chain_length, a);
    int low_index = 200;
    int high_index = 300;
    bool in_between;
    if (high_index - low_index <= chain_length/2)
    {
        in_between = true;
    }
    else
    {
        in_between = false;
    }
    double angle = 1.0;

    // cout << "Real - Initial low Theta: " << xyz_line(chain[low_index].Theta);
    // cout << "Real - Initial high Theta: " << xyz_line(chain[high_index].Theta);

    crank_shaft_circular_attempt attempt = benergy_try_crank_shaft_circular(positions, chain, chain_length, low_index, high_index, in_between, angle, a, Theta_0, stiffness);
    double new_benergy = attempt.new_low_benergy + attempt.new_high_benergy;
    cout << "New benergy according to attempt: " << new_benergy << " (low " << attempt.new_low_benergy << ", high " << attempt.new_high_benergy << ")" << endl;

    crank_shaft_circular_move(chain, chain_length, low_index, high_index, in_between, angle, Theta_0);
    set_Thetas(chain, chain_length, Theta_0, closed);
    set_benergy(chain, chain_length, a, beta, F, Theta_0, stiffness, closed);
    cout << "Real new benergy: " << chain[low_index].benergy + chain[high_index].benergy << " (low " << chain[low_index].benergy << ", high " << chain[high_index].benergy << ")" << endl;

    append_file(test_filename, write_chain_pos(chain, chain_length, a));

    // cout << "Real - Final low Theta: " << xyz_line(chain[low_index].Theta);
    // cout << "Real - Final high Theta: " << xyz_line(chain[high_index].Theta);

    // ============================================================================================================ */
    }
    int last_safe_step = 0;
    int last_gone_back = 0;

    set_Thetas(chain, chain_length, Theta_0, closed);
    set_benergy(chain, chain_length, a, beta, F, Theta_0, stiffness, closed);
    chain_copy(chain, backup_chain, chain_length);

    int steps_worked = 0;

    create_file("distances.dat");

    time_t start_time = time(NULL);

    for (int step = 0; step < steps; ++step)
    {
        if (OLD_LOOP)
        {
            set_benergy(chain, chain_length, a, beta, F, Theta_0, stiffness, closed);

            double current_benergy = chain_benergy(chain, chain_length);

            chain_copy(chain, poss_chain, chain_length);

            if (mode == CRANK_SHAFT)
            {
                crank_shaft_move(poss_chain, chain_length, sigma, Theta_0);
            }
            else if (mode == CRANK_SHAFT_CIRCULAR)
            {
                crank_shaft_circular_move_random(false, poss_chain, chain_length, sigma, Theta_0);
            }
            else if (mode == PIVOT)
            {
                pivot_move(poss_chain, chain_length, sigma, Theta_0);
            }
            else
            {
                cout << "No valid move specified :(" << endl;
            }

            set_benergy(poss_chain, chain_length, a, beta, F, Theta_0, stiffness, closed);

            double poss_benergy = chain_benergy(poss_chain, chain_length);
            double benergy_diff = poss_benergy - current_benergy;

            // cout << "current benergy = " << current_benergy << endl;
            // cout << "possible benergy = " << poss_benergy << endl;

            if (benergy_accept(benergy_diff) && collision_accept(neighbour_lists, poss_chain, chain_length, exclusion, a, radius))
            {
                ++unsafe_accepts;
                chain_copy(poss_chain, chain, chain_length);
                // cout << "Accept" << endl;
            }
            else
            {
                // cout << "Reject" << endl;
            }
        }
        else
        {
            set_chain_position(positions, chain, chain_length, a);
            // set_chain_position(positions_a, chain, chain_length, a);
            crank_shaft_circular_index_pair crank_indices = rand_crank_shaft_circular_pair(chain_length);
            double angle = rand_double(-1*sigma, sigma);
            crank_shaft_circular_attempt attempt = benergy_try_crank_shaft_circular(positions, chain, chain_length, crank_indices, angle, a, Theta_0, stiffness);
            if (attempt.benergy_accept)
            {
                int low_index = crank_indices.low_index;
                int high_index = crank_indices.high_index;
                bool in_between = crank_indices.in_between;

                rotate_chain_closed(chain, chain_length, low_index, high_index, in_between, attempt.M);

                if (collision_accept(neighbour_lists, chain, chain_length, exclusion, a, 0.99*a))
                {
                    chain[low_index].Theta = attempt.new_low_Theta;
                    chain[low_index].benergy = attempt.new_low_benergy;
                    chain[high_index].Theta = attempt.new_high_Theta;
                    chain[high_index].benergy = attempt.new_high_benergy;
                    ++unsafe_accepts;
                }
                else
                {
                    rotate_chain_closed(chain, chain_length, low_index, high_index, in_between, attempt.M.inverse()); //Rotate back
                    // cout << "Collision reject" << endl;
                }
            }

            // set_chain_position(positions_b, chain, chain_length, a);
            /* double max_move_distance = 0;
            for (int index = 0; index < chain_length; ++index)
            {
                double move_distance = (positions_b[index] - positions_a[index]).norm();
                if (move_distance > max_move_distance)
                {
                    max_move_distance = move_distance;
                }
            }
            append_file("distances.dat", to_string(max_move_distance) + "\n");
            */

        }

        check_pair current_pair = lk_check_pairs.back();

        if (output_now(fix_triads_threshold, step))
        {
            for (int index = 0; index < chain_length; ++index)
            {
                chain[index].triad = fix_triad(chain[index].triad);
            }
        }

        if (output_now(fix_Thetas_threshold, step))
        {
            set_Thetas(chain, chain_length, Theta_0, closed);
        }

        if (output_now(fix_benergy_threshold, step))
        {
            set_benergy(chain, chain_length, a, beta, F, Theta_0, stiffness, closed);
        }

        if (output_now(xyz_threshold, step))
        {
            //cout << "Output xyz at step " << step << "/" << steps << " (" << 1.0 * step / steps << ")" << endl;
            unsafe_xyz += write_chain_pos(chain, chain_length, a);
            unsafe_tan += write_chain_tan(chain, chain_length);
            unsafe_theta += write_chain_theta(chain, chain_length);
            // append_file(xyz_filename, write_chain_pos(chain, chain_length, a));
            // append_file(tan_filename, write_chain_tan(chain, chain_length));
            // append_file(theta_filename, write_chain_theta(chain, chain_length));
        }

        if (output_now(correl_threshold, step))
        {
            //cout << "Output correlation at step " << step << "/" << steps << " (" << 1.0 * step / steps << ")" << endl;
            // unsafe_corr += string_from_matrix(chain_correlation(chain, chain_length, Theta_0)) + "\n";
            append_file(correl_filename, string_from_matrix(chain_correlation(chain, chain_length, Theta_0)) + "\n");
        }

        if (output_now(lk_check_pairs.back().check_threshold, step - lk_check_pairs.back().start_point) || output_now(lk_check_pairs.back().end_point, step) || output_now(steps - 1, step))
        {
            double current_lk = linking_number(chain, chain_length, a, Theta_0);
            double efficiency;
            double accept_rate;
            // cout << "Current Lk found to be " << current_lk << endl;
            if (linksafe(target_lk, current_lk))
            {
                accepts += unsafe_accepts;
                chain_copy(chain, backup_chain, chain_length);
                last_safe_step = step;

                append_file(xyz_filename, unsafe_xyz);
                append_file(tan_filename, unsafe_tan);
                append_file(theta_filename, unsafe_theta);
                append_file(correl_filename, unsafe_corr);

                efficiency = 1.0*step/steps_worked;
                accept_rate = accepts/step;

                cout << "Effective accept rate " << efficiency * accept_rate << " (" << accept_rate << "; " << efficiency << "); Lk " << current_lk << endl;
            }
            else
            {
                efficiency = 1.0*last_safe_step/steps_worked;

                check_pair new_pair;
                new_pair.end_point = step;
                new_pair.start_point = last_safe_step;
                new_pair.check_threshold = adaptive_threshold_factor * lk_check_pairs.back().check_threshold;
                lk_check_pairs.push_back(new_pair);

                cout << "Unsafe at step " << step << "; going back to " << last_safe_step << "; efficiency " << efficiency << endl;
                cout << "Threshold " << new_pair.check_threshold << endl;

                step = last_safe_step;
                chain_copy(backup_chain, chain, chain_length);
            }
            unsafe_accepts = 0;
            unsafe_xyz = "";
            unsafe_tan = "";
            unsafe_theta = "";
            unsafe_corr = "";
            dump_chain(chain, chain_length, "chain.dat");
        }

        if (output_now(neighbour_list_threshold, step) || output_now(lk_check_pairs.back().check_threshold, step - lk_check_pairs.back().start_point))
        {
            // cout << "Recalculate neighbour lists at step " << step << "/" << steps << " (" << 1.0 * step / steps << ")" << endl;
            calculate_neighbour_lists(neighbour_lists, chain, chain_length, a, cutoff, 10);
        }

        if (output_now(progress_threshold, step))
        {
            cout << "Now at step " << step << " of " << steps << " (" << 1.0 * step / steps << ")" << " (latest safe step " << last_safe_step << "; time elapsed " << time(NULL) - start_time << ")" << endl;
            cout << "Threshold " << lk_check_pairs.back().check_threshold << endl;
        }

        if (lk_check_pairs.back().end_point <= step)
        {
            lk_check_pairs.pop_back();
        }

        ++steps_worked;
    }

    calc_overtwist = overtwist(chain, chain_length, Theta_0)/(2*PI);
    calc_writhe = writhe(chain, chain_length, a);
    cout << "overtwist: " << calc_overtwist << endl;
    cout << "writhe: " << calc_writhe << endl;
    cout << "Calculated linking number as " << calc_overtwist + calc_writhe << endl;

    report out;
    out.accept_rate = accepts/steps;
    out.steps_worked = steps_worked;

    return out;
}

int main()
{
    bool G_zero = true;

    int chain_length = 500;
    run_args run;
    run.F = 0.00;
    run.sigma = 0.10;
    run.steps = 1000000; //1.0 * pow(10,4);
    run.new_chain = false;
    run.Theta_0 << 0, 0, (0.1 * 2 * PI);

    if (G_zero)
    {
        run.A_1 = 88.47;
        run.A_2 = 26.07;
        run.C = 77.78;
        run.G = 0.00;
    }
    else
    {
        run.A_1 = 96.7;
        run.A_2 = 34.3;
        run.C = 104.9;
        run.G = 30.5;
    }

    fix_args fix;
    fix.fix_triads_frequency = 0.0001;
    fix.fix_Thetas_frequency = 0.0001;
    fix.fix_benergy_frequency = 0.0001;

    xyz_args xyz;
    string xyz_filename = next_filename(0, "logs", ".xyz");
    string tan_filename = next_filename(0, "logs", ".tan");
    string theta_filename = next_filename(0, "logs", ".theta");
    xyz.xyz_filename = xyz_filename;
    xyz.tan_filename = tan_filename;
    xyz.theta_filename = theta_filename;
    xyz.frequency = 0.001 - PRECISION;

    correl_args correl;
    string correl_filename = next_filename(0, "logs", ".corr");
    correl.filename = correl_filename;
    correl.frequency = 0.001 - PRECISION;

    Joint chain[chain_length];

    neighbour_list_args neighbour_list;
    neighbour_list.recalc_frequency = 0.001 - PRECISION;
    neighbour_list.check_lk_frequency = 0.001 - PRECISION;
    neighbour_list.adaptive_threshold_factor = 0.5;

    progress_args progress;
    progress.frequency = 0.001 - PRECISION;

    chain_type type;
    type.closed = true;
    type.mode = CRANK_SHAFT_CIRCULAR;

    if (!run.new_chain)
    {
        load_chain(chain, chain_length, "chain.dat");
    }

    report out = simulate(run, type, fix, xyz, correl, neighbour_list, progress, chain, chain_length);

    cout << "Dumping chain" << endl;
    dump_chain(chain, chain_length, "chain.dat");
    cout << "Chain dumped" << endl;

    cout << "Accept rate = " << out.accept_rate << endl;
    cout << "Steps worked = " << out.steps_worked << " (efficiency " << 1.0*run.steps/out.steps_worked << ")" << endl;

    string x;
    cin >> x;
    return 0;
}
