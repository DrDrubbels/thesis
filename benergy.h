#ifndef BENERGY_H
#define BENERGY_H

#include <Eigen/Dense>

#include "joint.h"

using namespace Eigen;

double Theta_elastic_benergy(Vector3d Theta, Vector3d Theta_0, Matrix3d stiffness, double a)
{
    Vector3d Theta_reduced = Theta - Theta_0;
    return 1/(2*a) * Theta_reduced.dot(stiffness * Theta_reduced);
}

double joint_elastic_benergy(Joint seg, Matrix3d stiffness, double a, Vector3d Theta_0) /* Not to be applied to first element in chain
                                                                                        (it can be, and it will yield the correct result, but it's pointless)
                                                                                        --> actually, don't: currently the Thetas of the first element become
                                                                                            NaN, not zero.
                                                                                        --> should be solved now    */
{
    Vector3d Theta_reduced = (seg).Theta - Theta_0;
    return 1/(2*a) * Theta_reduced.dot(stiffness * Theta_reduced);
}

double joint_extension_benergy(Joint seg, double a, double beta, double force) // Never apply to final element in chain
{
    return -1 * beta * force * a * seg.triad(2,2);
}

#endif // BENERGY_H
