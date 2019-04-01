#ifndef JOINT_H
#define JOINT_H

#include <Eigen/Dense>

#include "constants.h"

using namespace std;
using namespace Eigen;

class Joint
{
    public:
    Matrix3d triad;
    Vector3d Theta; // Represents rotations relative to the previous triad (always set to zero for segment 0)
    double benergy;

    Joint(Matrix3d triadarg = Matrix3d::Identity(), Vector3d Thetaarg = Vector3d::Zero(), double benergyarg = 0.0) : triad(triadarg), Theta(Thetaarg), benergy(benergyarg)
    {}

    bool operator==(const Joint & obj)
	{
		return (triad == obj.triad) && (Theta == obj.Theta) && (benergy == obj.benergy);
	}

	/*
	 * Write the member variables to stream objects
	 */

    friend std::ostream & operator << (std::ostream &out, const Joint & obj)
	{
		for (int m = 0; m < 3; ++m)
        {
            for (int n = 0; n < 3; ++n)
            {
                out << obj.triad(m,n) << endl;
            }
        }

        for (int index = 0; index < 3; ++index)
        {
            out << obj.Theta(index) << endl;
        }

		out << obj.benergy << endl;

		return out;
	}
	/*
	 * Read data from stream object and fill it in member variables
	 */

	friend std::istream & operator >> (std::istream &in,  Joint &obj)
	{
	    for (int m = 0; m < 3; ++m)
        {
            for (int n = 0; n < 3; ++n)
            {
                in >> obj.triad(m,n);
            }
        }

        for (int index = 0; index < 3; ++index)
        {
            in >> obj.Theta(index);
        }

		in >> obj.benergy;

		return in;
	}
};

#endif // JOINT_H
