#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
public:
    vector<double> mpc_x;
    vector<double> mpc_y;

    double prev_delta{0.0};
    double prev_a{0.5};

    MPC();

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

};

#endif /* MPC_H */
