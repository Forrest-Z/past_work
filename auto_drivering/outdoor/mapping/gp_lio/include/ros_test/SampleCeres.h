//
// Created by wchen on 2019/12/2.
//

#ifndef SRC_SAMPLECERES_H
#define SRC_SAMPLECERES_H

#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>

#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/numeric_diff_cost_function.h>


namespace gp_lio{
    namespace SampleCeres {

        //user defined cost functor: residual, 最后在cost function里表达的是 0.5×residual^2
        struct CostFunctor {
            template <typename T>
            bool operator()(const T* const x, T* residual) const {
                residual[0] = T(10.0) - x[0];
                return true;
            }
        };

        //Generally speaking we recommend automatic differentiation instead of numeric differentiation.
        //The use of C++ templates makes automatic differentiation efficient, whereas numeric differentiation
        // is expensive, prone to numeric errors, and leads to slower convergence.
        struct NumericDiffCostFunctor {
            bool operator()(const double* const x, double* residual) const {
                residual[0] = 10.0 - x[0];
                return true;
            }
        };

        // Analytic Derivatives
        //  it may be the case that it is more efficient to compute the derivatives in closed form instead of
        //  relying on the chain rule used by the automatic differentiation code.
        class QuadraticCostFunction : public ceres::SizedCostFunction<1, 1> {
        public:
            virtual ~QuadraticCostFunction() {}
            //an input array of parameters, an output array residuals_ for
            //residuals_ and an output array jacobians_ for Jacobians.
            virtual bool Evaluate(double const* const* parameters,
                                  double* residuals,
                                  double** jacobians) const {
                const double x = parameters[0][0];
                residuals[0] = 10 - x;

                // Compute the Jacobian if asked for.
                if (jacobians != NULL && jacobians[0] != NULL) {
                    jacobians[0][0] = -1;
                }
                return true;
            }
        };


        // cost_function_ = 0.5*(10-x)^2
        double FirstExample(double &initial_x){

            double x = initial_x;

            ros::Time tic = ros::Time::now();

            //build the problem
            ceres::Problem problem;

            // Set up the only cost function (also known as residual). This uses
            // auto-differentiation to obtain the derivative (jacobian).
//            ceres::CostFunction* cost_function_ =
//                    new ceres::AutoDiffCostFunction< CostFunctor, 1, 1>(new CostFunctor);
//            problem.AddResidualBlock(cost_function_, NULL, &x);

            // Numeric Differentiation
            ceres::CostFunction* cost_function =
                    new ceres::NumericDiffCostFunction<NumericDiffCostFunctor, ceres::CENTRAL, 1, 1>(
                            new NumericDiffCostFunctor);
            problem.AddResidualBlock(cost_function, NULL, &x);

            //Run the solver
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = true;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            std::cout << summary.FullReport() << "\n";
            std::cout << "x: " << initial_x
                      << " -> " << x << "\n";
            return  (ros::Time::now() - tic).toSec();
        };



    }
}



#endif //SRC_SAMPLECERES_H
