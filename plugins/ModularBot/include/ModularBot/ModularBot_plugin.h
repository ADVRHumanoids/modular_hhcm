/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef ModularBot_PLUGIN_H_
#define ModularBot_PLUGIN_H_

#include <XCM/XBotControlPlugin.h>

#include <boost/numeric/odeint.hpp>
//#include <gsl/gsl_integration.h>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include <boost/numeric/odeint/external/eigen/eigen_algebra.hpp>
//#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>

// namespace boost {
// namespace numeric {
// namespace odeint {

// template<typename B,int S1,int S2,int O, int M1, int M2>
// struct algebra_dispatcher< Eigen::Matrix<B,S1,S2,O,M1,M2> >
// {
//     typedef vector_space_algebra algebra_type;
// };

// }}}

// namespace Eigen {

// template<typename D, int Rows, int Cols>
// Matrix<D, Rows, Cols>
// abs(Matrix<D, Rows, Cols> const& m) {
//     return m.cwiseAbs();
// }

// }

namespace XBotPlugin {

/**
 * @brief ModularBot XBot RT Plugin
 *
 **/
class ModularBot : public XBot::XBotControlPlugin
{

public:

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual bool close();

    virtual void on_start(double time);

    virtual void on_stop(double time);
    
    virtual ~ModularBot();

protected:

    virtual void control_loop(double time, double period);

private:

    XBot::RobotInterface::Ptr _robot;
    Eigen::VectorXd _q0, _q_home, _q, _qdot, _k, _d, _k0, _d0, _qref, q_i;
    double _time, _homing_time, _first_loop_time;

    XBot::MatLogger::Ptr _logger;

    XBot::ModelInterface::Ptr _model;
    double max_qdot;
    Eigen::VectorXd q_ref, qdot_ref, theta_ref, thetadot_ref;

    int N;

    Eigen::MatrixXd M, B, B_inv, B_theta, B_theta_inv, I, K_theta, D_theta, D_m, K_s, D_s, nu, K_s_inv;

    Eigen::VectorXd tau, tau_m, u, tau_meas;

    Eigen::VectorXd _theta, _thetadot, _thetadotdot, _theta0, _theta_home, _delta;

    Eigen::VectorXd gcomp, gcomp_analytic, gcomp_i;

    Eigen::VectorXd _qmodel;

    KDL::Vector gravity;

    KDL::Vector COM;

    double mass;

    Eigen::MatrixXd COMJacobian;

    Eigen::Affine3d link_pose;
    
    //ODEINT

    /* The type of container used to hold the state vector */
    typedef Eigen::VectorXd state_type;

    state_type dxout, xout;

    /*type of stepper*/
    boost::numeric::odeint::runge_kutta_dopri5<state_type,double,state_type,double, boost::numeric::odeint::vector_space_algebra> stepper; //
    //boost::numeric::odeint::symplectic_euler<state_type,state_type,double,state_type,state_type,double, boost::numeric::odeint::vector_space_algebra> stepper; //

    /*class defining the system*/
    class motor_dynamics {

        Eigen::VectorXd theta_ref;
        Eigen::VectorXd thetadot_ref;
        Eigen::VectorXd q;
        Eigen::VectorXd qdot;
        Eigen::MatrixXd K_theta;
        Eigen::MatrixXd D_theta;
        Eigen::MatrixXd K_s;
        Eigen::MatrixXd D_s;
        Eigen::MatrixXd B_theta_inv;
        Eigen::MatrixXd D_m;

    public:
        motor_dynamics(Eigen::VectorXd theta_ref_i, 
                        Eigen::VectorXd thetadot_ref_i, 
                        Eigen::VectorXd q_i, 
                        Eigen::VectorXd qdot_i, 
                        Eigen::MatrixXd K_theta_i,
                        Eigen::MatrixXd D_theta_i,
                        Eigen::MatrixXd K_s_i,
                        Eigen::MatrixXd D_s_i,
                        Eigen::MatrixXd B_theta_inv_i,
                        Eigen::MatrixXd D_m_i) : 

        theta_ref(theta_ref_i), 
        thetadot_ref(thetadot_ref_i),
        q(q_i),
        qdot(qdot_i),
        K_theta(K_theta_i),
        D_theta(D_theta_i),
        K_s(K_s_i),
        D_s(D_s_i),
        B_theta_inv(B_theta_inv_i),
        D_m(D_m_i) { }
    
        void operator() ( const state_type &x , state_type &dxdt, const double t )
        {
            //"trick" to avoid errors given by the size of the dxdt vector 
            dxdt=x;

            //
            Eigen::VectorXd u_inte = - K_theta * (x.head(x.size()/2) - theta_ref) - D_theta * (x.tail(x.size()/2) - thetadot_ref);
            // std::cout << "u_inte: \n"
            //     << u_inte << std::endl; //;
            Eigen::VectorXd tau_int = K_s * (x.head(x.size()/2) - q) + D_s * (x.tail(x.size()/2) - qdot);
            // std::cout << "tau_int: \n"
            //     << tau_int << std::endl; //;
            dxdt.head(dxdt.size()/2) = x.tail(x.size()/2);
            // std::cout << "dxdt vel: \n"
            //     << dxdt.head(dxdt.size()/2) << std::endl; //;
            dxdt.tail(dxdt.size()/2) = B_theta_inv * (u_inte - D_m * x.tail(x.size()/2) - tau_int); //, u_inte
            // std::cout << "dxdt acc): \n"
            //     << dxdt.tail(dxdt.size()/2) << std::endl; //;
        }
    };

};

}

#endif // ModularBot_PLUGIN_H_
