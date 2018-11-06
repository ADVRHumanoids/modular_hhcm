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

#include <ModularBot_plugin.h>

/* Specify that the class XBotPlugin::ModularBot is a XBot RT plugin with name "ModularBot" */
REGISTER_XBOT_PLUGIN_(XBotPlugin::ModularBot)

namespace XBotPlugin
{

bool ModularBot::init_control_plugin(XBot::Handle::Ptr handle)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */

    /* Save robot to a private member. */
    _robot = handle->getRobotInterface();

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */

    _logger = XBot::MatLogger::getLogger("/tmp/ModularBot_log");  

    /* Get ModelInterface */
    _model = XBot::ModelInterface::getModel(handle->getPathToConfigFile());

    /* Get home configuration from SRDF file */
    if (!_robot->getRobotState("home", _theta_home))
    {
        /* If the requested configuration does not exist within the SRDF file,
         * return false. In this case, the plugin will not be executed. */
        return false;
    }

    /* Print the homing */
    std::cout << "_q_home from SRDF : " << _theta_home << std::endl;

    /* Set an homing time */
    //_homing_time = 20;

    /* Print the robot state */
    _robot->print();
    // _model->print();

    /* Get number of joints */
    N = _robot->getJointNum();

    /* load yaml */
    YAML::Node root_param = YAML::LoadFile("/home/edoardo/catkin_ws/src/modular/web/static/yaml/module_joint_M.yaml");

    // P << 5000, 5000, 5000, 5000, 5000, 5000;
    // D << 30, 30, 30, 30, 30, 30;

    YAML::Node control = root_param["control"];

    /* Resize matrices according to number of joints + define inertia and reduced-inertia matrix */
    I.resize(N,N);
    I.setIdentity();
    nu = I * control["nu"].as<double>();
    B = nu * nu * I * control["B"].as<double>();
    B_inv = B.inverse();
    B_theta = B * 0.5;
    B_theta_inv = B_theta.inverse();

    /* Impedance stiffness and damping matrices */
    K_theta = I * control["K_theta"].as<double>();
    D_theta = I * control["D_theta"].as<double>(); 

    /* SEA Spring stiffness and damping matrices */
    K_s = I * control["K_s"].as<double>();
    D_s = I * control["D_s"].as<double>();
    
    /* Motor viscous friction matrix */
    D_m = I  * nu * nu * control["D_m"].as<double>();

    /* Save actual robot q to a private member */
    _robot->getJointPosition(_theta0);
    
    /* Print gravity and mass of the robot */
    _model->getGravity(gravity);
    std::cout << "gravity vector : " << gravity << std::endl;
    mass = _model->getMass();
    std::cout << "mass : " << mass << std::endl;
    // KDL::Vector new_gravity(-9.81, 0.0, 0.0);
    // _model->setGravity(new_gravity);
    // _model->getGravity(gravity);
    // std::cout << "gravity vector : " << gravity << std::endl;

    /* Compute gravity term at equilibrum condition */
    _model->computeGravityCompensation(gcomp);
    std::cout << "gravity term = joint torque at equilibrum : " << gcomp << std::endl;
    std::cout << "model : " << _model << std::endl;
    // _model->setGravity(gravity);
    // std::cout << "gravity vector : " << gravity << std::endl;
    // _model->getGravity(gravity);
    // std::cout << "gravity vector : " << gravity << std::endl;
    // _model->computeGravityCompensation(gcomp);
    // std::cout << "gravity term = joint torque at equilibrum : " << gcomp << std::endl;
    //gcomp = _theta.array().sin()*1*9.81*0.175;

    /* Compute the inverse of the spring stiffness matrix */
    K_s_inv = K_s.inverse();
    std::cout << "K_s_inv : " << K_s_inv << std::endl;
    /* Get displacement at equilibrum */
    _delta = K_s_inv * gcomp;
    std::cout << "delta: " << _delta << std::endl;
    /* Get initial value for link position */
    _q0 = _theta0 - _delta;
    std::cout << "q0 : " << _q0 << std::endl;
   
    /* Debug prints */
    _model->getCOMJacobian(COMJacobian);
    std::cout << "COM jacobian: " << COMJacobian << std::endl;
    _model->getCOM(COM);
    std::cout << "COM: " << COM << std::endl;
    _model->getPose("L_1_link_2_A", "world", link_pose);
    std::cout << "link pose rotation: " << link_pose.rotation() << std::endl;
    std::cout << "link pose translation: " << link_pose.translation() << std::endl;
    _model->getPose("L_1_link_2_B", "world", link_pose);
    std::cout << "link pose rotation: " << link_pose.rotation() << std::endl;
    std::cout << "link pose translation: " << link_pose.translation() << std::endl;
    _model->getPose("L_3_link_2_A", "world", link_pose);
    std::cout << "link pose rotation: " << link_pose.rotation() << std::endl;
    std::cout << "link pose translation: " << link_pose.translation() << std::endl;
    
    /* Print Inertia Matrix */
    _model->getInertiaMatrix(M);
    std::cout << "Inertia Matrix: \n"
              << M << std::endl; //;

    /* Get some joint velocity that does not violate velocity limits */
    Eigen::VectorXd qdot_max;
    _robot->getVelocityLimits(qdot_max);
    max_qdot = qdot_max.minCoeff() * 0.2; // [rad/s]
    std::cout << "Joint velocity limits: \n"
              << qdot_max << std::endl; //;

    /* Get default impedance stiffness and damping */
    _robot->getStiffness(_k0);
    _robot->getDamping(_d0);
    std::cout << "Stiffness: \n"
              << _k0 << std::endl;
    std::cout << "Damping: \n"
              << _d0 << std::endl;

    /* Set to 0 the impedance stiffness and damping. 
    This is done to not use the standard JointImpedanceController, 
    but only send the command as a torque feedforward term to it */
    _k = _k0 * 0;//qdot_max*600;//
    _d = _d0 * 0;//qdot_max/5;//
    std::cout << "Stiffness: \n"
              << _k << std::endl;
    std::cout << "Damping: \n"
              << _d << std::endl;
    _robot->setStiffness(_k);
    _robot->setDamping(_d);

    /* Check that the stiffness and damping are actually 0 */
    _robot->getStiffness(_k);
    _robot->getDamping(_d);
    std::cout << "Actual Stiffness: \n"
              << _k << std::endl;
    std::cout << "Actual Damping: \n"
              << _d << std::endl;

        
    _robot->getJointPosition(_theta);
    _robot->getJointPosition(_thetadot);
    _robot->getJointPosition(_thetadotdot);

    // xout.resize(2*N);
    // xout << _theta, _thetadot;
    // std::cout << "initial state: \n"
    //             << K_theta * xout.head(N) << std::endl; //;

    // dxout.resize(2*N);
    // dxout << _theta, _thetadot;

    return true;
}

void ModularBot::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /ModularBot_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the starting time to a variable */
    _first_loop_time = time;

    /* Save the robot starting config to a variable */
    _robot->getJointPosition(_q0);

    /* Compute gravity term to prevent the robot from falling */
    // _robot->getJointPosition(_q);
    // _robot->getJointVelocity(_qdot);

    // _model->setJointPosition(_q);
    // _model->setJointVelocity(_qdot);
    // _model->update();

    // _model->computeGravityCompensation(gcomp);
    // _logger->add("gcomp", gcomp);

    // _robot->setEffortReference(gcomp);

    // _robot->move();
}

void ModularBot::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /ModularBot_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}

void ModularBot::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    
    /* Get the current position and velocity of the robot joints (link-side) */
    _robot->getJointPosition(_q);
    _robot->getJointVelocity(_qdot);

    // _robot->getMotorPosition(_theta);
    // _robot->getMotorVelocity(_thetadot);

    /* Update the model interface with the current state of the robot */
    _model->setJointPosition(_q);    
    _model->setJointVelocity(_qdot);
    _model->update();
    //_robot->sense();
    // _model->syncFrom(*_robot);
    // _model->update();

    /* Log the link-side joint positions */
    _model->getJointPosition(_qmodel);
    _logger->add("model_pos", _qmodel);
    _logger->add("link_pos", _q);

    /* Debug */
    // _model->getCOM(COM);
    // _model->getCOMAcceleration(COM);
    // std::cout << "com acc : " << COM << std::endl;
    // _model->getPose("L_1_link_2_B", link_pose);
    // _logger->add("link_pose", link_pose.rotation());


    /* Generate a joint-space trajectory using a 5th order polynomial */
    XBot::Utils::FifthOrderTrajectory(_first_loop_time,
                                      _theta0,
                                      _theta_home,
                                      max_qdot,
                                      time,
                                      theta_ref,
                                      thetadot_ref,
                                      _homing_time);

    /* Set the position and velocity references (to be used only if the standard JointImpedanceController from GazeboXBotPlugin wants to be used) */
    // _robot->setPositionReference(theta_ref);
    // _robot->setVelocityReference(thetadot_ref);
    
    /* Compute gravity compensation term */
    _model->computeGravityCompensation(gcomp);
    _logger->add("gcomp", gcomp);
    
    /* Compute analytically gravity compensation term */
    //gcomp_analytic = _theta.array().sin()*2*9.81*0.425;
    //gcomp_analytic = _theta.array().sin()*1*9.81*0.175;
    //_logger->add("gcomp_analytic", gcomp_analytic);    

    /* Iteration to estimate the link-side position in a steady-state configuration starting from motor-side position */
    q_i = _theta;
    for(int i=0; i<=2; i++)
    {   
        _model->setJointPosition(q_i); 
        _model->update();
        _model->computeGravityCompensation(gcomp_i);
        q_i = _theta - K_s_inv * gcomp_i;
    }

    /* Compensation of the link-side gravity torques using the quasi-static estimate of the link-side position */
    _model->setJointPosition(q_i);
    _model->update();
    _model->computeGravityCompensation(gcomp);
    _logger->add("gcomp_iteration", gcomp);

    //tau = K_s * (xout.head(N) - _q) + D_s * (xout.tail(N) - _qdot);
    tau = K_s * (_theta - _q) + D_s * (_thetadot - _qdot);
    //u = - K_theta * (xout.head(N) - theta_ref) - D_theta * (xout.tail(N) - thetadot_ref);
    u = - K_theta * (_theta - theta_ref) - D_theta * (_thetadot - thetadot_ref) + gcomp;
    tau_m = B*B_theta_inv * u + (I - B*B_theta_inv) * tau;
    
    /* Set the torque to send to Gazebo as the joint torque (motor-side dynamics is simulated in the plugin) */
    _robot->setEffortReference(tau);
    _robot->move();

    /* Logs */
    _robot->getJointEffort(tau_meas);
    _logger->add("torque_meas", tau_meas);
    _logger->add("torque_joint", tau);

    _logger->add("torque_cmd", u);
  
    //_logger->add("motor_pos", xout.head(N));
    _logger->add("motor_pos", _theta);
    _logger->add("theta_ref", theta_ref);

    /* Integration using Odeint */
    // motor_dynamics md(theta_ref, thetadot_ref, _q, _qdot, K_theta, D_theta, K_s, D_s, B_theta_inv, D_m);
    // stepper.do_step(md, xout, dxout, (time - _first_loop_time), period);

    /* Integration of the motor acceleration using semi-implicit Euler */
    _thetadotdot = B_inv * (tau_m - D_m*_thetadot - tau);    
    _thetadot += _thetadotdot*period;
    _theta += _thetadotdot * period * period / 2 + _thetadot * period;

    // // Go to homing
    // if( (time - _first_loop_time) <= _homing_time ){
    //     _q = _q0 + 0.5*(1-std::cos(3.1415*(time - _first_loop_time)/_homing_time))*(_q_home-_q0);
    //     _robot->setPositionReference(_q);
    //     _robot->move();
    //     return;
    // }

    /* The following code checks if any command was received from the plugin standard port
     * (e.g. from ROS you can send commands with
     *         rosservice call /ModularBot_cmd "cmd: 'MY_COMMAND_1'"
     * If any command was received, the code inside the if statement is then executed. */

    if (!current_command.str().empty())
    {

        if (current_command.str() == "DLR")
        {
            /* Handle command */
            // _robot->getStiffness(_k0);
            // _robot->getDamping(_d0);
            // std::cout << "Stiffness: \n" << _k0 << std::endl;
            // std::cout << "Damping: \n" << _d0 << std::endl;
        }

        if (current_command.str() == "flush")
        {
            /* Handle command */
            _logger->flush();
        }
    }
}

bool ModularBot::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}

ModularBot::~ModularBot()
{
}

} // namespace XBotPlugin
