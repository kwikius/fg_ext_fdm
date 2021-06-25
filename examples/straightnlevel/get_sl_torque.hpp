#ifndef ARDUIMU_VISUALISATION_GET_PID_TORQUE_HPP_INCLUDED
#define ARDUIMU_VISUALISATION_GET_PID_TORQUE_HPP_INCLUDED

///**
//No idea why GL/glut.h needs to be included but without including it integrator doesnt work correctly!!
//**/
//#include <GL/glut.h>

//#define QUAN_STRAIGHT_N_LEVEL_FILTER

#include <quan/time.hpp>
#include <quan/torque.hpp>
#include <quan/moment_of_inertia.hpp>
#include <quan/angular_velocity.hpp>
#include <quan/three_d/vect.hpp>
#include <quan/reciprocal_time2.hpp>

/**
 * @brief Proportional term
 * @param[in] B The body frame expressed as xyz unit vectors
 * @param[in] I The inertia vector
 * @param[in] accelk the required angular acceleration 
**/
quan::three_d::vect<quan::torque::N_m> 
get_P_torque(
   quan::three_d::vect< quan::three_d::vect<double> > const & B, 
   quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & I, 
   quan::reciprocal_time2::per_s2 const & accelK
);

/**
 * @brief Integral term
 * @param[in] B The body frame expressed as xyz unit vectors
 * @param[in] I The inertia vector
 * @param[in] dt time_step from last integrator call
**/
quan::three_d::vect<quan::torque::N_m>
get_I_torque(
   quan::three_d::vect< quan::three_d::vect<double> > const & B, 
   quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & I, 
   quan::time::ms const & dt
);

/**
* @brief differential term
* @param[in] B The body frame expressed as xyz unit vectors
* @param[in] I The inertia vector
* @param[in] tstop where t = -u/a ( u = current turn_rate a = rotational deceleration )
**/
quan::three_d::vect<quan::torque::N_m> 
get_D_torque(
   quan::three_d::vect<quan::angular_velocity::rad_per_s> const & turn_rate, 
   quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & I,
   quan::time::s const & tstop
);

#endif // ARDUIMU_VISUALISATION_GET_PID_TORQUE_HPP_INCLUDED
