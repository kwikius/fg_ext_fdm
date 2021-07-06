#ifndef ARDUIMU_VISUALISATION_AIRCRAFT_HPP_INCLUDED
#define ARDUIMU_VISUALISATION_AIRCRAFT_HPP_INCLUDED

#include <quan/time.hpp>
#include <quan/torque.hpp>
#include <quan/moment_of_inertia.hpp>
#include <quan/angular_velocity.hpp>
#include <quan/reciprocal_time2.hpp>
#include <quan/three_d/quat.hpp>

/**
   aircraft structure holds the values of constant parameters relating toa particular aircraft
**/

struct aircraft{

   aircraft();

   quan::three_d::vect<quan::moment_of_inertia::kg_m2> 
   get_inertia() const;

   quan::time::s get_Kd() const;

   quan::reciprocal_time2::per_s2 get_Kp() const;

   quan::three_d::vect<quan::torque::N_m> const & 
   get_control_torque() const
   { return m_control_torque;}

   void set_control_torque(quan::three_d::vect<quan::torque::N_m> const & v)
   { m_control_torque = v;}


   float get_roll_control_value() const;
   float get_pitch_control_value() const;
   float get_yaw_control_value() const;

private:
   
//   void set_control_deflections(quan::three_d::vect<quan::angle::deg> const & v) 
//   { m_control_deflections = v;}

   quan::three_d::vect<quan::torque::N_m> m_control_torque;
  // quan::three_d::vect<quan::angle::deg> m_control_deflections;
};

#endif // ARDUIMU_VISUALISATION_AIRCRAFT_HPP_INCLUDED
