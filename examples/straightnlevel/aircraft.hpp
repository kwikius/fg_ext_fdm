#ifndef ARDUIMU_VISUALISATION_AIRCRAFT_HPP_INCLUDED
#define ARDUIMU_VISUALISATION_AIRCRAFT_HPP_INCLUDED


#include <quan/time.hpp>
#include <quan/torque.hpp>
#include <quan/moment_of_inertia.hpp>
#include <quan/angular_velocity.hpp>
#include <quan/reciprocal_time2.hpp>
#include <quan/three_d/quat.hpp>
//#include <straight_n_level/pid/get_sl_torque.hpp>

struct aircraft{

   aircraft();
   
   quan::three_d::quat<double> const & 
   get_pose() const 
   { return m_pose;}

   void set_pose(quan::three_d::quat<double> const & q)
   { m_pose = q;}

   quan::three_d::vect<quan::angular_velocity::rad_per_s> const &
   get_angular_velocity()const 
   {return m_angular_velocity;}

   void set_angular_velocity(
      quan::three_d::vect<quan::angular_velocity::rad_per_s> const & v
   )
   { m_angular_velocity = v;}

   quan::three_d::vect<quan::moment_of_inertia::kg_m2> 
   get_inertia() const;

   quan::time::s get_Kd() const;

   quan::reciprocal_time2::per_s2 get_Kp() const;

   quan::three_d::vect<quan::torque::N_m> const & 
   get_control_torque() const
   { return m_control_torque;}

   void set_control_torque(quan::three_d::vect<quan::torque::N_m> const & v)
   { m_control_torque = v;}

//   quan::three_d::vect<quan::angle::deg>
//   get_control_deflections() const { return m_control_deflections;}

   float get_roll_control_value() const;
   float get_pitch_control_value() const;
   float get_yaw_control_value() const;

private:
   
   void set_control_deflections(quan::three_d::vect<quan::angle::deg> const & v) 
   { m_control_deflections = v;}

   quan::three_d::quat<double> 
   m_pose;

   quan::three_d::vect<quan::angular_velocity::rad_per_s>
   m_angular_velocity;

   quan::three_d::vect<quan::torque::N_m> m_control_torque;
   quan::three_d::vect<quan::angle::deg> m_control_deflections;
};

#endif // ARDUIMU_VISUALISATION_AIRCRAFT_HPP_INCLUDED
