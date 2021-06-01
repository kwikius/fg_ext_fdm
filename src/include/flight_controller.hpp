#ifndef FG_EXTERNAL_FLIGHT_CONTROLLER_HPP_INCLUDED
#define FG_EXTERNAL_FLIGHT_CONTROLLER_HPP_INCLUDED

#include <control_dimension.hpp>

struct flight_controller{

   flight_controller()
   :m_roll{&m_default_roll}
   ,m_pitch{&m_default_pitch}
   ,m_yaw{&m_default_yaw}
   ,m_throttle{&m_default_throttle}
   ,m_flap{&m_default_flap}
   ,m_spoiler{&m_default_spoiler}
   {

   }
   void set_roll_dimension(control_dimension<FlightDimension::Roll> const & p) { m_roll = &p;}
   void set_pitch_dimension(control_dimension<FlightDimension::Pitch> const & p) { m_pitch = &p;}
   void set_yaw_dimension(control_dimension<FlightDimension::Yaw> const & p) { m_yaw = &p;}
   void set_throttle_dimension(control_dimension<FlightDimension::Throttle> const & p) { m_throttle = &p;}
   void set_spoiler_dimension(control_dimension<FlightDimension::Spoiler> const & p) { m_spoiler = &p;}
   void set_flap_dimension(control_dimension<FlightDimension::Flap> const & p) { m_flap = &p;}

   using float_type = quan::quantity_traits::default_value_type;

   float_type get_roll() const { return m_roll->get();}
   float_type get_pitch() const { return m_pitch->get();}
   float_type get_yaw() { return m_yaw->get();}
   float_type get_throttle() const  { return m_throttle->get();}
   float_type get_spoiler() const { return m_spoiler->get();}
   float_type get_flap() const { return m_flap->get();}
private:

   control_dimension<FlightDimension::Roll> const * m_roll;
   control_dimension<FlightDimension::Pitch> const * m_pitch;
   control_dimension<FlightDimension::Yaw> const * m_yaw;
   control_dimension<FlightDimension::Throttle> const * m_throttle;
   control_dimension<FlightDimension::Spoiler> const * m_spoiler;
   control_dimension<FlightDimension::Flap> const * m_flap;

   static default_control_dimension<FlightDimension::Roll> const m_default_roll;
   static default_control_dimension<FlightDimension::Pitch> const  m_default_pitch;
   static default_control_dimension<FlightDimension::Yaw> const  m_default_yaw;
   static default_control_dimension<FlightDimension::Throttle> const  m_default_throttle;
   static default_control_dimension<FlightDimension::Spoiler> const m_default_spoiler;
   static default_control_dimension<FlightDimension::Flap> const  m_default_flap;
};

#endif // FG_EXTERNAL_FLIGHT_CONTROLLER_HPP_INCLUDED
