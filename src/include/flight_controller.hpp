#ifndef FG_EXTERNAL_FLIGHT_CONTROL_SOURCE_HPP_INCLUDED
#define FG_EXTERNAL_FLIGHT_CONTROL_SOURCE_HPP_INCLUDED

#include <flight_control_source.hpp>

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
   void set_roll_source(flight_control_source<FlightDimension::Roll> const & p) { m_roll = &p;}
   void set_pitch_source(flight_control_source<FlightDimension::Pitch> const & p) { m_pitch = &p;}
   void set_yaw_source(flight_control_source<FlightDimension::Yaw> const & p) { m_yaw = &p;}
   void set_throttle_source(flight_control_source<FlightDimension::Throttle> const & p) { m_throttle = &p;}
   void set_spoiler_source(flight_control_source<FlightDimension::Spoiler> const & p) { m_spoiler = &p;}
   void set_flap_source(flight_control_source<FlightDimension::Flap> const & p) { m_flap = &p;}

   using float_type = quan::quantity_traits::default_value_type;

   float_type get_roll() const { return m_roll->get();}
   float_type get_pitch() const { return m_pitch->get();}
   float_type get_yaw() { return m_yaw->get();}
   float_type get_throttle() const  { return m_throttle->get();}
   float_type get_spoiler() const { return m_spoiler->get();}
   float_type get_flap() const { return m_flap->get();}
private:

   flight_control_source<FlightDimension::Roll> const * m_roll;
   flight_control_source<FlightDimension::Pitch> const * m_pitch;
   flight_control_source<FlightDimension::Yaw> const * m_yaw;
   flight_control_source<FlightDimension::Throttle> const * m_throttle;
   flight_control_source<FlightDimension::Spoiler> const * m_spoiler;
   flight_control_source<FlightDimension::Flap> const * m_flap;

   static default_flight_control_source<FlightDimension::Roll> const m_default_roll;
   static default_flight_control_source<FlightDimension::Pitch> const  m_default_pitch;
   static default_flight_control_source<FlightDimension::Yaw> const  m_default_yaw;
   static default_flight_control_source<FlightDimension::Throttle> const  m_default_throttle;
   static default_flight_control_source<FlightDimension::Spoiler> const m_default_spoiler;
   static default_flight_control_source<FlightDimension::Flap> const  m_default_flap;
};

#endif // FG_EXTERNAL_FLIGHT_CONTROL_SOURCE_HPP_INCLUDED
