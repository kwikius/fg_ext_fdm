#ifndef FG_EXTERNAL_FLIGHT_CONTROL_SOURCE_HPP_INCLUDED
#define FG_EXTERNAL_FLIGHT_CONTROL_SOURCE_HPP_INCLUDED

#include <quan/quantity_traits.hpp>
#include "flight_dimensions.h"
#include <quan/constrain.hpp>

/**
* @file Flight control source These have no units and furthermore may cause side effeects
* in other dimensions  (For example Throttle may be offset from centre
* flap will change Cm so affect pitch etc
* These effects are dependent on the actual aircraft
**/
template <FlightDimension D>
struct flight_control_source{
   static constexpr FlightDimension flight_dimension = D;
   using float_type = quan::quantity_traits::default_value_type;

   /**
     * @brief get the value of the control_source 
     * @return current value constrained to +-1 for signed and 0 to 1 for unsigned
   **/
   float_type get() const
   { 
      float_type constexpr min_value = flight_dimension_is_signed<D> ? -1 : 0;
      float_type constexpr max_value = 1; 

      return quan::constrain(get_impl(),min_value , max_value);
   }
   virtual ~flight_control_source(){}
protected: 
   virtual float_type get_impl() const = 0;
   constexpr flight_control_source(){};
};

/**
*  @brief default control source returns 0;
**/
template <FlightDimension D>
struct default_flight_control_source : flight_control_source<D>{
   using float_type = quan::quantity_traits::default_value_type;
   float_type get_impl() const override
   {
      return 0;
   }
};

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
