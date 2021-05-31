#ifndef FG_EXT_JOYSTICK_CONTROL_SOURCE_HPP_INCLUDED
#define FG_EXT_JOYSTICK_CONTROL_SOURCE_HPP_INCLUDED

#include <flight_controller.hpp>
#include <quan/joystick.hpp>

template <FlightDimension D>
struct joystick_control_source final : flight_control_source<D>
{
   joystick_control_source(quan::joystick const & js);

   using float_type = quan::quantity_traits::default_value_type;
   float_type get_impl() const final;

   private:
     quan::joystick const & m_joystick;
};

#endif // FG_EXT_JOYSTICK_CONTROL_SOURCE_HPP_INCLUDED
