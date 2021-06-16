#ifndef FG_EXT_JOYSTICK_HPP_INCLUDED
#define FG_EXT_JOYSTICK_HPP_INCLUDED

#include <autoconv_net_fdm.hpp>
#include <control_source_concept.hpp>
#include <joystick_dimension.hpp>
/**
  model of Ccontrol_source
**/

struct joystick_t;

namespace detail{
  
  template<>
  inline constexpr bool is_control_source_impl<joystick_t> = true;

}

struct joystick_t{

  joystick_t( const char * path)
  : m_js{path}
  , roll{m_js}
  , pitch{m_js}
  , yaw{m_js}
  , throttle{m_js}
  , flap{m_js}
  , spoiler{m_js}
  {}

   
private:
   quan::joystick m_js;
public:
   joystick_dimension<FlightDimension::Roll> const roll;
   joystick_dimension<FlightDimension::Pitch> const pitch;
   joystick_dimension<FlightDimension::Yaw> const yaw;
   joystick_dimension<FlightDimension::Throttle> const throttle;
   joystick_dimension<FlightDimension::Flap> const  flap;
   joystick_dimension<FlightDimension::Spoiler> const spoiler;

   using fdm_update_callback_t = bool(*)(autoconv_FGNetFDM const &);
   // joystick doesnt require an up callback
   // since it doesnt depend on update of fdm
   fdm_update_callback_t get_update_callback()const { return nullptr;}
};

#endif // FG_EXT_JOYSTICK_HPP_INCLUDED
