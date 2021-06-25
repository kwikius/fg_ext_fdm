#ifndef FG_EXT_FDM_MANUAL_FLIGHT_CONTROLLER_HPP_INCLUDED
#define FG_EXT_FDM_MANUAL_FLIGHT_CONTROLLER_HPP_INCLUDED

#include "flight_controller.hpp"
#include "joystick.hpp"

struct manual_flight_controller final : abc_flight_controller{

   manual_flight_controller(fgfs_telnet const & t,const char * joystick_path)
   : abc_flight_controller{t}, m_joystick{joystick_path}
   {}

   float_type get_roll() const override{ return m_joystick.roll.get();}
   float_type get_pitch() const override{ return m_joystick.pitch.get();}
   float_type get_yaw() const override{ return m_joystick.yaw.get();}
   float_type get_throttle() const override { return m_joystick.throttle.get();}
   float_type get_spoiler() const override{ return m_joystick.spoiler.get();}
   float_type get_flap() const override{ return m_joystick.flap.get();}

   private:
      joystick_t m_joystick;
};

#endif // FG_EXT_FDM_MANUAL_FLIGHT_CONTROLLER_HPP_INCLUDED
