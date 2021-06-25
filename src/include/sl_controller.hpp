#ifndef EXT_FDM_SL_CONTROLLER_HPP_INCLUDED
#define EXT_FDM_SL_CONTROLLER_HPP_INCLUDED

#include "flight_controller.hpp"

struct sl_controller final : abc_flight_controller{

  sl_controller(fgfs_telnet const & t)
   : abc_flight_controller{t}{}

   float_type get_roll() const  override;
   float_type get_pitch() const  override;
   float_type get_yaw() const  override;

   float_type get_throttle() const override{return 1; }
   float_type get_spoiler() const override{return 0; }
   float_type get_flap() const  override{return 0; }

   bool pre_update(autoconv_FGNetFDM const & fdm, quan::time::ms const & time_step) override;

};
#endif // EXT_FDM_SL_CONTROLLER_HPP_INCLUDED
