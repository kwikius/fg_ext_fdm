#ifndef FG_EXTERNAL_FLIGHT_CONTROLLER_HPP_INCLUDED
#define FG_EXTERNAL_FLIGHT_CONTROLLER_HPP_INCLUDED

#include <control_dimension.hpp>
#include "fgfs_telnet.hpp"
#include <autoconv_net_fdm.hpp>
#include <quan/time.hpp>

struct abc_flight_controller{

   using float_type = quan::quantity_traits::default_value_type;

   virtual float_type get_roll() const = 0;
   virtual float_type get_pitch() const  = 0;
   virtual float_type get_yaw() const = 0;
   virtual float_type get_throttle() const  = 0; 
   virtual float_type get_spoiler() const = 0;
   virtual float_type get_flap() const  = 0;

   virtual bool pre_update(autoconv_FGNetFDM const & fdm, quan::time::ms const & time_step) { return true;}

   bool update(autoconv_FGNetFDM const & fdm, quan::time::ms const & time_step)
   {
      try {
         bool result = pre_update(fdm,time_step);
         return 
            result &&
            set_control("/controls/flight/aileron",this->get_roll(),
               m_flight_controls_cache[static_cast<int>(FlightDimension::Roll)]) &&
            set_control("/controls/flight/elevator",this->get_pitch(),
               m_flight_controls_cache[static_cast<int>(FlightDimension::Pitch)]) &&
            set_control("/controls/flight/rudder",this->get_yaw(),
               m_flight_controls_cache[static_cast<int>(FlightDimension::Yaw)]) &&
            set_control("/controls/engines/engine[0]/throttle",this->get_throttle(),
               m_flight_controls_cache[static_cast<int>(FlightDimension::Throttle)]);
      }catch(...){
         fprintf(stderr,"set controls failed\n");
         return false;
      }
   };

protected:
   abc_flight_controller(fgfs_telnet const & t)
   : m_telnet(t){}
private:
   /// @brief only set a property if its value changed
   bool set_control ( const char * prop, float_type const & latest, float_type& cached)
   {
      if ( latest == cached){
         return true;
      }else{
         cached = latest;
         return m_telnet.set(prop,latest);
      }
   };
   fgfs_telnet const & m_telnet;
   float_type m_flight_controls_cache[8] = {0.0};
};

#endif // FG_EXTERNAL_FLIGHT_CONTROLLER_HPP_INCLUDED
