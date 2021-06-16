#ifndef FG_EXTERNAL_FLIGHT_CONTROLLER_HPP_INCLUDED
#define FG_EXTERNAL_FLIGHT_CONTROLLER_HPP_INCLUDED

#include <control_dimension.hpp>
#include <control_source_concept.hpp>
#include "fgfs_telnet.hpp"
#include  <autoconv_net_fdm.hpp>

/**
*  flightcontroller encapsulates the available set of individual control dimensions
*  Though internally held as pointers does not own them so control_dimension's lifetime must
*  exceed flight_controller. For example say you have a joystick and an autopilot;
*  To fly manually assign the joystick flight dimensions to the flight controller.
*  Similarly to fly via autopilot, assign autopilot flight dimensions to the flight controller.
*  The flight controlle has its own default set of flight_dimensions which do nothing
*  and these are installed in the constructor.
**/

struct flight_controller{

#if 1
   using fdm_update_callback_t = bool(*)(autoconv_FGNetFDM const & );
#endif
   using float_type = quan::quantity_traits::default_value_type;

   flight_controller(fgfs_telnet const & t)
   : m_telnet(t)
   ,m_roll{&m_default_roll}
   ,m_pitch{&m_default_pitch}
   ,m_yaw{&m_default_yaw}
   ,m_throttle{&m_default_throttle}
   ,m_flap{&m_default_flap}
   ,m_spoiler{&m_default_spoiler}
   ,m_update_callback{nullptr}
   {}

   template <control_source CS>
   flight_controller(fgfs_telnet & t, CS const & cs)
   :flight_controller(t)
   {
      set_control_source(cs);
   }

   void set_roll_dimension(control_dimension<FlightDimension::Roll> const & p) { m_roll = &p;}
   void set_pitch_dimension(control_dimension<FlightDimension::Pitch> const & p) { m_pitch = &p;}
   void set_yaw_dimension(control_dimension<FlightDimension::Yaw> const & p) { m_yaw = &p;}
   void set_throttle_dimension(control_dimension<FlightDimension::Throttle> const & p) { m_throttle = &p;}
   void set_spoiler_dimension(control_dimension<FlightDimension::Spoiler> const & p) { m_spoiler = &p;}
   void set_flap_dimension(control_dimension<FlightDimension::Flap> const & p) { m_flap = &p;}
   void set_update_callback(fdm_update_callback_t pf) { m_update_callback = pf;}

   float_type get_roll() const { return m_roll->get();}
   float_type get_pitch() const { return m_pitch->get();}
   float_type get_yaw() { return m_yaw->get();}
   float_type get_throttle() const  { return m_throttle->get();}
   float_type get_spoiler() const { return m_spoiler->get();}
   float_type get_flap() const { return m_flap->get();}

   /**
     *@brief output all the modified flight controls to FlightGear
    * but use set_control to only actually write if changed
    **/
   bool update(autoconv_FGNetFDM const & fdm)
   {
      try {
         bool result = true;
         if ( m_update_callback != nullptr){
           result &= m_update_callback(fdm) ;
         }
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

   template <control_source CS>
   void set_control_source( CS const & cs)
   {
      this->set_roll_dimension(cs.roll);
      this->set_pitch_dimension(cs.pitch);
      this->set_yaw_dimension(cs.yaw);
      this->set_throttle_dimension(cs.throttle);
      this->set_flap_dimension(cs.flap);
      this->set_spoiler_dimension(cs.spoiler);
      this->set_update_callback(cs.get_update_callback());
   }
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
   control_dimension<FlightDimension::Roll> const * m_roll;
   control_dimension<FlightDimension::Pitch> const * m_pitch;
   control_dimension<FlightDimension::Yaw> const * m_yaw;
   control_dimension<FlightDimension::Throttle> const * m_throttle;
   control_dimension<FlightDimension::Spoiler> const * m_spoiler;
   control_dimension<FlightDimension::Flap> const * m_flap;
   
   float_type m_flight_controls_cache[8] = {0.0};
   fdm_update_callback_t m_update_callback = nullptr;

   static default_control_dimension<FlightDimension::Roll> const m_default_roll;
   static default_control_dimension<FlightDimension::Pitch> const  m_default_pitch;
   static default_control_dimension<FlightDimension::Yaw> const  m_default_yaw;
   static default_control_dimension<FlightDimension::Throttle> const  m_default_throttle;
   static default_control_dimension<FlightDimension::Spoiler> const m_default_spoiler;
   static default_control_dimension<FlightDimension::Flap> const  m_default_flap;
};

#endif // FG_EXTERNAL_FLIGHT_CONTROLLER_HPP_INCLUDED
