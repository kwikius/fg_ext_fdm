
#include <joystick_dimension.hpp>

namespace {

  /**
   * @brief range of raw Taranis joystick input is nominally +- 32767
   **/
   static constexpr double joystick_half_range = 32767.0;
   
   /**
    * @brief channel numbers of joystick channels
   **/
   static constexpr uint8_t roll_idx = 0;  // roll on ch 0
   static constexpr uint8_t pitch_idx = 1; // pitch on ch 1
   static constexpr uint8_t throttle_idx = 2;
   static constexpr uint8_t yaw_idx = 3;   // yaw on ch 3
   static constexpr uint8_t flap_idx = 4;   // yaw on ch 3
   static constexpr uint8_t spoiler_idx = 5;   // yaw on ch 3

   /**
    * @brief joystick channel direction of raw joystick input, 
    * either 1 or -1 dependent on joystick setup
   **/
   static constexpr int32_t js_sign []= {
      1,   // roll
      1,   // pitch
      1,   // throttle
      -1,   // yaw
      1,   // flap
      1,    // spoiler
      1,   //
      1,   // mode
   };

   template <FlightDimension D>
   constexpr int16_t get_joystick_channel_idx = -1;

   template <>
   constexpr int16_t get_joystick_channel_idx<FlightDimension::Roll> = roll_idx;

   template <>
   constexpr int16_t get_joystick_channel_idx<FlightDimension::Pitch> = pitch_idx;

   template <>
   constexpr int16_t get_joystick_channel_idx<FlightDimension::Yaw> = yaw_idx;

   template <>
   constexpr int16_t get_joystick_channel_idx<FlightDimension::Throttle> = throttle_idx;

   template <>
   constexpr int16_t get_joystick_channel_idx<FlightDimension::Flap> = flap_idx;

   template <>
   constexpr int16_t get_joystick_channel_idx<FlightDimension::Spoiler> = spoiler_idx;
}

template <FlightDimension D>
joystick_dimension<D>::joystick_dimension(quan::joystick const & js)
: m_joystick{js}{}

template <FlightDimension D>
typename joystick_dimension<D>::float_type
joystick_dimension<D>::get_impl() const
{
   int constexpr i = get_joystick_channel_idx<D>;
   float_type const v = (m_joystick.get_channel(i) * js_sign[i]) / joystick_half_range;
   return(flight_dimension_is_signed<D>) ? v : ((v + 1.0) / 2.0) ;
}

template  class joystick_dimension<FlightDimension::Roll>;
template  class joystick_dimension<FlightDimension::Pitch>;
template  class joystick_dimension<FlightDimension::Yaw>;
template  class joystick_dimension<FlightDimension::Throttle>;
template  class joystick_dimension<FlightDimension::Flap>;
template  class joystick_dimension<FlightDimension::Spoiler>;







