#ifndef FG_EXT_FLIGHT_MODE_HPP_INCLUDED
#define FG_EXT_FLIGHT_MODE_HPP_INCLUDED


   enum class flight_mode : uint8_t {
      Manual,
      StraightnLevel
   };

   flight_mode get_flight_mode();

#endif // FG_EXT_FLIGHT_MODE_HPP_INCLUDED
