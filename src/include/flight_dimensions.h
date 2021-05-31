#ifndef FG_EXTERNAL_TEST_FLIGHT_DIMENSIONS_H_INCLUDED
#define FG_EXTERNAL_TEST_FLIGHT_DIMENSIONS_H_INCLUDED


/**
* available flight dimensions
**/

enum class FlightDimension : uint8_t {
   Yaw,
   Pitch,
   Roll,
   Throttle,
   Flap,
   Spoiler
};

template <FlightDimension FD>
inline constexpr bool flight_dimension_is_signed = true;

template <>
inline constexpr bool flight_dimension_is_signed<FlightDimension::Throttle> = false;

template <>
inline constexpr bool flight_dimension_is_signed<FlightDimension::Flap> = false;

template <>
inline constexpr bool flight_dimension_is_signed<FlightDimension::Spoiler> = false;
  
#endif // FG_EXTERNAL_TEST_FLIGHT_DIMENSIONS_H_INCLUDED
