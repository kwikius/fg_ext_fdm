#ifndef FG_EXT_FLIGHT_CONTROL_SOURCE_HPP_INCLUDED
#define FG_EXT_FLIGHT_CONTROL_SOURCE_HPP_INCLUDED

#include <quan/quantity_traits.hpp>
#include <quan/constrain.hpp>
#include "flight_dimensions.h"

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
struct default_flight_control_source final : flight_control_source<D>{
   using float_type = quan::quantity_traits::default_value_type;
   float_type get_impl() const final
   {
      return 0;
   }
};

#endif // FG_EXT_FLIGHT_CONTROL_SOURCE_HPP_INCLUDED
