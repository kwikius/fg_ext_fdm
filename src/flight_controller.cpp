
#include <flight_controller.hpp>

default_flight_control_source<FlightDimension::Roll> const flight_controller::m_default_roll;
default_flight_control_source<FlightDimension::Pitch> const  flight_controller::m_default_pitch;
default_flight_control_source<FlightDimension::Yaw> const  flight_controller::m_default_yaw;
default_flight_control_source<FlightDimension::Throttle> const  flight_controller::m_default_throttle;
default_flight_control_source<FlightDimension::Spoiler> const flight_controller::m_default_spoiler;
default_flight_control_source<FlightDimension::Flap> const  flight_controller::m_default_flap;