#include <sensors.hpp>

///flightgear-2020.3.8/src/Time/TimeManager.cxx
//sim/model-hz

namespace {

   QUAN_QUANTITY_LITERAL(frequency,Hz)

   quan::frequency::Hz m_frame_rate = 0_Hz;
}

/**
 * @brief get FlightGear current framerate via telnet
 * Generally only do at startup
 * alternatively could try at runtime via /fdm[0]/jsbsim[0]/simulation[0]/sim-time-sec (double)
 * but difficult to know guarantee that is tied to fdm output
**/
quan::frequency::Hz get_frame_rate(fgfs_telnet & t)
{

   int32_t frame_rate;
   if ( t.get("/sim[0]/frame-rate",frame_rate)){
      m_frame_rate = quan::frequency::Hz{frame_rate};
      return  m_frame_rate;
   }else{
      throw std::runtime_error("get_frame_rate from telnet failed\n");
   }
}
/**
* @brief get locally stored framerate ( dont telnet to Flightgear)
**/
quan::frequency::Hz get_frame_rate()
{
   if (m_frame_rate != 0_Hz){
      return m_frame_rate;
   }else{
      throw std::runtime_error("get_frame_rate not initialised\n"); 
   }
}