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
**/
quan::frequency::Hz get_frame_rate(fgfs_telnet & t)
{

   int32_t model_hz;
   if ( t.get("/sim[0]/model-hz",model_hz)){
      m_frame_rate = quan::frequency::Hz{model_hz};
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