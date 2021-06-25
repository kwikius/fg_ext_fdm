#!/bin/bash
/usr/games/fgfs \
--in-air \
--aircraft=Easystar \
--prop:/input/joysticks/js[0]=0 \
--enable-terrasync \
--fg-scenery=/home/andy/.fgfs/TerraSync \
--fg-aircraft=/home/andy/cpp/projects/aerfpilot/Tools/autotest/aircraft/ \
--units-meters \
--altitude=2000 \
--lat=50.7381 \
--lon=0.2494 \
--vc=10 \
--glideslope=-3 \
--native-fdm=socket,out,10,127.0.0.1,5600,udp \
--telnet=socket,bi,30,localhost,5501,tcp


