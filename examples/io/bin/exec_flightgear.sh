#!/bin/bash
/usr/games/fgfs \
--in-air \
--aircraft=EasyStar \
--prop:/input/joysticks/js[0]=0 \
--units-meters \
--altitude=800 \
--lat=51.56856 \
--lon=-4.2869 \
--vc=10 \
--glideslope=-3 \
--native-fdm=socket,out,50,127.0.0.1,5600,udp \
--telnet=socket,bi,50,localhost,5501,tcp


 