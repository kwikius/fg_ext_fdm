#!/bin/bash
# net_ctrls
fgfs \
--in-air \
--aircraft=Dragonfly \
--prop:/input/joysticks/js[0]=0 \
--disable-random-objects \
--fg-root=/usr/share/games/flightgear \
--fg-scenery=/usr/share/games/flightgear/Scenery \
--enable-terrasync \
--units-meters \
--altitude=250 \
--lat=50.7381 \
--lon=0.2494 \
--vc=10 \
--glideslope=-3 \
--timeofday=noon \
--native-ctrls=socket,in,20,,5600,udp 
