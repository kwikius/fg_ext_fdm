#!/bin/bash
# net_ctrls
fgfs \
--in-air \
--aircraft=Dragonfly \
--prop:/input/joysticks/js[0]=0 \
--disable-random-objects \
--fg-root=/usr/share/games/flightgear \
--fg-scenery=/usr/share/games/flightgear/Scenery \
--units-meters \
--altitude=250 \
--lat=50.7381 \
--lon=0.2494 \
--vc=10 \
--glideslope=-3 \
--native-ctrls=socket,in,50,,5600,udp

