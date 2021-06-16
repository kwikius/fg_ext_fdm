#!/bin/bash
fgfs \
--in-air \
--aircraft=ask13 \
--prop:/input/joysticks/js[0]=0 \
--units-meters \
--altitude=250 \
--lat=50.7381 \
--lon=0.2494 \
--vc=10 \
--glideslope=-3 \
--fdm=external \
--native-fdm=socket,in,50,,5500,udp

