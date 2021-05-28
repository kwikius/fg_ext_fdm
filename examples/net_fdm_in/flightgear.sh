#!/bin/bash
#!/bin/bash
fgfs \
--in-air \
--aircraft=easystar  \
--fg-scenery=/usr/share/games/flightgear/Scenery \
--fg-aircraft=/home/andy/cpp/projects/aerfpilot/Tools/autotest/aircraft/ \
--units-meters \
--altitude=800 \
--lat=50.7381 \
--lon=0.2494 \
--vc=10 \
--glideslope=-3 \
--native-fdm=socket,out,50,127.0.0.1,5500,udp

