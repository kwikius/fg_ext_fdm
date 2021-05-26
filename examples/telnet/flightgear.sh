#!/bin/bash
#!/bin/bash
fgfs \
--in-air \
--aircraft=EasyStar  \
--fg-scenery=/usr/share/games/flightgear/Scenery \
--units-meters \
--altitude=800 \
--lat=50.7381 \
--lon=0.2494 \
--vc=10 \
--glideslope=-3 \
--telnet=socket,bi,5,localhost,5501,tcp


