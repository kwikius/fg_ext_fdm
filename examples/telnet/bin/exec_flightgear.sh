#!/bin/bash
#!/bin/bash
fgfs \
--in-air \
--aircraft=ask13  \
--prop:/input/joysticks/js[0]=0 \
--units-meters \
--altitude=800 \
--lat=50.7381 \
--lon=0.2494 \
--vc=10 \
--glideslope=-3 \
--telnet=socket,bi,50,localhost,5501,tcp


