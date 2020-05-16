C:
cd C:\Program Files\FlightGear

SET FG_ROOT=C:\Program Files\FlightGear\data

SET FG_SCENERY=C:\Program Files\FlightGear\data\Scenery;C:\Program Files\FlightGear\scenery;C:\Program Files\FlightGear\terrasync

.\\bin\fgfs --aircraft=f18 --fdm=null --enable-auto-coordination --disable-freeze  --native-fdm=socket,in,30,localhost,5502,udp --enable-clouds --start-date-lat=2020:03:05:09:00:00 --enable-sound --visibility=5000 --in-air --disable-freeze --airport=LKPR --runway=06 --altitude=2000 --heading=0 --offset-distance=0 --offset-azimuth=0 --enable-rembrandt