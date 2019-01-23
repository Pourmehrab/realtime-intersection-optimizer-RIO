# Intersection Geometry Spec

[Google Maps markers can be exported to CSV](https://www.quora.com/Is-there-a-way-to-export-Google-map-pointers-latitude-longitude-to-an-excel-sheet). We used high-precision GPS to extract Lane GPS, and Google Maps to create the optimization zones for the RTS test intersection. 

## Lanes

Lanes are represented by a sequence of GPS points, stored in a CSV file. The CSV header is ```Latitude,Longitude```. The first data line contains the GPS point corresponding to the center of the stop bar. Subsequent lines contain the GPS points leading away from the stop bar towards the far end of the lane. Files are named ```Lane_X.csv``` where X is the pre-determined unique lane number for the intersection.

## Optimization Zones

Zones are described by 4 GPS points for straight lanes forming a polygon. The front of the zone is approximately ```min_dist_to_stop_bar``` meters from the stop bar for the particular lane. They are stored in a CSV file under INTERSECTION_NAME/opt_zones.csv.
The 