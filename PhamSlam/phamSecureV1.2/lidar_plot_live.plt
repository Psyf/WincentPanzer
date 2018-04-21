plot [-100:100][-100:100] "lidar_map_reading.dat" using 1:2 pt 4 ps 0.2 lc rgb "purple", "scale.dat" using 1:2 pt 4 ps 0.4 lc rgb "red"
pause 0.1
reread
