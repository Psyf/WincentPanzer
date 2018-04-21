plot [-100:100][-100:100] "lidar_better_map_reading.dat" u 1:2 pt 4 ps 0.2 lc rgb "orange", "scale.dat" u 1:2 pt 4 ps 0.4 lc rgb "red"
pause 0.1
reread
