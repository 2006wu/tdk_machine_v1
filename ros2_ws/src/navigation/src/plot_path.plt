set title 'Robot Path'
set xlabel 'X (m)'
set ylabel 'Y (m)'
set grid
set size ratio -1
plot 'path.dat' with linespoints title 'Path'
pause -1
