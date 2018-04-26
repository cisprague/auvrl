#!/bin/bash

ffmpeg \
-i empty0.webm \
-i empty1.webm \
-i empty2.webm \
-i fewlarge0.webm \
-i fewlarge1.webm \
-i fewlarge2.webm \
-i manylarge0.webm \
-i manylarge1.webm \
-i manylarge2.webm \
-filter_complex \
"
[0:v][1:v]hstack[top]; \
[top][2:v]hstack[top]; \
[3:v][4:v]hstack[middle]; \
[middle][5:v]hstack[middle]; \
[6:v][7:v]hstack[bottom]; \
[bottom][8:v]hstack[bottom]; \
[top][middle]vstack[top]; \
[top][bottom]vstack
" \
grid.mp4 &
