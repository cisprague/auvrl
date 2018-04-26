#!/bin/bash

# RF 0
ffmpeg -framerate 120 -i results0/img/empty_img_%d.jpg video/empty0.webm &
ffmpeg -framerate 120 -i results0/img/fewlarge_img_%d.jpg video/fewlarge0.webm &
ffmpeg -framerate 120 -i results0/img/manysmall_img_%d.jpg video/manylarge0.webm &

# RF 1
ffmpeg -framerate 120 -i results1/img/empty_img_%d.jpg video/empty1.webm &
ffmpeg -framerate 120 -i results1/img/fewlarge_img_%d.jpg video/fewlarge1.webm &
ffmpeg -framerate 120 -i results1/img/manysmall_img_%d.jpg video/manylarge1.webm &

# RF 2
ffmpeg -framerate 120 -i results2/img/empty_img_%d.jpg video/empty2.webm &
ffmpeg -framerate 120 -i results2/img/fewlarge_img_%d.jpg video/fewlarge2.webm &
ffmpeg -framerate 120 -i results2/img/manysmall_img_%d.jpg video/manylarge2.webm &

# RF 3
ffmpeg -framerate 120 -i results3/img/empty_img_%d.jpg video/empty3.webm &
ffmpeg -framerate 120 -i results3/img/fewlarge_img_%d.jpg video/fewlarge3.webm &
ffmpeg -framerate 120 -i results3/img/manysmall_img_%d.jpg video/manylarge3.webm &

# RF 4
ffmpeg -framerate 120 -i results3/img/empty_img_%d.jpg video/empty4.webm &
ffmpeg -framerate 120 -i results3/img/fewlarge_img_%d.jpg video/fewlarge4.webm &
ffmpeg -framerate 120 -i results3/img/manysmall_img_%d.jpg video/manylarge4.webm &

wait
