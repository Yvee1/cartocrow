ffmpeg -framerate 60 -pattern_type glob -i '*.png' \
  -c:v libx264 -pix_fmt yuv420p -crf 18 out.mp4
