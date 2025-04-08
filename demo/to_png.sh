for file in *.svg; do inkscape "$file" --export-filename="${file%svg}png" --export-area=25:-60:335:-370 --export-width=1000 --export-height=1000 -b white; done
