for file in *.svg; do inkscape "$file" --export-filename="${file%svg}png" --export-area=0:-300:300:-600 --export-width=1080 --export-height=1080; done
