Get-ChildItem "\\wsl.localhost\Ubuntu\home\steven\Downloads\test\cartocrow\airplanes" -Filter *.svg | 
Foreach-Object {
     & "C:\Program Files\Inkscape\bin\inkscape.com" $_.FullName --export-type="png" --export-area=30:0:1030:-1000 --export-width=1000 --export-height=1000 -b white
}