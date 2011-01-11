#!/bin/bash

i=0; cat numbers | while read num; do i=$((i+1)); ./tag.py "$num" && /Applications/Inkscape.app/Contents/Resources/bin/inkscape -f out.svg -A "team$i.pdf"; done
