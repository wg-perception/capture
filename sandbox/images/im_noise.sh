#!/bin/bash -ex

for i in f1 f2 f3 f4 f5 f6 f7 f8 f9 f10
do
	convert -size 850x1100 -density 100x100 \
	  plasma:fractal $i.png
    #convert to pdf
	convert $i.png \
		-charcoal 10 -paint 10 -sigmoidal-contrast 15x50% -negate \
		   res_$i.png
    convert res_$i.png res_$i.pdf
    rm $i.png
done
