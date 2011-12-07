convert -size 500x500  plasma:fractal fractal.png
convert fractal.png  -charcoal 10 -paint 10 -sigmoidal-contrast 15x50% -negate \
           result.png
