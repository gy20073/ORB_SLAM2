for f in `find . -name "*.jpg"`
do
    convert $f -resize 50% "../images_50/"$f
done
