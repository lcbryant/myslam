cd image
mkdir L
mkdir R
cd image_0
mogrify -path ../L -format pgm  *.png
cd ..
cd image_1
mogrify -path ../R -format pgm  *.png
echo "done!"
cd ..
cd ..
./elas slam

#cd image1
#mkdir L
#mkdir R
#cd image_0
#mogrify -path ../L -format pgm  *.png
#cd ..
#cd image_1
#mogrify -path ../R -format pgm  *.png
#echo "done!"
#cd ..
#cd ..
#./elas slam
