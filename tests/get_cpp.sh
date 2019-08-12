echo "START COMPILATION..."

pio_output="../speeduino/speeduino.ino.cpp"
my_output="src/speeduino.cpp"

python -m platformio --version > /dev/null 2>&1
if [ $? -ne 0 ]; then
  echo "INSTALLING PLATFORMIO..."
  python -m pip install --user platformio > /dev/null

  if [ $? -ne 0 ]; then
    echo "UNABLE TO INSTALL PLATFORMIO, EXITING"
    exit 1
  fi

  echo "PLATFORMIO INSTALLED"
fi

#start platformio compilation
cd ../ && python -m platformio run -e megaatmega2560 >/dev/null &

#wait until the cpp file is created by platformio
while ! test -f $pio_output; do
  sleep 0.5;
done;

echo "CPP FILE FIRST APPARITION"

#try to get the latest file, so it is fully post-processed
while test -f $pio_output; do
  cp $pio_output ./ >/dev/null 2>&1;
  sleep 0.1;
done;

echo "GOT CPP FILE LAST ITERATION"

#copy speeduino/speeduino.cpp
sed -e 's/# [0-9]*/\/\/\0/' -e 's/#line [0-9]*/\/\/\0/' speeduino.ino.cpp > $my_output
rm speeduino.ino.cpp

echo "WAITING FOR PLATFORMIO TO FINISH"

wait

echo "SUCCESS!"

