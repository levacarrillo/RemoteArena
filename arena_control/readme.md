# sketchbook 

#INSTALATION

```
sudo apt-get update
sudo apt-get install arduino-mk
sudo apt-get install screen
```

#COMPILE WITH:
```
make
```
#COMPILE AND UPLOAD:
```
make upload
```
#COMPILE, UPLOAD AND WATCH SERIAL SCREEN
```
make upload monitor clean
```
#LIST ALL SCREEN
```
screen -list
```
#OPEN SCREEN
```
screen -r
```
#QUIT SCREEN
```
screen -X quit
```
#PROGRAMS ARE MADE FOR ARDUINO MEGA 2560, IF YOU WANT TO USE ANOTHER ARDUINO CHANGE Makefile
