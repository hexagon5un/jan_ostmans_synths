import serial
import time

s = serial.Serial("/dev/ttyACM0", 9600)
letters = ["a","s","d","f","j","k","l",";"]

pattern = ['fdk', 'f', 'a', 'fj',
        'fk','a','fdj','f',
         'fk','f','aj','fk',
         'fjk','f','as','f']

patLength = len(pattern)
time.sleep(2)

for i in range(2*patLength):
    time.sleep(0.25)
    s.write(pattern[i%patLength])

