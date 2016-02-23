import serial
import time

s = serial.Serial("/dev/ttyACM0", 9600)
letters = ["a","s","d","f","j","k","l",";"]

pattern = ['jdl', 'j', 'a', 'jj',
        'jl','a','jdj','j;',
         'jl','j','aj','jl',
         'jjl','j','as','jk']

patLength = len(pattern)
time.sleep(2)

for i in range(2*patLength):
    time.sleep(0.25)
    s.write(pattern[i%patLength])

