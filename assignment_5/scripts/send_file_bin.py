import serial
import argparse

parser = argparse.ArgumentParser(description='Send a file via UART')
parser.add_argument('-t',  '--tty', default = '/dev/ttyUSB1', type = str, help= 'Path to the tty device')
parser.add_argument('-b',  '--baudrate', default = '115200', type = int, help ='Baudrate')
parser.add_argument('file', type = str, help ='Path to the file to send')
parser.add_argument('-d',  '--debug', help="Debug mode", action="store_true")
parser.add_argument('-f',  '--follow', help="Follow mode", action="store_true")
args = parser.parse_args()

def printErr (*args):
  import builtins
  builtins.print (*args)
  
if not args.debug:
  def print(*args):
    pass


ser = serial.Serial(args.tty, args.baudrate)

f = open(args.file, "rb")
  
print ("File: {}".format(args.file) )

#print (ser.readline()) # waits for "Started" string

  
s=0
b = f.read(1)
i=0
while b:
  print ("\n\n\n\n####### Sending byte {} #######".format(i))
  print (b)
  print ("#######################")
  ser.write(b)
 # if args.debug:
  #  resp=ser.readline()
   # print (resp)
  s+=int.from_bytes(b, byteorder='big')
  b = f.read(1)
  i+=1

print ("Done!")
if args.follow:
  while(1):
    printErr (ser.readline())


f.close()
  

