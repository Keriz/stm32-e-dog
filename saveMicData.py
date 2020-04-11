import serial
import sys
import os
import struct

# The script need Python 3 and pyserial to run: pip3 install pyserial.
# The script need the serial port to open as arg when executing it (e.g. python read_microphones COM56).
# The script reads 10 ms of data for all 4 microphones and save the data in a csv file.
# Robot selector must be in position 1.

def main():
	if len(sys.argv) > 2:
		baud = sys.argv[2]
	else:
		baud = 115200

	if len(sys.argv) == 1:
		print('Please give the serial port to use')
		return
	else:
		fdesc = serial.Serial(sys.argv[1], baudrate=baud, timeout=0.1)

	# flush input and output buffers
	fdesc.reset_input_buffer()
	fdesc.reset_output_buffer()
	
	# clear the command line
	fdesc.write(b'\r\n')
	fdesc.flush()
	data = fdesc.read(10)
	
	# request mic data
	fdesc.write(b'start\r\n')
	fdesc.flush()
	garbage = fdesc.read(10) # garbage??
	header = fdesc.read(5)
	print("garbage: "+ garbage)
	print("header: "+ header)
	
	if(header[0]==0x73 and header[1]==0x74): # correct sync
		data = fdesc.read(2*1024*4)
	else:
		print("data not received correctly")
		return
	
	with open('micleft.csv','w') as file:
		for i in range(0, len(data)-1, 8):
			# Convert to signed int and write to csv
			#mic left complex real
			file.write(str(struct.unpack_from('<f',data, i*4)[0]))
			file.write(",")
			#mic left complex imaginary
			file.write(str(struct.unpack_from('<f',data, i*4+4)[0])))
			flie.write('\n')


if __name__ == '__main__':
	main()