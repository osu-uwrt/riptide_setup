import serial

def main():

	ser = serial.Serial('/dev/ttyUSB7', baudrate=9600, timeout=None)
	while (1):
		print ser.readline()

if __name__ == "__main__":
	main()
