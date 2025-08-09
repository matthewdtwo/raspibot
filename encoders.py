from __future__ import print_function
import qwiic_dual_encoder_reader
import time
import sys

def runExample():

	#  roughly 2400 

	print("\nSparkFun Qwiic Dual Encoder Reader   Example 1\n")
	myEncoders = qwiic_dual_encoder_reader.QwiicDualEncoderReader()

	if myEncoders.connected == False:
		print("The Qwiic Dual Encoder Reader device isn't connected to the system. Please check your connection", \
			file=sys.stderr)
		return

	myEncoders.begin()

	while True:

		print("Count1: %d, Count2: %s" % (myEncoders.count1, \
			myEncoders.count2, \
			))

		time.sleep(.3)

if __name__ == '__main__':
	try:
		runExample()
	except (KeyboardInterrupt, SystemExit) as exErr:
		print("\nEnding Example 1")
		sys.exit(0)