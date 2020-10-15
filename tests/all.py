import stbridge as st

import time

if __name__ == '__main__':
	st.open()
	print('Connected! Starting tests...')

	# Target Voltage Test
	print(f'Target Voltage: {round(st.getTargetVoltage(), 5)}v')

	# SPI Test
	print('\nInitiating SPI at 1MHz... ', end='')
	spiFreq = st.initSPI(1000, st.bitorderSPI.LSB, st.modeSPI.MODE3)
	print('Initiated at', spiFreq, '\bkHz')

	print('Sending out 0x69... ', end='')
	st.setnssSPI(0)
	st.writeSPI([0x69])
	st.setnssSPI(1)

	st.setnssSPI(0)
	print('Read in', hex(st.readSPI(1)[0]).upper(), '\b!')
	st.setnssSPI(1)

	# I2C Test
	print('\nScanning for I2C devices at 1MHz...')
	st.initI2C(1000)

	for addr in range(128):
		try:
			if st.readI2C(addr, 1):
				print('Found!:', hex(addr))
		except Exception as e:
			pass

	# TODO CAN Test

	# GPIO Test
	print("\nTesting GPIO...")
	st.initGPIO()

	for i in range(st.numGPIO):
		print('Flashing GPIO', i, '\b...')
		st.pinmodeGPIO(i, st.modeGPIO.OUTPUT)
		st.writeGPIO(i, 0)
		for _ in range(6):
			st.writeGPIO(i, not st.readGPIO(i))
			time.sleep(0.1)

	print('Reading GPIO using internal pull-ups, so disconnect any loads')
	for i in range(st.numGPIO):
		st.pinmodeGPIO(i, st.modeGPIO.INPUT_PULLUP if (i % 2 == 0) else st.modeGPIO.INPUT_PULLDOWN)
	for i in range(st.numGPIO):
		print("Reading GPIO", i, "\b...", "SUCCESS" if (st.readGPIO(i) == (i % 2 == 0)) else "FAIL")
	for i in range(st.numGPIO):
		st.pinmodeGPIO(i, st.modeGPIO.INPUT_PULLDOWN if (i % 2 == 0) else st.modeGPIO.INPUT_PULLUP)
	for i in range(st.numGPIO):
		print("Reading GPIO", i, "\b...", "SUCCESS" if (st.readGPIO(i) != (i % 2 == 0)) else "FAIL")

	# Cleanup
	st.close()

	print('\nDone!')