import stbridge as st
import time

if __name__ == '__main__':
    dev = st.USBInterface.list_devices()[0]
    print('Connected! Starting tests...')

    # Target Voltage Test
    print(f'Target Voltage: {round(dev.adc_read(), 5)}V')

    # SPI Test
    print('\nInitiating SPI at 187kHz, MODE3, LSB first... ', end='')
    dev.spi_set_freq(187000)
    dev.spi_set_mode(3)
    dev.spi_set_bitorder(False)

    print('Sending out 0x69... ', end='')
    dev.spi_set_nss(False)
    dev.spi_write([0x69])
    dev.spi_set_nss(True)

    dev.spi_set_nss(False)
    print('Read in', hex(dev.spi_read(1)[0]).upper(), '\b!')
    dev.spi_set_nss(True)

    # I2C Test
    print('\nScanning for I2C devices at 1MHz...')
    dev.i2c_set_freq(st.I2CFreq.FAST_PLUS)

    for addr in range(128):
        try:
            if dev.i2c_read(addr, 1):
                print('Found!:', hex(addr))
        except:
            pass

    # CAN Test
    print("\nInitializing CAN at 1Mbps...")
    try:
        dev.can_set_rate(125000)

        print("Sending some CAN messages...")
        for _ in range(5):
            dev.can_write(st.CANMessage(42, list(b'hola!')))
            print("Sent!")
            time.sleep(0.1)

        print("Listening to CAN for 1 sec...")
        time.sleep(1.0)
        while (msg := dev.can_read()):
            print("\t", msg)
    except:
        print("Failed CAN! Make sure transceiver and another node connected!")

    # GPIO Test
    print("\nTesting GPIO...")
    NUM_GPIO = 4
    for i in range(NUM_GPIO):
        print('Flashing GPIO', i, '\b...')
        dev.gpio_set_mode(i, st.GPIODir.OUTPUT)
        dev.gpio_write(i, 0)
        for _ in range(6):
            dev.gpio_write(i, not dev.gpio_read(i))
            time.sleep(0.1)

    print('Reading GPIO using internal pull-ups, so disconnect any loads')
    for i in range(NUM_GPIO):
        dev.gpio_set_mode(i, st.GPIODir.INPUT, st.GPIOPull.UP if (i % 2 == 0) else st.GPIOPull.DOWN)
    for i in range(NUM_GPIO):
        print("Reading GPIO", i, "\b...", "SUCCESS" if (dev.gpio_read(i) == (i % 2 == 0)) else "FAIL")
    for i in range(NUM_GPIO):
        dev.gpio_set_mode(i, st.GPIODir.INPUT, st.GPIOPull.DOWN if (i % 2 == 0) else st.GPIOPull.UP)
    for i in range(NUM_GPIO):
        print("Reading GPIO", i, "\b...", "SUCCESS" if (dev.gpio_read(i) != (i % 2 == 0)) else "FAIL")

    print('\nDone!')
