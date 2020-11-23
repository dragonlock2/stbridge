#include "stbridge.h"

namespace stbridge {

Brg* m_pBrg = NULL;
STLinkInterface *m_pStlinkIf = NULL;

void open() {
	m_pStlinkIf = new STLinkInterface(STLINK_BRIDGE);

	Brg_StatusT brgStat = BRG_NO_ERR;
	STLinkIf_StatusT ifStat = m_pStlinkIf->LoadStlinkLibrary(""); // doesn't do anything on !WIN32 but needed

	if (ifStat == STLINKIF_NO_ERR) {
		brgStat = Brg::ConvSTLinkIfToBrgStatus(m_pStlinkIf->EnumDevices(NULL, false));
	}
	if (brgStat == BRG_NO_ERR) {
		m_pBrg = new Brg(*m_pStlinkIf);
		m_pBrg->SetOpenModeExclusive(true);
		brgStat = m_pBrg->OpenStlink(0); // open first device
		if (brgStat == BRG_OLD_FIRMWARE_WARNING) {
			std::cout << "OLD FIRMWARE BUT PROBS FINE" << std::endl;
			brgStat = BRG_NO_ERR;
		}
	}
	if (brgStat != BRG_NO_ERR || ifStat != STLINKIF_NO_ERR) {
		close();
		throw fmt::format("BRG_ERR: {} ST_ERR: {}", brgStat, ifStat);
	}
}

void close() {
	// make sure it's closed properly, doesn't actually matter that much
	if(m_pBrg != NULL) {
		m_pBrg->CloseBridge(COM_UNDEF_ALL);
		m_pBrg->CloseStlink();
		delete m_pBrg;
		m_pBrg = NULL;
	}
	if(m_pStlinkIf != NULL) {
		delete m_pStlinkIf;
		m_pStlinkIf = NULL;
	}
}

float getTargetVoltage() {
	checkNull(m_pBrg);
	float v;
	checkError(m_pBrg->GetTargetVoltage(&v));
	return v;
}

uint32_t initSPI(int kHz, bitorderSPI bitorder, modeSPI mode) {
	checkNull(m_pBrg);
	Brg_SpiInitT spiParams;
	spiParams.Direction = SPI_DIRECTION_2LINES_FULLDUPLEX;
	spiParams.Mode = SPI_MODE_MASTER;
	spiParams.DataSize = SPI_DATASIZE_8B;
	switch (mode) {
		case modeSPI::MODE0:
			spiParams.Cpol = SPI_CPOL_LOW;
			spiParams.Cpha = SPI_CPHA_1EDGE;
			break;
		case modeSPI::MODE1:
			spiParams.Cpol = SPI_CPOL_LOW;
			spiParams.Cpha = SPI_CPHA_2EDGE;
			break;
		case modeSPI::MODE2:
			spiParams.Cpol = SPI_CPOL_HIGH;
			spiParams.Cpha = SPI_CPHA_1EDGE;
			break;
		case modeSPI::MODE3:
			spiParams.Cpol = SPI_CPOL_HIGH;
			spiParams.Cpha = SPI_CPHA_2EDGE;
			break;
		default:
			throw "Invalid SPI mode!";
			break;
	}
	switch (bitorder) {
		case bitorderSPI::LSB:
			spiParams.FirstBit = SPI_FIRSTBIT_LSB;
			break;
		case bitorderSPI::MSB:
			spiParams.FirstBit = SPI_FIRSTBIT_MSB;
			break;
		default:
			throw "Invalid SPI bit order!";
			break;
	}
	spiParams.FrameFormat = SPI_FRF_MOTOROLA;
	spiParams.Nss = SPI_NSS_SOFT;
	spiParams.NssPulse = SPI_NSS_NO_PULSE;
	uint32_t spiFreq;
	m_pBrg->GetSPIbaudratePrescal(kHz, &spiParams.Baudrate, &spiFreq);
	spiParams.Crc = SPI_CRC_DISABLE;
	spiParams.CrcPoly = 0;
	spiParams.SpiDelay = DEFAULT_NO_DELAY;
	checkError(m_pBrg->InitSPI(&spiParams));

	return spiFreq;
}

boost::python::list readSPI(uint16_t len) {
	checkNull(m_pBrg);
	uint8_t buff[len];
	checkError(m_pBrg->ReadSPI(buff, len, NULL));

	boost::python::list arr;
	for (auto i = 0; i < len; i++) {
		arr.append(buff[i]);
	}

	return arr;
}

void writeSPI(boost::python::list data) {
	checkNull(m_pBrg);
	uint8_t buff[boost::python::len(data)];

	for (auto i = 0; i < boost::python::len(data); i++) {
		boost::python::extract<uint8_t> ext(data[i]);

		if (ext.check()) {
			buff[i] = ext();
		} else {
			ext(); // throw the error
			return; // not needed
		}
	}

	checkError(m_pBrg->WriteSPI(buff, boost::python::len(data), NULL));
}

void setnssSPI(bool level) {
	checkNull(m_pBrg);
	checkError(m_pBrg->SetSPIpinCS(level ? SPI_NSS_HIGH : SPI_NSS_LOW));
}

void initI2C(int kHz) {
	checkNull(m_pBrg);
	Brg_I2cInitT i2cParams;
	i2cParams.OwnAddr = 0x00; // slave mode, not needed
	i2cParams.AddrMode = I2C_ADDR_7BIT; // defaulting to 7 bit
	i2cParams.AnFilterEn = I2C_FILTER_DISABLE; // disabling filters
	i2cParams.DigitalFilterEn = I2C_FILTER_DISABLE;
	i2cParams.Dnf = 0;

	I2cModeT speed;
	if (kHz <= 100) {
		speed = I2C_STANDARD;
	} else if (kHz <= 400) {
		speed = I2C_FAST;
	} else if (kHz <= 1000) {
		speed = I2C_FAST_PLUS;
	} else {
		throw "Slow down there buckaroo!";
	}

	checkError(m_pBrg->GetI2cTiming(speed, kHz, 0, 0, 0, 0, &i2cParams.TimingReg));

	checkError(m_pBrg->InitI2C(&i2cParams));
}

boost::python::list readI2C(uint16_t addr, uint16_t len) {
	checkNull(m_pBrg);
	if (len == 0) {
		throw "Must read at least 1 byte!";
	}

	uint8_t buff[len];
	checkError(m_pBrg->ReadI2C(buff, addr, len, NULL));

	boost::python::list arr;
	for (auto i = 0; i < len; i++) {
		arr.append(buff[i]);
	}

	return arr;
}

void writeI2C(uint16_t addr, boost::python::list data) {
	checkNull(m_pBrg);
	if (boost::python::len(data) == 0) {
		throw "Must write at least 1 byte!";
	}

	uint8_t buff[boost::python::len(data)];

	for (auto i = 0; i < boost::python::len(data); i++) {
		boost::python::extract<uint8_t> ext(data[i]);

		if (ext.check()) {
			buff[i] = ext();
		} else {
			ext(); // throw the error
			return; // not needed
		}
	}

	checkError(m_pBrg->WriteI2C(buff, addr, boost::python::len(data), NULL));
}

Brg_GpioConfT gpioConf[BRG_GPIO_MAX_NB];

void initGPIO() {
	checkNull(m_pBrg);
	// all inputs initially
	for (int i = 0; i < BRG_GPIO_MAX_NB; i++) {
		gpioConf[i].Mode = GPIO_MODE_INPUT;
		gpioConf[i].Speed = GPIO_SPEED_LOW;
		gpioConf[i].Pull = GPIO_NO_PULL;
	}

	Brg_GpioInitT gpioParams;
	gpioParams.GpioMask = BRG_GPIO_ALL;
	gpioParams.ConfigNb = BRG_GPIO_MAX_NB;
	gpioParams.pGpioConf = gpioConf;

	checkError(m_pBrg->InitGPIO(&gpioParams));
}

void pinmodeGPIO(uint8_t pin, modeGPIO mode) {
	checkNull(m_pBrg);
	if (pin >= BRG_GPIO_MAX_NB) {
		throw "Invalid pin number!";
	}

	switch (mode) {
		case modeGPIO::OUTPUT:
			gpioConf[pin].Mode = GPIO_MODE_OUTPUT;
			gpioConf[pin].Pull = GPIO_NO_PULL;
			break;
		case modeGPIO::INPUT:
			gpioConf[pin].Mode = GPIO_MODE_INPUT;
			gpioConf[pin].Pull = GPIO_NO_PULL;
			break;
		case modeGPIO::INPUT_PULLUP:
			gpioConf[pin].Mode = GPIO_MODE_INPUT;
			gpioConf[pin].Pull = GPIO_PULL_UP;
			break;
		case modeGPIO::INPUT_PULLDOWN:
			gpioConf[pin].Mode = GPIO_MODE_INPUT;
			gpioConf[pin].Pull = GPIO_PULL_DOWN;
			break;
		default:
			throw "Invalid mode!";
			break;
	}
	gpioConf[pin].Speed = GPIO_SPEED_LOW;
	gpioConf[pin].OutputType = GPIO_OUTPUT_PUSHPULL;

	Brg_GpioInitT gpioParams;
	gpioParams.GpioMask = BRG_GPIO_ALL; // for some reason the mask doesn't work
	gpioParams.ConfigNb = BRG_GPIO_MAX_NB;
	gpioParams.pGpioConf = gpioConf;

	checkError(m_pBrg->InitGPIO(&gpioParams));
}

void writeGPIO(uint8_t pin, bool level) {
	checkNull(m_pBrg);
	if (pin >= BRG_GPIO_MAX_NB) {
		throw "Invalid pin number!";
	}

	Brg_GpioValT gpioVals[BRG_GPIO_MAX_NB];
	uint8_t gpioErr;
	gpioVals[pin] = level ? GPIO_SET : GPIO_RESET;
	checkError(m_pBrg->SetResetGPIO(1 << pin, gpioVals, &gpioErr));

	if (gpioErr != 0) {
		throw "GPIO Error?!";
	}
}

bool readGPIO(uint8_t pin) {
	checkNull(m_pBrg);
	if (pin >= BRG_GPIO_MAX_NB) {
		throw "Invalid pin number!";
	}

	Brg_GpioValT gpioVals[BRG_GPIO_MAX_NB];
	uint8_t gpioErr;
	checkError(m_pBrg->ReadGPIO(1 << pin, gpioVals, &gpioErr));

	if (gpioErr != 0) {
		throw "GPIO Error?!";
	}

	return gpioVals[pin] == GPIO_SET;
}

inline void checkError(Brg_StatusT stat) {
	if (stat != BRG_NO_ERR) {
		throw fmt::format("BRG_ERR: {}", stat);
	}
}

inline void checkNull(void* ptr) {
	if (ptr == NULL) {
		throw "You forgot to initialize?! 😱";
	}
}

}