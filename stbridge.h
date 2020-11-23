#ifndef STBRIDGE_H
#define STBRIDGE_H

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/python.hpp>

#include <iostream>
#include <string>
// #include <format> // no compiler support yet :(
#include <fmt/format.h> // once support added switch back to std::format

#include "bridge.h"

/* 
 *  Overarching design philosophy is simplicity, this is made for the average user
 */

namespace stbridge {
	void open();
	void close();

	float getTargetVoltage();

	enum class bitorderSPI { LSB = SPI_FIRSTBIT_LSB, MSB = SPI_FIRSTBIT_MSB };
	enum class modeSPI { MODE0, MODE1, MODE2, MODE3 };

	uint32_t initSPI(int kHz, bitorderSPI bitorder, modeSPI mode);
	boost::python::list readSPI(uint16_t len);
	void writeSPI(boost::python::list data);
	void setnssSPI(bool level);

	void initI2C(int kHz);
	boost::python::list readI2C(uint16_t addr, uint16_t len);
	void writeI2C(uint16_t addr, boost::python::list data);

	enum class modeGPIO { OUTPUT, INPUT, INPUT_PULLUP, INPUT_PULLDOWN };

	void initGPIO();
	void pinmodeGPIO(uint8_t pin, modeGPIO mode);
	void writeGPIO(uint8_t pin, bool level);
	bool readGPIO(uint8_t pin);

	void checkError(Brg_StatusT stat);
}

void translate_c_str(const char* s) {
	PyErr_SetString(PyExc_RuntimeError, s);
}

void translate_str(std::string& s) {
	PyErr_SetString(PyExc_RuntimeError, s.c_str());
}

BOOST_PYTHON_MODULE(stbridge) {
	boost::python::register_exception_translator<const char*>(translate_c_str);
	boost::python::register_exception_translator<std::string&>(translate_str);

	boost::python::def("open", stbridge::open);
	boost::python::def("close", stbridge::close);

	boost::python::def("getTargetVoltage", stbridge::getTargetVoltage);

	boost::python::enum_<stbridge::bitorderSPI>("bitorderSPI")
		.value("LSB", stbridge::bitorderSPI::LSB)
		.value("MSB", stbridge::bitorderSPI::MSB);

	boost::python::enum_<stbridge::modeSPI>("modeSPI")
		.value("MODE0", stbridge::modeSPI::MODE0)
		.value("MODE1", stbridge::modeSPI::MODE1)
		.value("MODE2", stbridge::modeSPI::MODE2)
		.value("MODE3", stbridge::modeSPI::MODE3);

	boost::python::def("initSPI", stbridge::initSPI);
	boost::python::def("readSPI", stbridge::readSPI);
	boost::python::def("writeSPI", stbridge::writeSPI);
	boost::python::def("setnssSPI", stbridge::setnssSPI);

	boost::python::def("initI2C", stbridge::initI2C);
	boost::python::def("readI2C", stbridge::readI2C);
	boost::python::def("writeI2C", stbridge::writeI2C);

	boost::python::enum_<stbridge::modeGPIO>("modeGPIO")
		.value("OUTPUT", stbridge::modeGPIO::OUTPUT)
		.value("INPUT", stbridge::modeGPIO::INPUT)
		.value("INPUT_PULLUP", stbridge::modeGPIO::INPUT_PULLUP)
		.value("INPUT_PULLDOWN", stbridge::modeGPIO::INPUT_PULLDOWN);

	boost::python::scope().attr("numGPIO") = BRG_GPIO_MAX_NB;

	boost::python::def("initGPIO", stbridge::initGPIO);
	boost::python::def("pinmodeGPIO", stbridge::pinmodeGPIO);
	boost::python::def("writeGPIO", stbridge::writeGPIO);
	boost::python::def("readGPIO", stbridge::readGPIO);
}

#endif