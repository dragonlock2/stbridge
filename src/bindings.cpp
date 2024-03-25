#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>
#include "stbridge.h"

namespace py = pybind11;
using namespace pybind11::literals;

py::object can_read_simple(Device &d) {
    CANMessage msg;
    if (d.can_read(msg) == -1) {
        return py::none();
    }
    return py::cast(msg);
}

PYBIND11_MODULE(stbridge, m) {
    py::enum_<CANMode>(m, "CANMode")
        .value("NORMAL", CANMode::NORMAL)
        .value("LOOPBACK", CANMode::LOOPBACK)
        .value("LISTENONLY", CANMode::LISTENONLY);

    py::class_<CANMessage>(m, "CANMessage")
        .def(py::init<>())
        .def(py::init<int, int>())
        .def(py::init<int, std::vector<uint8_t>>())
        .def_readwrite("id", &CANMessage::id)
        .def_readwrite("ext", &CANMessage::ext)
        .def_readwrite("rtr", &CANMessage::rtr)
        .def_readwrite("data", &CANMessage::data) // Note can't set individual elements
        .def("__repr__", [](const CANMessage &m){ std::stringstream s; s << m; return s.str(); });

    py::enum_<I2CFreq>(m, "I2CFreq")
        .value("STANDARD", I2CFreq::STANDARD)
        .value("FAST", I2CFreq::FAST)
        .value("FAST_PLUS", I2CFreq::FAST_PLUS);

    py::enum_<GPIODir>(m, "GPIODir")
        .value("INPUT", GPIODir::INPUT)
        .value("OUTPUT", GPIODir::OUTPUT)
        .value("OPEN_DRAIN", GPIODir::OPEN_DRAIN);

    py::enum_<GPIOPull>(m, "GPIOPull")
        .value("NONE", GPIOPull::NONE)
        .value("UP", GPIOPull::UP)
        .value("DOWN", GPIOPull::DOWN);

    m.attr("BRG_GPIO_MAX_NB") = 4; // from bridge.h

    py::enum_<ADCChannel>(m, "ADCChannel")
        .value("TARGET_VOLTAGE", ADCChannel::TARGET_VOLTAGE);

    py::class_<Device>(m, "Device")
        .def("serial", &Device::serial)

        .def("can_set_filter", &Device::can_set_filter)
        .def("can_set_rate", &Device::can_set_rate)
        .def("can_set_mode", &Device::can_set_mode)
        .def("can_write", &Device::can_write)
        .def("can_read", &can_read_simple)

        .def("i2c_set_freq", &Device::i2c_set_freq)
        .def("i2c_write", &Device::i2c_write)
        .def("i2c_read", &Device::i2c_read)

        .def("gpio_set_mode", &Device::gpio_set_mode, "idx"_a, "dir"_a=GPIODir::INPUT, "pull"_a=GPIOPull::NONE)
        .def("gpio_write", &Device::gpio_write)
        .def("gpio_write_all", &Device::gpio_write_all)
        .def("gpio_read", &Device::gpio_read)

        .def("adc_read", &Device::adc_read, "chan"_a=ADCChannel::TARGET_VOLTAGE)

        .def("spi_set_freq", &Device::spi_set_freq)
        .def("spi_set_mode", &Device::spi_set_mode)
        .def("spi_set_bitorder", &Device::spi_set_bitorder)
        .def("spi_set_nss", &Device::spi_set_nss)
        .def("spi_write", &Device::spi_write)
        .def("spi_read", &Device::spi_read);

    py::class_<USBInterface>(m, "USBInterface")
        .def("get_device", &USBInterface::get_device)
        .def("list_devices", &USBInterface::list_devices);
}
