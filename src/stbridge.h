#ifndef STBRIDGE_H
#define STBRIDGE_H

#include <memory>
#include <vector>

/*
 * API designed to closely match JABI
 */
class USBInterface;

enum class CANMode {
    NORMAL,
    LOOPBACK,
    LISTENONLY,
};

struct CANMessage {
    int  id;
    bool ext;
    bool rtr;
    std::vector<uint8_t> data;

    CANMessage() : id(0), ext(false), rtr(false) {}
    CANMessage(int id, int req_len) : id(id), ext(id & ~0x7FF), rtr(true), data(req_len, 0) {}
    CANMessage(int id, std::vector<uint8_t> data) : id(id), ext(id & ~0x7FF), rtr(false), data(data) {}
};

std::ostream &operator<<(std::ostream &os, CANMessage const &m);

enum class I2CFreq {
    STANDARD,   // 100kHz
    FAST,       // 400kHz
    FAST_PLUS , // 1MHz
};

enum class GPIODir {
    INPUT,
    OUTPUT,
    OPEN_DRAIN,
};

enum class GPIOPull {
    NONE,
    UP,
    DOWN,
};

enum class ADCChannel {
    TARGET_VOLTAGE = 0,
};

class Device {
public:
    /* Metadata */
    std::string serial();

    /* CAN */
    void can_set_filter(int id, int id_mask, bool rtr, bool rtr_mask);
    void can_set_rate(int bitrate);
    void can_set_mode(CANMode mode);
    void can_write(CANMessage msg);
    int can_read(CANMessage &msg);

    /* I2C */
    void i2c_set_freq(I2CFreq preset);
    void i2c_write(int addr, std::vector<uint8_t> data);
    std::vector<uint8_t> i2c_read(int addr, size_t len);

    /* GPIO */
    void gpio_set_mode(int idx, GPIODir dir=GPIODir::INPUT, GPIOPull pull=GPIOPull::NONE);
    void gpio_write(int idx, bool val);
    void gpio_write_all(int pin_vals);
    bool gpio_read(int idx);

    /* ADC */
    float adc_read(ADCChannel chan=ADCChannel::TARGET_VOLTAGE); // V

    /* SPI */
    void spi_set_freq(int freq);
    void spi_set_mode(int mode);
    void spi_set_bitorder(bool msb);
    void spi_set_nss(bool val);
    void spi_write(std::vector<uint8_t> data);
    std::vector<uint8_t> spi_read(size_t len);

private:
    // using PImpl to workaround libusb.h and pyconfig.h defining ssize_t
    struct device_data;
    std::shared_ptr<device_data> pdata;

    Device(std::shared_ptr<device_data> pdata);
    friend class USBInterface;
};

class USBInterface {
public:
    static Device get_device(std::string sn);
    static std::vector<Device> list_devices();
};

#endif // STBRIDGE_H
