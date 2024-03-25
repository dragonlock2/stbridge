#include <sstream>
#include "bridge.h"
#include "stbridge.h"

static inline void check_error(Brg_StatusT stat) {
    if (stat != BRG_NO_ERR &&
        stat != BRG_OLD_FIRMWARE_WARNING &&
        stat != BRG_COM_FREQ_MODIFIED) {
        throw std::runtime_error("BRG_ERROR: " + std::to_string(stat));
    }
}

struct Device::device_data {
    std::shared_ptr<STLinkInterface> stlink;
    std::shared_ptr<Brg> brg;
    std::string sn;
    Brg_CanInitT can_params;
    Brg_CanFilterConfT can_filter_params;
    Brg_I2cInitT i2c_params;
    Brg_GpioConfT gpio_conf[BRG_GPIO_MAX_NB];
    Brg_SpiInitT spi_params;

    device_data(std::string sn, std::shared_ptr<Brg> brg, std::shared_ptr<STLinkInterface> stlink) : stlink(stlink), brg(brg), sn(sn) {}
};

Device::Device(std::shared_ptr<device_data> pdata) : pdata(pdata) {
    /* init CAN */
    // http://www.bittiming.can-wiki.info/
    uint32_t baud = 125000; // bps
    uint32_t baud_final;
    pdata->can_params.BitTimeConf.PropSegInTq   = 1;
    pdata->can_params.BitTimeConf.PhaseSeg1InTq = 4;
    pdata->can_params.BitTimeConf.PhaseSeg2InTq = 2;
    pdata->can_params.BitTimeConf.SjwInTq       = 1;
    check_error(pdata->brg->GetCANbaudratePrescal(&pdata->can_params.BitTimeConf, baud, &pdata->can_params.Prescaler, &baud_final));
    if (baud != baud_final) {
        throw std::runtime_error("actual baud rate mismatch: " + std::to_string(baud_final) + "bps");
    }

    // https://stackoverflow.com/questions/57094729/what-is-the-meaning-of-canbus-function-mode-initilazing-settings-for-stm32
    pdata->can_params.bIsAbomEn = false;
    pdata->can_params.bIsAwumEn = false;
    pdata->can_params.bIsNartEn = false;
    pdata->can_params.bIsRflmEn = false;
    pdata->can_params.bIsTxfpEn = false;
    pdata->can_params.Mode = CAN_MODE_NORMAL;

    // default let all messages through
    pdata->can_filter_params.FilterBankNb = 0;
    pdata->can_filter_params.bIsFilterEn  = true;
    pdata->can_filter_params.FilterMode   = CAN_FILTER_ID_MASK;
    pdata->can_filter_params.FilterScale  = CAN_FILTER_32BIT;
    pdata->can_filter_params.Id[0].RTR    = CAN_DATA_FRAME;
    pdata->can_filter_params.Id[0].IDE    = CAN_ID_STANDARD;
    pdata->can_filter_params.Id[0].ID     = 0x00000000;
    pdata->can_filter_params.Mask[0].RTR  = CAN_DATA_FRAME;
    pdata->can_filter_params.Mask[0].IDE  = CAN_ID_STANDARD;
    pdata->can_filter_params.Mask[0].ID   = 0x00000000;
    pdata->can_filter_params.AssignedFifo = CAN_MSG_RX_FIFO0;

    /* I2C */
    pdata->i2c_params.OwnAddr = 0x00; // for responder mode, not needed
    pdata->i2c_params.AddrMode = I2C_ADDR_7BIT; // defaulting to 7 bit
    pdata->i2c_params.AnFilterEn = I2C_FILTER_DISABLE; // disabling filters
    pdata->i2c_params.DigitalFilterEn = I2C_FILTER_DISABLE;
    pdata->i2c_params.Dnf = 0;
    check_error(pdata->brg->GetI2cTiming(I2C_STANDARD, 100, 0, 0, 0, 0, &pdata->i2c_params.TimingReg)); // 100kHz default
    check_error(pdata->brg->InitI2C(&pdata->i2c_params));

    /* GPIO */
    for (int i = 0; i < BRG_GPIO_MAX_NB; i++) {
        pdata->gpio_conf[i].Mode       = GPIO_MODE_INPUT;
        pdata->gpio_conf[i].Speed      = GPIO_SPEED_LOW;
        pdata->gpio_conf[i].Pull       = GPIO_NO_PULL;
        pdata->gpio_conf[i].OutputType = GPIO_OUTPUT_PUSHPULL;
    }
    Brg_GpioInitT gpio_params;
    gpio_params.GpioMask  = BRG_GPIO_ALL;
    gpio_params.ConfigNb  = BRG_GPIO_MAX_NB;
    gpio_params.pGpioConf = pdata->gpio_conf;
    check_error(pdata->brg->InitGPIO(&gpio_params));

    /* SPI */
    pdata->spi_params.Direction   = SPI_DIRECTION_2LINES_FULLDUPLEX;
    pdata->spi_params.Mode        = SPI_MODE_MASTER;
    pdata->spi_params.DataSize    = SPI_DATASIZE_8B;
    pdata->spi_params.Cpol        = SPI_CPOL_LOW;
    pdata->spi_params.Cpha        = SPI_CPHA_1EDGE;
    pdata->spi_params.FirstBit    = SPI_FIRSTBIT_LSB;
    pdata->spi_params.FrameFormat = SPI_FRF_MOTOROLA;
    pdata->spi_params.Nss         = SPI_NSS_SOFT; // software-controlled NSS
    pdata->spi_params.NssPulse    = SPI_NSS_NO_PULSE;
    pdata->spi_params.Crc         = SPI_CRC_DISABLE;
    pdata->spi_params.CrcPoly     = 0;
    pdata->spi_params.SpiDelay    = DEFAULT_NO_DELAY;
    uint32_t spi_freq = 750; // kHz
    uint32_t spi_freq_final;
    check_error(pdata->brg->GetSPIbaudratePrescal(spi_freq, &pdata->spi_params.Baudrate, &spi_freq_final));
    if (spi_freq != spi_freq_final) {
        throw std::runtime_error("actual SPI freq mismatch: " + std::to_string(spi_freq_final) + "kHz");
    }
    check_error(pdata->brg->InitSPI(&pdata->spi_params));
}

std::ostream &operator<<(std::ostream &os, CANMessage const &m) {
    std::stringstream s;
    s << std::hex << std::showbase << "CANMessage(";
    s <<  "id="  << m.id  << ",ext=" << m.ext << ",rtr=" << m.rtr;
    if (m.rtr) {
        s << ",data.size()=" << m.data.size();
    } else {
        s << ",data={";
        for (auto i : m.data) { s << static_cast<int>(i) << ","; }
        s << "}";
    }
    s << ")";
    return os << s.str();
}

std::string Device::serial() {
    return pdata->sn;
}

void Device::can_set_filter(int id, int id_mask, bool rtr, bool rtr_mask) {
    pdata->can_filter_params.Id[0].ID    = id;
    pdata->can_filter_params.Mask[0].ID  = id_mask;
    pdata->can_filter_params.Id[0].RTR   = rtr ? CAN_REMOTE_FRAME : CAN_DATA_FRAME;
    pdata->can_filter_params.Mask[0].RTR = rtr_mask ? CAN_REMOTE_FRAME : CAN_DATA_FRAME;
    check_error(pdata->brg->InitCAN(&pdata->can_params, BRG_INIT_FULL)); // need a transceiver hooked up for this
    check_error(pdata->brg->InitFilterCAN(&pdata->can_filter_params));
    check_error(pdata->brg->StartMsgReceptionCAN()); // never gonna call stop :P
}

void Device::can_set_rate(int bitrate) {
    uint32_t baud = bitrate;
    uint32_t baud_final;
    check_error(pdata->brg->GetCANbaudratePrescal(&pdata->can_params.BitTimeConf, baud, &pdata->can_params.Prescaler, &baud_final));
    if (baud != baud_final) {
        throw std::runtime_error("actual baud rate mismatch: " + std::to_string(baud_final));
    }
    check_error(pdata->brg->InitCAN(&pdata->can_params, BRG_INIT_FULL)); // need a transceiver hooked up for this
    check_error(pdata->brg->InitFilterCAN(&pdata->can_filter_params));
    check_error(pdata->brg->StartMsgReceptionCAN()); // never gonna call stop :P
}

void Device::can_set_mode(CANMode mode) {
    switch (mode) {
        case CANMode::NORMAL:     pdata->can_params.Mode = CAN_MODE_NORMAL;   break;
        case CANMode::LOOPBACK:   pdata->can_params.Mode = CAN_MODE_LOOPBACK; break;
        case CANMode::LISTENONLY: pdata->can_params.Mode = CAN_MODE_SILENT;   break;
        default: throw std::runtime_error("invalid mode"); break;
    }
    check_error(pdata->brg->InitCAN(&pdata->can_params, BRG_INIT_FULL)); // need a transceiver hooked up for this
    check_error(pdata->brg->InitFilterCAN(&pdata->can_filter_params));
    check_error(pdata->brg->StartMsgReceptionCAN()); // never gonna call stop :P
}

void Device::can_write(CANMessage msg) {
    if (msg.data.size() > 8) {
        throw std::runtime_error("message too long!");
    }
    Brg_CanTxMsgT bmsg;
    bmsg.IDE = msg.ext ? CAN_ID_EXTENDED : CAN_ID_STANDARD;
    bmsg.ID  = msg.id;
    bmsg.RTR = msg.rtr ? CAN_REMOTE_FRAME : CAN_DATA_FRAME;
    bmsg.DLC = msg.data.size();
    check_error(pdata->brg->WriteMsgCAN(&bmsg, msg.data.data(), bmsg.DLC));
}

int Device::can_read(CANMessage &msg) {
    uint16_t num_messages = 0;
    check_error(pdata->brg->GetRxMsgNbCAN(&num_messages));
    if (num_messages > 0) {
        Brg_CanRxMsgT bmsg;
        uint8_t data[8] = {0};
        uint16_t data_size; // unused
        check_error(pdata->brg->GetRxMsgCAN(&bmsg, 1, data, 8, &data_size));
        msg.ext  = bmsg.IDE == CAN_ID_EXTENDED;
        msg.id   = bmsg.ID;
        msg.rtr  = bmsg.RTR == CAN_REMOTE_FRAME;
        msg.data = std::vector<uint8_t>(bmsg.DLC, 0);
        if (!msg.rtr) {
            memcpy(msg.data.data(), data, bmsg.DLC);
        }
    }
    return num_messages - 1;
}

void Device::i2c_set_freq(I2CFreq preset) {
    I2cModeT speed;
    int kHz;
    switch (preset) {
        case I2CFreq::STANDARD:  speed = I2C_STANDARD;  kHz = 100;  break;
        case I2CFreq::FAST:      speed = I2C_FAST;      kHz = 400;  break;
        case I2CFreq::FAST_PLUS: speed = I2C_FAST_PLUS; kHz = 1000; break;
        default: throw std::runtime_error("invalid i2c speed!"); break;
    }
    check_error(pdata->brg->GetI2cTiming(speed, kHz, 0, 0, 0, 0, &pdata->i2c_params.TimingReg));
    check_error(pdata->brg->InitI2C(&pdata->i2c_params));
}

void Device::i2c_write(int addr, std::vector<uint8_t> data) {
    if (data.size() == 0) {
        throw std::runtime_error("must write at least 1 byte!");
    }
    check_error(pdata->brg->WriteI2C(data.data(), addr, data.size(), NULL));
}

std::vector<uint8_t> Device::i2c_read(int addr, size_t len) {
    if (len == 0) {
        throw std::runtime_error("must read at least 1 byte!");
    }
    std::vector<uint8_t> data(len, 0);
    check_error(pdata->brg->ReadI2C(data.data(), addr, len, NULL));
    return data;
}

void Device::gpio_set_mode(int idx, GPIODir dir, GPIOPull pull) {
    if (idx >= BRG_GPIO_MAX_NB) {
        throw std::runtime_error("invalid pin number!");
    }
    switch (dir) {
        case GPIODir::INPUT:
            pdata->gpio_conf[idx].Mode = GPIO_MODE_INPUT;
            break;
        case GPIODir::OUTPUT:
            pdata->gpio_conf[idx].Mode = GPIO_MODE_OUTPUT;
            pdata->gpio_conf[idx].OutputType = GPIO_OUTPUT_PUSHPULL;
            break;
        case GPIODir::OPEN_DRAIN:
            pdata->gpio_conf[idx].Mode = GPIO_MODE_OUTPUT;
            pdata->gpio_conf[idx].OutputType = GPIO_OUTPUT_OPENDRAIN;
            break;
        default:
            throw std::runtime_error("invalid direction!");
            break;
    }
    switch (pull) {
        case GPIOPull::NONE: pdata->gpio_conf[idx].Pull = GPIO_NO_PULL;   break;
        case GPIOPull::UP:   pdata->gpio_conf[idx].Pull = GPIO_PULL_UP;   break;
        case GPIOPull::DOWN: pdata->gpio_conf[idx].Pull = GPIO_PULL_DOWN; break;
        default: throw std::runtime_error("invalid pull!"); break;
    }
    Brg_GpioInitT gpio_params;
    gpio_params.GpioMask  = BRG_GPIO_ALL;
    gpio_params.ConfigNb  = BRG_GPIO_MAX_NB;
    gpio_params.pGpioConf = pdata->gpio_conf;
    check_error(pdata->brg->InitGPIO(&gpio_params));
}

void Device::gpio_write(int idx, bool val) {
    if (idx >= BRG_GPIO_MAX_NB) {
        throw std::runtime_error("invalid pin number!");
    }
    Brg_GpioValT gpio_vals[BRG_GPIO_MAX_NB];
    uint8_t gpio_err;
    gpio_vals[idx] = val ? GPIO_SET : GPIO_RESET;
    check_error(pdata->brg->SetResetGPIO(1 << idx, gpio_vals, &gpio_err));
    if (gpio_err != 0) {
        throw std::runtime_error("GPIO error??");
    }
}

void Device::gpio_write_all(int pin_vals) {
    if (pin_vals >= 1 << BRG_GPIO_MAX_NB) {
        throw std::runtime_error("pin_vals too large!");
    }
    Brg_GpioValT gpio_vals[BRG_GPIO_MAX_NB];
    uint8_t gpio_err;
    for (int i = 0; i < BRG_GPIO_MAX_NB; ++i) {
        gpio_vals[i] = pin_vals & (1 << i) ? GPIO_SET : GPIO_RESET;
    }
    check_error(pdata->brg->SetResetGPIO((1 << BRG_GPIO_MAX_NB) - 1, gpio_vals, &gpio_err));
    if (gpio_err != 0) {
        throw std::runtime_error("GPIO error??");
    }
}

bool Device::gpio_read(int idx) {
    if (idx >= BRG_GPIO_MAX_NB) {
        throw std::runtime_error("invalid pin number!");
    }
    Brg_GpioValT gpio_vals[BRG_GPIO_MAX_NB];
    uint8_t gpio_err;
    check_error(pdata->brg->ReadGPIO(1 << idx, gpio_vals, &gpio_err));
    if (gpio_err != 0) {
        throw std::runtime_error("GPIO error??");
    }
    return gpio_vals[idx] == GPIO_SET;
}

float Device::adc_read(ADCChannel chan) {
    if (chan != ADCChannel::TARGET_VOLTAGE) {
        throw std::runtime_error("invalid ADC channel!");
    }
    float v;
    check_error(pdata->brg->GetTargetVoltage(&v));
    return v;
}

void Device::spi_set_freq(int freq) {
    uint32_t spi_freq = freq / 1000;
    uint32_t spi_freq_final;
    check_error(pdata->brg->GetSPIbaudratePrescal(spi_freq, &pdata->spi_params.Baudrate, &spi_freq_final));
    if (spi_freq != spi_freq_final) {
        throw std::runtime_error("actual SPI freq mismatch: " + std::to_string(spi_freq_final) + "kHz");
    }
    check_error(pdata->brg->InitSPI(&pdata->spi_params));
}

void Device::spi_set_mode(int mode) {
    switch (mode) {
        case 0:
            pdata->spi_params.Cpol = SPI_CPOL_LOW;
            pdata->spi_params.Cpha = SPI_CPHA_1EDGE;
            break;
        case 1:
            pdata->spi_params.Cpol = SPI_CPOL_LOW;
            pdata->spi_params.Cpha = SPI_CPHA_2EDGE;
            break;
        case 2:
            pdata->spi_params.Cpol = SPI_CPOL_HIGH;
            pdata->spi_params.Cpha = SPI_CPHA_1EDGE;
            break;
        case 3:
            pdata->spi_params.Cpol = SPI_CPOL_HIGH;
            pdata->spi_params.Cpha = SPI_CPHA_2EDGE;
            break;
        default:
            throw std::runtime_error("invalid SPI mode!");
            break;
    }
    check_error(pdata->brg->InitSPI(&pdata->spi_params));
}

void Device::spi_set_bitorder(bool msb) {
    pdata->spi_params.FirstBit = msb ? SPI_FIRSTBIT_MSB : SPI_FIRSTBIT_LSB;
    check_error(pdata->brg->InitSPI(&pdata->spi_params));
}

void Device::spi_set_nss(bool val) {
    check_error(pdata->brg->SetSPIpinCS(val ? SPI_NSS_HIGH : SPI_NSS_LOW));
}

void Device::spi_write(std::vector<uint8_t> data) {
    check_error(pdata->brg->WriteSPI(data.data(), data.size(), NULL));
}

std::vector<uint8_t> Device::spi_read(size_t len) {
    std::vector<uint8_t> data(len, 0);
    check_error(pdata->brg->ReadSPI(data.data(), len, NULL));
    return data;
}

/* device acquisition */
Device USBInterface::get_device(std::string sn) {
    auto stlink = std::make_shared<STLinkInterface>(STLINK_BRIDGE);
    if (stlink->LoadStlinkLibrary("") != STLINKIF_NO_ERR) {
        throw std::runtime_error("couldn't load stlink library??");
    }

    auto brg = std::make_shared<Brg>(*stlink);
    check_error(Brg::ConvSTLinkIfToBrgStatus(stlink->EnumDevices(NULL, false)));
    check_error(brg->OpenStlink(sn.c_str(), true));

    return Device(std::make_shared<Device::device_data>(sn, brg, stlink));
}

std::vector<Device> USBInterface::list_devices() {
    auto stlink = std::make_shared<STLinkInterface>(STLINK_BRIDGE);
    if (stlink->LoadStlinkLibrary("") != STLINKIF_NO_ERR) {
        throw std::runtime_error("couldn't load stlink library??");
    }

    uint32_t num_devices = 0;
    if (stlink->EnumDevices(&num_devices, false) != STLINKIF_NO_ERR) {
        throw std::runtime_error("failed to enumerate devices??");
    }

    std::vector<Device> devices;
    devices.reserve(num_devices);
    for (uint32_t i = 0; i < num_devices; i++) {
        STLink_DeviceInfo2T info;
        if (stlink->GetDeviceInfo2(i, &info, sizeof info) != STLINKIF_NO_ERR) {
            throw std::runtime_error("failed to get device info??");
        }
        info.EnumUniqueId[SERIAL_NUM_STR_MAX_LEN - 1] = 0; // just in case
        std::string sn(info.EnumUniqueId);

        auto brg = std::make_shared<Brg>(*stlink);
        brg->SetOpenModeExclusive(false);
        check_error(brg->OpenStlink(sn.c_str(), true));
        devices.push_back(Device(std::make_shared<Device::device_data>(sn, brg, stlink)));
    }
    return devices;
}
