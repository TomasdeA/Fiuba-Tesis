#include "gpio_rosbag_controller/gpio_interface.hpp"

#include <gpiod.h>

#include <stdexcept>
#include <string>

namespace gpio_rosbag_controller
{

    GpioInterface::GpioInterface(
        const std::string &chip_name,
        unsigned int line_offset,
        GpioDirection direction,
        const std::string &consumer,
        int initial_value)
        : direction_(direction), offset_(line_offset)
    {
        chip_ = gpiod_chip_open_by_name(chip_name.c_str());
        if (!chip_)
        {
            throw std::runtime_error(
                "GpioInterface: cannot open chip '" + chip_name +
                "'. Is libgpiod installed and the chip accessible?");
        }

        line_ = gpiod_chip_get_line(chip_, line_offset);
        if (!line_)
        {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
            throw std::runtime_error(
                "GpioInterface: cannot get line " + std::to_string(line_offset) +
                " from chip '" + chip_name + "'");
        }

        int ret = 0;
        if (direction == GpioDirection::Input)
        {
            ret = gpiod_line_request_input(line_, consumer.c_str());
        }
        else
        {
            ret = gpiod_line_request_output(line_, consumer.c_str(), initial_value);
        }

        if (ret < 0)
        {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
            line_ = nullptr;
            throw std::runtime_error(
                "GpioInterface: cannot request line " + std::to_string(line_offset) +
                " as " + (direction == GpioDirection::Input ? "input" : "output") +
                ". Another process may hold this GPIO.");
        }
    }

    GpioInterface::~GpioInterface()
    {
        if (line_)
        {
            gpiod_line_release(line_);
            line_ = nullptr;
        }
        if (chip_)
        {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
        }
    }

    int GpioInterface::read() const
    {
        const int v = gpiod_line_get_value(line_);
        if (v < 0)
        {
            throw std::runtime_error(
                "GpioInterface: read failed on line " + std::to_string(offset_));
        }
        return v;
    }

    void GpioInterface::write(int value)
    {
        const int ret = gpiod_line_set_value(line_, value ? 1 : 0);
        if (ret < 0)
        {
            throw std::runtime_error(
                "GpioInterface: write failed on line " + std::to_string(offset_));
        }
    }

} // namespace gpio_rosbag_controller
