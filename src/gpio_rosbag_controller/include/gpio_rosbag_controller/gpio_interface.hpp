#pragma once

#include <cstdint>
#include <string>

// Forward-declare libgpiod types to avoid polluting every TU with gpiod.h
struct gpiod_chip;
struct gpiod_line;

namespace gpio_rosbag_controller
{

    /// Direction of a GPIO line.
    enum class GpioDirection
    {
        Input,
        Output,
    };

    /// Abstraction over a single libgpiod GPIO line.
    /// Owns the chip and line handles; closes them on destruction.
    class GpioInterface
    {
    public:
        /// @param chip_name  e.g. "gpiochip0"
        /// @param line_offset  BCM GPIO number (same as gpiochip0 line offset on RPi)
        /// @param direction  Input or Output
        /// @param consumer   String label for the kernel (shows up in /sys and GPIO tools)
        /// @param initial_value  Only used for Output direction (0 or 1)
        GpioInterface(
            const std::string &chip_name,
            unsigned int line_offset,
            GpioDirection direction,
            const std::string &consumer = "gpio_rosbag_ctrl",
            int initial_value = 0);

        ~GpioInterface();

        // Non-copyable, non-movable (owns raw handles)
        GpioInterface(const GpioInterface &) = delete;
        GpioInterface &operator=(const GpioInterface &) = delete;
        GpioInterface(GpioInterface &&) = delete;
        GpioInterface &operator=(GpioInterface &&) = delete;

        /// Read current value. Returns 0 or 1. Throws on error.
        int read() const;

        /// Write value (0 or 1). Throws on error.
        void write(int value);

    private:
        gpiod_chip *chip_{nullptr};
        gpiod_line *line_{nullptr};
        GpioDirection direction_;
        unsigned int offset_;
    };

} // namespace gpio_rosbag_controller
