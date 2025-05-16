#include "max14830.h"
#include "max310x.h"

#define RETURN_ON_FALSE(x)                                       \
    if (!(x))                                                    \
    {                                                            \
        ESP_LOGE(TAG, "Error in %s:%d", __FUNCTION__, __LINE__); \
        return false;                                            \
    }

namespace esphome
{
    namespace max14830
    {

        static const char *const TAG = "max14830";

        void MAX14830::dump_config()
        {
            ESP_LOGCONFIG(TAG, "dump_config called!");
        }

        float MAX14830::get_setup_priority() const
        {
            return setup_priority::IO;
        }

        void MAX14830::setup()
        {
            ESP_LOGCONFIG(TAG, "Setting up MAX14830...");

            this->spi_setup();

            // Detect chip
            bool detected = Detect();
            if (!detected)
            {
                ESP_LOGE(TAG, "Chip not detected");
                return;
            }

            // Set the clock source to the internal PLL
            uint32_t clk;
            bool clkOk = SetRefClock(&clk);
            if (clkOk)
            {
                ESP_LOGI(TAG, "Clock set to %u Hz", clk);
            }
            else
            {
                ESP_LOGE(TAG, "Failed to set clock");
                return;
            }

            // Init ports
            for (int i = 0; i < 4; i++)
            {
                bool portReady = portInit(i);
                if (!portReady)
                {
                    ESP_LOGE(TAG, "Failed to initialize port %d", i);
                    return;
                }
            }

            // Setup ISR, todo
        }

        bool MAX14830::digital_read(uint8_t pin)
        {
            uint32_t port = pin / 4;
            uint8_t bit = pin % 4;
            uint8_t reg = max310x_port_read(port, MAX310X_GPIODATA_REG);
            return ((reg >> 4) & (1 << bit)) != 0;
        }

        void MAX14830::digital_write(uint8_t pin, bool value)
        {
            uint32_t port = pin / 4;

            if (value)
                gpioDataBuffer |= 1 << pin;
            else
                gpioDataBuffer &= ~(1 << pin);

            uint8_t reg = gpioDataBuffer >> (4 * port);

            if (!max310x_port_write(port, MAX310X_GPIODATA_REG, reg))
            {
                ESP_LOGE(TAG, "Failed to write GPIO data");
            }
        }

        void MAX14830::pin_mode(uint8_t pin, gpio::Flags flags)
        {
            uint32_t port = pin / 4;
            uint8_t mask = (1 << (pin % 4));

            ESP_LOGD(TAG, "Setting pin %u (%01x, %01x) to mode %u", pin, port, mask, flags);

            uint8_t od = (gpioConfBuffer[port] >> 4) & 0xf;
            uint8_t ou = (gpioConfBuffer[port]) & 0xf;

            if (flags & gpio::Flags::FLAG_INPUT)
            {
                ou &= ~mask;
            }
            if (flags & gpio::Flags::FLAG_OUTPUT)
            {
                ou |= mask;
                od &= ~mask;
            }
            if (flags & gpio::Flags::FLAG_OPEN_DRAIN)
            {
                ou |= mask;
                od |= mask;
            }
            if (flags & gpio::Flags::FLAG_PULLUP)
            {
                ESP_LOGE(TAG, "Pullup not supported");
                return;
            }
            if (flags & gpio::Flags::FLAG_PULLDOWN)
            {
                ESP_LOGE(TAG, "Pullup not supported");
                return;
            }

            gpioConfBuffer[port] = (od << 4) | ou;

            if (!max310x_port_write(port, MAX310X_GPIOCFG_REG, gpioConfBuffer[port]))
            {
                ESP_LOGE(TAG, "Failed to write GPIO configuration");
            }
        }

        // --------------------------------------------------------
        // ------------------ MAX14830 functions ------------------
        // --------------------------------------------------------

        bool MAX14830::Detect()
        {

            regmap_write(MAX310X_GLOBALCMD_REG, MAX310X_EXTREG_ENBL);
            uint8_t value = max310x_port_read(0x00, MAX310X_REVID_EXTREG);
            regmap_write(MAX310X_GLOBALCMD_REG, MAX310X_EXTREG_DSBL);

            if (((value & MAX310x_REV_MASK) != MAX14830_REV_ID))
            {
                ESP_LOGE(TAG, "Chip not found revid %d", value);
                return false;
            }
            return true;
        }

        bool MAX14830::SetRefClock(uint32_t *clk)
        {
            uint32_t div, clksrc, pllcfg = 0;
            int64_t besterr = -1;
            uint64_t fdiv, fmul, bestfreq = MAX14830_CLK;
            bool clkError = false;

            // First, update error without PLL
            max310x_update_best_err(MAX14830_CLK, &besterr);

            // Try all possible PLL dividers
            for (div = 1; (div <= 63) && besterr; div++)
            {
                fdiv = MAX14830_CLK / div;

                /* Try multiplier 6 */
                if ((fdiv >= 500000) && (fdiv <= 800000))
                {
                    fmul = fdiv * 6;
                    if (!max310x_update_best_err(fmul, &besterr))
                    {
                        pllcfg = (0 << 6) | div;
                        bestfreq = fmul;
                    }
                }

                /* Try multiplier 48 */
                if ((fdiv >= 850000) && (fdiv <= 1200000))
                {
                    fmul = fdiv * 48;
                    if (!max310x_update_best_err(fmul, &besterr))
                    {
                        pllcfg = (1 << 6) | div;
                        bestfreq = fmul;
                    }
                }

                /* Try multiplier 96 */
                if ((fdiv >= 425000) && (fdiv <= 1000000))
                {
                    fmul = fdiv * 96;
                    if (!max310x_update_best_err(fmul, &besterr))
                    {
                        pllcfg = (2 << 6) | div;
                        bestfreq = fmul;
                    }
                }

                /* Try multiplier 144 */
                if ((fdiv >= 390000) && (fdiv <= 667000))
                {
                    fmul = fdiv * 144;
                    if (!max310x_update_best_err(fmul, &besterr))
                    {
                        pllcfg = (3 << 6) | div;
                        bestfreq = fmul;
                    }
                }
            }

            /* Configure clock source */
#ifdef MAX14830_USE_XTAL
            clksrc = MAX310X_CLKSRC_CRYST_BIT;
#else
            clksrc = MAX310X_CLKSRC_EXTCLK_BIT;
#endif

            /* Configure PLL */
            if (pllcfg)
            {
                clksrc |= MAX310X_CLKSRC_PLL_BIT;
                regmap_write(MAX310X_PLLCFG_REG, pllcfg);
            }
            else
            {
                clksrc |= MAX310X_CLKSRC_PLLBYP_BIT;
            }
            regmap_write(MAX310X_CLKSRC_REG, clksrc);

#ifdef MAX14830_USE_XTAL
            /* Wait for crystal */
            uint8_t val;
            uint8_t escape = 0;
            while (escape < 100)
            {
                val = regmap_read(MAX310X_STS_IRQSTS_REG);
                if (!(val & MAX310X_STS_CLKREADY_BIT))
                {
                    vTaskDelay(pdMS_TO_TICKS(20));
                    // ESP_LOGE(TAG, "Clock is not stable yet");
                }
                else
                {
                    escape = 255;
                }
            }
            if (escape != 255)
            {
                ESP_LOGE(TAG, "Clock not stable");
                clkError = true;
            }
#endif
            *clk = (uint32_t)bestfreq;
            return !clkError;
        }

        bool MAX14830::portInit(uint8_t port)
        {
            // Configure MODE2 register
            RETURN_ON_FALSE(max310x_port_update(port, MAX310X_MODE2_REG, MAX310X_MODE2_RXEMPTINV_BIT, MAX310X_MODE2_RXEMPTINV_BIT));

            // Clear IRQ status registers
            max310x_port_read(port, MAX310X_IRQSTS_REG);
            max310x_port_read(port, MAX310X_LSR_IRQSTS_REG);
            max310x_port_read(port, MAX310X_SPCHR_IRQSTS_REG);
            max310x_port_read(port, MAX310X_STS_IRQSTS_REG);
            max310x_port_read(port, MAX310X_GLOBALIRQ_REG);

            // Route GlobalIRQ to IRQPIN
            RETURN_ON_FALSE(max310x_port_update(port, MAX310X_MODE1_REG, MAX310X_MODE1_IRQSEL_BIT, MAX310X_MODE1_IRQSEL_BIT));

            /* Enable STS, RX, TX, CTS change interrupts */
            RETURN_ON_FALSE(max310x_port_write(port, MAX310X_IRQEN_REG, MAX310X_IRQ_RXEMPTY_BIT | MAX310X_IRQ_STS_BIT));
            RETURN_ON_FALSE(max310x_port_write(port, MAX310X_LSR_IRQEN_REG, 0));
            RETURN_ON_FALSE(max310x_port_write(port, MAX310X_SPCHR_IRQEN_REG, 0));

            return true;
        }

        // --------------------------------------------------------
        //
        // --------------------------------------------------------

        // Functions for reading and writing registers
        bool MAX14830::regmap_write(uint8_t cmd, uint8_t value)
        {
            return Max14830_WriteBufferPolled(cmd, &value, 1);
        }

        uint8_t MAX14830::regmap_read(uint8_t cmd)
        {
            uint8_t replyData[1];
            Max14830_ReadBufferPolled(cmd, nullptr, replyData, 1);
            return replyData[0];
        }

        uint8_t MAX14830::max310x_port_read(uint8_t port, uint8_t cmd)
        {
            cmd = (port << 5) | cmd;
            return regmap_read(cmd);
        }

        bool MAX14830::max310x_port_write(uint8_t port, uint8_t cmd, uint8_t value)
        {
            cmd = (port << 5) | cmd;
            return regmap_write(cmd, value);
        }

        bool MAX14830::max310x_port_update(uint8_t port, uint8_t cmd, uint8_t mask, uint8_t value)
        {
            uint8_t val = max310x_port_read(port, cmd);
            val &= ~mask;
            val |= (mask & value);
            return max310x_port_write(port, cmd, val);
        }

        uint8_t MAX14830::max310x_update_best_err(uint64_t f, int64_t *besterr)
        {
            /* Use baudrate 115200 for calculate error */
            int64_t err = f % (115200 * 16);
            if ((*besterr < 0) || (*besterr > err))
            {
                *besterr = err;
                return 0;
            }
            return 1;
        }

        bool MAX14830::Max14830_WriteBufferPolled(uint8_t cmd, const uint8_t *cmdData, uint8_t count)
        {
            this->enable();
            this->write_byte(0x80 | cmd);
            for (uint8_t i = 0; i < count; i++)
            {
                this->write_byte(cmdData[i]);
            }
            this->disable();
            return true;
        }

        bool MAX14830::Max14830_ReadBufferPolled(uint8_t cmd, uint8_t *cmdData, uint8_t *replyData, uint8_t count)
        {
            this->enable();
            this->write_byte(cmd);
            for (uint8_t i = 0; i < count; i++)
            {
                replyData[i] = this->transfer_byte(0xFF); // Dummy TX, read RX
            }
            this->disable();
            return true;
        }

    } // namespace max14830
} // namespace esphome
