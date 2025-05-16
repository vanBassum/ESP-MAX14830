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

        void MAX14830::UartConfigure(uint8_t port, uint32_t baud, esphome::uart::UARTParityOptions parity, uint8_t stop_bits, uint8_t data_bits, FlowControl flow_control)
        {
            uint8_t flowCtrlRegVal = 0;

            // TODO: Implement other flow control settings.
            switch (flow_control)
            {
            case FlowControl::UART_CFG_FLOW_CTRL_NONE:
                max310x_port_update(port, MAX310X_MODE1_REG, MAX310X_MODE1_TXDIS_BIT, 0);
                max310x_port_update(port, MAX310X_MODE1_REG, (MAX310X_MODE1_TRNSCVCTRL_BIT | MAX310X_MODE1_IRQSEL_BIT), (0 | MAX310X_MODE1_IRQSEL_BIT));
                max310x_port_write(port, MAX310X_HDPIXDELAY_REG, 0);
                break;

            case FlowControl::UART_CFG_FLOW_CTRL_RS485:
                max310x_port_update(port, MAX310X_MODE1_REG, (MAX310X_MODE1_TRNSCVCTRL_BIT | MAX310X_MODE1_IRQSEL_BIT), (MAX310X_MODE1_TRNSCVCTRL_BIT | MAX310X_MODE1_IRQSEL_BIT));
                max310x_port_write(port, MAX310X_HDPIXDELAY_REG, 0x11);
                break;

            case FlowControl::UART_CFG_FLOW_CTRL_RTS_CTS:
                max310x_port_update(port, MAX310X_MODE1_REG, MAX310X_MODE1_TXDIS_BIT, MAX310X_MODE1_TXDIS_BIT);
                max310x_port_update(port, MAX310X_MODE1_REG, (MAX310X_MODE1_TRNSCVCTRL_BIT | MAX310X_MODE1_IRQSEL_BIT), (0 | MAX310X_MODE1_IRQSEL_BIT));
                max310x_port_write(port, MAX310X_HDPIXDELAY_REG, 0);
                flowCtrlRegVal |= MAX310X_FLOWCTRL_AUTOCTS_BIT;
                break;
            default:
                ESP_LOGE(TAG, "Flow control %d not supported", flow_control);
                return;
            }

            // TODO: Implement other parity settings.
            // TODO: Implement other databits settings.
            // TODO: Implement other stopbits settings.


            max310x_set_baud(port, baud, NULL);
            max310x_port_write(port, MAX310X_LCR_REG, MAX310X_LCR_LENGTH0_BIT | MAX310X_LCR_LENGTH1_BIT); // 8 bit - no parity - 1 stopbit
            max310x_port_write(port, MAX310X_FLOWCTRL_REG, flowCtrlRegVal);

            // Reset FIFOs
            max310x_port_update(port, MAX310X_MODE2_REG, MAX310X_MODE2_FIFORST_BIT, MAX310X_MODE2_FIFORST_BIT);
            max310x_port_update(port, MAX310X_MODE2_REG, MAX310X_MODE2_FIFORST_BIT, 0);
        }

        int MAX14830::UartAvailable(uint8_t port)
        {
            return max310x_port_read(port, MAX310X_RXFIFOLVL_REG);
        }

        int MAX14830::UartWrite(uint8_t port, const uint8_t *data, int size)
        {
            size_t bytesWrittenTotal = 0;

            // Loop until all data is written
            while (bytesWrittenTotal < size)
            {
                // Calculate the number of bytes to write
                size_t bytesToWrite = size - bytesWrittenTotal;

                // Ensure it fits int the FIFO
                uint8_t fifolvl = max310x_port_read(port, MAX310X_TXFIFOLVL_REG);
                if (bytesToWrite > MAX14830_FIFO_MAX - fifolvl)
                    bytesToWrite = MAX14830_FIFO_MAX - fifolvl;

                // If no space in the FIFO, wait for the data to be clocked out.
                if (bytesToWrite == 0)
                {
                    ESP_LOGE(TAG, "No space in FIFO of port %d, %d out of %d bytes written", port, bytesWrittenTotal, size);
                    return bytesWrittenTotal;
                }

                // Perform the write operation
                if (!Max14830_WriteBufferPolled(((uint32_t)port << 5), data, bytesToWrite))
                {
                    ESP_LOGE(TAG, "Failed to write data to port %d, %d out of %d bytes written", port, bytesWrittenTotal, size);
                    return bytesWrittenTotal;

                }

                // Update total bytes written and shift the data pointer
                bytesWrittenTotal += bytesToWrite;
                data += bytesToWrite;
            }
            return bytesWrittenTotal;
        }

        int MAX14830::UartRead(uint8_t port, uint8_t *data, int size)
        {
            size_t bytesReadTotal = 0;

            // Loop until all data is read
            while (bytesReadTotal < size)
            {
                // Calculate the number of bytes to read
                size_t bytesToRead = size - bytesReadTotal;

                // Ensure data is available in the FIFO
                uint8_t fifolvl = max310x_port_read(port, MAX310X_RXFIFOLVL_REG);
                if (bytesToRead > fifolvl)
                    bytesToRead = fifolvl;

                // If no data in the FIFO, wait for the data to be received.
                if (bytesToRead == 0)
                {
                    ESP_LOGE(TAG, "No data in FIFO of port %d, %d out of %d bytes read", port, bytesReadTotal, size);
                    return bytesReadTotal;
                }

                // Perform the read operation
                if (!Max14830_ReadBufferPolled(((uint32_t)port << 5), nullptr, data, bytesToRead))
                {
                    ESP_LOGE(TAG, "Failed to read data from port %d, %d out of %d bytes read", port, bytesReadTotal, size);
                    return bytesReadTotal;
                }

                // Update total bytes read and shift the data pointer
                bytesReadTotal += bytesToRead;
                data += bytesToRead;
            }

            return bytesReadTotal;
        }


        void MAX14830::max310x_set_baud(uint8_t port, uint32_t baud, uint32_t *actualBaud)
        {
            uint8_t mode = 0;
            uint32_t fref = max310x_get_ref_clk();
            uint32_t clk = fref;
            uint32_t div = clk / baud;

            // Check for minimal value for divider
            if (div < 16)
                div = 16;
            //
            if ((clk % baud) && ((div / 16) < 0x8000))
            {
                /* Mode x2 */
                mode = MAX310X_BRGCFG_2XMODE_BIT;
                clk = fref * 2;
                div = clk / baud;
                if ((clk % baud) && ((div / 16) < 0x8000))
                {
                    /* Mode x4 */
                    mode = MAX310X_BRGCFG_4XMODE_BIT;
                    clk = fref * 4;
                    div = clk / baud;
                }
            }

            max310x_port_write(port, MAX310X_BRGDIVMSB_REG, (div / 16) >> 8);
            max310x_port_write(port, MAX310X_BRGDIVLSB_REG, div / 16);
            max310x_port_write(port, MAX310X_BRGCFG_REG, (div % 16) | mode);
            if (actualBaud != NULL)
                *actualBaud = clk / div; // actual baudrate, this will never be exactly the value requested..
        }

        uint32_t MAX14830::max310x_get_ref_clk()
        {
            uint64_t clk = MAX14830_CLK;  // 14.7456 MHz, or whatever
            uint8_t value = max310x_port_read(0x00, MAX310X_PLLCFG_REG);

            uint32_t clkDiv = value & MAX310X_PLLCFG_PREDIV_MASK;

            if (clkDiv == 0) {
                ESP_LOGE("max14830", "PLLCFG: clkDiv is 0! Invalid config.");
                return 0;
            }

            uint32_t mul = (value >> 6) & 0x3;
            if (mul == 0) {
                mul = 6;
            } else {
                mul = mul * 48;
            }

            return (clk * mul) / clkDiv;
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
