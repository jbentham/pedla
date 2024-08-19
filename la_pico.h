// Pi Pico logic analyser interface definitions

// Copyright (c) 2022, Jeremy P Bentham
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Pin definitions
#define PIN_PCNT        17
#define PIN_SDA         18
#define PIN_SCL         19
#define PIN_SER_TX      20
#define PIN_SER_RX      21
#define PIN_SCK         22
#define PIN_RAM_CS      26
#define PIN_DAC_CS      27
#define PIN_LED         28

// Serial console
#define UART            uart1
#define UART_BAUD       115200

// Master clock frequency
#define PWM_CLOCK       120000000

// External memory, there are 2 samples per addressable byte
// 23LC1024 is 1 Mbit (128K x 8), IS62WVS2568 is 2 Mbit (256K x 8)
#define RAM_MAX_ADDR    0x20000
#define RAM_MAX_SAMP    (RAM_MAX_ADDR * 2)

// Logic analyser
#define NUM_RAMS        4       // Number of RAM chips
#define NCHANS          (NUM_RAMS * 4)
#define TXBUFF_NSAMP    702
#define TXBUFF_LEN      (TXBUFF_NSAMP * 2)
#define MAX_SAMP_RATE   20000000

// Select serial console on USB or external serial
#define EXT_SERIAL_CONSOLE  0
#if EXT_SERIAL_CONSOLE
#define DEBUG       Serial2     // Debug on external serial link
#else
#define DEBUG       Serial      // Debug on USB serial link
#endif

// Parameters for sample capture
typedef struct {
    uint32_t xsamp, xrate, nsamp;
    bool capturing;
} CAP_PARAMS;

void ram_data_sim(int oset, int count);
int la_ram_test(void);
void la_ram_write_bytes(int oset, BYTE *data, int len);
void la_ram_read_bytes(int oset, BYTE *data, int len);
void la_ram_write_qspi_words(int oset, uint16_t *data, int count);
void la_ram_read_qspi_bytes(int oset, BYTE *data, int count);
void la_ram_read_qspi_words(int oset, void *data, int count);
void la_ram_read_start(int oset);
void la_ram_send_cmd(BYTE *cmd, int len);
void la_ram_send_bytes(BYTE *data, int len);
void la_ram_recv_bytes(BYTE *data, int len);
void la_ram_write_start(int oset);
void la_ram_write_end(void);
void la_ram_read_end(void);
void la_pins_init(void);
void la_pins_config(uint32_t ins, uint32_t outs);
void gpio_out_spi(uint32_t val);
void gpio_out_bus(uint32_t val);
void spi_bus_init(void);
void qspi_bus_init(void);
int busbit_pin(int bitnum);
void bus_send_spi_cmd(BYTE *cmd, int len);
void bus_send_spi_bytes(BYTE *txd, int len);
void bus_recv_spi_bytes(BYTE *data, int len);
void bus_send_qspi_cmd(BYTE *cmd, int len);
void bus_send_qspi_bytes(BYTE *txd, int len);
void bus_recv_qspi_bytes(BYTE *rxd, int len, bool skip);
void bus_send_qspi_sim(int count);
void bus_send_qspi_words(uint16_t *txd, int count);
void bus_recv_qspi_words(void *rxd, int count);
void qspi_begin(void);
void qspi_input(void);
void qspi_output(void);
void qspi_end(void);
void pcnt_init(int pin);
void pcnt_reset(void);
void pwm_start(void);
void pwm_stop(void);

void cap_init(CAP_PARAMS *cp);
void cap_start(CAP_PARAMS *cp);
bool cap_capturing(CAP_PARAMS *cp);
void cap_end(CAP_PARAMS *cp);
void cap_read_start(int addr);
void cap_read_block(uint16_t *buff, int nsamp);
void cap_read_end(void);
uint32_t samp_wrap(uint32_t n);

void dac_init(void);
void dac_out(int mv);
void set_threshold(int v);
void la_set_led(bool on);
void set_state(STATE_VALS val);
void dump_bytes(BYTE *data, int len);
void dump_words(void *data, int len);
uint16_t prng16(uint16_t x);

void i2c_eeprom_init(void);
bool i2c_eeprom_read(int addr, BYTE *data, int dlen);
bool i2c_eeprom_write(int addr, BYTE *data, int dlen);
void i2c_eeprom_test(int addr);
int i2c_scan(void);
void serial_init(void);
void debug_printf(const char *format, ...);

// EOF
