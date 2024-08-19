// Pi Pico logic analyser hardware interface

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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "picowi/picowi_defs.h"
#include "picowi/picowi_pico.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include <hardware/pwm.h>
#include <hardware/dma.h>
#include <hardware/clocks.h>

#include "pedla_cfg.h"
#include "la_pico.h"

// If non-zero, fake data for testing
#define FAKE_DATA       0

// Data pin numbers for one RAM chip
#define RAM_SPI_DIN     0
#define RAM_SPI_DOUT    1
#define RAM_SPI_HOLD    3
// Corresponding SPI bit masks
#define MSK_SPI_DOUT    (1 << RAM_SPI_DIN)
#define MSK_SPI_DIN     (1 << RAM_SPI_DOUT)
#define MSK_SPI_HOLD    (1 << RAM_SPI_HOLD)
#define MSK_SPI_IN      (MSK_SPI_DIN)
#define MSK_SPI_OUTS    (MSK_SPI_DOUT | MSK_SPI_HOLD)
#define MSK_SQI_IOS     0xf
#define ALL_RAM_WORD(b) ((b) | (b)<<4 | (uint32_t)(b)<<8 | (uint32_t)(b)<<12)
#define RAM_TEST_OSET   0x1bcde
#define IO_READ()       gpio_get_all()

// I/O macros
#define DAC_SELECT      IO_WR_LO(PIN_DAC_CS)
#define DAC_DESELECT    IO_WR_HI(PIN_DAC_CS)
#define RAM_SELECT      IO_WR_LO(PIN_RAM_CS)
#define RAM_DESELECT    IO_WR_HI(PIN_RAM_CS)
#define SET_SCK         IO_WR_HI(PIN_SCK)
#define CLR_SCK         IO_WR_LO(PIN_SCK)

#define I2C             i2c1
#define I2C_FREQ        100000
#define EEPROM_ADDR     0x50
#define EEPROM_PAGE_LEN 64
#define EEPROM_WRITE_US 11000
#define EEPROM_XFER_US  100000

const char *state_strs[NUM_STATES] = { "Idle", "Ready", "PreLoad", "PreTrig", "PostTrig", "UpLoad" };
char hexchar[] = "0123456789ABCDEF";
bool qspi_mode;

#define word_busval(x) (x)
#define bus_wordval(x) (x)
uint32_t spi_in_pins   = word_busval(ALL_RAM_WORD(MSK_SPI_IN));
uint32_t spi_out_pins  = word_busval(ALL_RAM_WORD(MSK_SPI_OUTS));
uint32_t spi_dout_pins = word_busval(ALL_RAM_WORD(MSK_SPI_DOUT));
uint32_t spi_hold_pins = word_busval(ALL_RAM_WORD(MSK_SPI_HOLD));
uint32_t sqi_io_pins   = word_busval(ALL_RAM_WORD(MSK_SQI_IOS));

uint pout_slice, pcnt_slice, pcnt_dma_chan, pend_dma_chan;
uint pout_csr_stopval, dma_dummy_val;

// Load simulated data into RAM
void ram_data_sim(int oset, int count) 
{
    qspi_begin();
    la_ram_write_start(oset);
    bus_send_qspi_sim(count);
    la_ram_write_end();
    qspi_end();
}

// Do logic analyser RAM test, return number of samples, or 0 if failed
int la_ram_test(void) 
{
    uint8_t obytes[12] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
        0x11, 0x22, 0x44, 0x88 };
    uint8_t ibytes[12];
    uint16_t iwords[6];
    uint16_t owords[6] = { 0x1234, 0x2345, 0x3456, 0x5678, 0x6789, 0x789a };
    uint16_t idata[6] = { 0x0000, 0x8421, 0x8421, 0x0000, 0x8421, 0x8421 };
    int n, addr=0, val, fail;
    
    // SPI byte read/write test
    qspi_end();
    spi_bus_init();
    la_ram_write_bytes(RAM_TEST_OSET, obytes, sizeof(obytes));
    la_ram_read_bytes(RAM_TEST_OSET, ibytes, sizeof(ibytes));
    if ((fail = memcmp(obytes, ibytes, sizeof(obytes))))
    {
        debug_printf("RAM SPI readback error\r\n");
        dump_bytes(obytes, sizeof(obytes));
        dump_bytes(ibytes, sizeof(ibytes));
    }
    else
    {
        // QSPI word read/write test
        qspi_begin();
        la_ram_write_qspi_words(RAM_TEST_OSET + 0x20, owords, sizeof(owords) / 2);
        la_ram_read_qspi_words(RAM_TEST_OSET + 0x20, iwords, sizeof(iwords) / 2);
        if ((fail = memcmp(owords, iwords, sizeof(owords))))
        {
            debug_printf("RAM QSPI readback error\r\n");
            dump_words(owords, sizeof(owords) / 2);
            dump_words(iwords, sizeof(iwords) / 2);
        }
        la_ram_read_qspi_words(RAM_TEST_OSET, iwords, sizeof(iwords) / 2);
        qspi_end();
    }
    // SPI to QSPI test
    if (!fail && (fail = memcmp(iwords, idata, sizeof(iwords))))
    {
        debug_printf("RAM SPI to QSPI error\r\n");
        dump_words(idata, sizeof(idata) / 2);
        dump_words(iwords, sizeof(iwords) / 2);
    }
    // QSPI word write & byte read
    if (!fail)
    {
        memset(ibytes, 0, sizeof(ibytes));
        qspi_begin();
        la_ram_write_bytes(RAM_TEST_OSET+2, obytes, sizeof(obytes));
        la_ram_read_bytes(RAM_TEST_OSET+2, ibytes, sizeof(ibytes));
        qspi_end();
        if ((fail = memcmp(ibytes, obytes, sizeof(ibytes)))) 
        {
            debug_printf("RAM QSPI byte read error\r\n");
            dump_bytes(ibytes, sizeof(ibytes));
            dump_bytes(obytes, sizeof(obytes));
        }
    }
    // Memory size test
    // Write to address 1, 2, 4, 8 etc., read back from address 0
    // When read-value equals write-value, address has wrapped around
    if (!fail)
    {
        fail = 1;
        qspi_begin();
        for (n=0; n<30 && fail; n++)
        {
            addr = 1 << n;
            la_ram_write_qspi_words(addr, (uint16_t *)&addr, 2);
            la_ram_read_qspi_words(0, (uint16_t *)&val, 2);
            if (n && val==addr)
                fail = 0;
        }
        qspi_end();
        if (fail)
            debug_printf("Memory size test failed\n");
        else
            debug_printf("RAM size %dk samples\n", addr / 1024);
    }
    return (fail ? 0 : addr);
}

// Write block of bytes to RAM
void la_ram_write_bytes(int oset, BYTE *data, int len) 
{
    la_ram_write_start(oset);
    la_ram_send_bytes(data, len);
    la_ram_write_end();
}

// Read block of bytes from RAM
void la_ram_read_bytes(int oset, BYTE *data, int len) 
{
    //qspi_begin();
    la_ram_read_start(oset);
    la_ram_recv_bytes(data, len);
    la_ram_read_end();
    //qspi_end();
    //dump_words((uint16_t *)data, len / 2);
}

// Write block of 16-bit words to RAM in QSPI mode
void la_ram_write_qspi_words(int oset, uint16_t *data, int count) 
{
    la_ram_write_start(oset);
    bus_send_qspi_words(data, count);
    la_ram_write_end();
}

// Read block of 16-bit words from RAM in QSPI mode
void la_ram_read_qspi_words(int oset, void *data, int count) 
{
    la_ram_read_start(oset);
    bus_recv_qspi_words(data, count);
    la_ram_read_end();
}

// Read block of bytes from RAM in QSPI mode
void la_ram_read_qspi_bytes(int oset, BYTE *data, int count) 
{
    la_ram_read_start(oset / 2);
    bus_recv_qspi_bytes(data, count, oset & 1);
    la_ram_read_end();
}

// Start SPI or QSPI memory write, given offset (number of samples)
void la_ram_write_start(int n) 
{
    BYTE cmd[4] = { 2, (BYTE)(n >> 17), (BYTE)(n >> 9), (BYTE)(n >> 1)};
    RAM_SELECT;
    la_ram_send_cmd(cmd, sizeof(cmd));
}

// Start SPI or QSPI memory read, given offset (number of samples)
void la_ram_read_start(int n) 
{
    BYTE cmd[4] = {3, (BYTE)(n >> 17), (BYTE)(n >> 9), (BYTE)(n >> 1)};
    BYTE padding[NUM_RAMS];
    RAM_SELECT;
    la_ram_send_cmd(cmd, sizeof(cmd));
    if (qspi_mode) 
        qspi_input();
    la_ram_recv_bytes(padding, sizeof(padding));
}

// Send command to all RAMs using SPI or QSPI
void la_ram_send_cmd(BYTE *cmd, int len) 
{
    if (qspi_mode)
        bus_send_qspi_cmd(cmd, len);
    else
        bus_send_spi_cmd(cmd, len);
}

// Write bytes to memory using SPI or QSPI
// Length value is total length in bytes for all RAMs
void la_ram_send_bytes(BYTE *data, int len) 
{
    if (qspi_mode)
        bus_send_qspi_bytes(data, len);
    else
        bus_send_spi_bytes(data, len);
}

// Read bytes from memory using SPI or QSPI
// Length value is total length in bytes for all RAMs
void la_ram_recv_bytes(BYTE *data, int len) 
{
    if (qspi_mode)
        bus_recv_qspi_bytes(data, len, false);
    else
        bus_recv_spi_bytes(data, len);
}

// End write to RAM
void la_ram_write_end(void) 
{
    RAM_DESELECT;
}

// End read from RAM
void la_ram_read_end(void) 
{
    if (qspi_mode)
        qspi_output();
    RAM_DESELECT;
}

// Initialise I/O pins
void la_pins_init(void)
{
    io_set(PIN_LED, IO_OUT, 0);
    io_set(PIN_RAM_CS, IO_OUT, 0);
    io_out(PIN_RAM_CS, 1);
    io_set(PIN_DAC_CS, IO_OUT, 0);
    io_out(PIN_DAC_CS, 1);
    io_set(PIN_SCK, IO_OUT, 0);
    io_out(PIN_SCK, 1);
    for (int i = 0; i < NUM_RAMS * 4; i++)
        io_set(i, IO_IN, 0);
}

// Set input & output pins, given bitmaps
void la_pins_config(uint32_t ins, uint32_t outs) 
{
    io_set_ins(ins);
    io_set_outs(outs);
}

// Send a 32-bit value to all RAM SPI inputs
void gpio_out_spi(uint32_t val) 
{
    io_write_masked(spi_dout_pins, val);
}

// Send a 32-bit value to all RAM SQI inputs
void gpio_out_bus(uint32_t val) 
{
    io_write_masked(sqi_io_pins, val);
}

// Set I/O bus configuration for SPI
void spi_bus_init(void) 
{
    CLR_SCK;
    la_pins_config(spi_in_pins, spi_out_pins);
    io_write_masked(spi_hold_pins, spi_hold_pins);
    qspi_mode = 0;
}

// Set I/O bus configuration for QSPI
void qspi_bus_init(void) 
{
    RAM_DESELECT;
    CLR_SCK;
    la_pins_config(0, sqi_io_pins);
}

// Return I/O pin number, given bus bit number
int busbit_pin(int bitnum) 
{
    uint32_t val = word_busval(1 << bitnum);
    int n=0;
    while (val) 
    {
        n++;
        val >>= 1;
    }
    return(n-1);
}

// Send byte command to all RAMs using SPI
void bus_send_spi_cmd(BYTE *cmd, int len)
{
    BYTE b;
    
    io_write_set(spi_hold_pins);
    while (len--) 
    {
        b = *cmd++;
        for (int n = 0; n < 8; n++) 
        {
            if (b & 0x80)
                io_write_set(spi_dout_pins);
            else
                io_write_clr(spi_dout_pins);
            SET_SCK;
            SET_SCK;
            b <<= 1;
            CLR_SCK;
        }
    }
}

// Send bytes to RAM chips using SPI
// Length value is total length in bytes for all RAMs
void bus_send_spi_bytes(BYTE *txd, int len)
{
    uint16_t w;
    
    len /= NUM_RAMS;
    while (len--) 
    {
        uint32_t d = (uint32_t)txd[0] | (uint32_t)txd[1] << 8 | (uint32_t)txd[2] << 16 | (uint32_t)txd[3] << 24;
        for (int n = 0; n < 8; n++)
        {
            w = (d >> 7 & 1) | (d >> (15-4) & 0x10) | (d >> (23-8) & 0x100) | (d >> (31-12) & 0x1000);
            io_write_masked(spi_dout_pins, w);
            SET_SCK;
            d <<= 1;
            CLR_SCK;
        }
        txd += NUM_RAMS;
    }
}

// Receive bytes from RAM chips using SPI
// Length value is total length in bytes for all RAMs
void bus_recv_spi_bytes(BYTE *rxd, int len) 
{
    uint16_t w;
    
    len /= NUM_RAMS;
    while (len--)
    {
        uint32_t d = 0;
        for (int n = 0; n < 8; n++) 
        {
            SET_SCK;
            w = bus_wordval(io_read() & spi_in_pins) >> RAM_SPI_DOUT;
            d <<= 1;
            CLR_SCK;
            d |= (w & 1) | (w & 0x10) << (8-4) | (w & 0x100) << (16-8) | (w & 0x1000) << (24-12);
        }
        *rxd++ = (BYTE)d;
        *rxd++ = (BYTE)(d >> 8);
        *rxd++ = (BYTE)(d >> 16);
        *rxd++ = (BYTE)(d >> 24);
    }
}

// Send a single command to all RAMs using QSPI
void bus_send_qspi_cmd(BYTE *cmd, int len) 
{
    while (len--) 
    {
        uint32_t b1 = *cmd >> 4, b2 = *cmd & 15, val = word_busval(ALL_RAM_WORD(b1));
        gpio_out_bus(val);
        SET_SCK;
        SET_SCK;
        val = word_busval(ALL_RAM_WORD(b2));
        CLR_SCK;
        gpio_out_bus(val);
        SET_SCK;
        SET_SCK;
        cmd++;
        CLR_SCK;
    }
}

// Send bytes to RAM chips using QSPI
// Length value is total length in bytes for all RAMs
void bus_send_qspi_bytes(BYTE *txd, int len) 
{
    uint16_t w;
    
    while (len > 0) 
    {
        w = *txd++;
        w |= (uint16_t)(*txd++) << 8;
        gpio_out_bus(word_busval(w));
        SET_SCK;
        SET_SCK;
        len -= 2;
        CLR_SCK;
    }
}

// Receive bytes from RAM chips using QSPI
// Length value is total length in bytes for all RAMs
void bus_recv_qspi_bytes(BYTE *rxd, int len, bool skip) 
{
    WORD w;
    
    if (skip)
    {
        SET_SCK;
        SET_SCK;
        w = (WORD)bus_wordval(IO_READ());
        *rxd++ = (BYTE)(w >> 8);
        CLR_SCK;
        CLR_SCK;
        len --;
    }
    while (len > 1) 
    {
        SET_SCK;
        SET_SCK;
        w = (WORD)bus_wordval(IO_READ());
        *rxd++ = (BYTE)w;
        CLR_SCK;
        *rxd++ = (BYTE)(w >> 8);
        len -= 2;
    }
    if (len == 1)
    {
        SET_SCK;
        SET_SCK;
        w = (WORD)bus_wordval(IO_READ());
        *rxd++ = (BYTE)w;
        CLR_SCK;
    }
}

// Send simulated data words to RAM chips using QSPI
void bus_send_qspi_sim(int count) 
{
    static int n=0;
    int i = 0;
    while (i < count)
    {
        gpio_out_bus(word_busval(i+n));
        SET_SCK;
        i++;
        CLR_SCK;
    }
    n += 100;
}

// Send words to RAM chips using QSPI
void bus_send_qspi_words(uint16_t *txd, int count) 
{
    while (count) 
    {
        gpio_out_bus(word_busval(*txd));
        SET_SCK;
        txd++;
        count--;
        CLR_SCK;
    }
}

// Receive words from RAM chips using QSPI
void bus_recv_qspi_words(void *rxd, int count) 
{
    BYTE *bp = rxd;
    uint16_t w;
    
    while (count--) 
    {
        SET_SCK;
        w = bus_wordval(IO_READ() & sqi_io_pins);
        *bp++ = (BYTE)w;
        CLR_SCK;
        *bp++ = (BYTE)(w >> 8);
    }
}

// Start QSPI mode
void qspi_begin(void) 
{
    RAM_DESELECT;
    RAM_DESELECT;
    RAM_SELECT;
    bus_send_spi_cmd((BYTE *)"\x38", 1);
    RAM_DESELECT;
    qspi_bus_init();
    qspi_mode = 1;
}

// Switch QSPI bus to input data from RAMs
void qspi_input(void) 
{
    la_pins_config(sqi_io_pins, 0);
}

// Set QSPI bus to output data
void qspi_output(void) 
{
    la_pins_config(0, sqi_io_pins);
}

// End QSPI mode
void qspi_end(void) 
{
    RAM_DESELECT;
    RAM_DESELECT;
    RAM_SELECT;
    bus_send_qspi_cmd((BYTE *)"\xff\xff", 2);
    RAM_DESELECT;
    spi_bus_init();
    qspi_mode = 0;
}

// Initialise PWM pulse counter
void pcnt_init(int pin) 
{
    pcnt_slice = pwm_gpio_to_slice_num(pin);
    io_rw_32 *pout_csr_ptr = &pwm_hw->slice[pout_slice].csr;
    
    // PWM channel to trigger DMA
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_set_clkdiv_mode(pcnt_slice, PWM_DIV_B_FALLING);
    pwm_set_clkdiv_int_frac(pcnt_slice, 1, 0);
    pwm_set_wrap(pcnt_slice, 0);
    pwm_set_phase_correct(pcnt_slice, false);
    
    // DMA channel to stop the PWM pulse output
    pend_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(pend_dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, DREQ_FORCE);
    pout_csr_stopval = *pout_csr_ptr;
    dma_channel_configure(pcnt_dma_chan, &cfg, pout_csr_ptr, &pout_csr_stopval, 1, false); // ???

    // DMA channel to count dummy cycles, then chain to channel above
    pcnt_dma_chan = dma_claim_unused_channel(true);
    cfg = dma_channel_get_default_config(pcnt_dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pwm_get_dreq(pcnt_slice));
    channel_config_set_chain_to(&cfg, pend_dma_chan);	
    dma_channel_configure(pcnt_dma_chan, &cfg, &dma_dummy_val, &dma_dummy_val, 1, false);
}

// Reset sample counter
void pcnt_reset(void) 
{
    pwm_set_counter(pcnt_slice, 0);
}

// Initialise PWM output
void pout_init(int pin, int freq) 
{
    int div = PWM_CLOCK / freq;
    int wrap = div & 0xffff;
    
    pout_slice = pwm_gpio_to_slice_num(pin);
    pwm_set_enabled(pout_slice, 0);
    pwm_set_clkdiv_int_frac(pout_slice, 1, 0);
    pwm_set_wrap(pout_slice, wrap - 1);
    pwm_set_chan_level(pout_slice, pwm_gpio_to_channel(pin), wrap / 2);
    pwm_set_phase_correct(pout_slice, 0);
}

// Start PWM output
void pout_start(int pin) 
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_set_counter(pout_slice, 0);
    pwm_set_enabled(pout_slice, 1);
}

// Stop PWM output
void pout_stop(int pin) 
{
    pwm_set_enabled(pout_slice, 0);
    gpio_set_function(pin, GPIO_FUNC_SIO);
}

// Output a given number of RAM clock pulses
void pout_pulses(int pin, int n)
{
    pout_stop(pin);
    dma_channel_abort(pend_dma_chan);
    dma_channel_abort(pcnt_dma_chan);
    pwm_set_wrap(pcnt_slice, 0);
    dma_channel_set_trans_count(pcnt_dma_chan, n, true);
    dma_channel_set_trans_count(pend_dma_chan, 1, true);
    dma_channel_start(pcnt_dma_chan);
    //dma_channel_start(pend_dma_chan);
    pwm_set_counter(pcnt_slice, 0);
    pwm_set_enabled(pcnt_slice, 1);
    pout_start(pin);
}

// Set up capture
void cap_init(CAP_PARAMS *cp) 
{
    pout_init(PIN_SCK, cp->xrate);
    pcnt_init(PIN_PCNT);
}

// Start capture
void cap_start(CAP_PARAMS *cp) 
{
    cp->capturing = true;
    set_threshold(get_param_int(ARG_THRESH));
    cp->nsamp = 0;
    qspi_begin();
    la_ram_write_start(0);
    qspi_input();
    pout_init(PIN_SCK, cp->xrate);
    pcnt_reset();
    pout_pulses(PIN_SCK, cp->xsamp);
}

// Check progress of capture
bool cap_capturing(CAP_PARAMS *cp)
{
    uint32_t rem = dma_channel_hw_addr(pcnt_dma_chan)->transfer_count;
    
    cp->nsamp = cp->xsamp >= rem ? cp->xsamp - rem : cp->xsamp; // ???
    set_param_int(ARG_NSAMP, cp->nsamp);
    return (rem != 0);
}

// Stop capturing data
void cap_end(CAP_PARAMS *cp) 
{
    cp->capturing = 0;
    pout_stop(PIN_SCK);
    qspi_output();
    qspi_end();
    cp->capturing = false;
}

// Start reading captured data, given offset (number of samples)
void cap_read_start(int addr) 
{
    spi_bus_init();
    qspi_begin();
    la_ram_read_start(addr);
}

// Read block of captured data
void cap_read_block(uint16_t *buff, int nsamp) 
{
    bus_recv_qspi_words(buff, nsamp);
}

// End reading captured data
void cap_read_end(void) 
{
    la_ram_read_end();
}

// Wrap sample count around RAM boundary
uint32_t samp_wrap(uint32_t n) 
{
    return(n & (RAM_MAX_ADDR-1));
}

// Initialise DAC hardware
void dac_init(void) 
{
    io_out(PIN_DAC_CS, 1);
}

// Output voltage from DAC; Vout = Vref * n / 4096
void dac_out(int mv) 
{
    uint16_t w = 0x7000 + ((mv * 4096) / 3300);
    BYTE cmd[2] = { (BYTE)(w >> 8), (BYTE)(w & 0xff) };
    
    RAM_DESELECT;
    DAC_SELECT;
    bus_send_spi_cmd(cmd, sizeof(cmd));
    DAC_DESELECT;
}

// Set threshold, given voltage
void set_threshold(int v) 
{
    dac_out(v * 1000 / get_param_int(ARG_ATTEN));
}

// Set or clear the LED
void la_set_led(bool on) 
{
    io_out(PIN_LED, !on);
}

// Set the current state
void set_state(STATE_VALS val) 
{
    set_param_int(ARG_STATE, val);
    la_set_led(val > STATE_READY);
    if (val < NUM_STATES)
        debug_printf("State: %s\r\n", state_strs[val]);
}

// Dump byte values in hex
void dump_bytes(BYTE *data, int len) 
{
    while (len--) 
        debug_printf("%02X ", *data++);
    debug_printf("\r\n");
}

// Dump 16-bit hex values
void dump_words(void *data, int len) 
{
    uint16_t w;
    BYTE *bp = data;
    
    while (len--)
    {
        w = *bp++;
        w |= ((WORD)*bp++) << 8;
        debug_printf("%04X ", w);
    }
    debug_printf("\r\n");
}

// 16-bit pseudo-random number generator
uint16_t prng16(uint16_t x) 
{
    x ^= x << 7;
    x ^= x >> 9;
    return (x ^= x << 8);
}

// Initialise EEPROM i2c interface
void i2c_eeprom_init(void)
{
    i2c_init(I2C, I2C_FREQ);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
}

// Read from EEPROM
bool i2c_eeprom_read(int addr, BYTE *data, int dlen)
{
    BYTE cmd[2] = {(BYTE)(addr >> 8), (BYTE)addr};
        
    return i2c_write_timeout_us(I2C, EEPROM_ADDR, cmd, sizeof(cmd), 1, EEPROM_XFER_US) > 0 &&
        i2c_read_timeout_us(I2C, EEPROM_ADDR, data, dlen, 0, EEPROM_XFER_US) > 0;
}
    
// Write to EEPROM
bool i2c_eeprom_write(int addr, BYTE *data, int dlen)
{
    BYTE cmd[EEPROM_PAGE_LEN+2];
    int n, i = 0, a = addr & (EEPROM_PAGE_LEN - 1);
    bool ret = 1;
    
    while (ret && dlen > 0)
    {
        n = a + dlen > EEPROM_PAGE_LEN ? EEPROM_PAGE_LEN - a : dlen;
        cmd[0] = (BYTE)((addr + i) >> 8);
        cmd[1] = (BYTE)(addr + i);
        memcpy(&cmd[2], &data[i], n);
        ret = i2c_write_timeout_us(I2C, EEPROM_ADDR, cmd, n + 2, 0, EEPROM_XFER_US) > 0;
        i += n;
        dlen -= n;
        a = 0;
        usdelay(EEPROM_WRITE_US);
    }
    return (ret);
}

// Test EEPROM
void i2c_eeprom_test(int addr)
{
    BYTE txd[8] = { 1, 2, 4, 8, 16, 32, 64, 128};
    BYTE rxd[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    
    i2c_eeprom_write(addr, txd, sizeof(txd));
    i2c_eeprom_read(addr, rxd, sizeof(rxd));
    if (memcmp(txd, rxd, sizeof(rxd)))
    {
        debug_printf("EEPROM test failed\n");
        dump_bytes(rxd, sizeof(rxd));
    }
    else
        debug_printf("EEPROM test passed\n");
}

// Scan i2c bus, return number of devices
int i2c_scan(void)
{
    BYTE b;
    int n = 0;
    
    debug_printf("I2C address scan:");
    for (int a = 0x10; a < 0x80; a+=2)
    {
        if (i2c_read_blocking(I2C, a, &b, 1, false) > 0)
        {
            debug_printf(" 0x%02X", a);
            n++;
        }
    }
    debug_printf("\n");
    return (n);
}

// Initialise serial console interface
// Console UART is set using compiler definition -DPICO_DEFAULT_UART=0 or 1
void serial_init(void)
{
    uart_init(UART, UART_BAUD);
    gpio_set_pulls(PIN_SER_RX, 1, 0);
    gpio_set_function(PIN_SER_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_SER_RX, GPIO_FUNC_UART);
}

// Debug print to console
void debug_printf(const char* format, ...)
{
    va_list argptr;
    va_start(argptr, format);
    vfprintf(stdout, format, argptr);
    va_end(argptr);
}

// EOF
