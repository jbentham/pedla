// PEDLA remote logic analyser, see https://iosoft.blog/pedla
//
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

// v0.01 JPB 16/4/23  Adapted from web_server v0.30
//                    1200 kbytes/s TCP Tx with 40 MHz SPI, in debug mode
//                    2000 kbytes/s in release mode
//                    2200 to 2600 kbyte/s with 62 MHz SPI
// v0.02 JPB 16/4/23  Added base64-encoded logic analyser display data
// v0.03 JPB 16/4/23  Corrected test data length calculation
// v0.04 JPB 17/4/23  Added octet-stream output for binary data transfer
// v0.05 JPB 17/4/23  Increased test data count, added browser data-rate display
// v0.06 JPB 17/4/23  Moved from picowix project to pedla, using PEDLA hardware
//                    Added UART init, hardware pins init, and extra GPIO functions
// v0.07 JPB 29/5/23  Added decode of HTTP request arguments
// v0.08 JPB 30/5/23  Split arguments into state, setup, and config
// v0.09 JPB 5/6/23   Moved config_args from picowi to pico_la
// v0.10 JPB 5/6/23   Moved configuration data from pedla into pedla_cfg
// v0.11 JPB 5/6/23   Added data capture interface
// v0.12 JPB 20/6/23  Corrected end-of-clock detection
//                    Use PIO programs for SET_SCK and CLR_SCK (slow!)
// v0.20 JPB 5/12/23  Modified pedla v0.12 to use Mongoose TCP stack
//                    Added code from mg_ether v0.05, pedla_test v0.25
// v0.21 JPB 1/1/24   Adapted from mg_wifi v0.15
// v0.22 JPB 2/1/24   Added logic analyser code from pedla_test v0.05 and v0.25
// v0.23 JPB 10/2/24  Added JSON-format status.txt response
// v0.24 JPB 12/2/24  Added binary file transfer, allow CORS
// v0.25 JPB 12/2/24  Do binary file transfers in bytes, not words
//                    (after 10 transfers, fs_read buffer has odd address)
// v0.26 JPB 13/2/24  Added base64 encoder, and encoding tests
// v0.27 JPB 14/2/24  Use base64 encoding for data.txt
// v0.28 JPB 14/2/24  Corrected discontinuity in base64 data
// v0.29 JPB 14/2/24  Added file read code to handle small-size requests
// v0.30 JPB 15/2/24  Simplified file read logic by using extra texblock
// v0.31 JPB 15/2/24  Simplified base64 read
// v0.32 JPB 15/2/24  Added binary & base64 test files, test.bin & test.txt
// v0.33 JPB 16/2/24  Linked force-down to mg_wifi_up
//                    Added is_draining to test file transfer start
// v0.34 JPB 17/2/24  Corrected la_fs_bin_data() handling odd start address
// v0.35 JPB 18/2/24  Corrected warnings generated by -Wextra
// v0.36 JPB 19/2/24  Added code to switch between test & LA file data
//                    Corrected bin_base64len calculation (don't round up)
// v0.37 JPB 20/2/24  Corrected padding of base64 data
// v0.38 JPB 26/2/24  Added code to handle HTTP query parameters
// v0.39 JPB 27/2/24  Set ADC and tri-state bus, to get input signals
// v0.40 JPB 28/2/24  Added console IP address printout
// v0.41 JPB 29/2/24  Changed utility filenames to have 'la_' prefix
// v0.42 JPB 1/3/24   Added serial console interface
// v0.43 JPB 2/3/24   Load setup from EEPROM on startup
//                    Restored clock-pulse-count test
// v0.44 JPB 3/3/24   Added dummy-cycle DMA to count clock pulses
// v0.45 JPB 3/3/24   Dropped system clock from 125 to 120 MHz
//                    Added simple data capture test
// v0.46 JPB 5/3/24   Added root file with version number
//                    Switch clock between pulse O/P (PWM) and GPIO (SIO)
// v0.47 JPB 6/3/24   Removed casts of byte buffer to uint32_t *
// v0.48 JPB 6/3/24   Added alternative bus_recv_qspi_bytes
// v0.49 JPB 10/3/24  Test with simulated data loaded into RAM
//                    Removed misaligned 16-bit transfers
// v0.50 JPB 10/3/24  Simplified QSPI read & write (removed nybble shifting)
//                    Corrected RAM address & length for odd byte-counts
// v0.51 JPB 11/3/24  Test of 2nd PIO instance to handle RAM clocking
// v0.52 JPB 11/3/24  Further testing of PIO RAM clocking
// v0.53 JPB 11/3/24  Reverted to v0.50, speed up bus_recv_qspi_bytes
// v0.54 JPB 12/3/24  Initial testing of data capture
// v0.55 JPB 12/3/24  Corrected set-function for RAM clock pin
//                    Corrected end-of-capture detection and cleanup
// v0.56 JPB 13/3/24  Set state to STATE_READY after capture
// v0.57 JPB 17/3/24  Added LED indication when capturing
//                    Added check for idle file pointers
// v0.58 JPB 18/3/24  Removed redundant code, tested release version
// v0.59 JPB 20/3/24  Added threshold setting
// v0.60 JPB 21/3/24  Changed EEPROM_PAGE_LEN from 128 to 64 for 24LC256
//                    Corrected console setting of unit number
// v0.61 JPB 23/3/24  Update parameter 'nsamp' when capturing
//                    Set initial 'nsamp' value to 0
// v0.62 JPB 24/3/24  Added SSID and password to NV store
//                    Added indication of incorrect password
// v0.63 JPB 25/3/24  Added security to NV store
//                    Use DHCP if base IP address is zero
// v0.64 JPB 25/3/24  Removed test mode
// v0.65 JPB 25/3/24  Corrected IP base address setting
// v0.66 JPB 26/3/24  Corrected version string

#define SW_VERSION   "0.66"

#include <stdio.h>
#include <string.h>

#include "pico/rand.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "picowi/picowi_defs.h"
#include "picowi/picowi_auth.h"
#include "mongoose.h"
#include "la_wifi.h"

#include "pedla_cfg.h" 
#include "la_pico.h"
#include "la_console.h"

// Web server
#define LISTEN_URL          "http://0.0.0.0:80"
#define ROOT_FILENAME       "/"
#define LA_FNAME_BASE64     "/data.txt"
#define LA_FNAME_BIN        "/data.bin"
#define STATUS_FILENAME     "/status.txt"
#define BASE64_SEG_SIZE     720
#define MAX_DATALEN         ((BASE64_SEG_SIZE*4)/3)
#define BASE64_TEXTBLOCKLEN 24
#define BASE64_BINBLOCKLEN  ((BASE64_TEXTBLOCKLEN*3)/4)

// Timeout values in msec
#define LINK_UP_BLINK       500
#define LINK_DOWN_BLINK     100
#define JOIN_DOWN_MS        3000   

// Maximum number of simultaneous TCP connections
#define MAXCONNS            8

// HTML header to disable client caching
#define NO_CACHE "Cache-Control: no-cache, no-store, must-revalidate\r\nPragma: no-cache\r\nExpires: 0\r\n"
#define ALLOW_CORS "Access-Control-Allow-Origin: *\r\n"
#define TEXT_PLAIN "Content-Type: text/plain\r\n"
    
// Structure to hold parameters for an open file
typedef struct {
    uint outpos, outlen, inpos, inlen, index, millis;
    bool inuse, base64;
} FILESTRUCT;

FILESTRUCT filestructs[MAXCONNS];
bool force_down;
extern struct mg_fs mg_test_fs;
int startval;
char base64_textblock[BASE64_TEXTBLOCKLEN+1];
char version[] = "PICO_LA v" SW_VERSION;

uint ready_ticks, led_ticks;
char temps[TEMPS_SIZE];
const char base64_chars[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

SERVER_PARAM server_params[] = { SERVER_PARAM_VALS };
CAP_PARAMS caparams = { .xsamp = XSAMP_DEFAULT, .xrate = 000000, .nsamp = 0, .capturing = 0 };

extern char wifi_ssid[SSID_MAXLEN + 1];
extern char wifi_passwd[PASSWD_MAXLEN + 1];
extern int wifi_security;

void cap_disp(int addr, int len);
void listener(struct mg_connection *c, int ev, void *ev_data, void *fn_data);
void web_get_params(struct mg_http_message *hm, SERVER_PARAM *args, int *cmdp);
void poll_console(void);
void la_fs_check(void);

int main(void) 
{
    bool ledon = 0;
    struct mg_mgr mgr;
    struct mg_tcpip_if mif = {.driver = &mg_tcpip_driver_wifi, .mgr = &mgr};
    
    set_sys_clock_khz(PWM_CLOCK/1000, true);
    stdio_init_all();
    serial_init();
    la_pins_init();
    dac_init();
    spi_bus_init();
    
    qspi_end();
    la_set_led(1);
    xprintf("\nPEDLA v" SW_VERSION "\n");
    la_ram_test();
    set_threshold(THRESH_DEFAULT);
    i2c_eeprom_init();
    xprintf("Loading setup...");
    xprintf(load_params(0, server_params, temps, NV_BLOCKLEN) ?
        " OK\n" : " failed\n");
    if (get_param_int(ARG_IP_BASE))
    {
        mif.ip = mg_htonl(get_param_int(ARG_IP_BASE) + get_param_int(ARG_UNIT));
        mif.mask = mg_htonl(MG_U32(255, 255, 255, 0));
        mif.gw = mg_htonl(get_param_int(ARG_GATEWAY));
        ip_addr_str(get_param_int(ARG_IP_BASE) + get_param_int(ARG_UNIT), temps);
        xprintf("Using static IP %s\n", temps);
    }
    else
        xprintf("Using dynamic IP (DHCP)\n");
    
    ram_data_sim(0, caparams.xsamp);
    cap_init(&caparams);

    mg_mgr_init(&mgr);
    mg_log_set(MG_LL_NONE);
    mg_tcpip_init(&mgr, &mif);
    mg_http_listen(&mgr, LISTEN_URL, listener, &mgr);
    la_set_led(0);
    for (;;) 
    {
        mg_mgr_poll(&mgr, 2);
        if (mstimeout(&led_ticks, link_check() > 0 ? LINK_UP_BLINK : LINK_DOWN_BLINK))
        {
            wifi_set_led(ledon = !ledon);
            if (ledon)
                la_fs_check();
        }
        if (caparams.capturing && !cap_capturing(&caparams))
        {
            cap_end(&caparams);
            set_state(STATE_READY);
        }
        if (!mif.driver->up(0))
            wifi_poll(0, 0);
        console_poll();
    }
    return 0;
}

// Display RAM in QSPI mode
void cap_disp(int addr, int len)
{
    qspi_begin();
    la_ram_read_qspi_words(addr, (WORD *)temps, len);
    qspi_end();
    dump_words((WORD *)temps, len);
}

// Return non-zero if timeout
bool mstimeout(uint *tickp, uint msec)
{
    uint t = mg_millis();
    uint dt = t - *tickp;

    if (msec == 0 || dt >= msec)
    {
        *tickp = t;
        return (1);
    }
    return (0);
}

// Return char count of base64-encoded string, given original length
// If len is not divisible by 3, add 2 chars + 2 pad, or 3 chars + 1 pad
int bin_base64len(int dlen)
{
    return (dlen / 3) * 4 + (dlen % 3 ? 4 : 0);
}

// Encode data into base64 (3 bytes -> 4 chars)
int base64_enc(void *inp, int inlen, void *outp)
{
    int olen=0, val, n=inlen/3, extra=inlen%3;
    BYTE *ip = (BYTE *)inp;
    char *op = (char *)outp;
    
    while (n--)
    {
        val =  (int)*ip++ << 16;
        val += (int)*ip++ << 8;
        val += (int)*ip++;
        op[olen++] = base64_chars[(val >> 18) & 0x3F];
        op[olen++] = base64_chars[(val >> 12) & 0x3F];
        op[olen++] = base64_chars[(val >> 6)  & 0x3F];
        op[olen++] = base64_chars[val         & 0x3F];
    }
    if (extra)
    {
        val = (int)*ip++ << 16;
        if (extra > 1)
            val += (int)*ip++ << 8;
        op[olen++] = base64_chars[(val >> 18) & 0x3F];
        op[olen++] = base64_chars[(val >> 12) & 0x3F];
        if (extra > 1)
            op[olen++] = base64_chars[(val >> 6) & 0x3F];
        while (extra++ < 3)
            op[olen++] = '=';
    }
    return (olen);
}

// Return descriptive string if state has changed
char *state_change(int state)
{
    static int last_state = 0;
    static char *states[] = { "DOWN", "UP", "REQ", "READY" };

    if (state != last_state && state >= MG_TCPIP_STATE_DOWN && state <= MG_TCPIP_STATE_READY)
    {
        last_state = state;
        return states[state];
    }
    return 0;
}

// Functions providing a file-like interface for data in analyser RAM

// Return file list (not implemented)
static void no_fs_list(const char *path, void(*fn)(const char *, void *), void *userdata)
{
    (void) path, (void) fn, (void) userdata;
}
// Write file (not implemented)
static size_t no_fs_write(void *fd, const void *buf, size_t len) 
{
    (void) fd, (void) buf, (void) len;
    return 0;
}
// Rename file (not implemented)
static bool no_fs_rename(const char *from, const char *to) 
{
    (void) from, (void) to;
    return false;
}
// Remove file (not implemented)
static bool no_fs_remove(const char *path) 
{
    (void) path;
    return false;
}
// Make directory (not implemented)
static bool no_fs_mkdir(const char *path) 
{
    (void) path;
    return false;
}

// Return status of logic analyser base64 file interface
static int la_fs_stat_base64(const char *path, size_t *size, time_t *mtime)
{
    static time_t t = 0;
    if (size)
    {
        *size = bin_base64len(caparams.nsamp * 2);
        xprintf("Stat  file %s size %u\n", path, *size);
    }
    if (mtime)
        *mtime = t++;
    return (caparams.nsamp * 2);
}

// Return status of logic analyser binary file interface
static int la_fs_stat_bin(const char *path, size_t *size, time_t *mtime)
{
    static time_t t = 0;
    if (size)
    {
        *size = caparams.nsamp * 2;
        xprintf("Stat  file %s size %u\n", path, *size);
    }
    if (mtime)
        *mtime = t++;
    return (caparams.nsamp * 2);
}

// Start analyser file transfer, binary mode
static void *la_fs_open_bin(const char *path, int flags) 
{
    (void) flags;
    if (strstr(path, ".gz"))
        return NULL;
    for (int i = 0; i < MAXCONNS; i++)
    {
        if (!filestructs[i].inuse) 
        {
            FILESTRUCT *fptr = &filestructs[i];
            fptr->inuse = true;
            fptr->base64 = false;            
            fptr->outlen = fptr->inlen = caparams.nsamp * 2;
            fptr->outpos = fptr->inpos = 0;
            fptr->index = i;
            fptr->millis = mg_millis();
            xprintf("Open  file %u %s\n", fptr->index, path);
            return(fptr);
        }
    }
    return(NULL);
}

// Start analyser file transfer, base64 mode
static void *la_fs_open_base64(const char *path, int flags) 
{
    FILESTRUCT *fptr = (FILESTRUCT *)la_fs_open_bin(path, flags);
    
    if (fptr)
    {
        fptr->outlen = bin_base64len(caparams.nsamp * 2);
        fptr->base64 = 1;
    }
    return (void *)fptr;
}

// Close file
static void la_fs_close(void *fp) 
{
    FILESTRUCT *fptr = fp;
    
    uint dt = mg_millis() - fptr->millis;
    uint speed = dt ? (fptr->outlen * 1000) / dt : 0;
    xprintf("Close file %u, %u msec, %u of %u bytes, %u bytes/sec\n", 
        fptr->index, dt, fptr->outpos, fptr->outlen, speed);
    fptr->outpos = fptr->outlen = fptr->inuse = 0;
    base64_textblock[0] = 0;
    startval += XSAMP_DEFAULT / 100;
}

// Check for old (unused) file pointers
void la_fs_check(void) 
{
    for (int i = 0; i < MAXCONNS; i++)
    {
        FILESTRUCT *fptr = &filestructs[i];
        if (fptr->inuse && mg_millis() - fptr->millis > 30000) 
        {
            fptr->outpos = fptr->outlen = fptr->inuse = 0;
            xprintf("Closing unused file transfer\n");
        }
    }
}

// Return next block of binary data
int la_fs_bin_data(FILESTRUCT *fptr, void *buf, int len)
{
    qspi_begin();
    la_ram_read_qspi_bytes(fptr->inpos, buf, len);
    qspi_end();
    return (len);
} 
    
// Read data stream, returning base64 encoded data
static size_t la_fs_read_base64(void *fd, void *buf, size_t length) 
{
    FILESTRUCT *fptr = fd;
    int outlen = 0, end = (fptr->outpos + length >= fptr->outlen);
    
    if (length && length < BASE64_TEXTBLOCKLEN && !base64_textblock[0])
    {
        la_fs_bin_data(fptr, temps, BASE64_BINBLOCKLEN);
        base64_enc(temps, end ? fptr->inlen-fptr->inpos : BASE64_BINBLOCKLEN, base64_textblock);
        fptr->inpos += BASE64_BINBLOCKLEN;
    }        
    if (length && base64_textblock[0])
    {
        int n = fptr->outpos % BASE64_TEXTBLOCKLEN;
        outlen = MIN((int)length, (int)BASE64_TEXTBLOCKLEN - n);
        memcpy(buf, &base64_textblock[n], outlen);
        if (n + outlen >= BASE64_TEXTBLOCKLEN || end)
            base64_textblock[0] = 0;
    }
    else
    {
        outlen = MIN(MAX_DATALEN, length);
        outlen -= outlen % BASE64_TEXTBLOCKLEN;
        int inlen = (outlen * 3) / 4;
        la_fs_bin_data(fptr, temps, inlen);
        fptr->inpos += inlen;
        base64_enc(temps, inlen, buf);
    }
#if DISP_BLOCKS    
    xprintf("Read  file %u req %4d dlen %4d pos %d len %d\n", fptr->index, length, outlen, fptr->outpos, fptr->len);
#endif    
    fptr->outpos += outlen;
    return(outlen);
}

// Read data stream, returning binary data
static size_t la_fs_read_bin(void *fd, void *buf, size_t length) 
{
    FILESTRUCT *fptr = fd;
    int outlen = MIN(fptr->outlen - fptr->outpos, length);
    
    la_fs_bin_data(fptr, buf, outlen);
#if DISP_BLOCKS    
    xprintf("Read  file %u req %4d dlen %4d pos %d len %d\n", 
        fptr->index, length, outlen, fptr->outpos, fptr->len);
#endif
    fptr->inpos += outlen;
    fptr->outpos += outlen;
    return (outlen);
}

// Move file pointer
static size_t la_fs_seek(void *fd, size_t offset) 
{
    (void) fd, (void) offset;
    FILESTRUCT *fptr = fd;
    xprintf("Seek  file %u offset %d\n", fptr->index, offset);
    fptr->outpos = MIN(fptr->outlen, offset);
    return(fptr->outpos);
}

// Pointers to logic analyser base64 file functions
struct mg_fs mg_la_fs_base64 = 
{
    la_fs_stat_base64,  no_fs_list,  la_fs_open_base64,  la_fs_close, la_fs_read_base64,
    no_fs_write,  la_fs_seek, no_fs_rename, no_fs_remove, no_fs_mkdir
 };

// Pointers to logic analyser binary file functions
struct mg_fs mg_la_fs_bin = 
{
    la_fs_stat_bin,  no_fs_list,  la_fs_open_bin,  la_fs_close, la_fs_read_bin,
    no_fs_write,  la_fs_seek, no_fs_rename, no_fs_remove, no_fs_mkdir
 };

// Connection callback
void listener(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{
    struct mg_mgr *mgrp = (struct mg_mgr *)fn_data;
    struct mg_tcpip_if *ifp = mgrp->priv;
    struct mg_http_serve_opts opts = {.extra_headers = NO_CACHE ALLOW_CORS};

    if (ev == MG_EV_HTTP_MSG) 
    {
        struct mg_http_message *hm = (struct mg_http_message *) ev_data;
        int len = hm->method.len + hm->uri.len + hm->query.len + 2, cmd=-1;
        
        for (int i = 0; i < len; i++)
            putchar(hm->method.ptr[i]);
        putchar('\n');

        web_get_params(hm, server_params, &cmd);
        if (cmd >= 0)
        {
            xprintf("Command %d\n", cmd);
            if (cmd == CMD_SINGLE || cmd == CMD_MULTI)
            {
                caparams.xsamp = get_param_int(ARG_XSAMP);
                caparams.xrate = get_param_int(ARG_XRATE);
                set_state(STATE_POSTTRIG);
                cap_start(&caparams);
            }
        }
        if (mg_http_match_uri(hm, ROOT_FILENAME)) 
        {
            mg_http_reply(c, 200, TEXT_PLAIN ALLOW_CORS, "%s", version); 
        }
        else if (mg_http_match_uri(hm, STATUS_FILENAME)) 
        {
            json_status(temps, sizeof(temps) - 1, 0);
            //xprintf("%s\n", temps);
            mg_http_reply(c, 200, NO_CACHE ALLOW_CORS, "%s", temps); 
        }
        else if (mg_http_match_uri(hm, LA_FNAME_BASE64)) 
        {
            opts.fs = &mg_la_fs_base64;
            mg_http_serve_dir(c, hm, &opts);
            c->is_draining = 1;
        }
        else if (mg_http_match_uri(hm, LA_FNAME_BIN)) 
        {
            opts.fs = &mg_la_fs_bin;
            mg_http_serve_dir(c, hm, &opts);
            c->is_draining = 1;
        }
        else 
        {
            mg_http_reply(c, 404, "", "Not Found\n");
        }
    }
    else 
    {
        char *s = state_change(ifp->state);
        if (s) 
        {
            if (ifp->state == MG_TCPIP_STATE_READY)
                xprintf("IP state: %s, IP: %M, GW: %M\n", s, mg_print_ip4, &ifp->ip, mg_print_ip4, &ifp->gw);
            else
                xprintf("IP state: %s\n", s);
            mstimeout(&ready_ticks, 0);
        }
        else if (ifp->state != MG_TCPIP_STATE_READY && mstimeout(&ready_ticks, JOIN_DOWN_MS))
            force_down = !force_down;
    }
}

// Get HTTP query parameter values, including a command value (if present)
void web_get_params(struct mg_http_message *hm, SERVER_PARAM *args, int *cmdp)
{
    while (args->type)
    {
        if (args->type==ARG_CONFIG_T && 
            mg_http_get_var(&hm->query, args->name, temps, sizeof(temps)) > 0)
            args->val = strtol(temps, NULL, 10);
        else if (args->type == ARG_COMMAND_T && cmdp &&
            mg_http_get_var(&hm->query, args->name, temps, sizeof(temps)) > 0)
            *cmdp = args->val = strtol(temps, NULL, 10);
        args++;
    }
}

// Get integer parameter value, given index number
int get_param_int(SERVER_ARG_NUM n)
{
    return (n <= ARG_SAVE) ? server_params[n].val : 0;       
}
// Set integer parameter value, given index number
void set_param_int(SERVER_ARG_NUM n, int val)
{
    if (n < ARG_SAVE)
        server_params[n].val = val;       
}

// Set integer parameter value, given name
bool set_param_name_int(char *name, int val)
{
    SERVER_PARAM *sp = server_params;
    bool ret = false;
    
    while (!ret && sp->name[0]) 
    {
        if (!strcmp(name, sp->name))
        {
            sp->val = val;
            ret = true;
        }
        sp++;
    }
    return (ret);
}

// Return server status as json string
int json_status(char *buff, int maxlen, int typ) 
{
    SERVER_PARAM *arg = server_params;
    int n = sprintf(buff, "{");
    
    while (arg->name[0] && n < maxlen - 20)
    {
        if ((typ == 0 && arg->type < ARG_SETUP_T) || arg->type == typ)
            n += sprintf(&buff[n], "%s\"%s\":%d", n > 2 ? "," : "", 
                arg->name, arg->val);
        arg++;
    }
    return (n += sprintf(&buff[n], "}"));
}

// Save parameters as JSON string in non-volatile memory
bool save_params(int addr, SERVER_PARAM *params, char *buff, int blocklen)
{
    int n;
    
    memset(buff, 0, blocklen);
    n = sprintf(buff, "{");
    while (params->name[0] && n < blocklen - (PARAM_MAXNAME + 10))
    {
        if (params->type == ARG_SETUP_T)
            n += sprintf(&buff[n], "%s\"%s\":%d", n > 2 ? "," : "", params->name, params->val);
        params++;
    }
    n += sprintf(&buff[n], "}");
    return (i2c_eeprom_write(addr, (BYTE *)buff, blocklen) &&
        i2c_eeprom_write(NV_BLOCKLEN, (BYTE *)wifi_ssid, strlen(wifi_ssid) + 1) &&
        i2c_eeprom_write(NV_BLOCKLEN + SSID_BLOCKLEN, (BYTE *)wifi_passwd, strlen(wifi_passwd) + 1));
}

// Load parameters from JSON string in non-volatile memory
int load_params(int addr, SERVER_PARAM *params, char *buff, int blocklen)
{
    struct mg_str mgs;
    int n = 0, i = 0, val;
    char *str=temps, s[PARAM_MAXNAME + 3] = "$.";
    
    i2c_eeprom_read(addr, (BYTE *)buff, blocklen);
    if (buff[0] == '{')
    {
        mgs = mg_str(buff);
        while (params[i].name[0])
        {
            strcpy(&s[2], params[i].name);
            if ((val=mg_json_get_long(mgs, s, -1)) != -1)
            {
                params[i].val = val;
                if (i == ARG_SECURITY)
                    wifi_security = val;
                n++;
            }
            i++;
        }
        i2c_eeprom_read(NV_BLOCKLEN, (BYTE *)temps, SSID_BLOCKLEN + PASSWD_BLOCKLEN);
        if (*str>0 && *str < 0x7f && strlen(str) <= SSID_MAXLEN)
            strcpy(wifi_ssid, str);
        str += SSID_BLOCKLEN;
        if (*str>0 && *str < 0x7f && strlen(str) <= PASSWD_MAXLEN)
            strcpy(wifi_passwd, str);
    }
    return (n);
}

// Convert IP address to a string
void ip_addr_str(uint ip, char *buff)
{
    mg_snprintf(buff, 16, "%u.%u.%u.%u", (BYTE)(ip >> 24), 
        (BYTE)(ip >> 16), (BYTE)(ip >> 8), (BYTE)ip);
}

// EOF