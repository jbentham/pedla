// Pi pico logic analyser configuration definitions
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

#define THRESH_DEFAULT  3           // Default threshold in V
#define ATTEN_DEFAULT   34          // Attenuation, scaling I/P mV -> DAC mV
#define XMAX_DEFAULT    500000      // Max number of samples
#define XSAMP_DEFAULT   1000        // Default number of samples
#define XRATE_DEFAULT   10000       // Default sample rate
#define IP_BASE_DEFAULT 0x0a010100  // Default IP base addr
#define GATEWAY_DEFAULT 0x0a010164  // Default gateway addr
#define UNIT_DEFAULT    201         // Default unit serial number
#define TEMPS_SIZE      2000        // Size of temporay string buffer
#define UNIT_IP_OSET    10          // Offset of last IP addr from serial num
#define NV_BLOCKLEN     256         // Non-volatile parameter storage block
#define SSID_BLOCKLEN   64
#define PASSWD_BLOCKLEN 64

#define PARAM_MAXNAME       15      // Maximum length of parameter name
typedef struct {
    char name[PARAM_MAXNAME + 1];
    int type;
    int val;
} SERVER_PARAM;

typedef enum { ARG_STATUS_T = 1, ARG_CONFIG_T, ARG_SETUP_T, ARG_WIFI_T, ARG_COMMAND_T } PARAM_TYPES;
typedef enum { \
    ARG_STATE, ARG_NSAMP, ARG_XSAMP, ARG_XRATE, ARG_THRESH, \
    ARG_TRIG_CHAN, ARG_TRIG_MODE, ARG_TRIG_POS, ARG_CMD,    \
    ARG_UNIT, ARG_IP_BASE, ARG_GATEWAY, ARG_ATTEN, ARG_XMAX,\
    ARG_SECURITY, ARG_SSID, ARG_PASSWD, ARG_SAVE, ARG_END } SERVER_ARG_NUM;

#define SERVER_PARAM_VALS                      \
/* Current state */                            \
    { "state", ARG_STATUS_T, STATE_READY},     \
    { "nsamp", ARG_STATUS_T, 0},               \
/* Current configuration */                    \
    { "xsamp", ARG_CONFIG_T, XSAMP_DEFAULT},   \
    { "xrate", ARG_CONFIG_T, XRATE_DEFAULT },  \
    { "thresh", ARG_CONFIG_T, THRESH_DEFAULT },\
    { "trig_chan", ARG_CONFIG_T, 0 },          \
    { "trig_mode", ARG_CONFIG_T, 0 },          \
    { "trig_pos", ARG_CONFIG_T, 1 },           \
/* Commands */                                 \
    { "cmd", ARG_COMMAND_T, 0},                \
/* Non-volatile setup */                       \
    { "unit", ARG_SETUP_T, UNIT_DEFAULT},      \
    { "ip_base", ARG_SETUP_T, IP_BASE_DEFAULT},\
    { "gateway", ARG_SETUP_T, GATEWAY_DEFAULT},\
    { "atten", ARG_SETUP_T, ATTEN_DEFAULT},    \
    { "xmax", ARG_SETUP_T, XMAX_DEFAULT },     \
/* Network settings */                         \
    { "security", ARG_SETUP_T, 0},             \
    { "ssid", ARG_WIFI_T, 0},                  \
    { "passwd", ARG_WIFI_T, 0},                \
/* End-marker */                               \
    { "", 0, 0 }

#define NUM_STATES  6
typedef enum { STATE_IDLE, STATE_READY, STATE_PRELOAD,
    STATE_PRETRIG, STATE_POSTTRIG, STATE_UPLOAD
} STATE_VALS;

#define NUM_CMDS    3
typedef enum { CMD_STOP, CMD_SINGLE, CMD_MULTI } CMD_VALS;

bool mstimeout(uint *tickp, uint msec);
int bin_base64len(int dlen);
int base64_enc(void *inp, int inlen, void *outp);
void serial_init(void);
int link_check(void);
int get_param_int(SERVER_ARG_NUM n);
void set_param_int(SERVER_ARG_NUM n, int val);
bool set_param_name_int(char *name, int val);
int json_status(char *buff, int maxlen, int typ);
bool save_params(int addr, SERVER_PARAM *params, char *buff, int blocklen);
int load_params(int addr, SERVER_PARAM *params, char *buff, int blocklen);
void ip_addr_str(uint ip, char *buff);

// EOF
