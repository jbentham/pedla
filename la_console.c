// Serial console for PEDLA remote logic analyser

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

#include <string.h>
#include "mongoose.h"
#include "picowi/picowi_defs.h"
#include "pedla_cfg.h" 
#include "la_pico.h"
#include "la_wifi.h"
#include "la_console.h"

extern char temps[TEMPS_SIZE];
char conbuff[100];
int conin;
extern int display_mode;
extern SERVER_PARAM server_params[];
extern char wifi_ssid[SSID_MAXLEN + 1];
extern char wifi_passwd[PASSWD_MAXLEN + 1];
extern int wifi_security;

// Display setup parameter
void console_disp_param(SERVER_PARAM *params, int arg, char *eol)
{
    int ip = get_param_int(arg);
    
    if (arg == ARG_IP_BASE || arg == ARG_GATEWAY)
        xprintf("%s=%d.%d.%d.%d%s", params[arg].name, 
            (BYTE)(ip >> 24), (BYTE)(ip >> 16), (BYTE)(ip >> 8), (BYTE)ip, eol);
    else if (arg == ARG_SSID)
        xprintf("%s=%s%s", params[arg].name, wifi_ssid, eol);
    else if (arg == ARG_PASSWD)
        xprintf("%s=%s%s", params[arg].name, wifi_passwd, eol);
    else if (arg == ARG_SECURITY)
        xprintf("%s=%s%s", params[arg].name, auth_type_str(wifi_security), eol);
    else
        xprintf("%s=%d%s", params[arg].name, params[arg].val, eol);
}

// Get a parameter value
int console_get_value(int argnum, char *str)
{
    int val=0, i=0;
    char *s;
    
    if (argnum == ARG_IP_BASE || argnum == ARG_GATEWAY)
    {
        s = strtok(str, ".");
        while (s && i < 4)
        {
            val = (val << 8) + atoi(s);
            i++;
            s = strtok(0, ".");
        }
        while (i++ < 4)
            val <<= 8;
    }
    else if (argnum == ARG_SAVE)
        val = str[0] == 'y' || str[0] == 'Y';
    else 
        val = atoi(str);
    return (val);
}

// Display prompt for setup parameters
void console_list_params(SERVER_PARAM *params)
{
    console_disp_param(params, ARG_UNIT, ", ");
    console_disp_param(params, ARG_IP_BASE, ", ");
    console_disp_param(params, ARG_GATEWAY, ", ");
    console_disp_param(params, ARG_ATTEN, ", ");
    console_disp_param(params, ARG_XMAX, ", ");
    console_disp_param(params, ARG_SECURITY, ", ");
    console_disp_param(params, ARG_SSID, ", ");
    console_disp_param(params, ARG_PASSWD, "");
}

// Poll the serial console
void console_poll(void)
{
    char c;
    SERVER_PARAM *params = server_params;
    static int argnum = -1, disp_mode=-1;
    int val=0;
        
    if (uart_is_readable(UART))
    {
        c = uart_getc(UART);
        if (c == '\b')
        {
            if (conin > 0)
            {
                conin--;
                conbuff[conin] = 0;
                putchar('\b');
                putchar(' ');
                putchar('\b');
            }
        }
        else if (c == 0x1b)
        {
            display_mode = conin = conbuff[0] = 0;
            argnum = -1;
            ip_addr_str(get_param_int(ARG_IP_BASE) + get_param_int(ARG_UNIT), temps);
            xprintf("\nIP address %s\n", temps);
            console_list_params(server_params);
            xprintf("\n");
            if (disp_mode > 0)
                display_mode = disp_mode;
            disp_mode = -1;
        }
        else if (c == '\r')
        {
            if (argnum < 0)
            {
                argnum = ARG_UNIT - 1;
                conin = 0;
                disp_mode = display_mode;
                display_mode = 0;
            }
            conbuff[conin] = 0;
            putchar('\n');
            if (conin > 0)
            {
                if (argnum == ARG_SSID && strlen(conbuff) <= SSID_MAXLEN)
                    strcpy(wifi_ssid, conbuff);
                else if (argnum == ARG_PASSWD && strlen(conbuff) <= PASSWD_MAXLEN)
                    strcpy(wifi_passwd, conbuff);                        
                else if (argnum == ARG_SECURITY)
                {
                    if ((val = auth_str_type(conbuff)) < 0)
                    {
                        xprintf("Security type not recognised\n");
                        argnum--;
                    }            
                    else
                        set_param_int(argnum, val);
                }
                else
                {
                    val = console_get_value(argnum, conbuff);
                    set_param_int(argnum, val);
                }
            }
            conin = 0;
            argnum++;
            if (argnum < ARG_SAVE)
                console_disp_param(params, argnum, "? ");
            else if (argnum == ARG_SAVE)
            {
                console_list_params(server_params);
                xprintf("\nSave setup (y/N)? ");
            }
            else
            {   
                if (val)
                {
                    xprintf("Saving setup...");
                    xprintf(save_params(0, server_params, temps, NV_BLOCKLEN) ?
                        " OK\n" : " failed\n");
                }
                else
                    xprintf("Not saved\n");
                argnum = -1;
                if (disp_mode > 0)
                    display_mode = disp_mode;
                disp_mode = -1;
            }
        }
        else if (conin < (int)sizeof(conbuff)-1)
        {
            conbuff[conin++] = c;
            putchar(c);
        }
    }
}

// EOF
