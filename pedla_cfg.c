// Pi pico logic analyser configuration
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pedla_cfg.h"

const char b64chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

SERVER_ARG server_args[] = { SERVER_ARGVALS };

// Decode integer web request arguments, return number decoded
int cfg_get_args(int sock, char *req, SERVER_ARG *args)
{
    char *p = strchr(req, '?'), *q, *r, name[ARGNAME_MAXLEN];
    int count = 0, n, val = 0;
    
    while (p && *p++ && (q = strchr(p, '=')) && (n = q - p) <= ARGNAME_MAXLEN &&
        ((r = strchr(q, '&')) || (r = strchr(q, 0))))
    {
        strncpy(name, p, n);
        name[n] = 0;
        val = atoi(q + 1);
        cfg_set_argname_int(args, name, val);
        p = r;
        count++;
    }
    return (count);
}

// Get type, give argument number
int cfg_get_argnum_type(SERVER_ARGNUM n) 
{
    return (server_args[n].type);
}

// Get integer value, give argument number
int cfg_get_argnum_int(SERVER_ARGNUM n) 
{
    return (server_args[n].val);
}

// Set integer value, give argument number
void cfg_set_argnum_int(SERVER_ARGNUM n, int val) 
{
    server_args[n].val = n;
}

// Get integer argument value, given name
int cfg_get_argname_int(SERVER_ARG *args, const char *name) 
{
    SERVER_ARG *arg = args;
    int val = -1;
    
    while (arg->name[0]) 
    {
        if (!(strcmp(name, arg->name)))
        {
            val = arg->val;
            break;
        }
        arg++;
    }
    return (val);
}

// Set integer argument value
void cfg_set_argname_int(SERVER_ARG *args, const char *name, int val) 
{
    SERVER_ARG *arg = args;
    while (arg->name[0]) 
    {
        if (!(strcmp(name, arg->name)))
        {
            arg->val = val;
            break;
        }
        arg++;
    }
}

// Start a JSON string
int cfg_json_start(char *buff)
{
    return (sprintf(buff, "{"));
}

// End a JSON string
int cfg_json_end(char *buff, int oset)
{
    return (sprintf(&buff[oset], "}"));
}

// Return arguments as json string, excluding commands
int cfg_json_args(char *buff, int oset, int maxlen, SERVER_ARG *args) 
{
    int n = 0;
    
    while (args->name[0] && n < maxlen-20) 
    {
        if (args->type != ARG_COMMAND_T)
            n += cfg_json_int(buff, oset + n, args->name, args->val);
        args++;
    }
    return (n);
}

// Add a JSON-encoded integer parameter to HTTP response, return length
int cfg_json_int(char *buff, int oset, char *name, int val)
{
    return (sprintf(&buff[oset], "%s\"%s\":%d", oset > 2 ? "," : "", name, val));
}

// Encode binary block as base64 string
int base64_encode(char *out, unsigned char *in, int ilen)
{
    int olen, i, j, v;

    olen = ilen % 3 ? ilen + 3 - ilen % 3 : ilen;
    olen = (olen / 3) * 4;
    out[olen] = '\0';
    for (i = 0, j = 0; i < ilen; i += 3, j += 4) 
    {
        v = in[i];
        v = i + 1 < ilen ? v << 8 | in[i + 1] : v << 8;
        v = i + 2 < ilen ? v << 8 | in[i + 2] : v << 8;
        out[j]   = b64chars[(v >> 18) & 0x3F];
        out[j + 1] = b64chars[(v >> 12) & 0x3F];
        if (i + 1 < ilen)
            out[j + 2] = b64chars[(v >> 6) & 0x3F];
        else
            out[j + 2] = '=';
        if (i + 2 < ilen)
            out[j + 3] = b64chars[v & 0x3F];
        else
            out[j + 3] = '=';
    }
    return (olen);
}

// EOF