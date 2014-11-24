#include "base64.h"
#include <string.h>

/*
** Translation Table as described in RFC1113
*/
static const char cb64[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/*
** Translation Table to decode
*/
static const char cd64[]="|$$$}rstuvwxyz{$$$$$$$>?@ABCDEFGHIJKLMNOPQRSTUVW$$$$$$XYZ[\\]^_`abcdefghijklmnopq";

/*
** encodeblock
**
** encode 3 8-bit binary bytes as 4 '6-bit' characters
*/
static void encodeblock(char *in, char *out, int len )
{
    out[0] = (unsigned char) cb64[ (int)(in[0] >> 2) ];
    out[1] = (unsigned char) cb64[ (int)(((in[0] & 0x03) << 4) | ((in[1] & 0xf0) >> 4)) ];
    out[2] = (unsigned char) (len > 1 ? cb64[ (int)(((in[1] & 0x0f) << 2) | ((in[2] & 0xc0) >> 6)) ] : '=');
    out[3] = (unsigned char) (len > 2 ? cb64[ (int)(in[2] & 0x3f) ] : '=');
}

int bin_to_base64(char *bin, uint32_t bin_length, char *base64, uint32_t *base64_conv_len, uint32_t base64_maxsize)
{
    char in[3];
  int i, enc_indx = 0, len;
    uint32_t output_length = 4 * ((bin_length + 2) / 3);

    if (output_length > base64_maxsize) 
        return 1;

    *in = 0;
    *base64_conv_len = 0;
    
    while( enc_indx < bin_length ) {
        len = 0;
        for( i = 0; i < 3; i++ ) {
            if( enc_indx >= bin_length) {
                in[i] = 0;
            }
            else {
                in[i] = bin[enc_indx];
                enc_indx++;
                len++;
            }
        }
        if( len > 0 ) {
            encodeblock( in, base64 + *base64_conv_len, len );
            *base64_conv_len += 4;
        }
    }
        
    return 0;
}

/*
** decodeblock
**
** decode 4 '6-bit' characters into 3 8-bit binary bytes
*/
static void decodeblock(char *in, char *out, int *conv_len)
{   
    out[0] = (char) (in[0] << 2 | in[1] >> 4);
    *conv_len = 1;
    out[1] = (char) (in[1] << 4 | in[2] >> 2);
    if (out[1] == '\n'){
        out[1] = 0;
        out[2] = 0;
        *conv_len = 1;
        return;
    }
    *conv_len = 2;
    out[2] = (char) (((in[2] << 6) & 0xc0) | in[3]);
    if (out[2] == '\n'){
        out[2] = 0;
        *conv_len = 2;
        return;
    }
    *conv_len = 3;
}

int base64_to_bin(char *base64, uint32_t base64_len, char *bin, uint32_t* bin_conv_len, uint32_t bin_maxsize)
{
    char in[4], out[3];
    int i, dec_indx = 0, len, v;
    uint32_t output_length = base64_len*3/4;

    if (output_length > bin_maxsize) 
        return 1;

    *in = 0;
    *out = 0;
    *bin_conv_len = 0;
    
    while( dec_indx < base64_len ) {
        for( len = 0, i = 0; i < 4 && dec_indx < base64_len; i++ ) {
            v = 0;
            while(dec_indx < base64_len && v == 0 ) {
                v = base64[dec_indx];
                dec_indx++;
                v = ((v < 43 || v > 122) ? 0 : (int) cd64[ v - 43 ]);
                if( v != 0 ) {
                    v = ((v == (int)'$') ? 0 : v - 61);
                }
            }
            if( dec_indx <= base64_len ) {
                len++;
                if( v != 0 ) 
                    in[i] = (char)(v - 1);
            }
            else{
                in[i] = (char)0;
            }
        }
        if( len > 0 ) {
            decodeblock(in, out, &len);
            memcpy(bin + *bin_conv_len, out, len);
            *bin_conv_len += len;
        }
  }

    return 0;
}

