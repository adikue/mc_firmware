#ifndef BASE_64_H
#define BASE_64_H

#include <stdint.h>

/**
    * @brief  Converts bin string to base64 format
    * @param  bin: Pointer to bin str

    * @param  bin_length: Length of bin string

    * @param  base64: Pointer to base64 buffer. Converted string will be stored there

    * @param  base64_conv_len: Pointer on return stores actual base64 string length.

    * @param  base64_maxsize: Max base64 size allowed to convert.

    * @retval Status of converting. The returned value can be one
    *         of the following:
    *               0: success
    *               1: base64 buffer overflow. Length of converted base64 string
    *                 more than base64_maxlen or more than base64_buflen
*/
int bin_to_base64(char *bin, uint32_t bin_length, char *base64, uint32_t *base64_conv_len, uint32_t base64_maxsize);

/**
    * @brief  Converts base64 string to bin format
    * @param  base64: Pointer to base64 str

    * @param  base64_len: Length of base64 string

    * @param  bin: Pointer to bin buffer. Converted string will be stored there

    * @param  bin_conv_len: Pointer on return stores actual base64 string length.
    
    * @param  bin_maxsize: Max bin size allowed to convert.

    * @retval Status of converting. The returned value can be one
    *         of the following:
    *               0: success
    *               1: bin buffer overflow. Length of converted bin string
    *                 more than bin_maxsize or more than bin_buflen
*/
int base64_to_bin(char *base64, uint32_t base64_len, char *bin, uint32_t* bin_conv_len, uint32_t bin_maxsize);

#endif /*BASE_64_H*/
