
#ifndef __BASE64_H__
#define __BASE64_H__

#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void base64_encode(const unsigned char *in, int in_len, int (*putc_func)(int));
    void base64_decode(const unsigned char *in, int in_len, int (*putc_func)(int));

#ifdef __cplusplus
}
#endif

#endif /* __BASE64_H__ */
