#ifndef _MD5_H
#define _MD5_H

typedef struct
{
    UINT32 total[2];
    UINT32 state[4];
    UINT8 buffer[64];
}
md5_context;

void md5_starts( md5_context *ctx );
void md5_update( md5_context *ctx, UINT8 *input, UINT32 length );
void md5_finish( md5_context *ctx, UINT8 digest[16] );

#endif /* md5.h */
