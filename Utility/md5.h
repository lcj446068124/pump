/* md5.h
 *
 * Copyright (C) 2006-2016 wolfSSL Inc.
 *
 * This file is part of wolfSSL.
 *
 * wolfSSL is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfSSL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA
 */


#ifndef WOLF_CRYPT_MD5_H
#define WOLF_CRYPT_MD5_H

#include "stdint.h"
#include "stdlib.h"
#include "string.h"

#ifdef __cplusplus
    extern "C" {
#endif

/* in bytes */
enum {
    MD5             =  0,      /* hash type unique */
    MD5_BLOCK_SIZE  = 64,
    MD5_DIGEST_SIZE = 16,
    MD5_PAD_SIZE    = 56
};



/* MD5 digest */
typedef struct Md5 {
    uint32_t  buffLen;   /* in bytes          */
    uint32_t  loLen;     /* length in bytes   */
    uint32_t  hiLen;     /* length in bytes   */
    uint32_t  buffer[MD5_BLOCK_SIZE  / sizeof(uint32_t)];
    uint32_t  digest[MD5_DIGEST_SIZE / sizeof(uint32_t)];
} Md5;

void init_md5(Md5*);
void md5_update(Md5*, const uint8_t*, uint32_t);
void md5_final(Md5*, uint8_t*);
int  md5_hash(const uint8_t*, uint32_t, uint8_t*);

#ifdef __cplusplus
    } /* extern "C" */
#endif

#endif /* WOLF_CRYPT_MD5_H */
