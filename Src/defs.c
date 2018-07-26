/*
 * defs.c
 *
 *  Created on: 20 ���. 2018 �.
 *      Author: Vanyw
 */

#include "defs.h"

void volmemcpy (void *dest, volatile const void *src, uint16_t len)
{
	uint8_t *d = dest;
  volatile const uint8_t *s = src;
  while (len--)
    *d++ = *s++;
}
