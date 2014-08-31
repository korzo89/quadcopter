/*
 * fifo.h
 *
 *  Created on: 31 sie 2014
 *      Author: Korzo
 */

#ifndef FIFO_H_
#define FIFO_H_

//-----------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

//-----------------------------------------------------------------

struct fifo
{
    uint8_t     *buffer;
    uint32_t    item_size;
    uint32_t    capacity;
    uint32_t    length;
    uint32_t    head;
    uint32_t    tail;
};

//-----------------------------------------------------------------

bool fifo_init(struct fifo *obj, void *buffer, uint32_t capacity, uint32_t item_size);

bool fifo_enqueue(struct fifo *obj, const void *item);
bool fifo_dequeue(struct fifo *obj, void *item);
bool fifo_peek(struct fifo *obj, void *item);

bool fifo_is_empty(struct fifo *obj);
bool fifo_is_full(struct fifo *obj);

uint32_t fifo_get_length(struct fifo *obj);

bool fifo_clear(struct fifo *obj);

//-----------------------------------------------------------------

#endif /* FIFO_H_ */
