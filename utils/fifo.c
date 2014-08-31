/*
 * fifo.c
 *
 *  Created on: 31 sie 2014
 *      Author: Korzo
 */

#include "fifo.h"
#include <string.h>

//-----------------------------------------------------------------

bool fifo_init(struct fifo *obj, void *buffer, uint32_t capacity, uint32_t item_size)
{
    if (!obj || !buffer || !capacity || !item_size)
        return false;

    obj->buffer = buffer;
    obj->capacity = capacity;
    obj->item_size = item_size;
    obj->length = 0;
    obj->head = 0;
    obj->tail = 0;
    return true;
}

//-----------------------------------------------------------------

bool fifo_enqueue(struct fifo *obj, const void *item)
{
    if (!obj || !item || (obj->length == obj->capacity))
        return false;

    uint32_t offset = obj->item_size * obj->tail;
    memcpy(obj->buffer + offset, item, obj->item_size);
    obj->tail = (obj->tail + 1) % obj->capacity;
    ++obj->length;
    return true;
}

//-----------------------------------------------------------------

bool fifo_dequeue(struct fifo *obj, void *item)
{
    if (!obj || !item || (obj->length == 0))
        return false;

    uint32_t offset = obj->item_size * obj->head;
    memcpy(item, obj->buffer + offset, obj->item_size);
    obj->head = (obj->head + 1) % obj->capacity;
    --obj->length;
    return true;
}

//-----------------------------------------------------------------

bool fifo_peek(struct fifo *obj, void *item)
{
    if (!obj || !item || (obj->length == 0))
        return false;

    uint32_t offset = obj->item_size * obj->head;
    memcpy(item, obj->buffer + offset, obj->item_size);
    return true;
}

//-----------------------------------------------------------------

bool fifo_is_empty(struct fifo *obj)
{
    return obj ? (obj->length == 0) : false;
}

//-----------------------------------------------------------------

bool fifo_is_full(struct fifo *obj)
{
    return obj ? (obj->length == obj->capacity) : false;
}

//-----------------------------------------------------------------

uint32_t fifo_get_length(struct fifo *obj)
{
    return obj ? obj->length : 0;
}

//-----------------------------------------------------------------

bool fifo_clear(struct fifo *obj)
{
    if (!obj)
        return false;

    obj->length = 0;
    obj->head = 0;
    obj->tail = 0;
    return true;
}
