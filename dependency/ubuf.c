#include "ubuf.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/pgmspace.h>

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define WLIM(buf) ((buf->_tail > 0) ? buf->_tail - 1 : buf->_memsize - 1)
#define SHIFT_TO_END(buf, pos) (((pos) == buf->_memsize) ? 0 : (pos))
#define SHIFT(buf, pos) (((pos) < buf->_memsize) ? (pos) : (pos) - buf->_memsize))

void Ubuf_Init (ubuf_st* buf, void* buffer, cb_size_t size)
{
	buf->_mem = buffer;
	buf->_tail = 0;
	buf->_head = 0;
	buf->_memsize = size;
}

void Ubuf_Clear (ubuf_st* buf)
{
	buf->_tail = 0;
	buf->_head = 0;
}

cb_size_t Ubuf_GetDataSize (ubuf_st* buf)
{
	cb_size_t wpos = buf->_head;
	if (wpos < buf->_tail){
		return buf->_memsize - buf->_tail + wpos;
	} else {
		return wpos - buf->_tail;
	}
}

cb_size_t Ubuf_GetFreeSize (ubuf_st* buf)
{
	cb_size_t rpos = buf->_tail;
	if (buf->_head < rpos){
		return rpos - 1 - buf->_head;
	} else {
		return rpos - 1 - buf->_head + buf->_memsize;
	}
}

bool Ubuf_IsEmpty (ubuf_st* buf)
{
	return buf->_head == buf->_tail;
}

bool Ubuf_IsFull (ubuf_st* buf)
{
	return (buf->_head + 1 == buf->_tail) || (buf->_tail == 0 && buf->_head + 1 == buf->_memsize);
}

uint8_t Ubuf_ReadB (ubuf_st* buf, bool* res)
{
	cb_size_t rpos = buf->_tail;
	if (rpos == buf->_head){
		*res = false;
		return 0;
	}
	buf->_tail = SHIFT_TO_END(buf, rpos + 1);
	*res = true;
	return buf->_mem[rpos];
}

bool Ubuf_WriteB (ubuf_st* buf, char value)
{
	cb_size_t wpos = buf->_head;
	cb_size_t next = SHIFT_TO_END(buf, wpos + 1);
	if (next == buf->_tail){
		return false;
	}
	buf->_mem[wpos] = value;
	buf->_head = next;
	return true;
}

uint8_t Ubuf_ReadBNoRes (ubuf_st* buf)
{
	cb_size_t rpos = buf->_tail;
	if (rpos == buf->_head){
		return 0;
	}
	buf->_tail = SHIFT_TO_END(buf, rpos + 1);
	return buf->_mem[rpos];
}

cb_size_t Ubuf_ReadData (ubuf_st* buf, void* data, cb_size_t size)
{
	char* loc_data = (char*)data;
	size = MIN(size, Ubuf_GetFreeSize(buf));
	cb_size_t res = size;
	cb_size_t cont_size = buf->_memsize - buf->_tail;
	if (size >= cont_size){
		memcpy(loc_data, buf->_mem + buf->_tail, cont_size);
		loc_data += cont_size;
		size -= cont_size;
		buf->_tail = 0;
	}
	if (size > 0) {
		memcpy(loc_data, buf->_mem + buf->_tail, size);
		buf->_tail += size;
	}
	return res;

}

cb_size_t Ubuf_WriteData (ubuf_st* buf, const void* data, cb_size_t size)
{
	const char* loc_data = (const char*)data;
	cb_size_t wpos = buf->_head;// fix wpos
	size = MIN(size, Ubuf_GetFreeSize(buf));
	cb_size_t res = size;
	cb_size_t cont_size = buf->_memsize - wpos;
	if (size >= cont_size){
		memcpy((char*)buf->_mem + wpos, loc_data, cont_size);
		loc_data += cont_size;
		size -= cont_size;
		wpos = 0;
	}
	if (size > 0) {
		memcpy((char*)buf->_mem + wpos, loc_data, size);
		wpos += size;
	}
	buf->_head = wpos;
	return res;
}

cb_size_t Ubuf_WriteData_P (ubuf_st* buf, const void* data, cb_size_t size)
{
	const char* loc_data = (const char*)data;
	cb_size_t wpos = buf->_head;// fix wpos
	size = MIN(size, Ubuf_GetFreeSize(buf));
	cb_size_t res = size;
	cb_size_t cont_size = buf->_memsize - wpos;
	if (size >= cont_size){
		memcpy_P((char*)buf->_mem + wpos, loc_data, cont_size);
		loc_data += cont_size;
		size -= cont_size;
		wpos = 0;
	}
	if (size > 0) {
		memcpy_P((char*)buf->_mem + wpos, loc_data, size);
		wpos += size;
	}
	buf->_head = wpos;
	return res;
}

bool Ubuf_GetFlatMemForRead (ubuf_st* buf, void** data, cb_size_t* size)
{
	if (buf->_tail == buf->_head){
		return false;
	}
	*data = (void*)(buf->_mem + buf->_tail);
	if (buf->_head < buf->_tail){
		*size = buf->_memsize - buf->_tail;
	} else {
		*size = buf->_head - buf->_tail;
	}
	return true;
}

bool Ubuf_GetFlatMemForWrite (ubuf_st* buf, void** data, cb_size_t* size)
{
	cb_size_t end_pos = WLIM(buf);
	if (buf->_head == end_pos)
	return false;

	*data = buf->_mem + buf->_head;
	if (end_pos < buf->_head)
	*size = buf->_memsize - buf->_head;
	else
	*size = end_pos - buf->_head;
	return true;
}

bool Ubuf_Remove (ubuf_st* buf, cb_size_t size)
{
	cb_size_t data_size = Ubuf_GetFreeSize(buf);
	if (size > data_size){
		return false;
	}
	buf->_tail = SHIFT(buf, buf->_tail + size);
	return true;
}

cb_size_t Ubuf_Move (ubuf_st* dest, ubuf_st* src)
{
	cb_size_t bytesLeft = MIN(Ubuf_GetFreeSize(dest), Ubuf_GetFreeSize(src));
	cb_size_t result = bytesLeft;
	while (bytesLeft > 0) {
		cb_size_t blockSize = MIN(bytesLeft, MIN(dest->_memsize - dest->_head, src->_memsize - src->_tail));
		memcpy((char*)dest->_mem + dest->_head, (char*)src->_mem + src->_tail, blockSize);
		dest->_head = SHIFT_TO_END(dest, dest->_head + blockSize);
		src->_tail = SHIFT_TO_END(src, src->_tail + blockSize);
		bytesLeft -= blockSize;
	}
	return result;
}

