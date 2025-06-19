#ifndef _UNIVERSAL_BUFFER
#define _UNIVERSAL_BUFFER

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

typedef uint8_t cb_size_t;

typedef struct ubuf_s {
	char* _mem;
	cb_size_t _memsize;
	cb_size_t _tail;
	volatile cb_size_t _head;
} ubuf_st;

#define UBUF_STATIC_INIT(p) {(char*)p, 0, 0, sizeof(p)}
	
cb_size_t Ubuf_GetDataSize (ubuf_st* buf);
cb_size_t Ubuf_GetFreeSize (ubuf_st* buf);
bool Ubuf_IsEmpty (ubuf_st* buf);
bool Ubuf_IsFull (ubuf_st* buf);
void Ubuf_Init (ubuf_st* buf, void* buffer, cb_size_t size);
void Ubuf_Clear (ubuf_st* buf);
bool Ubuf_Remove (ubuf_st* buf, cb_size_t size);
bool Ubuf_WriteB (ubuf_st* buf, char value);
uint8_t Ubuf_ReadB (ubuf_st* buf, bool* res);
uint8_t Ubuf_ReadBNoRes (ubuf_st* buf);
cb_size_t Ubuf_WriteData (ubuf_st* buf, const void* data, cb_size_t size);
cb_size_t Ubuf_WriteData_P(ubuf_st* buf, const void* data, cb_size_t size);
cb_size_t Ubuf_ReadData (ubuf_st* buf, void* data, cb_size_t size);
bool Ubuf_GetFlatMemForRead (ubuf_st* buf, void** data, cb_size_t* size);
bool Ubuf_GetFlatMemForWrite (ubuf_st* buf, void** data, cb_size_t* size);
cb_size_t Ubuf_Move (ubuf_st* dest, ubuf_st* src);

#endif
