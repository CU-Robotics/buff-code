#include <Arduino.h>

#ifndef H_BUFF_BUFF_BUFFERS
#define H_BUFF_BUFF_BUFFERS

class CircularBuffer {
public:
	CircularBuffer();

	void init(int size);
	float get(int);					// index [0,size], 0 is most recent item
	float* get(int, int);
	int push(float);
	int size();

	void print_buff();

	void reset();

	float mean();

private:
	int head;
	int tail;
	int max_size;
	int num_items;
	float* buff;
};

struct Vector3 {
	float x = 0;
	float y = 0;
	float z = 0;
};

class Buffer3 {
public:
	Buffer3();
	Buffer3(int n);
	void incr_seek();

	Vector3 get(int);					// index [0,size], 0 is most recent item
	int push(float, float, float);

	void reset();
	Vector3 mean();

private:
	int seek_ptr;
	int max_size;
	Vector3* buff;
};



typedef union
{
  float number;
  byte bytes[4];
} FLOATUNION_t;


class HIDBuffer {
public:
	int seek_ptr = 0;
	byte data[64];

	HIDBuffer();

	void reset();

	int put(byte);
	int put_u16(uint16_t);
	int put_f32(float);

	byte seek();
	byte seek(int);
	uint16_t seek_u16();
	float seek_f32();


	void incr_seek();
	bool check_of(int l);

private:
	unsigned long update_t;
	int length = 64;
};

#endif

