#include <Arduino.h>

#ifndef H_CIRCULARBUFFER
#define H_CIRCULARBUFFER
/* A circular buffer template class implementation, which can store data of any type needed
modified code from source: https://github.com/embeddedartistry/embedded-resources/blob/master/examples/cpp/circular_buffer.cpp
more info: https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/
usage instructions (copied & modified from link above):
    To instantiate a circular buffer, we just declare an object and specify the templated type for our buffer. Here’s an example using a buffer of 10 uint32_t entries:
        CircularBuffer<uint32_t> buffer(10);
    Adding data is easy:
        uint32_t x = 100;
        buffer.put(x);
    And getting data is equally easy:
        x = buffer.pop(); // removes value
		x = buffer.peek(); // does not remove value
	extra feature:
		x = buffer.pad(); // fills with zeros up to BLOCK_SIZE
*/

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

#endif

