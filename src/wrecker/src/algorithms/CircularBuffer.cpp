#include "CircularBuffer.h"

CircularBuffer::CircularBuffer(int max_size) {
	this->max_size = max_size;
	head = 0;
	tail = 0;
}

int CircularBuffer::size() {
	int length = tail - head;
	if (length < 0)
		length = max_size;

	return length;
}

float CircularBuffer::peek(int idx){
	idx += head;

	if (idx > max_size)
		idx -= tail;

	return buffer[idx];
}

float* CircularBuffer::peek(int start, int stop){
	int length = size();
	float temp[length];

	for (int i = 0; i < stop - start; i++)
		temp[i] = get(i);

	return temp;
}

int CircularBuffer::push(float value){
	head ++;
	
	if (tail <= head)
		tail++;

	else if (head >= size){
		head = 0;
		tail++;
	}

	buffer[head] = value;

	return size();
}

void CircularBuffer::reset(){
	for (int i = 0; i < max_size; i++)
		push(0.0);

	head = 0;
	tail = 0;
}

int CircularBuffer::push(float value){
    if (num_items() != 0)
        head ++;

    if (tail <= head)
        tail++;

    else if (head >= max_size){
        head = 0;
        tail++;
    }

    buff[head] = value;

    return num_items();
}