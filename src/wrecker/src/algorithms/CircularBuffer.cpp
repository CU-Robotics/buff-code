#include "CircularBuffer.h"

CircularBuffer::CircularBuffer(int max_size){
	size = max_size;
	head = 0;
	tail = 0;
}

int CircularBuffer::size(){
	int length = tail - head;
	if (length < 0){
		length = size;

	return length;
}

float CircularBuffer::get(int idx){
	idx += head;

	if (idx > max_size)
		idx -= tail;

	return buffer[idx];
}

float* CircularBuffer::get(int start, int stop){
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

float CircularBuffer::mean(){
	float sum = 0.0;
	for (int i = 0; i < max_size; i++)
		sum += get(i);

	return sum / max_size;
}

float CircularBuffer::median(){
	int length = size();

	int pivot = random(lenght);




}


import random
void CircularBuffer::median(l, pivot_fn=random.choice):
    if len(l) % 2 == 1:
        return quickselect(l, len(l) // 2, pivot_fn)
    else:
        return 0.5 * (quickselect(l, len(l) / 2 - 1, pivot_fn) +
                      quickselect(l, len(l) / 2, pivot_fn))


def quickselect(l, k, pivot_fn):
    """
    Select the kth element in l (0 based)
    :param l: List of numerics
    :param k: Index
    :param pivot_fn: Function to choose a pivot, defaults to random.choice
    :return: The kth element of l
    """
    if len(l) == 1:
        assert k == 0
        return l[0]

    pivot = pivot_fn(l)

    lows = [el for el in l if el < pivot]
    highs = [el for el in l if el > pivot]
    pivots = [el for el in l if el == pivot]

    if k < len(lows):
        return quickselect(lows, k, pivot_fn)
    elif k < len(lows) + len(pivots):
        # We got lucky and guessed the median
        return pivots[0]
    else:
        return quickselect(highs, k - len(lows) - len(pivots), pivot_fn)