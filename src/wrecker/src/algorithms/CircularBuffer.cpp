#include "CircularBuffer.h"

CircularBuffer::CircularBuffer(int n){
	max_size = max_size;
	head = 0;
	tail = 0;
}


int CircularBuffer::num_items(){
	int length = tail - head;
	if (length < 0)
		length = max_size;

	return length;
}

float CircularBuffer::get(int idx){
	idx += head;

	if (idx > max_size)
		idx -= tail;

	return buff[idx];
}

float* CircularBuffer::get(int start, int stop){
	int length = num_items();
	float temp[length];

	for (int i = 0; i < stop - start; i++)
		temp[i] = get(i);

	return temp;
}

int CircularBuffer::push(float value){
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

void CircularBuffer::reset(){
	for (int i = 0; i < max_size; i++)
		push(0.0);

	head = 0;
	tail = 0;
}

float CircularBuffer::mean(){
	float sum = 0.0;
	for (int i = 0; i < num_items(); i++)
		sum += get(i);

	return sum / max_size;
}


// float CircularBuffer::median():
// 	int length = size();

//     if length % 2 == 1:
//         return quickselect(get(0,length), int(length / 2));
//     else:
//         return 0.5 * (quickselect(get(0,length), int(length / 2) - 1) +
//                       quickselect(get(0,length), int(length / 2)));


// void CircularBuffer::quickselect(l, length):
//     """
//     Select the kth element in l (0 based)
//     :param l: List of numerics
//     :param k: Index
//     :param pivot_fn: Function to choose a pivot, defaults to random.choice
//     :return: The kth element of l
//     """
//     if length == 1:
//         assert k == 0
//         return l[0]

//     pivot = random(length)

//     for (int i = 0; i < length)
//     lows = [el for el in l if el < pivot]
//     highs = [el for el in l if el > pivot]
//     pivots = [el for el in l if el == pivot]

//     if k < len(lows):
//         return quickselect(lows, k, pivot_fn)
//     elif k < len(lows) + len(pivots):
//         # We got lucky and guessed the median
//         return pivots[0]
//     else:
//         return quickselect(highs, k - len(lows) - len(pivots), pivot_fn)