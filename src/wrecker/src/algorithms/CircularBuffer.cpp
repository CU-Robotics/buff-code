#include "CircularBuffer.h"

CircularBuffer::CircularBuffer(){

}

void CircularBuffer::init(int n) {
	max_size = n;
	num_items = 0;
	head = 0;
	tail = 0;

	buff = new float[max_size];
}

float CircularBuffer::get(int idx){

	idx = head - idx;

	if (idx < 0)
		idx += max_size;

	return buff[idx];
}

// float* CircularBuffer::get(int start, int stop){
// 	int length = num_items;
// 	float temp[length];

// 	for (int i = 0; i < stop - start; i++)
// 		temp[i] = get(i);

// 	return temp;
// }

int CircularBuffer::push(float value){
    if (num_items > 0){
        head ++;

    	if (head >= max_size)
        	head = 0;

        if (tail >= head)
        	tail++;
        
        	if (tail >= max_size)
        		tail = 0;
	}

	if (num_items < max_size) {
		num_items ++;
	}
    buff[head] = value;

    return num_items;
}

void CircularBuffer::reset(){
	for (int i = 0; i < max_size; i++)
		push(0.0);

	head = 0;
	tail = 0;
	num_items = 0;
}

float CircularBuffer::mean(){
	float sum = 0.0;
	for (int i = 0; i < num_items; i++)
		sum += get(i);

	return sum / max_size;
}

int CircularBuffer::size() {
	return num_items;
}

void CircularBuffer::print_buff()
{
	for (int i = 0; i < num_items; i++) {
		Serial.print(get(i));
		Serial.print(" // ");
	}

	Serial.print(num_items);
	Serial.print(" // ");
	Serial.print(head);
	Serial.print(" // ");
	Serial.print(tail);
	Serial.print(" // ");
	Serial.println(max_size);
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
