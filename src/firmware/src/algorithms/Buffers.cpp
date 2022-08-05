#include "Buffers.h"

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


Buffer3::Buffer3(){
	seek_ptr = 0;
	max_size = 10;

	buff = new Vector3[max_size];
}

Buffer3::Buffer3(int n){
	seek_ptr = 0;
	max_size = n;

	buff = new Vector3[max_size];
}

void Buffer3::incr_seek(){
	seek_ptr ++;

	if (seek_ptr > max_size - 1)
		seek_ptr = 0;
}

Vector3 Buffer3::get(int idx){

	idx = seek_ptr - idx;

	if (idx < 0)
		idx += max_size;

	return buff[idx];
}

int Buffer3::push(float x, float y, float z){

	
	Vector3 xyz = {x,y,z};
    buff[seek_ptr] = xyz;
    incr_seek();
    return seek_ptr;
}

void Buffer3::reset(){
	for (int i = 0; i < max_size; i++)
		push(0.0, 0.0, 0.0);

	seek_ptr = 0;
}

Vector3 Buffer3::mean(){
	Vector3 sum;
	for (int i = 0; i < max_size; i++){
		Vector3 v = get(i);
		sum.x += v.x;
		sum.y += v.y;
		sum.z += v.z;
	}

	sum.x /= max_size;
	sum.y /= max_size;
	sum.z /= max_size;
	return sum;
}

// int Buffer3::size() {
// 	return num_items;
// }

// void Buffer3D::print_buff()
// {
// 	for (int i = 0; i < num_items; i++) {
// 		Serial.print(get(i));
// 		Serial.print(" // ");
// 	}

// 	Serial.print(num_items);
// 	Serial.print(" // ");
// 	Serial.print(head);
// 	Serial.print(" // ");
// 	Serial.print(tail);
// 	Serial.print(" // ");
// 	Serial.println(max_size);
// }

// float Buffer3D::median():
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


	HIDBuffer::HIDBuffer() {
		update_t = micros();
		seek_ptr = 0;
		length = 64;
	}

	void HIDBuffer::reset(){
		seek_ptr = 0;
		while (put(0)) {}
	}

	void HIDBuffer::incr_seek(){
		if (seek_ptr == length - 1){
			seek_ptr = 0;
		}
		else {
			seek_ptr += 1;
		}
	}

	int HIDBuffer::put(byte b){
		data[seek_ptr] = b;
		incr_seek();
		return seek_ptr;
	}

	int HIDBuffer::put_u16(uint16_t value){
		if (length - seek_ptr < 2){
			return seek_ptr;
		}

		put(byte((value & 0xFF00) >> 8));
		return put(byte(value & 0x00FF));
	}

	int HIDBuffer::put_f32(float value){
		if (length - seek_ptr < 4) {
			return seek_ptr;
		}

		FLOATUNION_t fu;
		fu.number = value;

		put(fu.bytes[0]);
		put(fu.bytes[1]);
		put(fu.bytes[2]);
		return put(fu.bytes[3]);
	}

	byte HIDBuffer::seek(int idx){
		seek_ptr = idx;
		incr_seek();
		return data[idx];
	}

	byte HIDBuffer::seek(){
		int idx = seek_ptr;
		incr_seek();
		return data[idx];
	}

	uint16_t HIDBuffer::seek_u16(){
		return uint16_t(seek()) << 8 | uint16_t(seek());
	}

	float HIDBuffer::seek_f32(){
		FLOATUNION_t fu;
		fu.bytes[0] = seek();
		fu.bytes[1] = seek();
		fu.bytes[2] = seek();
		fu.bytes[3] = seek();

		return fu.number;
	}

	bool HIDBuffer::check_of(int l){
		return length - seek_ptr < l;
	}