#include <Arduino.h>
#include "buff_cpp/loggers.h"

// template<typename T>
// int assert_eq(T a, T b, String message){
// 	if (a != b) {
// 		Serial.println(message);
// 		Serial.printf("%i != %i\n", a, b);
// 		return 1;
// 	}

// 	return 0;
// }

// template<typename T>
// int assert_arr_eq(T* a, T* b, int n, String message){
// 	int e = 0;
// 	for (int i = 0; i < n; i++) {
// 		e += assert_equal(a[i], b[i], message);
// 	}
	
// 	return e;
// }

// template<typename T>
// int assert_arr_eq(T a, T* b, int n, String message){
// 	int e = 0;
// 	for (int i = 0; i < n; i++) {
// 		e += assert_equal(a, b[i], message);
// 	}
	
// 	return e;
// }

int int_eq(int a, int b, String message){
	if (a != b) {
		Serial.print("[BUFF_TEST]\t");
		Serial.println(message);
		Serial.printf("\t%i != %i\n", a, b);
		return 1;
	}

	return 0;
}

int ints_eq(int* a, int* b, int n, String message){
	int e = 0;
	for (int i = 0; i < n; i++) {
		e += int_eq(a[i], b[i], message);
	}
	
	return e;
}

int ints_eq(int a, int* b, int n, String message){
	int e = 0;
	for (int i = 0; i < n; i++) {
		e += int_eq(a, b[i], message);
	}
	
	return e;
}

int byte_eq(byte a, byte b, String message){
	if (a != b) {
		Serial.print("[BUFF_TEST]\t");
		Serial.println(message);
		Serial.printf("\t%i != %i\n", a, b);
		return 1;
	}

	return 0;
}

int bytes_eq(byte* a, byte* b, int n, String message){
	int e = 0;
	for (int i = 0; i < n; i++) {
		e += int_eq(a[i], b[i], message);
	}
	
	return e;
}

int bytes_eq(byte a, byte* b, int n, String message){
	int e = 0;
	for (int i = 0; i < n; i++) {
		e += int_eq(a, b[i], message);
	}
	
	return e;
}

int float_eq(float a, float b, String message){
	if (abs(a - b) > 0.0005) {
		Serial.print("[BUFF_TEST]\t");
		Serial.println(message);
		Serial.printf("\t%f != %f\n", a, b);
		return 1;
	}

	return 0;
}

int floats_eq(float* a, float* b, int n, String message){
	int e = 0;
	for (int i = 0; i < n; i++) {
		e += float_eq(a[i], b[i], message);
	}
	
	return e;
}

int float_neq(float a, float b, String message){
	if (abs(a - b) < 0.0005) {
		Serial.print("[BUFF_TEST]\t");
		Serial.println(message);
		Serial.printf("\t%f != %f\n", a, b);
		return 1;
	}

	return 0;
}

int float_nan(float a){
	if (isnan(a)) {
		Serial.print("[BUFF_TEST]\t");
		Serial.print("Value is ");
		Serial.printf("%f\n", a);
		return 1;
	}

	return 0;
}


int float_leq(float a, float b, String message){
	if (b < a) {
		Serial.print("[BUFF_TEST]\t");
		Serial.println(message);
		Serial.printf("\t%f !<= %f\n", a, b);
		return 1;
	}

	return 0;
}

int float_geq(float a, float b, String message){
	if (a < b) {
		Serial.print("[BUFF_TEST]\t");
		Serial.println(message);
		Serial.printf("\t%f !>= %f\n", a, b);
		return 1;
	}

	return 0;
}


void timer_print(uint32_t time, String message) {
	Serial.print("[BUFF_TIMER]\t");
		Serial.print(message);
		Serial.printf("\t%i\n", time);
}











