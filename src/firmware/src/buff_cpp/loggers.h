

#ifndef BUFF_LOGGERS
#define BUFF_LOGGERS


// #define IS_EQ(a, b) (a = b) ? 1 : 0
// #define ASSERT_EQ(a, b, message) IS_EQ(a, b)


// Data types 
// dr16		float[7]
/*	Basic serial print functions */

// template<typename T>
// int assert_eq(T, T , String);

// template<typename T>
// int assert_arr_eq(T*, T*, int, String);

// template<typename T>
// int assert_arr_eq(T, T*, int, String);

int int_eq(int, int, String);
int ints_eq(int, int*, int, String);
int ints_eq(int*, int*, int, String);
int byte_eq(int, int, String);
int bytes_eq(byte, byte*, int, String);
int bytes_eq(byte*, byte*, int, String);
int float_eq(float, float, String);
int floats_eq(float*, float*, int, String);
int float_neq(float, float, String);
int float_nan(float);
int float_leq(float, float, String);
int float_geq(float, float, String);

void timer_print(uint32_t, String);

#endif