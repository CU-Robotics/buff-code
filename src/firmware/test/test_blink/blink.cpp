#include "unity.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/blink.h"

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void test_setup_blink(void) {
  // test stuff
  setup_blink();
}

void test_blink(void) {
  // more test stuff
  timer_set(0);
  blink();
  bool init_status = blinker_status;
  while (init_status == blinker_status) {
    blink();
  }
  // blinker should be precise to 0 microseconds
  TEST_ASSERT_EQUAL_INT32(BLINK_RATE_US, timer_info_us(0));
}

int runUnityTests(void) {
  UNITY_BEGIN();
  RUN_TEST(test_setup_blink);
  RUN_TEST(test_blink);
  return UNITY_END();
}


/**
  * For Arduino framework
  */
void setup() {
  // Wait ~2 seconds before the Unity test runner
  // establishes connection with a board Serial interface
  delay(2000);

  runUnityTests();
}
void loop() {}