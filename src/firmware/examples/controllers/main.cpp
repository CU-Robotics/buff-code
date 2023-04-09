// #include "unity.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/blink.h"
#include "buff_cpp/loggers.h"
#include "buff_cpp/controllers.h"

int test_rotate2D(void) {
	int errors = 0;
	// test stuff
	float x[2] = {1, 0};
	float xR[2] = {0, 0};
	float y[2] = {0, 1};
	float yR[2] = {0, 0};
	float xy[2] = {2, 1};
	float xyR[2] = {0, 0};
	float yx[2] = {-1, 2};
	float yxR[2] = {0, 0};

	rotate2D(x, xR, PI/2);
	rotate2D(xy, xyR, PI/2);
	rotate2D(y, yR, -PI/2);
	rotate2D(yx, yxR, -PI/2);

	
	errors += floats_eq(xR, y, 2, "failed rotation x->y");
	errors += floats_eq(xyR, yx, 2, "failed rotation xy->yx");
	errors += floats_eq(yR, x, 2, "failed rotation y->x");
	errors += floats_eq(yxR, xy, 2, "failed rotation yx->xy");

	return errors;
}

int test_vector_product(void) {
	int errors = 0;

	float x[2] = {1, 0};
	float y[2] = {0, 1};
	float xy[2] = {2, 1};
	float yx[2] = {-1, 2};

	errors += float_eq(vector_product(x, y, 2), 0, "failed vector product x^T y");
	errors += float_eq(vector_product(xy, yx, 2), 0, "failed vector product xy^T yx");
	errors += float_eq(vector_product(x, x, 2), 1, "failed vector product x^T x");
	errors += float_eq(vector_product(y, y, 2), 1, "failed vector product y^T y");
	errors += float_eq(vector_product(xy, xy, 2), 5, "failed vector product xy^T xy");

	return errors;
}

int test_cross_product2D(void) {
	int errors = 0;

	float x[2] = {1, 0};
	float y[2] = {0, 1};
	float xy[2] = {2, 1};
	float yx[2] = {-1, 2};

	errors += float_eq(cross_product2D(x, y), 1, "failed cross product x * y");
	errors += float_eq(cross_product2D(xy, yx), 5, "failed cross product xy * yx");
	errors += float_eq(cross_product2D(x, x), 0, "failed cross product x * x");
	errors += float_eq(cross_product2D(y, y), 0, "failed cross product y * y");
	errors += float_eq(cross_product2D(xy, xy), 0, "failed cross product xy * xy");

	return errors;
}

int test_weighted_vector_addition(void) {
	int errors = 0;

	float x[2] = {1, 0};
	float xR[2] = {0, 0};
	float y[2] = {0, 1};
	float yR[2] = {0, 0};
	float xyR[2] = {0, 0};
	float yxR[2] = {0, 0};

	weighted_vector_addition(x, x, 1, 1, 2, xR);
	weighted_vector_addition(y, y, 1, 1, 2, yR);
	weighted_vector_addition(x, y, 2, 1, 2, xyR);
	weighted_vector_addition(x, y, 1, -2, 2, yxR);

	for (int i = 0; i < 2; i++) {
		errors += float_eq(xR[i], 2 * x[i], "failed weighted addition x + x");
		errors += float_eq(yR[i], 2 * y[i], "failed weighted addition y + y");
		errors += float_eq(xyR[i], (2 * x[i]) + y[i], "failed weighted addition 2x + y");
		errors += float_eq(yxR[i], x[i] - (2 * y[i]), "failed weighted addition x - 2y");
	}

	return errors;
}


int test_feedback_controller(void) {
	int errors = 0;
	float reference[2] = {1, 0};
	float feedback[2] = {0, 0};
	Feedback_Controller c;

	// kinda just for safety
	errors += float_eq(c.step(reference, feedback), 0, "Uninitialized controller with non-zero input, shutdown robot");

	float gains[3] = {1, 0, 0};
	float limits[4] = {0, 0, 0, 0};

	c.init(gains, limits);

	float ref[2] = {1e8, 1e8};
	c.bound_reference(ref);
	errors += float_eq(ref[0], 1e8, "Failed unlimited reference check");
	errors += float_eq(ref[1], 1e8, "Failed unlimited reference check");

	limits[0] = -1;
	limits[1] = 1;
	c.init(gains, limits);

	ref[1] = 5;
	c.bound_reference(ref);
	errors += float_eq(ref[0], 2 * PI, "Failed roll over limited Reference check (positive)");
	errors += float_eq(ref[1], 0, "Failed roll over limited Reference check (positive)");

	ref[0] = -1e8;
	ref[1] = -1e8;
	c.bound_reference(ref);
	errors += float_eq(ref[0], -2 * PI, "Failed roll over limited Reference check (negative)");
	errors += float_eq(ref[1], 0, "Failed roll over limited Reference check (negative)");

	limits[0] = 0;
	limits[1] = 0;
	limits[2] = -1;
	limits[3] = 1;
	c.init(gains, limits);


	ref[0] = 1e8;
	ref[1] = 5;
	c.bound_reference(ref);
	errors += float_eq(ref[0], 1, "Failed angle limited Reference check (positive)");
	errors += float_eq(ref[1], 0, "Failed angle limited Reference check (positive)");

	ref[0] = -1e8;
	ref[1] = -1e8;
	c.bound_reference(ref);
	errors += float_eq(ref[0], -1, "Failed angle limited Reference check (negative)");
	errors += float_eq(ref[1], 0, "Failed angle limited Reference check (negative)");

	return errors;
}

int test_controller_manager(void) {
	int errors = 0;

	Controller_Manager cm;

	float gains[3] = {1, 0, 0};
	float limits[4] = {0, 0, 0, 0};
	float limits_pend[4] = {-1/2, 1/2, 0, 0};

	for (int i = 0; i < MAX_NUM_RM_MOTORS - 1; i++) {
		cm.init_controller(i, 0, gains, limits);
		cm.step()
	}
	cm.init_controller(MAX_NUM_RM_MOTORS - 1, 1, gains, limits_pend);




	return errors;
}


int runTests(void) {
	int errs = test_rotate2D();
	errs += test_vector_product();
	errs += test_cross_product2D();
	errs += test_weighted_vector_addition();
	errs += test_feedback_controller();
	errs += test_controller_manager();
	return errs;

	// UNITY_BEGIN();
	// RUN_TEST(test_setup_blink);
	// RUN_TEST(test_blink);
	// return UNITY_END();
}


int main() {
	// Wait ~2 seconds before the Unity test runner
	// establishes connection with a board Serial interface
	while (!Serial) {};
	Serial.println("Start control tests");
	delay(2000);

	int errors = runTests();

	Serial.println("Finished tests");
	Serial.printf("%i failed\n", errors);

	return 0;
}