

#ifndef LP_FILTER_H
#define LP_FILTER_H


class LPFilter {
private:
    float K = 0.4;
    float output = 0;
    float prev_output = 0.0;

public:
    LPFilter() {}

    void set_gain(float k) {
        K = k;
    }

    float filter(float measurement) {
        prev_output = (K * prev_output) + ((1 - K) * measurement);
        return prev_output;
    }
};

#endif