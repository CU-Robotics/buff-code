

#ifndef LP_FILTER_H
#define LP_FILTER_H


struct LPFilter {
    float K = 0.4;
    float output = 0;
    float prev_measurement = 0.0;

    float filter(float measurement, float dt) {
        output += K * (measurement - prev_measurement);
        return output;
    }
};

#endif