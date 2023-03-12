#ifndef ROLLING_AVERAGE_FILTER_H
#define ROLLING_AVERAGE_FILTER_H

#include <vector>

struct RollingAverageFilter {
    float sum = 0.0;
    std::vector<int> samples;
    int max_samples;
    int curr_sample_offset = 0;

    RollingAverageFilter(int max_samples) {
        this->max_samples = max_samples;
        samples.resize(max_samples);
    }

    void addMeasurement(float measurement) {
        sum += measurement;
        sum -= samples[curr_sample_offset];
        samples[curr_sample_offset] = measurement;
        curr_sample_offset++;
        if (curr_sample_offset >= max_samples) curr_sample_offset = 0;
    }

    float getMean() { return sum / (float)(max_samples); };
};

#endif