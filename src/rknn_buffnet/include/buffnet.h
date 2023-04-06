/*-------------------------------------------
                Includes
-------------------------------------------*/
#include "buff_realsense.h"

// #include "opencv2/core/core.hpp"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/imgproc.hpp"
// #include "rknn_api.h"

// #include <stdint.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <sys/time.h>

// const int MODEL_IN_WIDTH    = 224;
// const int MODEL_IN_HEIGHT   = 224;
// const int MODEL_IN_CHANNELS = 3;

#ifndef BUFFNET_H
#define BUFFNET_H

class Buffnet {
private:

public:
    Buff_RealSense rs;

    
    Buffnet();
    void spin_realsense();
};

#endif