#include "data_generator_options.h"

DataGeneratorOptions DataGeneratorOptions::legacyDefaults()
{
    DataGeneratorOptions o;
    o.num_cam = 1;
    o.camera_ids = {0};
    o.ric[0] << 0, 0, -1, -1, 0, 0, 0, 1, 0;
    o.tic[0] << 0.0, 0.2, 0.6;
    return o;
}
