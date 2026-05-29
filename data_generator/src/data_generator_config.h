#pragma once

#include <string>

#include "data_generator_options.h"

/** Default path relative to repository root. */
inline const char *defaultDataGeneratorConfigPath() { return "data_generator/config/data_generator.yaml"; }

/**
 * Build DataGeneratorOptions from already-loaded VinsParameters (ric/tic, num_of_cam, camera_ids).
 * Caller must invoke readParameters() first.
 */
DataGeneratorOptions dataGeneratorOptionsFromVinsParameters();

/**
 * Load fov_deg / num_points / imu_per_img from data_generator/config/data_generator.yaml (top-level keys).
 * Does not call readParameters().
 */
void dataGeneratorLoadConfig(const std::string &config_file, DataGeneratorOptions &options);

/** Load default data_generator.yaml (repo root or build-relative path). */
void dataGeneratorLoadDefaultConfig(DataGeneratorOptions &options);
