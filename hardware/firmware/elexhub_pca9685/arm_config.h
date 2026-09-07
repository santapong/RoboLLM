// GENERATED from the selected Elexhub profile. Never edit limits here.
#pragma once
const uint8_t NJOINTS = 5;
const uint8_t NSERVOS = NJOINTS + 1;
const uint8_t CHANNEL[NSERVOS] = {0, 1, 2, 3, 4, 5};
const uint16_t PULSE_MIN[NSERVOS] = {0, 0, 0, 0, 0, 0};
const uint16_t PULSE_MAX[NSERVOS] = {0, 0, 0, 0, 0, 0};
const float MIN_RAW[NSERVOS] = {10.000000f, 10.000000f, 10.000000f, 10.000000f, 10.000000f, 0.000000f};
const float MAX_RAW[NSERVOS] = {170.000000f, 170.000000f, 170.000000f, 170.000000f, 170.000000f, 100.000000f};
const float HOME_RAW[NSERVOS] = {90.000000f, 100.000000f, 140.000000f, 155.000000f, 90.000000f, 0.000000f};
const float MAX_RATE[NSERVOS] = {30.000000f, 30.000000f, 30.000000f, 30.000000f, 30.000000f, 100.000000f};
const float COMMISSION_MIN[NSERVOS] = {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f};
const float COMMISSION_MAX[NSERVOS] = {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f};
const bool ARM_CALIBRATED = false;
const bool COMMISSIONING_ALLOWED = false;
const char PROFILE_HASH[] = "caf74d8f39ce1d4702819ffc";
const uint8_t PCA_ADDRESS = 0x40;
const uint8_t OE_PIN = 22;
const uint16_t TICK_MS = 50;
const uint16_t COMMAND_TIMEOUT_MS = 750;
