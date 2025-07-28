#pragma once
#include "parameters/parameters.hpp"
#include "utils/InputField.hpp"
#include "telemetry/Communication.h"
#include "telemetry/Data.h"

void sinusoidalSpeed();
void selectSimCar();
void selectRealCar();
void selectStartSimulation();
void selectNeutralSimulation();
void selectStopSimulation();
void selectEnabled();
void selectDisabled();
void updateValueOnFieldChange(double& valueToUpdate, InputField inputField);
void emergencyStop(class Communication &communication, struct TelemetryData &telemetryData);
void addTab(const char tabName[], Page pageToSelect, std::vector<Button>& tabsVector);
void displayPages();
void displayOnlyMenu();