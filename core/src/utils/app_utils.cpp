#include "app_utils.hpp"
#include "parameters/parameters.hpp"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <chrono>
#include "raylib.h"

void sinusoidalSpeed() {
  if (useSinusoidalSpeed) {
    static auto lastUpdate = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUpdate).count();

    if (elapsed >= sindt) {
        lastUpdate = now;
        double timeSec = GetTime();
        double multiplier = 1.0 + amplitude * sin(frequency * timeSec);
        currentDesiredSpeed = desiredSpeed * multiplier;
    }
  } else {
    currentDesiredSpeed = desiredSpeed;
  }
}

void selectSimCar(){
  currentSelectedCar= SIM_CAR;
  mustSwitchCar= true;
}

void selectRealCar(){
  currentSelectedCar = REAL_CAR;
  mustSwitchCar= true;
}

void selectStartSimulation(){
  currentSimulation= START;
}

void selectNeutralSimulation(){
  currentSimulation= NEUTRAL;
}

void selectStopSimulation(){
  currentSimulation= STOP;
}

void selectEnabled(){
  commandsStatus= ENABLED;
}

void selectDisabled(){
  commandsStatus= DISABLED;
}

void updateValueOnFieldChange(double& valueToUpdate, InputField inputField){
  try {
    double updatedValue;
    std::string inputFieldText= inputField.getText();
    if (inputFieldText.empty()) return;

    updatedValue= std::stod(inputFieldText);
    valueToUpdate= updatedValue;
  }
  catch(const std::exception& e) {
    std::cerr << "Invalid input" << std::endl;
    std::cerr << e.what() << '\n';
  }
}

void emergencyStop(class Communication &communication, struct TelemetryData &telemetryData) {
  if (telemetryData.vehicleState.u > 0.1) {
    visualizepid = -100.0;
    communication.sendVehicleCommands(-100.0, 0);
  } else {
    communication.sendVehicleCommands(0.0, 0);
    visualizepid = 0.0;
  }
}

void addTab(const char tabName[], Page pageToSelect, std::vector<Button>& tabsVector){
  int prevRight= tabsVector.size() > 0 ? tabsVector.back().getRight() : 0;

  tabsVector.push_back(
    Button(
      prevRight,
      0,
      TAB_FONT_SIZE,
      TAB_PADDING,
      currentPage==pageToSelect ? WHITE : DARKGRAY,
      tabName,
      currentPage==pageToSelect ? BLACK : WHITE,
      [pageToSelect]() { currentPage=pageToSelect; }
    )
  );
}

void displayPages(){
  DrawRectangle(0, 0, SCREEN_WIDTH, TAB_FONT_SIZE+TAB_PADDING*2, DARKGRAY);

  std::vector<Button> buttons;
  addTab("MENU", MENU, buttons);
  addTab("LONGITUDINAL", LONGITUDINAL, buttons);
  addTab("CIRCUIT", CIRCUIT, buttons);

  for (Button& button: buttons){
    button.setHoverColor(WHITE);
    button.setHoverTextColor(BLACK);
    button.draw();
  }

  DrawLine(0, TAB_FONT_SIZE+TAB_PADDING*2, SCREEN_WIDTH, TAB_FONT_SIZE+TAB_PADDING*2, WHITE);
}

void displayOnlyMenu(){
  DrawRectangle(0, 0, SCREEN_WIDTH, TAB_FONT_SIZE+TAB_PADDING*2, DARKGRAY);

  std::vector<Button> buttons;
  addTab("MENU", MENU, buttons);

  for (Button& button: buttons){
    button.setHoverColor(WHITE);
    button.setHoverTextColor(BLACK);
    button.draw();
  }

  DrawLine(0, TAB_FONT_SIZE+TAB_PADDING*2, SCREEN_WIDTH, TAB_FONT_SIZE+TAB_PADDING*2, WHITE);
}