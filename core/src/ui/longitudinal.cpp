#include <string>
#include "telemetry/Communication.h"
#include "telemetry/Data.h"
#include "pid/pid.h"
#include "parameters/parameters.hpp"
#include "utils/Button.hpp"
#include "utils/InputField.hpp"
#include "utils/CheckBox.hpp"
#include "utils/utils.hpp"
#include "trajectory/trajectory.h"
#include "map/spline_cones.hpp"
#include "raylib.h"
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>

void longitudinal(class Communication &communication, struct TelemetryData &telemetryData, std::string &vehicleId, 
  SimulationType &mode, enum State &currentState, PID &pid) {
  ClearBackground(BLACK);
  
  const int HORIZONTAL_PAGE_PADDING = 50;
  const int VERTICAL_PAGE_PADDING = 20;
  const int CAPTION_FONT_SIZE = 30;
  const int LABEL_FONT_SIZE = 20;
  const int SPACING = 20;
  const int BUTTON_FONT_SIZE = 20;
  const int BUTTON_PADDING = 10;
  const int LABEL_WIDTH = 250;

  // Title
  Rectangle captionBounds = { 
      HORIZONTAL_PAGE_PADDING, 
      TAB_FONT_SIZE + TAB_PADDING*2 + VERTICAL_PAGE_PADDING, 
      MeasureText("LONGITUDINAL CONTROL", CAPTION_FONT_SIZE), 
      CAPTION_FONT_SIZE 
  };
  DrawText("LONGITUDINAL CONTROL", captionBounds.x, captionBounds.y, captionBounds.height, WHITE);
  DrawLine(HORIZONTAL_PAGE_PADDING, calcBottom(captionBounds)+SPACING/2, 
           SCREEN_WIDTH-HORIZONTAL_PAGE_PADDING, calcBottom(captionBounds)+SPACING/2, WHITE);

  // Input fields with labels
  static bool inputsInitializedlong = false;
  static InputField* Kp_min_field;
  static InputField* Kp_max_field;
  static InputField* Ki_min_field;
  static InputField* Ki_max_field;
  static InputField* Kd_min_field;
  static InputField* Kd_max_field;
  static InputField* DesiredSpeed_field;
  static InputField* TimeDuration_field;
  static InputField* CycleNumber_field;
  static InputField* throttle_power_field;
  static InputField* brake_power_field;
  static InputField* sin_field;
  static InputField* amplitude_field;
  static InputField* frequency_field;
  static CheckBox* sinusoidalSpeedCheckbox;
  static CheckBox* uselongitudinalCheckbox;
  static CheckBox* usegamepadCheckbox;
  static CheckBox* usetrajectoryCheckbox;

  int startY = calcBottom(captionBounds) + SPACING;

  if (!inputsInitializedlong) {
    // Initialize all fields first
    Kp_min_field = new InputField(
      HORIZONTAL_PAGE_PADDING + LABEL_WIDTH,
      startY,
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      DARKGRAY,
      WHITE
    );
    Kp_min_field->setPlaceholder("Kp min");
    Kp_min_field->setPlaceholderColor(GRAY);
    Kp_min_field->setText(TextFormat("%.2f", pidAt01kmh.Kp));
    Kp_min_field->onChange([](){
      updateValueOnFieldChange(pidAt01kmh.Kp, *Kp_min_field);
    });

    Kp_max_field = new InputField(
      HORIZONTAL_PAGE_PADDING + LABEL_WIDTH + Kp_min_field->getWidth() + SPACING,
      startY,
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      DARKGRAY,
      WHITE
    );
    Kp_max_field->setPlaceholder("Kp max");
    Kp_max_field->setPlaceholderColor(GRAY);
    Kp_max_field->setText(TextFormat("%.2f", pidAt100kmh.Kp));
    Kp_max_field->onChange([](){
      updateValueOnFieldChange(pidAt100kmh.Kp, *Kp_max_field);
    });

    Ki_min_field = new InputField(
      HORIZONTAL_PAGE_PADDING + LABEL_WIDTH,
      startY + (BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      DARKGRAY,
      WHITE
    );
    Ki_min_field->setPlaceholder("Ki min");
    Ki_min_field->setPlaceholderColor(GRAY);
    Ki_min_field->setText(TextFormat("%.2f", pidAt01kmh.Ki));
    Ki_min_field->onChange([](){
      updateValueOnFieldChange(pidAt01kmh.Ki, *Ki_min_field);
    });

    Ki_max_field = new InputField(
      Ki_min_field->getRight() + SPACING,
      startY + (BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      DARKGRAY,
      WHITE
    );
    Ki_max_field->setPlaceholder("Ki max");
    Ki_max_field->setPlaceholderColor(GRAY);
    Ki_max_field->setText(TextFormat("%.2f", pidAt100kmh.Ki));
    Ki_max_field->onChange([](){
      updateValueOnFieldChange(pidAt100kmh.Ki, *Ki_max_field);
    });
              
    Kd_min_field = new InputField(
      HORIZONTAL_PAGE_PADDING + LABEL_WIDTH,
      startY + 2*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      DARKGRAY,
      WHITE
    );
    Kd_min_field->setPlaceholder("Kd min");
    Kd_min_field->setPlaceholderColor(GRAY);
    Kd_min_field->setText(TextFormat("%.2f", pidAt01kmh.Kd));
    Kd_min_field->onChange([](){
      updateValueOnFieldChange(pidAt01kmh.Kd, *Kd_min_field);
    });

    Kd_max_field = new InputField(
      Kd_min_field->getRight() + SPACING,
      startY + 2*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      DARKGRAY,
      WHITE
    );
    Kd_max_field->setPlaceholder("Kd max");
    Kd_max_field->setPlaceholderColor(GRAY);
    Kd_max_field->setText(TextFormat("%.2f", pidAt100kmh.Kd));
    Kd_max_field->onChange([](){
      updateValueOnFieldChange(pidAt100kmh.Kd, *Kd_max_field);
    });

    DesiredSpeed_field = new InputField(
      HORIZONTAL_PAGE_PADDING + LABEL_WIDTH,
      startY + 3*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      DARKGRAY,
      WHITE
    );
    DesiredSpeed_field->setPlaceholder("m/s");
    DesiredSpeed_field->setPlaceholderColor(GRAY);
    DesiredSpeed_field->setText(TextFormat("%.2f", desiredSpeed));
    DesiredSpeed_field->onChange([](){
      updateValueOnFieldChange(desiredSpeed, *DesiredSpeed_field);
    });
        
    TimeDuration_field = new InputField(
      HORIZONTAL_PAGE_PADDING + LABEL_WIDTH,
      startY + 4*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      DARKGRAY,
      WHITE
    );
    TimeDuration_field->setPlaceholder("seconds");
    TimeDuration_field->setPlaceholderColor(GRAY);
    TimeDuration_field->setText(TextFormat("%.2f", timeDuration));
    TimeDuration_field->onChange([](){
      updateValueOnFieldChange(timeDuration, *TimeDuration_field);
    });

    CycleNumber_field = new InputField(
      HORIZONTAL_PAGE_PADDING + LABEL_WIDTH,
      startY + 5*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      DARKGRAY,
      WHITE
    );
    CycleNumber_field->setPlaceholder("min 1");
    CycleNumber_field->setPlaceholderColor(GRAY);
    CycleNumber_field->setText(TextFormat("%.0f", cycleNumber));
    CycleNumber_field->onChange([](){
      updateValueOnFieldChange(cycleNumber, *CycleNumber_field);
    });

    throttle_power_field = new InputField(
      Kp_max_field->getRight() + SPACING * 2,
      Kp_max_field->getTop(),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      DARKGRAY,
      WHITE
    );
    throttle_power_field->setPlaceholder("Throttle power");
    throttle_power_field->setPlaceholderColor(GRAY);
    throttle_power_field->setText(TextFormat("%.2f", throttle_power));
    throttle_power_field->onChange([](){
      updateValueOnFieldChange(throttle_power, *throttle_power_field);
    });

    brake_power_field = new InputField(
      Ki_max_field->getRight() + SPACING * 2,
      Ki_max_field->getTop(),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      DARKGRAY,
      WHITE
    );
    brake_power_field->setPlaceholder("Brake power");
    brake_power_field->setPlaceholderColor(GRAY);
    brake_power_field->setText(TextFormat("%.2f", brake_power));
    brake_power_field->onChange([](){
      updateValueOnFieldChange(brake_power, *brake_power_field);
    });

    sin_field = new InputField(
      HORIZONTAL_PAGE_PADDING + LABEL_WIDTH,
      startY + 6.5*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      DARKGRAY,
      WHITE
    );
    sin_field->setPlaceholder("sinusoidal speed");
    sin_field->setPlaceholderColor(GRAY);
    sin_field->setText(TextFormat("%.2f", sindt));
    sin_field->onChange([](){
      updateValueOnFieldChange(sindt, *sin_field);
    });

    amplitude_field = new InputField(
      sin_field->getRight() + SPACING,
      startY + 6.5*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      DARKGRAY,
      WHITE
    );
    amplitude_field->setPlaceholder("amplitude");
    amplitude_field->setPlaceholderColor(GRAY);
    amplitude_field->setText(TextFormat("%.2f", amplitude));
    amplitude_field->onChange([](){
      updateValueOnFieldChange(amplitude, *amplitude_field);
    });

    frequency_field = new InputField(
      sin_field->getRight() + SPACING,
      startY + 7.5*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      DARKGRAY,
      WHITE
    );
    frequency_field->setPlaceholder("frequency");
    frequency_field->setPlaceholderColor(GRAY);
    frequency_field->setText(TextFormat("%.2f", frequency));
    frequency_field->onChange([](){
      updateValueOnFieldChange(frequency, *frequency_field);
    });

    sinusoidalSpeedCheckbox = new CheckBox(
      HORIZONTAL_PAGE_PADDING,
      startY + 6*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      useSinusoidalSpeed ? GREEN : RED,
      "Use Sinusoidal Speed",
      BLACK
    );
    sinusoidalSpeedCheckbox->onToggle([](bool checked) {
        useSinusoidalSpeed = checked;});
    sinusoidalSpeedCheckbox->isChecked();

    uselongitudinalCheckbox = new CheckBox(
      brake_power_field->getRight() + SPACING * 7,
      startY + 9*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      useLongitudinal ? GREEN : RED,
      "Use Longitudinal",
      BLACK
    );
    uselongitudinalCheckbox->onToggle([](bool checked) {
        useLongitudinal = checked;});
    uselongitudinalCheckbox->isChecked();

    flagforstop = useGamepad;
    usegamepadCheckbox = new CheckBox(
      brake_power_field->getRight() + SPACING * 7,
      startY + 10*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING),
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      useGamepad ? GREEN : RED,
      "Use Gamepad",
      BLACK
    );
    usegamepadCheckbox->onToggle([](bool checked) {
        useGamepad = checked;});
    usegamepadCheckbox->isChecked();
    
    usetrajectoryCheckbox = new CheckBox(
      SCREEN_WIDTH - HORIZONTAL_PAGE_PADDING - 300,
      startY + SPACING * 2.5,
      BUTTON_FONT_SIZE,
      BUTTON_PADDING,
      useTrajectory ? GREEN : RED,
      "Use Trajectory",
      BLACK
    );
    usetrajectoryCheckbox->onToggle([](bool checked) {
      useTrajectory = checked;});
    usetrajectoryCheckbox->isChecked();
      
    if (!useGamepad && flagforstop) {
      pidisActivated = false;
    }

    inputsInitializedlong = true;
  }

  // Draw labels and fields
  if (Kp_min_field && Kp_max_field) {
    DrawText("Kp ->", HORIZONTAL_PAGE_PADDING, startY, LABEL_FONT_SIZE, WHITE);
    Kp_min_field->draw();
    Kp_max_field->draw();
  }
  
  if (Ki_min_field && Ki_max_field) {
    DrawText("Ki ->:", HORIZONTAL_PAGE_PADDING, startY + (BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING), 
             LABEL_FONT_SIZE, WHITE);
    Ki_min_field->draw();
    Ki_max_field->draw();
  }
  
  if (Kd_min_field && Kd_max_field) {
    DrawText("Kd ->", HORIZONTAL_PAGE_PADDING, startY + 2*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING), 
             LABEL_FONT_SIZE, WHITE);
    Kd_min_field->draw();
    Kd_max_field->draw();
  }
  
  if (DesiredSpeed_field) {
    DrawText("Desired speed (m/s) ->", HORIZONTAL_PAGE_PADDING, 
             startY + 3*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING), LABEL_FONT_SIZE, WHITE);
    DesiredSpeed_field->draw();
  }
  
  if (TimeDuration_field) {
    DrawText("Time duration (sec) ->", HORIZONTAL_PAGE_PADDING, 
             startY + 4*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING), LABEL_FONT_SIZE, WHITE);
    TimeDuration_field->draw();
  }
  
  if (CycleNumber_field) {
    DrawText("Cycle number ->", HORIZONTAL_PAGE_PADDING, 
             startY + 5*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING), LABEL_FONT_SIZE, WHITE);
    CycleNumber_field->draw();
  }

  if (throttle_power_field) {
    DrawText("<- Throttle power ", throttle_power_field->getRight() + SPACING, 
             throttle_power_field->getTop(), LABEL_FONT_SIZE, WHITE);
    throttle_power_field->draw();
  }
  if (brake_power_field) {
    DrawText("<- Brake power ", brake_power_field->getRight() + SPACING, 
             brake_power_field->getTop(), LABEL_FONT_SIZE, WHITE);
    brake_power_field->draw();
  }
  if (sin_field) {
    DrawText("delta of sin ->", HORIZONTAL_PAGE_PADDING, 
             startY + 6.5*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING), LABEL_FONT_SIZE, WHITE);
    sin_field->draw();
  }
  if (amplitude_field) {
    DrawText("<- amplitude", amplitude_field->getRight() + SPACING, 
            startY + 6.5*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING), LABEL_FONT_SIZE, WHITE);
    amplitude_field->draw();
  }

  if (frequency_field) {
    DrawText("<- frequency", frequency_field->getRight() + SPACING, 
            startY + 7.5*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING), LABEL_FONT_SIZE, WHITE);
    frequency_field->draw();
  }

  sinusoidalSpeedCheckbox->draw();
  uselongitudinalCheckbox->draw();
  usegamepadCheckbox->draw();
  usetrajectoryCheckbox->draw();

  // Start Button
  Button startButton(
    HORIZONTAL_PAGE_PADDING,
    sin_field->getBottom() + SPACING * 4,
    BUTTON_FONT_SIZE,
    BUTTON_PADDING,
    GREEN,
    "START",
    BLACK,
    [&communication, &telemetryData, &mode, &currentState, &pid]() { 
      if (commandsStatus == ENABLED && !useGamepad && useLongitudinal) {
        selectStartSimulation();
        issimulationrunning = true;
        numberofcyclesdone = 0;
        pid.reset();
        currentState = ACCELERATING;
        if (mode == SimulationType::SIMULATOR) {
          communication.setInitialState();
        }
      }
    }   
  );

  // Neutral Button -> initial condition
  Button neutralButton(
    startButton.getRight() + SPACING * 2,
    sin_field->getBottom() + SPACING * 4,
    BUTTON_FONT_SIZE,
    BUTTON_PADDING,
    BLUE,
    "NEUTRAL",
    BLACK,
    [&communication, &mode, &currentState]() {
      if (mode == SimulationType::SIMULATOR) {
        communication.setInitialState();
      }
      visualizepid = 0.0;
      numberofcyclesdone = 0;
      communication.sendVehicleCommands(0.0, 0);
      issimulationrunning = false;
      currentState = STOPPED;
      selectNeutralSimulation();
    }   
  );

  // Stop Button
  Button stopButton(
    neutralButton.getRight() + SPACING * 2,
    sin_field->getBottom() + SPACING * 4,
    BUTTON_FONT_SIZE,
    BUTTON_PADDING,
    RED,
    "STOP",
    BLACK,
    [&communication, &telemetryData]() { 
      if (commandsStatus == ENABLED) {
        selectStopSimulation();
        // emergencyStop(communication, telemetryData);
        issimulationrunning = false;
        emergencystop = true;
      }   
    }
  );

  // Enable Commands Button
  Button enableCommandsButton(
    stopButton.getRight() + SPACING * 2,
    sin_field->getBottom() + SPACING * 4,
    BUTTON_FONT_SIZE,
    BUTTON_PADDING,
    YELLOW,
    "ENABLE COMMANDS",
    BLACK,
    [&communication]() { 
      // Enable steering, throttle, and brakes
      Serializers::Data::ASStatus setStatus;
      setStatus.steerStatus = Serializers::Data::Status::status_enabled;
      setStatus.throttleStatus = Serializers::Data::Status::status_enabled;
      setStatus.brakesStatus = Serializers::Data::Status::status_enabled;
      
      // std::cout << "Sending setStaus enabled\n";
      communication.sendStatus(setStatus);
      std::cout << "Enabled setStaus\n";
      selectEnabled();
    }   
  );

  // Disable Commands Button
  Button disableCommandsButton(
    stopButton.getRight() + SPACING * 2,
    enableCommandsButton.getBottom() + SPACING,
    BUTTON_FONT_SIZE,
    BUTTON_PADDING,
    YELLOW,
    "DISABLE COMMANDS",
    BLACK,
    [&communication]() { 
      // Disable steering, throttle, and brakes
      Serializers::Data::ASStatus setStatus;
      setStatus.steerStatus = Serializers::Data::Status::status_disabled;
      setStatus.throttleStatus = Serializers::Data::Status::status_disabled;
      setStatus.brakesStatus = Serializers::Data::Status::status_disabled;
      
      // std::cout << "Sending setStaus disabled\n";
      communication.sendStatus(setStatus);
      std::cout << "Disabled setStaus\n";
      selectStopSimulation();
      selectDisabled();
      issimulationrunning = false;
    }   
  );

  // Generate centerline
  // Generate centerline button aligned to the right of the page
  Button generateCenterlineButton(
    SCREEN_WIDTH - HORIZONTAL_PAGE_PADDING - 300,
    startY,
    BUTTON_FONT_SIZE,
    BUTTON_PADDING,
    YELLOW,
    "GENERATE CENTERLINE",
    BLACK,
    []() { 
      LoadConesFromCsv(cones_csv, left_cones, right_cones);
      // centerline.clear();
      // centerline = CreateCenterlineFromCones(left_cones, right_cones, centerline);
      // CalculateVelocityProfile(centerline);
      // std::cout << "Centerline generated with " << centerline.size() << " points." << std::endl;

      // ClothoidList
      CreateClothoidListFromCones(left_cones, right_cones);
      CalculateVelocityProfileFromClothoidList();
      
      buildSplineFromCones_l(left_cones);
      buildSplineFromCones_r(right_cones);


    }
  );

  vector<Button> simulationButtons = {startButton, neutralButton, stopButton, enableCommandsButton, disableCommandsButton, generateCenterlineButton};
  for (Button& button : simulationButtons) {
    button.setHoverColor(WHITE);
    button.setHoverTextColor(BLACK);
    button.draw();
  }

  const int simulationBorderOffsetPx = 5;
  const int simulationBorderThickness = 2;
  switch (currentSimulation) {
    case START:
      startButton.drawBorder(simulationBorderOffsetPx, YELLOW, simulationBorderThickness);
      break;
    case NEUTRAL:
      neutralButton.drawBorder(simulationBorderOffsetPx, YELLOW, simulationBorderThickness);
      break;
    case STOP:
      stopButton.drawBorder(simulationBorderOffsetPx, YELLOW, simulationBorderThickness);
      break;
  }

  const int controllersStatusBorderOffsetPx = 5;
  const int controllersStatusBorderThickness = 2;
  switch (commandsStatus) {
    case ENABLED:
      enableCommandsButton.drawBorder(controllersStatusBorderOffsetPx, RED, controllersStatusBorderThickness);
      break;
    case DISABLED:
      disableCommandsButton.drawBorder(controllersStatusBorderOffsetPx, RED, controllersStatusBorderThickness);
      break;
  }
  
  DrawText("LONGITUDINAL CONTROL SETTINGS", 10, startButton.getBottom() + SPACING * 11, 20, WHITE);
  DrawText(("Connectet to " + host_number).c_str(), 10, startButton.getBottom() + SPACING * 12, 15, WHITE);
  DrawText(("VehicleId " + vehicleId).c_str(), 10, startButton.getBottom() + SPACING * 13, 15, WHITE);
  std::string modeStr = (mode == SimulationType::ONBOARD) ? "ONBOARD" : "SIMULATOR";
  DrawText(("Mode is " + modeStr).c_str(), 10, startButton.getBottom() + SPACING * 14, 15, WHITE);

  // Display current speed
  DrawText(
    TextFormat("Current Speed: %.2f m/s", telemetryData.vehicleState.u),
    HORIZONTAL_PAGE_PADDING,
    startButton.getBottom() + SPACING,
    LABEL_FONT_SIZE,
    WHITE
  );

  // Display PID output
  DrawText(
    TextFormat("PID Output: %.2f", visualizepid),
    HORIZONTAL_PAGE_PADDING,
    startButton.getBottom() + SPACING * 2,
    LABEL_FONT_SIZE,
    WHITE
  );

  // Display numberofcyclesdone
  DrawText(
    TextFormat("Number of cycles done: %d", numberofcyclesdone),
    HORIZONTAL_PAGE_PADDING,
    startButton.getBottom() + SPACING * 3,
    LABEL_FONT_SIZE,
    WHITE
  );

  // Display sinusoidal checkbox status
  DrawText(
    TextFormat("Sinusoidal speed: %s", useSinusoidalSpeed ? "Enabled" : "Disabled"),
    HORIZONTAL_PAGE_PADDING + SPACING,
    startY + 6*(BUTTON_FONT_SIZE + BUTTON_PADDING + SPACING),
    LABEL_FONT_SIZE,
    WHITE
  );

  // Display sinusoidal speed

  DrawText(
    TextFormat("Current desired speed: %.2f m/s", currentDesiredSpeed),
    HORIZONTAL_PAGE_PADDING,
    startButton.getBottom() + SPACING * 4,
    LABEL_FONT_SIZE,
    WHITE
  );

  // Display longitudinal checkbox status
  DrawText(
    TextFormat("Longitudinal control: %s", useLongitudinal ? "Enabled" : "Disabled"),
    uselongitudinalCheckbox->getLeft() + SPACING,
    uselongitudinalCheckbox->getTop(),
    LABEL_FONT_SIZE,
    WHITE
  );

  // Display gamepad checkbox status
  DrawText(
    TextFormat("GamePad control: %s", useGamepad ? "Enabled" : "Disabled"),
    usegamepadCheckbox->getLeft() + SPACING,
    usegamepadCheckbox->getTop(),
    LABEL_FONT_SIZE,
    WHITE
  );

  // Display trajectory checkbox status
  DrawText(
    TextFormat("Trajectory control: %s", useTrajectory ? "Enabled" : "Disabled"),
    usetrajectoryCheckbox->getLeft() + SPACING,
    usetrajectoryCheckbox->getTop(),
    LABEL_FONT_SIZE,
    WHITE
  );
  
}