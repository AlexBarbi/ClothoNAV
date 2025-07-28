// #include "menu.hpp"
#include <algorithm>
#include <thread>
#include "utils/Button.hpp"
#include "utils/InputField.hpp"
#include "telemetry/Communication.h"
#include "telemetry/Data.h"
#include "parameters/parameters.hpp"
#include "raylib.h"

void menu(std::string &vehicleId, SimulationType &mode, bool &saveandclose) {
    ClearBackground(BLACK);
    const int HORIZONTAL_PAGE_PADDING = 20;
    const int VERTICAL_PAGE_PADDING = 20;
    const int CAPTION_FONT_SIZE = 30;
    const int LABEL_FONT_SIZE = 20;
    const int SPACING = 20;
    const int BUTTON_FONT_SIZE = 20;
    const int BUTTON_PADDING = 10;
    const int INPUT_FIELD_WIDTH = 200;
    const int LABEL_WIDTH = 250;

    // Car selection
    Rectangle carTypeSelectionCaptionBounds = {
        HORIZONTAL_PAGE_PADDING,
        TAB_FONT_SIZE + TAB_PADDING * 2 + VERTICAL_PAGE_PADDING,
        MeasureText("SELECT THE CAR TO USE", CAPTION_FONT_SIZE),
        CAPTION_FONT_SIZE
    };
    DrawText("SELECT THE CAR TO USE", carTypeSelectionCaptionBounds.x, carTypeSelectionCaptionBounds.y, carTypeSelectionCaptionBounds.height, WHITE);
    DrawLine(HORIZONTAL_PAGE_PADDING, carTypeSelectionCaptionBounds.y + carTypeSelectionCaptionBounds.height + SPACING / 2,
             SCREEN_WIDTH - HORIZONTAL_PAGE_PADDING, carTypeSelectionCaptionBounds.y + carTypeSelectionCaptionBounds.height + SPACING / 2, WHITE);

    // Input fields with labels
    static bool inputsInitialized = false;
    static InputField* vehicleIdField;

    int startY = carTypeSelectionCaptionBounds.y + carTypeSelectionCaptionBounds.height + SPACING;

    // Vehicle ID input with label
    DrawText("Vehicle ID:",
             HORIZONTAL_PAGE_PADDING,
             startY,
             LABEL_FONT_SIZE,
             WHITE);

    if (!inputsInitialized) {
        vehicleIdField = new InputField(
            HORIZONTAL_PAGE_PADDING + LABEL_WIDTH,
            startY,
            BUTTON_FONT_SIZE,
            BUTTON_PADDING,
            DARKGRAY,
            WHITE
        );
        vehicleIdField->setPlaceholder("es. hydra");
        vehicleIdField->setPlaceholderColor(GRAY);
        vehicleIdField->setText(TextFormat("dcp"));
        inputsInitialized = true;
    }

    vehicleIdField->draw();
    vehicleId = vehicleIdField->getText();
    std::transform(vehicleId.begin(), vehicleId.end(), vehicleId.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (IsKeyPressed(KEY_ENTER)) {
        SimulationType modeLocal = (SimulationType)currentSelectedCar;
        std::string vehicleIdLocal = vehicleIdField->getText();
        std::transform(vehicleIdLocal.begin(), vehicleIdLocal.end(), vehicleIdLocal.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        saveandclose = true;
        if (modeLocal == SimulationType::ONBOARD) {
            vehicleIdName = "hydra";
            host_number = "control.local";
        } else if (modeLocal == SimulationType::SIMULATOR) {
            vehicleIdName = vehicleIdLocal;
            host_number = "localhost";
        }
        // Initialize telemetry data and communication
        TelemetryData telemetryData;
        Communication communication(host_number, vehicleIdName, modeLocal, &telemetryData);

        // Wait until the connection is established
        while (communication.getConnection()->getStatus() != PAHOMQTTConnectionStatus::CONNECTED) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // Car selection buttons
    Button simCarButton(
        HORIZONTAL_PAGE_PADDING,
        vehicleIdField->getBottom() + SPACING * 2,
        BUTTON_FONT_SIZE,
        BUTTON_PADDING,
        YELLOW,
        "SIM CAR",
        BLACK,
        []() { selectSimCar(); }
    );

    Button realCarButton(
        HORIZONTAL_PAGE_PADDING,
        simCarButton.getBottom() + SPACING,
        BUTTON_FONT_SIZE,
        BUTTON_PADDING,
        YELLOW,
        "REAL CAR",
        BLACK,
        []() { selectRealCar(); }
    );

    std::vector<Button> carButtons = {simCarButton, realCarButton};
    for (Button& button : carButtons) {
        button.setHoverColor(WHITE);
        button.setHoverTextColor(BLACK);
        button.draw();
    }

    const int carSelectionBorderOffsetPx = 5;
    const int carSelectionBorderThickness = 2;
    switch (currentSelectedCar) {
        case AvailableCars::SIM_CAR:
            simCarButton.drawBorder(carSelectionBorderOffsetPx, YELLOW, carSelectionBorderThickness);
            mode = SimulationType::SIMULATOR;
            break;
        case AvailableCars::REAL_CAR:
            realCarButton.drawBorder(carSelectionBorderOffsetPx, YELLOW, carSelectionBorderThickness);
            mode = SimulationType::ONBOARD;
            break;
    }

    // Save and Close Menu Button
    Button saveAndCloseButton(
        HORIZONTAL_PAGE_PADDING,
        realCarButton.getBottom() + SPACING * 4,
        BUTTON_FONT_SIZE,
        BUTTON_PADDING,
        GREEN,
        "SAVE",
        BLACK,
        [&saveandclose, &mode, &vehicleId]() {
            SimulationType modeLocal = (SimulationType)currentSelectedCar;
            std::string vehicleIdLocal = vehicleIdField->getText();
            std::transform(vehicleIdLocal.begin(), vehicleIdLocal.end(), vehicleIdLocal.begin(),
                           [](unsigned char c) { return std::tolower(c); });
            saveandclose = true;
            if (modeLocal == SimulationType::ONBOARD) {
                vehicleIdName = "hydra";
                host_number = "control.local";
            } else if (modeLocal == SimulationType::SIMULATOR) {
                vehicleIdName = vehicleIdLocal;
                host_number = "localhost";
            }
            // Initialize telemetry data and communication
            TelemetryData telemetryData;
            Communication communication(host_number, vehicleIdName, modeLocal, &telemetryData);

            // Wait until the connection is established
            while (communication.getConnection()->getStatus() != PAHOMQTTConnectionStatus::CONNECTED) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
    );

    saveAndCloseButton.setHoverColor(WHITE);
    saveAndCloseButton.setHoverTextColor(BLACK);
    saveAndCloseButton.draw();
}