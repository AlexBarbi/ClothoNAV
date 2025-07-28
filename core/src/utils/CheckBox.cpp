// CheckBox.cpp

#include "CheckBox.hpp"

CheckBox::CheckBox(int x, int y, int fontSize, int padding, Color color, std::string text, Color textColor)
    : x(x), y(y), fontSize(fontSize), padding(padding), color(color), text(text),
      textColor(textColor), checked(false) {}

void CheckBox::draw() {
    // Draw checkbox square
    Rectangle box = { (float)x, (float)y, (float)fontSize, (float)fontSize };
    DrawRectangleRec(box, checked ? GREEN : RED);
    DrawRectangleLinesEx(box, 2, BLACK);

    // Draw checkmark if checked
    if (checked) {
        DrawText("", x + 2, y, fontSize, BLACK);
    }

    // Draw label text
    DrawText(text.c_str(), x + fontSize + padding, y, fontSize, textColor);

    // Handle click
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        Rectangle totalArea = {
            (float)x,
            (float)y,
            (float)(MeasureText(text.c_str(), fontSize) + fontSize + padding),
            (float)fontSize
        };

        if (CheckCollisionPointRec(GetMousePosition(), totalArea)) {
            checked = !checked;
            if (toggleCallback) {
                toggleCallback(checked);
            }
        }
    }
}

void CheckBox::onToggle(std::function<void(bool)> callback) {
    toggleCallback = callback;
}

bool CheckBox::isChecked() const {
    return checked;
}

int CheckBox::getBottom() const {
    return y + fontSize;
}

int CheckBox::getRight() const {
    return x + fontSize + padding + MeasureText(text.c_str(), fontSize);
}

int CheckBox::getLeft() const {
    return x;
}

int CheckBox::getTop() const {
    return y;
}
