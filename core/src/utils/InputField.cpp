/**
 * @file InputField.cpp
 * @author Donatello Donini (donatello.donini@eagletrt.it)
 * @brief Implementation of an input field utility in the context of the raylib library.
 * @version 0.1
 * @date 2024-12-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "InputField.hpp"

InputField::InputField(int x, int y, int fontSize, int padding, Color backgroundColor, Color textColor) :
    x(x),
    y(y),
    fontSize(fontSize),
    padding(padding),
    minWidth(64),
    borderWidth(1),      // Now before maxWidth
    maxWidth(128),       // Matches declaration order
    borderOffset(0),
    backgroundColor(backgroundColor),
    textColor(textColor),
    borderColor(BLANK),
    placeholderColor(BLANK),
    text(""),
    placeholder(""),
    focused(false),
    onChangeCallback(nullptr),
    wasFocused(false),
    textBeforeFocus("")
{}

int InputField::getX() { return x; }
void InputField::InputField::setX(int x) { this->x = x; }

int InputField::getY() { return y; }
void InputField::setY(int y) { this->y = y; }

int InputField::getFontSize() { return fontSize; }
void InputField::setFontSize(int fontSize) { this->fontSize = fontSize; }

int InputField::getPadding() { return padding; }
void InputField::setPadding(int padding) { this->padding = padding; }

int InputField::getHeight() {
    return fontSize + padding * 2;
}

int InputField::getWidth() {
    int idealWidth= MeasureText(getText().c_str(), fontSize) + padding * 2;

    if (idealWidth < minWidth) return minWidth;
    if (idealWidth > maxWidth) return maxWidth;
    return idealWidth;
}

int InputField::getMinWidth() { return minWidth; }
void InputField::setMinWidth(int minWidth) { this->minWidth = minWidth; }

int InputField::getMaxWidth() { return maxWidth; }
void InputField::setMaxWidth(int maxWidth) { this->maxWidth = maxWidth; }

bool InputField::isFocused() { return focused; }
void InputField::setFocused(bool focused) { this->focused = focused; }

std::string InputField::getText() { return text; }
void InputField::setText(std::string text) { this->text= text; }

void InputField::setPlaceholder(std::string placeholder) { this->placeholder = placeholder; }
void InputField::setPlaceholderColor(Color placeholderColor) { this->placeholderColor = placeholderColor; }
void InputField::setBackgroundColor(Color backgroundColor) { this->backgroundColor = backgroundColor; }
void InputField::setTextColor(Color textColor) { this->textColor = textColor; }
void InputField::setBorderColor(Color borderColor) { this->borderColor = borderColor; }

int InputField::getTop() { return y; }
int InputField::getRight() { return x + getWidth(); }
int InputField::getBottom() { return y + getHeight(); }
int InputField::getLeft() { return x; }

void InputField::onChange(std::function<void()> callback) { this->onChangeCallback= callback; }

void InputField::draw() {
    DrawRectangle(x, y, getWidth(), getHeight(), backgroundColor);
    DrawText(text.c_str(), (x + padding), (y + padding), fontSize, textColor);

    ///                      ///
    /// Handling placeholder ///
    ///                      ///

    if (text.empty() && !placeholder.empty()) {
        DrawText(placeholder.c_str(), (x + padding), (y + padding), fontSize, placeholderColor);
    }

    ///                  ///
    /// Handling borders ///
    ///                  ///

    if (borderWidth > 0 && (!areColorsEqual(borderColor, BLANK))) {
        DrawRectangleLinesEx(Rectangle{(float)(x - borderOffset), (float)(y - borderOffset), (float)(getWidth()+ borderOffset * 2), (float)(getHeight() + borderOffset * 2)}, borderWidth, borderColor);
    }

    ///                ///
    /// Handling focus ///
    ///                ///

    Vector2 mousePosition = GetMousePosition();
    if (mousePosition.x >= x && mousePosition.x <= x + getWidth() && mousePosition.y >= y && mousePosition.y <= y + getHeight()) {
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            focused = true;
            wasFocused= true;
            textBeforeFocus= text;
        }
    } else {
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            focused = false;
        }
    }

    // Draw the cursor
    if (focused) {
        int cursorX= x + padding + MeasureText(text.c_str(), fontSize) + 2;
        DrawLine(cursorX, y + padding, cursorX, y + padding + fontSize, textColor);
    }
    else if (wasFocused && (textBeforeFocus.compare(text)!= 0)){
        if (onChangeCallback != nullptr) {
            onChangeCallback();
        }
        wasFocused= false;
    }

    ///                 ///
    /// Handling typing ///
    ///                 ///

    if (focused) {
        if (IsKeyPressed(KEY_BACKSPACE) && text.length() > 0) {
            text.pop_back();
        }
        else if (IsKeyPressed(KEY_ENTER)) {
            focused = false;
        }
        else{
            int key = GetKeyPressed();
            if (key > 0) {
                text.push_back((char)key);
            }
        }
    }
}