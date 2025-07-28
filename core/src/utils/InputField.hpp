/**
 * @file InputField.hpp
 * @author Donatello Donini (donatello.donini@eagletrt.it)
 * @brief Structure of an input field utility in the context of the raylib library.
 * @version 0.1
 * @date 2024-12-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "raylib.h"
#include "utils.hpp"

#include <string>
#include <functional>

class InputField {
public:
InputField(int x, int y, int fontSize, int padding, Color backgroundColor, Color textColor);

int getX();
void setX(int x);

int getY();
void setY(int y);

int getFontSize();
void setFontSize(int fontSize);

int getPadding();
void setPadding(int padding);

int getHeight();
int getWidth();

int getMinWidth();
void setMinWidth(int minWidth);

int getMaxWidth();
void setMaxWidth(int maxWidth);

bool isFocused();
void setFocused(bool focused);

std::string getText();
void setText(std::string text);

void setPlaceholder(std::string placeholder);
void setPlaceholderColor(Color placeholderColor);
void setBackgroundColor(Color backgroundColor);
void setTextColor(Color textColor);
void setBorderColor(Color borderColor);

int getTop();
int getRight();
int getBottom();
int getLeft();

void onChange(std::function<void()> callback);

void draw();

private:

// Measures
int x;
int y;
int fontSize;
int padding;
int minWidth;
int borderWidth;
int maxWidth;
int borderOffset;

// Colors
Color backgroundColor;
Color textColor;
Color borderColor;
Color placeholderColor;

// Internal
std::string text;
std::string placeholder;
bool focused;
std::function<void()> onChangeCallback;
bool wasFocused;
std::string textBeforeFocus;
};