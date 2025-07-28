#pragma once
#include "raylib.h"
#include <string>
#include <functional>

class Button {
public:
/**
 * @brief Construct a new Button object
 *
 * @param x The x position (left to right) of the button on the screen
 * @param y The y position (top to bottom) of the button on the screen
 * @param fontSize The size of the font used to draw the text
 * @param padding The padding around the text
 * @param backgroundColor The color of the button
 * @param text The text to display on the button
 * @param textColor The color of the text
 * @param callback The function to call when the button is clicked
 */
Button(int x, int y, int fontSize, int padding, Color backgroundColor, std::string text, Color textColor, std::function<void()> callback);

int getX();
void setX(int x);

int getY();
void setY(int y);

int getFontSize();
void setFontSize(int fontSize);

int getPadding();
void setPadding(int padding);

int getTop();
int getRight();
int getBottom();
int getLeft();

int getWidth();
int getHeight();

/**
 * @brief Draws the button on the screen
 */
void draw();

/**
 * @brief Draws a border around the button
 *
 * @param offset How much the border should be offset from the button bounds
 * @param borderColor The color of the drawn border
 * @param borderWidth The width of the drawn border
 */
void drawBorder(int offset, Color borderColor, int borderWidth);

void setHoverColor(Color hoverColor);
void setHoverTextColor(Color hoverTextColor);
void setClickColor(Color clickColor);
void setClickTextColor(Color clickTextColor);

Rectangle getBouds();

private:
    int x;
    int y;
    int fontSize;
    int padding;
    Color backgroundColor;
    std::string text;
    Color textColor;
    std::function<void()> callback;
    int width;
    Color hoverColor;
    Color hoverTextColor;
    Color clickColor;
    Color clickTextColor;
};