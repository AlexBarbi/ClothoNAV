#include "Button.hpp"

Button::Button(int x, int y, int fontSize, int padding, Color backgroundColor, std::string text, Color textColor, std::function<void()> callback) :
    x(x),
    y(y),
    fontSize(fontSize),
    padding(padding),
    backgroundColor(backgroundColor),
    text(text),
    textColor(textColor),
    callback(callback) {}

int Button::getX() { return x; }
void Button::setX(int x) { this->x = x; }

int Button::getY() { return y; }
void Button::setY(int y) { this->y = y; }

int Button::getFontSize() { return fontSize; }
void Button::setFontSize(int fontSize) { this->fontSize= fontSize; }

int Button::getPadding() { return padding; }
void Button::setPadding(int padding) { this->padding= padding; }

int Button::getWidth() { return MeasureText(text.c_str(), fontSize) + padding*2; }
int Button::getHeight() { return fontSize + padding*2; }

int Button::getTop() { return y; }
int Button::getRight() { return x + getWidth(); }
int Button::getBottom() { return y + getHeight(); }
int Button::getLeft() { return x; }

void Button::draw() {
    DrawRectangle(x, y, getWidth(), getHeight(), backgroundColor);
    DrawText(text.c_str(), x + getWidth() / 2 - MeasureText(text.c_str(), fontSize) / 2, y + getHeight() / 2 - fontSize / 2, fontSize, textColor);

    Vector2 mousePos = GetMousePosition();
    if (mousePos.x >= x && mousePos.x <= getRight() && mousePos.y >= y && mousePos.y <= getBottom()) {
        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            DrawRectangle(x, y, getWidth(), getHeight(), clickColor);
            DrawText(text.c_str(), x + getWidth() / 2 - MeasureText(text.c_str(), fontSize) / 2, y + getHeight() / 2 - fontSize / 2, fontSize, clickTextColor);
            callback();
        } else {
            DrawRectangle(x, y, getWidth(), getHeight(), hoverColor);
            DrawText(text.c_str(), x + getWidth() / 2 - MeasureText(text.c_str(), fontSize) / 2, y + getHeight() / 2 - fontSize / 2, fontSize, hoverTextColor);
        }
    }
}

void Button::drawBorder(int offset, Color borderColor, int borderWidth) {
    DrawRectangleLinesEx({(float)x - offset, (float)y - offset, (float)getWidth() + offset * 2, (float)getHeight() + offset * 2}, borderWidth, borderColor);
}

void Button::setHoverColor(Color hoverColor) { this->hoverColor = hoverColor; }
void Button::setHoverTextColor(Color hoverTextColor) { this->hoverTextColor = hoverTextColor; }
void Button::setClickColor(Color clickColor) { this->clickColor = clickColor; }
void Button::setClickTextColor(Color clickTextColor) { this->clickTextColor = clickTextColor; }

Rectangle Button::getBouds() { return { (float)x, (float)y, (float)getWidth(), (float)getHeight() }; }