#ifndef CHECKBOX_HPP
#define CHECKBOX_HPP

#include <string>
#include <functional>
#include "raylib.h"

class CheckBox {
private:
    int x, y;
    int fontSize;
    int padding;
    Color color;
    std::string text;
    Color textColor;
    bool checked;
    std::function<void(bool)> toggleCallback;

public:
    CheckBox(int x, int y, int fontSize, int padding, Color color, std::string text, Color textColor);

    void draw();
    void onToggle(std::function<void(bool)> callback);

    bool isChecked() const;

    int getBottom() const;
    int getRight() const;
    int getLeft() const;
    int getTop() const;
};

#endif // CHECKBOX_HPP
