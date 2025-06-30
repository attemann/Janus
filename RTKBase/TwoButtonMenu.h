#ifndef TWO_BUTTON_MENU_H
#define TWO_BUTTON_MENU_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <vector>

class SelectableMenuItem {
public:
    String label;
    std::vector<String> options;
    int currentOption = 0;

    SelectableMenuItem(const String& name, std::initializer_list<String> opts)
        : label(name), options(opts) {
    }

    const String& getValue() const {
        return options[currentOption];
    }

    void nextOption() {
        currentOption = (currentOption + 1) % options.size();
    }
};

class TwoButtonMenu {
public:
    TwoButtonMenu(LiquidCrystal_I2C& lcd)
        : _lcd(lcd), _currentItem(0) {
    }

    void begin() {
        _lcd.clear();
        show();
    }

    void addItem(const String& label, std::initializer_list<String> options) {
        _items.emplace_back(label, options);
    }

    void nextItem() {
        if (_items.empty()) return;
        _currentItem = (_currentItem + 1) % _items.size();
        show();
    }

    void selectValue() {
        if (_items.empty()) return;
        _items[_currentItem].nextOption();
        show();
    }

    void show() {
        _lcd.clear();
        if (_items.empty()) {
            _lcd.setCursor(0, 0);
            _lcd.print("(no items)");
            return;
        }
        auto& item = _items[_currentItem];
        _lcd.setCursor(0, 0);
        _lcd.print("> ");
        _lcd.print(item.label);
        _lcd.setCursor(0, 1);
        _lcd.print("  ");
        _lcd.print(item.getValue());
    }

    const String& getValueByLabel(const String& label) const {
        for (const auto& item : _items) {
            if (item.label == label) return item.getValue();
        }
        static String empty = "";
        return empty;
    }

private:
    LiquidCrystal_I2C& _lcd;
    std::vector<SelectableMenuItem> _items;
    int _currentItem;
};

#endif