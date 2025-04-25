#pragma once

class Cell {
   public:
    consteval Cell() : voltage{} {};
    float voltage{};
};