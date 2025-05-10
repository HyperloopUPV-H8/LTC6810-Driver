#pragma once

struct Cell {
    consteval Cell() : voltage{} {};
    float voltage{};
};