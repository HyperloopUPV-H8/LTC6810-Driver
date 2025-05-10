#ifndef BMS_HPP
#define BMS_HPP

#include <cstddef>
#include <cstdint>

#include "DataManager.hpp"
#include "Driver.hpp"

template <size_t N_BATTERIES, size_t N_CELLS,
          void (*const SPI_TRANSMIT)(const std::span<uint8_t>),
          void (*const SPI_RECEIVE)(std::span<uint8_t>),
          void (*const SPI_CS_TURN_ON)(void),
          void (*const SPI_CS_TURN_OFF)(void), uint32_t (*const GET_TICK)(void),
          uint8_t TICK_RESOLUTION_MS, uint32_t PERIOD_MS>
class BMS {
    DataManager<N_CELLS, N_BATTERIES> data_manager{};
    Driver<N_BATTERIES, SPI_TRANSMIT, SPI_RECEIVE, SPI_CS_TURN_ON,
           SPI_CS_TURN_OFF, GET_TICK, TICK_RESOLUTION_MS, PERIOD_MS>
        driver{};

   public:
    void update() { driver.update(); }
};

#endif
