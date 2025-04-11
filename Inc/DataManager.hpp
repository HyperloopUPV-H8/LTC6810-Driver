#pragma once
#include "Battery.hpp"

template <uint8_t N_CELLS, uint8_t N_BATTERIES>
class DataManager {
   private:
    std::array<Battery<N_CELLS>, N_BATTERIES> batteries;

   public:
    DataManager() = default;
    std::array<Battery<N_CELLS>, N_BATTERIES>& get_references();
};

template <uint8_t N_CELLS, uint8_t N_BATTERIES>
std::array<Battery<N_CELLS>, N_BATTERIES>&
DataManager<N_CELLS, N_BATTERIES>::get_references() {
    return batteries;
}