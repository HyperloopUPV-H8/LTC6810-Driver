#pragma once
#include "Battery.hpp"

template <std::size_t N_CELLS, std::size_t N_BATTERIES>
class DataManager {
   private:
    std::array<Battery<N_CELLS>, N_BATTERIES> batteries{};

   public:
    consteval DataManager() = default;
    std::array<Battery<N_CELLS>, N_BATTERIES>& get_references();
};

template <std::size_t N_CELLS, std::size_t N_BATTERIES>
std::array<Battery<N_CELLS>, N_BATTERIES>&
DataManager<N_CELLS, N_BATTERIES>::get_references() {
    return batteries;
}