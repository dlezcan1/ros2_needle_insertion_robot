//
//  Galil.h
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 3/31/22.
//

#pragma once

#include "Controller.hpp"
#include "Axis.hpp"
#include "Robot.hpp"

#include <limits>

namespace Galil
{
    // helpful typedefs
    template <typename T> using Array = std::array<T, GALIL_NUM_AXES>;

    // NULL AXES operations
    const long NULL_LONG_AXIS = std::numeric_limits<long>::lowest();
    const float NULL_FLOAT_AXIS = std::numeric_limits<float>::lowest();

    inline bool isNullAxis(long axis) { return (axis == NULL_LONG_AXIS); }
    inline bool isNullAxis(float axis){ return (axis == NULL_FLOAT_AXIS); }

    // helpful functions
    template <typename T, size_t N> std::array<T, N> newSTDArray  () { return std::array<T, N>(); } // default
    template <typename T, size_t N> std::array<bool, N>  newSTDArray<T, N>(T fillval) { std::array<T, N> retval; retval.fill(T); return retval; }
    template <size_t N> std::array<bool, N>  newSTDArray<bool, N> () { return newSTDArray<bool, N> ( false ); }
    template <size_t N> std::array<long, N>  newSTDArray<long, N> () { return newSTDArray<long, N> ( NULL_LONG_AXIS ); }
    template <size_t N> std::array<float, N> newSTDArray<float, N>() { return newSTDArray<float, N>( NULL_FLOAT_AXIS ); }
    
    template <typename T> Array<T> newArray () { return Array<T>(); } // default
    template <typename T> Array<T> newArray (T fillval) { Array<T> retval; retval.fill(fillval); return retval; }
    template <> Array<bool>  newArray<bool> () { return newArray<bool> ( false ); }
    template <> Array<long>  newArray<long> () { return newArray<long> ( NULL_LONG_AXIS ); }
    template <> Array<float> newArray<float>() { return newArray<float>( NULL_FLOAT_AXIS ); }

} // namespace: Galil