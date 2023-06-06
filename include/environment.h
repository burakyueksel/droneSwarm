/*
 * File: drone.cpp
 * Author: Burak Yueksel
 * Date: 2023-06-04
 */
#pragma once

struct Environment
{
    static constexpr double GRAVITY = 9.81; // m/s^2
    static constexpr double timeStep = 0.001; // s
    static constexpr double timeEnd = 10; // s
};