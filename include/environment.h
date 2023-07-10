/**
 * @file environment.h
 * @brief This file contains the static constant expressions for the environment
 */
 /*
 * Author: Burak Yueksel
 * Date: 2023-06-04
 */
#pragma once

/**
 * @brief Envrionment struct that contains the constant expressions
 */
struct Environment
{
    static constexpr double GRAVITY = 9.81; // m/s^2
    static constexpr double timeStep = 0.001; // s
    static constexpr double timeEnd = 10; // s
    static constexpr double PI      = 3.141592; // constant
};