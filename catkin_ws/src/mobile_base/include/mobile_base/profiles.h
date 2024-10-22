#ifndef PROFILE_H
#define PROFILE_H

#include <iostream>
#include <geometry_msgs/Twist.h>

enum Profile {
    RECTANGULAR,
    TRIANGULAR,
    TRAPEZOIDAL,
    CUSTOM_1,
    CUSTOM_2,
    POLYNOMIAL,
};

float rectangularProfile(float v_max, float error){
    if (error < 0)
        return -v_max;
    else
        return v_max;
}

float triangularProfile(float v_max, float init, float curr, float goal, float v_init) {
    float midGoal = (goal - init) / 2;
    float m1 = 2 * (v_max - v_init) / (goal - init);
    float m2 = - v_max  / (goal - init);
    if (curr <= midGoal) {
        return v_init + m1 * (curr + init);
    } else {
        return v_max + m2 * (curr + init);
    }
    return v_max;
}

float trapezoidalProfile(float max_angular) {
    return max_angular;
}


float customProfile1(float max_angular, float angle_error, float alpha) {
    return max_angular * (2 / (1 + exp(-angle_error / alpha)) - 1);
}
float customProfile2(float max_angular) {
    return max_angular;
}

float polynomialProfile(float max_angular) {
    return max_angular;
}

geometry_msgs::Twist getAngularVelocity(
                                            float init,
                                            float curr,
                                            float goal,
                                            float angle_error,
                                            Profile profile) {

    float w;
    float start_angular = 0.5;
    float max_vel_angular = 0.5;
    float initial_angular = 0.2;
    float auto_increment = 0.1;
    float alpha = 0.45;

    switch(profile) {
        case RECTANGULAR:
            w = rectangularProfile(max_vel_angular, angle_error);
        break;
        case TRIANGULAR:
            w = triangularProfile(max_vel_angular, init, curr, goal, initial_angular);
        break;
        case TRAPEZOIDAL:
            w = trapezoidalProfile(max_vel_angular);
        break;
        case CUSTOM_1:
            w = customProfile1(max_vel_angular, angle_error, alpha);
        break;
        case CUSTOM_2:
            w = customProfile2(max_vel_angular);
        break;
        case POLYNOMIAL:
            w = polynomialProfile(max_vel_angular);
        break;
        default:
            w = rectangularProfile(max_vel_angular, angle_error);
        break;
    }

    geometry_msgs::Twist angular_vel;
    angular_vel.linear.x = 0.0;
    angular_vel.linear.y = 0.0;
    angular_vel.angular.z = w;

    return angular_vel;
}

geometry_msgs::Twist getLinearVelocity(float distance_error, Profile profile) {
    float v_max = 0.2;
    float v = v_max;
    float alpha = 1.2;

    switch(profile) {
        case RECTANGULAR:
            v = rectangularProfile(v_max, distance_error);
        break;
        // case TRIANGULAR:
        //     // v = triangularProfile(v_max);
        // break;
        // case TRAPEZOIDAL:
        //     v = trapezoidalProfile(v_max);
        // break;
        // case CUSTOM_1:
        //     v = customProfile1(v_max, distance_error, alpha);
        // break;
        // case CUSTOM_2:
        //     v = customProfile2(v_max);
        // break;
        // case POLYNOMIAL:
        //     v = polynomialProfile(v_max);
        // break;
        // default:
        //     v = rectangularProfile(v_max, distance_error);
        // break;
    }

    geometry_msgs::Twist linear_velocity;
    linear_velocity.linear.x = v;
    linear_velocity.linear.y = 0.0;
    linear_velocity.angular.z = 0.0;

    return linear_velocity;
}

geometry_msgs::Twist stop() {
    geometry_msgs::Twist move;
    move.linear.x = 0.0;
    move.linear.y = 0.0;
    move.angular.z = 0.0;

    return move;
}

#endif
