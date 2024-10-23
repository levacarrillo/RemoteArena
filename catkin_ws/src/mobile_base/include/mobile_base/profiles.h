#ifndef PROFILE_H
#define PROFILE_H

#include <iostream>
#include <geometry_msgs/Twist.h>

enum Profile {
    RECTANGULAR,
    TRIANGULAR,
    TRAPEZOIDAL,
    GAUSSIAN,
    SIGMOIDE,
    POLYNOMIAL,
};

float incremetal_speed = 0;

float rectangularProfile(float v_max, float error){
    if (error < 0)
        return -v_max;
    else
        return v_max;
}

float triangularProfile(float v_max, float v_init, float curr, float goal) {
    float dir = goal / fabs(goal);
    v_init = fabs(v_init);
    curr   = fabs(curr);
    goal   = fabs(goal);
    // CALCULING TWO GRADIENTS
    float m1 = 2 * (v_max - v_init) / goal;
    float m2 = - (2 * v_max) / goal;
    if (curr <= goal / 2) {
        return dir * (v_init + m1 * curr);
    } else if (curr > goal / 2 && curr < goal) {
        return dir * (v_max + m2 * (curr - goal / 2));
    }
    return 0;
}

float trapezoidalProfile(float v_max, float auto_increment, float curr, float goal) {
    float dir = goal / fabs(goal);
    incremetal_speed += auto_increment;
    if (incremetal_speed > v_max) incremetal_speed = v_max;
    curr   = fabs(curr);
    goal   = fabs(goal);
    if (curr >= goal) incremetal_speed = 0;
    return dir * incremetal_speed;
}

float gaussianProfile(float v_max, float error, float alpha) {
    return v_max * exp(- pow(error, 2) / alpha);
}

float sigmoideProfile(float v_max, float error, float alpha) {
    return v_max * (2 / (1 + exp(- error / alpha)) - 1);
}

float polynomialProfile(float v_max, float v_init, float goal) {
    float a, b, c, x, v_top;
    float xp_0 = 6 * v_init / (v_max + 3 * v_init);
    a = (xp_0 - 2)/pow(goal, 2);
    b = (3 - 2*xp_0) / (goal);
    c = xp_0;
    float x_inf = - b /(3*a);
    float vel = 3*a*pow(x_inf, 2) + 2*b*x_inf + c;
    return vel;
}

geometry_msgs::Twist getAngularVelocity(
                                            float curr,
                                            float goal,
                                            float angle_error,
                                            Profile profile) {

    float w = 0;
    float start_angular = 0.5;
    float max_vel_angular = 0.5;
    float initial_vel_angular = 0.2;
    float auto_increment = 0.1;
    float gaussian_alpha = 30.2;
    float sigmoide_alpha = 0.45;

    switch(profile) {
        case RECTANGULAR:
            w = rectangularProfile(max_vel_angular, angle_error);
        break;
        case TRIANGULAR:
            w = triangularProfile(max_vel_angular, initial_vel_angular, curr, goal);
        break;
        case TRAPEZOIDAL:
            w = trapezoidalProfile(max_vel_angular, auto_increment, curr, goal);
        break;
        case GAUSSIAN:
            w = gaussianProfile(max_vel_angular, angle_error, gaussian_alpha);
        break;
        case SIGMOIDE:
            w = sigmoideProfile(max_vel_angular, angle_error, sigmoide_alpha);
        break;
        case POLYNOMIAL:
            w = polynomialProfile(max_vel_angular, initial_vel_angular, goal);
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

geometry_msgs::Twist getLinearVelocity(
                                        float curr,
                                        float goal,
                                        float distance_error,
                                        Profile profile) {
    float v;
    float v_max = 0.2;
    float initial_vel = 0.1;
    float auto_increment = 0.1;

    float gaussian_alpha = 20.2;
    float sigmoide_alpha = 1.2;

    switch(profile) {
        case RECTANGULAR:
            v = rectangularProfile(v_max, distance_error);
        break;
        case TRIANGULAR:
            v = triangularProfile(v_max, initial_vel, curr, goal);
        break;
        case TRAPEZOIDAL:
            v = trapezoidalProfile(v_max, auto_increment, curr, goal);
        break;
        case GAUSSIAN:
            v = gaussianProfile(v_max, distance_error, gaussian_alpha);
        break;
        case SIGMOIDE:
            v = sigmoideProfile(v_max, distance_error, sigmoide_alpha);
        break;
        case POLYNOMIAL:
            v = polynomialProfile(v_max, initial_vel, goal);
        break;
        default:
            v = rectangularProfile(v_max, distance_error);
        break;
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
