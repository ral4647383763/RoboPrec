#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include "inverse_kinematics.hpp"

int main(){

    double goal_position[3] = {-0.253370, 0.307280, 0.384357};
    double goal_rotation[3][3] = {
        {-0.807539, -0.043014, -0.588243},
        {-0.587219, 0.152085, 0.795012},
        {0.055267, 0.987431, -0.148073}
    };

    // q_start could be a random position or the current position of the robot.
    double q_start[6] = {2.07809, -0.645178, 1.72935, 1.82307, 2.51461, -2.26985};
    double q_out[6] = {0};

    inverse_kinematics_roarm(
        goal_position,
        goal_rotation,
        q_start,
        q_out
    );

    //printf("Output joint angles: ");
    //for (int i = 0; i < 6; i++) {
    //    printf("%f ", q_out[i]);
    //}
    //printf("\n");


}