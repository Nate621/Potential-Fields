#include <iostream>
#include <armadillo>
#include <matplot/matplot.h>
#include <cmath>

#define ATTRACTIVE_INTENSITY_GAIN 1.0
#define REPULSIVE_INTENSITY_GAIN 20.0
#define REPULSIVE_SENSING_DISTANCE 1.0
#define TIME_STEP 0.05
#define DIMENSION 2
#define TOTAL_ATTEMPTS 200
#define GOAL_REACHED_THRESHOLD 0.001
// Forgive the obscene amount of comments: this is how I normally learn. I delete them later on, don't worry, lol
/*
It is correct to call these positions in the geometric sense (referring to the goal, the obstacle, and the robot), but they should NOT be interpreted 
as states of a Newtonian dynamical system, since the algorithm does not simulate physical mass, velocity, or acceleration. Instead, 
they are points in a configuration space updated by a potential field based motion rule.

i.e. Newtonian Mechanics - Simulating a ball rolling on a surface where momentum, gravity and inertia exist. Potential Fields (and most path planning algorithms)
just focus on "given the obstacles positions and the goal position, take a step in the best direction toward the goal" while ignoring the physics causing the change 
in position (can't call this kinematics either, but it's a somewhat kinematic rule to impart motion). 

    - Robot State = Current Robot's geometric position
    - Reference State = Robot's geometric goal position
    - Repulsive Source = Obstacle's geometric location 

*/
// Want them to be const so that the current position/goal will not be modified during the function call
/* Attractive energy equation:
(1/2) * ATTRACTIVE_INTENSITY_GAIN * (current pose - goal pose)^2

Taking the gradient of this yields the following (Attractive Force):
*/
arma::vec attractive_force(const arma::vec& robot_pos, const arma::vec& goal_pos){
    return -ATTRACTIVE_INTENSITY_GAIN*(robot_pos - goal_pos);
}

// We are given multiple obstacles! 
arma::vec repulsive_force(const arma::vec& robot_pos, const std::vector<arma::vec>& obstacles){
    
    arma::vec total_repulsive_force(DIMENSION, arma::fill::zeros);

    // For loop to iterate through all obstacles
    for(const auto& obs_pos: obstacles){
        
        // Norm: Any function that takes a vector and returns a number
        // Magnitude: Eucludian Norm (L2 norm)
        arma::vec displacement_vec = robot_pos - obs_pos;
        double magnitude = arma::norm(displacement_vec);
        arma::vec force(DIMENSION, arma::fill::zeros);

        // Do not just immediately return the repulsive force! We need to first check if the 
        // obstacle is close enough to an obstacle to apply a repulsive force
        if(magnitude > REPULSIVE_SENSING_DISTANCE || magnitude == 0){
            // Not close enough to obstacle or we made it to goal! Return zero force vector    
            continue;
        }

        // Normalize our vector to only get direction (we want force to only be influenced by JUST the magnitude, NOT the direction!)
        // If we don't normalize, our strength increases incorrectly because our displacement_vec vector contains magnitude information! (So normalizea and get the unit vector to only get direction)
        arma::vec direction = displacement_vec / magnitude;
        double repulsive_strength = REPULSIVE_INTENSITY_GAIN * ((1/magnitude) - (1/REPULSIVE_SENSING_DISTANCE)) * std::pow((1/magnitude), 2);
        total_repulsive_force += magnitude * direction;
    
    }

    // Find sum total of repulsive forces and return that!
    return total_repulsive_force;

}

int main(){

    arma::vec att_force, rep_force, total_force,
        robot_pos{0, 0},
        goal_pos{5, 3};

    // Obstacle locations
    std::vector<arma::vec> obstacles{
        {-1, 1},
        {2, 4},
        {2, 2}
    };

    for(int i = 0; i < TOTAL_ATTEMPTS; i++){

        att_force = attractive_force(robot_pos, goal_pos);
        rep_force = repulsive_force(robot_pos, obstacles);

        total_force = att_force + rep_force;
        robot_pos += TIME_STEP * total_force; // Should initially be extremely high; this will reach zero once we reach goal 
        std::cout << "Step " << i << ":" << std::endl;
        std::cout << "Robot's Position: (" << robot_pos.at(0) << ", " << robot_pos.at(1) << ")" << std::endl;  

        if(arma::norm(robot_pos - goal_pos) < GOAL_REACHED_THRESHOLD){
            std::cout << "Reached goal within tolerance! Exiting loop..." << std::endl;
            break;
        }
    }    

    
    

    return 0;
}