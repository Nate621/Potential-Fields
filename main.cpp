#include <iostream>
#include <armadillo>
#include <matplot/matplot.h>
#include <cmath>

#define ATTRACTIVE_INTENSITY_GAIN 1.0
#define REPULSIVE_INTENSITY_GAIN 120.0
#define REPULSIVE_SENSING_DISTANCE 2.0
#define TIME_STEP 0.05
#define DIMENSION 2
#define TOTAL_ATTEMPTS 1000
#define GOAL_REACHED_THRESHOLD 0.001

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

/* Attractive energy equation:
(1/2) * ATTRACTIVE_INTENSITY_GAIN * (current pose - goal pose)^2
*/
arma::vec attractive_force(const arma::vec& robot_pos, const arma::vec& goal_pos){
    return -ATTRACTIVE_INTENSITY_GAIN * (robot_pos - goal_pos);
}

arma::vec repulsive_force(const arma::vec& robot_pos, const std::vector<arma::vec>& obstacles){

    arma::vec total_repulsive_force(DIMENSION, arma::fill::zeros);

    for(const auto& obs_pos : obstacles){

        arma::vec displacement_vec = robot_pos - obs_pos;
        double magnitude = arma::norm(displacement_vec);

        if(magnitude > REPULSIVE_SENSING_DISTANCE || magnitude == 0){
            continue;
        }

        arma::vec direction = displacement_vec / magnitude;

        double repulsive_strength = REPULSIVE_INTENSITY_GAIN *
            ((1.0 / magnitude) - (1.0 / REPULSIVE_SENSING_DISTANCE)) *
            std::pow((1.0 / magnitude), 2);

        total_repulsive_force += repulsive_strength * direction;
    }

    return total_repulsive_force;
}

int main(){

    arma::vec robot_pos{0, 0};
    arma::vec goal_pos{8, 7};

    std::vector<arma::vec> obstacles{
        {2, 4},
        {6, 3},
        {2, 2}
    };

    std::vector<double> x_path, y_path;

    for(int i = 0; i < TOTAL_ATTEMPTS; i++){

        arma::vec att_force = attractive_force(robot_pos, goal_pos);
        arma::vec rep_force = repulsive_force(robot_pos, obstacles);
        arma::vec total_force = att_force + rep_force;

        robot_pos += TIME_STEP * total_force;

        x_path.push_back(robot_pos.at(0));
        y_path.push_back(robot_pos.at(1));

        if(arma::norm(robot_pos - goal_pos) < GOAL_REACHED_THRESHOLD){
            std::cout << "Reached goal at step " << i << "\n";
            break;
        }
    }

    matplot::figure()->size(1200, 1000);
    matplot::hold(matplot::on);

    matplot::xlim({-10, 10});
    matplot::ylim({-10, 10});
    matplot::axis(matplot::equal);
    matplot::grid(matplot::on);
    matplot::title("Potential Field Path Planning");

    // Obstacles
    std::vector<double> obs_x, obs_y;
    for(const auto& obs : obstacles){
        obs_x.push_back(obs.at(0));
        obs_y.push_back(obs.at(1));
    }

    auto obs_plot = matplot::scatter(obs_x, obs_y, 40);
    obs_plot->marker_face_color("red");

    // Goal
    auto goal_plot = matplot::scatter(
        std::vector<double>{goal_pos.at(0)},
        std::vector<double>{goal_pos.at(1)},
        30
    );
    goal_plot->marker_face_color("blue");

    // Trajectory (final only)
    auto traj = matplot::plot(x_path, y_path, "-o");
    traj->line_width(1);

    // This is for the vector field
    std::vector<double> X, Y, U, V;

    double xmin = -10, xmax = 10;
    double ymin = -10, ymax = 10;
    double step = 1.0;

    for(double x = xmin; x <= xmax; x += step){
        for(double y = ymin; y <= ymax; y += step){

            arma::vec pos{ x, y };

            arma::vec att = attractive_force(pos, goal_pos);
            arma::vec rep = repulsive_force(pos, obstacles);
            arma::vec total = att + rep;

            X.push_back(x);
            Y.push_back(y);

            double scale = 0.2;
            U.push_back(scale * total(0));
            V.push_back(scale * total(1));
        }
    }

    auto q = matplot::quiver(X, Y, U, V);
    q->color("black");

    matplot::show();

    return 0;
}