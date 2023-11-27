#ifndef UR_MOVE_H
#define UR_MOVE_H
#include <vector>

#define CTRL_FREQ 128

enum contol_mode {
    stop,
    moveJ,
    moveL,
    servol,
    force,
    speedL,
    teach,
    contol_mode_end
};

typedef struct
{
    double velocity = 0.5;
    double acceleration = 1.2;
    double lookaheadTime = 0.1;
    double gain = 500;//2000;
}Servol_Arg;

typedef struct
{
    double velocity = 1;
    double acceleration = 1;
    std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
    std::vector<int> selection_vector = {1, 0, 0, 0, 0, 0};
    std::vector<double> wrench = {0, 0, 0, 0, 0, 0};
    int force_type = 2;
    std::vector<double> limitsSpeed = {2, 2, 2, 1, 1, 1};
}ForceMode_Arg;

typedef struct
{
    std::vector<double> speed;
    double acceleration = 0.5;
    double returnTime = 1.0 / CTRL_FREQ;
}SpeedL_Arg;

typedef struct
{
    const double velocity = 0.5;
    const double acceleration = 0.5;
    Servol_Arg servol;
    ForceMode_Arg force;
    SpeedL_Arg speedL;
    std::vector<double> initPose = {0.0, -194.0, 1000.0, 0.0055, 2.2262, -2.2214};
    contol_mode mode = stop;
    contol_mode lastMode = stop;
    bool newPoseTarget = false;
}Move_Arg;

#endif // UR_MOVE_H
