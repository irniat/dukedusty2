#include <stdio.h>
#include <stdlib.h>

int main(int argc, char** argv) {
    double FORWARD_DELTA = 0.01, TURN_DELTA = 0.01;
    double forward = 0.0;
    double turnrate = 0.0;
    char c;
    int state = 0;
    char command[512];
    bool automatic = true;
    
    if (argc >= 3) {
        FORWARD_DELTA = atof(argv[1]);
        TURN_DELTA = atof(argv[2]);
    }
    
    while (1) {
        c = getchar();
        if (c == 'z') {
            state++;
            state = state % 3;
            sprintf(command, "rosparam set /hallwaydrive/automatic %i", state);
            printf("Now in state %i\n", state);
        }
        else if (c == 'a') {
            turnrate += TURN_DELTA;
            sprintf(command, "rosparam set /hallwaydrive/turnrate %.3lf", turnrate);
        }
        else if (c == 'd') {
            turnrate -= TURN_DELTA;
            sprintf(command, "rosparam set /hallwaydrive/turnrate %.3lf", turnrate);
        }
        else if (c == 'w') {
            forward += FORWARD_DELTA;
            sprintf(command, "rosparam set /hallwaydrive/forward %.3lf", forward);
        }
        else if (c == 's') {
            forward -= FORWARD_DELTA;
            sprintf(command, "rosparam set /hallwaydrive/forward %.3lf", forward);
        }
        else if (c == ' ') {
            automatic = !automatic;
            printf("\nautomatic = %i\n", automatic);
            sprintf(command, "rosparam set /hallwaydrive/automatic %i", automatic);
        }
        else if (c == 'r') {
            //reset
            automatic = 0;
            forward = 0.0;
            turnrate = 0.0;
            sprintf(command, "rosparam set /hallwaydrive/forward %.3lf", forward);
        }
        else {
            continue;
        }
        system(command);
        printf("forward = %.3lf, turnRate = %.3lf\n", forward, turnrate);
    }
    return 0;
}
