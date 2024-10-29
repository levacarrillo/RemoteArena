/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_destination.h           		*
 *                                                      *
 *              Jesus Savage                            *
 *              Diego Cordero                           *
 *              FI-UNAM                                 *
 *              13-2-2019                               *
 *                                                      *
 ********************************************************/

#define THRESHOLD_DEST 7.1

bool sm_destination(float max_intensity, int dest, movement *movements, int *next_state, float Mag_Advance, float max_twist)
{
    int state = *next_state;
    bool finished = false;

    switch ( state ) {
        case 1:
            if (max_intensity > THRESHOLD_DEST) {
                *movements=generate_output(STOP, Mag_Advance, max_twist);
                printf("\n **************** Reached light source ******************************\n");
                *next_state = 1;
                finished = true;
            } else {
                *movements=generate_output(FORWARD, Mag_Advance, max_twist);
                *next_state = 2;
            }
            break;
	case 2:
            if (dest == 0){
                // go right
                *movements=generate_output(RIGHT, Mag_Advance,max_twist);
                *next_state = 3;
            } else if (dest == 1){
                // go left
                *movements=generate_output(LEFT, Mag_Advance,max_twist);
                *next_state = 4;
            } else if (dest == 2){
                // go forward 
                *movements=generate_output(FORWARD, Mag_Advance,max_twist);
                *next_state = 3;
            } else if (dest == 3){
                // go forward 
                *movements=generate_output(FORWARD, Mag_Advance,max_twist);
                //printf("Present State: %d FORWARD\n", state);
                *next_state = 4;
            } else if (dest == 4){
                // go forward 
                *movements=generate_output(FORWARD, Mag_Advance,max_twist);
                //printf("Present State: %d FORWARD\n", state);
                *next_state = 1;
            }
            break;
        case 3: // right turn
            *movements=generate_output(RIGHT, Mag_Advance, max_twist);
            *next_state = 1;
            break;
        case 4: // left turn
            *movements=generate_output(LEFT, Mag_Advance,max_twist);
            *next_state = 1;
            break;

        default:
            *movements=generate_output(STOP, Mag_Advance,max_twist);
            *next_state = 1;
            break;
    }

    return finished;
}