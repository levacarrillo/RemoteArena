enum move {
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

movement generate_output(int move_output, float advance, float twist) {

    movement output;

    switch(move_output) {
        case STOP:
            output.advance = 0.0f;
            output.twist = 0.0f;
            break;

        case FORWARD:
            output.advance = advance;
            output.twist = 0.0f;
            break;

        case BACKWARD:
            output.advance = -advance;
            output.twist = 0.0f;
            break;

        case LEFT:
            output.advance = 0.0f;
            output.twist = twist;
            break;

        case RIGHT:
            output.advance = 0.0f;
            output.twist = -twist;
            break;

        default:
            printf("OUTPUT %d NOT DEFINED ", move_output);
            output.advance = 0.0f;
            output.twist = 0.0f;
            break;
    }

    return(output);

}

int quantize_light(float *light_readings){
    int sensor = 0;

    for(int i = 1; i < 8; i+=2 )
    {
        if( light_readings[i] > light_readings[sensor] )
            sensor = i;
    }

    if(sensor == 0)      return 2;
    else if(sensor == 1) return 3;
    else if(sensor == 3) return 1;
    else if(sensor == 5) return 0;
    else if(sensor == 7) return 2;
    else
        return 0;
}