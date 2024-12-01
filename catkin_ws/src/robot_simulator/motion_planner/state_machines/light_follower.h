/***********************************************
*                                              *
*      light_follower.h	                       *
*                                              *
*      Diego Cordero                           *
*      Jesus Savage			       *
*					       *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 2019                      *
*                                              *
*                                              *
************************************************/


bool light_follower(float intensity, float* light_values, movement* movements, float max_advance, float THRESHOLD_FOLLOWER) {

    int sensor_max_value = 0;
    bool finished;

    if(intensity > THRESHOLD_FOLLOWER) {
        movements->twist   = 0.0;
        movements->advance = 0.0;
        std::cout << "\n ****************** light_follower.-> Reached light source ***************\n" << std::endl;
        finished = true;
    } else {
        for (size_t i=1; i<sizeof(light_values); i++) {
            if (light_values[i] > light_values[sensor_max_value]) 
                sensor_max_value = i;
        }
        if (sensor_max_value > 4) {
            sensor_max_value = - (sizeof(light_values) - sensor_max_value);
        }

        movements->twist   = sensor_max_value * M_PI / 16;
        movements->advance = max_advance;
        
        finished = false;
    }
    
    return finished;
}