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

#define THRESHOLD_FOLLOWER 7.1

bool light_follower(float max_intensity, float* light_readings, movement* movement, float max_advance) {

    int sensor_max_value = 0;

    if(max_intensity > THRESHOLD_FOLLOWER) {
        movement->twist   = 0.0;
        movement->advance = 0.0;
        std::cout << "\n ****************** Reached light source ***************\n" << std::endl;
        return true;
    } else {
        for (size_t i=1; i<sizeof(light_readings); i++) {
            if (light_readings[i] > light_readings[sensor_max_value]) 
                sensor_max_value = i;
        }
        if (sensor_max_value > 4) {
            sensor_max_value = - (sizeof(light_readings) - sensor_max_value);
        }
        
        movement->twist   = sensor_max_value * M_PI / 16;
        movement->advance = max_advance;
    }
    return false;
}