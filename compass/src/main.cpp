// hier testen fuer compass

#include <robo4you/robo4you.hpp>
#include <iostream>
#include <time.h>
#include <kipr/botball.h>
#include <stdlib.h>
#include <cmath>
#include <stdexcept>
#include <algorithm>


using namespace std;

int main()
{
    
    int speed = 22;
	//motor(1, speed);
    //motor(0, -speed);
    /*
    motor_power(leftMotor, speed);
    motor_power(rightMotor, -speed);
	calibrate_compass();
    ao();
    msleep(4000);
    */
    set_compass_params(-86.568001, 90.694000, 249.798004, -0.110612, 0.067605, 1.006099, 1.014518);
    
    set_distances(7.3, 12.5);
    set_tick_factor(1);
    set_round_ticks(1600, 1755);

	int speed_drive = 90;
    
    double cal_angle;
    int calal = 50;
    
    for (int i = 0; i < calal; ++i) {
        cal_angle += ((get_compass_angle() * 180) / PI);
    }
    cal_angle = cal_angle / calal;
    
    
    double P = 0.1;
    
    int i;
    double sum;
    //int avg;
    double diff;
    double angle_thrashhold=5.0;
    
    int j=0;
    while(true){
        double curr_angle = ((get_compass_angle() * 180) / PI);
        sum += (cal_angle - curr_angle);
        ++i;
        if (i==50){
            diff = (sum/i); 	//avg // maybe + 5
            //diff = abs(avg-cal_angle);
            cout << "diff1: " << diff << endl;
            if ((diff < -angle_thrashhold || diff > angle_thrashhold) && j > 500) {
                double error = diff * P;
                cout << "err: " << error << endl;
        		motor_power(leftMotor, get_left_motor_speed(speed_drive) + error);
        		motor_power(rightMotor,get_right_motor_speed(speed_drive) - error);
            } else {
                motor_power(leftMotor, get_left_motor_speed(speed_drive));
        		motor_power(rightMotor,get_right_motor_speed(speed_drive));
            }
            
            sum = 0;
            i = 0;
            cout << "diff: " << diff << endl;
        }
        //cout << (curr_angle) << endl;
        //msleep(100);
        ++j;
    }   
}