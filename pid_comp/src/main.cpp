// hier testen fuer pid + compass mit calman

#include <kipr/botball.h>
#include <robo4you/robo4you.hpp>
#include <iostream>

using namespace std;


class kalman{
    public:
    static const double R = 10; //stC$rke der anpassung - noise covariance
    static const double H = 1.00;   //wertigkeit - measurement map scalar
    static const double Q = 20;   //anfangs wertaenderung - initial estimated covariance

    double P;    //irgenwie wie Q - initial error covariance
    double U_hat;    //startwert - initial estimated state
    double K;    //initial Kalman gain
    
    kalman() {
        P = 10;
        U_hat = 0;
    }
    
  	double filter(double U){
        
    	K =  P * H / (H * P * H + R);   //update kalman gain
    	U_hat +=  K *(U - H * U_hat);   //update estimated

    	P = (1 - K * H)* P + Q; //update error covariance
        
    	return U_hat;
    }
};

void pid_drive (double distance, double spd) {
    int reverse = 0;
    
    clear_motor_position_counter(leftMotor);
    clear_motor_position_counter(rightMotor);
    
    double start_angl = ((get_compass_angle() * 180) / PI) - 10;
	
    bool forw = true;
    int speed_curr = 0;
    
    double error = 0.0;
    double out = 0.0;
    double setpoint = 0;
    
    // tune the 3 numbers in order to fix the pid controller (hopefuly)
    PID pid = PID(&error, &out, &setpoint, 0.5, 0.0, 0.0, 1, reverse);
    pid.SetMode(1);
    pid.SetSampleTime(100);
    pid.SetOutputLimits(-100, 100);
    cout << "Mode" << pid.GetMode() << endl;
    cout << "Direction" << pid.GetDirection() << endl;

    if (distance < 0) {
        forw = false;
        distance = abs(distance);
    }
    distance = distance / 10.0;
    
    //hier wird da kali kali-briert
    kalman comp = kalman(); // Kompass

    while (fabs(getLeftDistance()) < distance && fabs(getRightDistance()) < distance) {
		if (forw) {
            if (spd >= speed_curr) {
                speed_curr += 1;
            }
        }
        else {
            if (-spd <= speed_curr) {
                speed_curr -= 1;
            }
        }
        
        pid.SetMode(1);
        
        ///magic
        
        double cmp = start_angl - comp.filter(((get_compass_angle() * 180) / PI));
        
        ///
        
        error = cmp * (-1);
        
        if (pid.Compute()) {
            cout << "--- ERR --- OUT ---"/* << pid.Compute()*/ << endl;
            cout << error << endl;
            //pid.Compute();
            //cout << "Comp " << pid.Compute() << endl;
            cout << out << endl;
        }
        
        motor_power(leftMotor, get_left_motor_speed(speed_curr) + out); // -
        motor_power(rightMotor,get_right_motor_speed(speed_curr) - out); // +
        msleep(10);
    }
    stop();
    printf("Done with action\n");
    
    cout << "Mode" << pid.GetMode() << endl;
}

int main() {

    /*for (int i = 0; i < size; i++) {
        filtered[i] = Kalman(input[i]);
        cout << i << ": " << filtered[i] << endl;
    }*/
    /*
    int speed = 23;
    motor_power(leftMotor, speed);
    motor_power(rightMotor, -speed);
	calibrate_compass();
    ao();
    msleep(6000);
    */
    set_compass_params(-87.983002, 103.579002, 250.606995, -0.112845, 0.047464, 1.006347, 1.013860);
    
    set_distances(7.3, 12.5);
    set_tick_factor(1);
    set_round_ticks(1600, 1690); // 1600 1685 muss man direkt vor dem fahren tunen: links, rechts (wie schnell welcher ist)
    
    double distance = 1700;
    double spd = 90;
    
    pid_drive(distance, spd);

    return 0;
}