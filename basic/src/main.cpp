// hier testen fC<r nix nur gerade aus

#include <kipr/botball.h>


void bd(int vel){
	mav(0,vel);
    mav(1,vel);
}

int main()
{
    accel_calibrate();
    bd(1200);
	msleep(10000);

    return 0;
} 	
