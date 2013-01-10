// Regler-Funktion fertigstellen!
// Wie soll geregelt werden? Motoren einzeln? Ein Motor konstant und der andere je nach Encoder-Differenz? Oder nach Geschwindigkeitsdifferenz? Oder sowohl einzeln als auch untereinander (wie in der Regelungs-Einführung).
// -> done
// IR-Sensoren einbeziehen? -> done


#include "robong/robong.h"
#include "robong_additionals/fout.h"
#include "robong_additionals/control.h"

#define DELAY 10         	// Verzögerung nach Encoder- und Cycle-Clears (nötig?)
#define PID_TIME 10         // Zeitabstand zwischen Regler-Aufrufen (in 10 ms)
#define PID_TIME_IR 10
#define DRIVE_SPEED 60      // Geschwindigkeit beim Fahren
#define TURN_SPEED 40       // Geschwindigkeit beim Drehen

void drive(unsigned long);
void drive_ir(unsigned long);
void turn(unsigned long, int);
void pid();
void pid_stop();
void pid_ir();
void pid_turn();
int ir_mean_l();
int ir_mean_r();
void rescue();

unsigned int sum = 0;
unsigned int left = 0;
unsigned int right = 0;
unsigned int left_ir = 0;
unsigned int right_ir = 0;
int t_left, t_right;

// Encoder
int e = 0;
int e_old = 0;
int e_sum = 0;
int Kp = 67.5/2;
int Ki = 21;
int Kd = 10;
int Ta = 100;
float P = 0;
float I = 0;
float D = 0;
float U = 0;
int left_speed = DRIVE_SPEED;
int right_speed = DRIVE_SPEED;

// IR
int e_ir = 0;
int e_old_ir = 0;
int e_sum_ir = 0;
int Kp_ir = 67.5/2;
int Ki_ir = 21;
int Kd_ir = 10;
float P_ir = 0;
float I_ir = 0;
float D_ir = 0;
float U_ir = 0;
int left_speed_ir = DRIVE_SPEED;
int right_speed_ir = DRIVE_SPEED;

float U_ges = 0;

int x;
int y=0;

// Umrechnung mm -> Inkremente
// int mm2inc = 1632/188;
// 90° turn = 575 inc.
int main()
{
    init_robong(1);
    mode_set(kModeStudent);
    battery_request(kRequestWait);
    lcdclr();
    lcdstr("Wettbewerb 1");
    lcdxy(0,1);
    lcdfout("Batterie: %d",battery_get());
    mdelay(1000);
    lcdclr();
    mdelay(1000);
    
    // Aktivierung der MW-Fkt. der IR-Sensoren
//    unsigned char id_ir_mean_l = time_cycle_add(&ir_mean_l,60);
//    unsigned char id_ir_mean_r = time_cycle_add(&ir_mean_r,60);
    // Aktivierung der Rescuefkright_speedt. bei zu geringem Abstand zu Objekt
//    unsigned char id_rescue = time_cycle_add(&rescue,100);
	
//    drive_ir(14497);
//    drive(868);
//    drive(12000);
//    drive(7000);
    drive_ir(200000);
//    turn(575,1);
//    drive(2000);
//    turn(575,0);


//    time_cycle_clear(id_ir_mean_l);
//    time_cycle_clear(id_ir_mean_r);
//    time_cycle_clear(id_rescue);

    motors_set(kMotorLeft | kMotorRight, 0);
    mdelay(500);

    return 0;
}

/*
int ir_mean_l()
{

	sharps_request(kRequestContinuous);
	int a = 1;
	int ir_sum_l = 0;
	while (a <= 5)
	{
		ir_sum_l += sharps_get(kSharpLeft);
		a++;
		mdelay(10);
	}
	lcdxy(0,0);
	lcdfout("links: %d",ir_sum_l/a);
	return ir_sum_l/a;
}

int ir_mean_r()
{

	sharps_request(kRequestContinuous);
	int a = 1;
	int ir_sum_r = 0;
	while (a <= 5)
	{
		ir_sum_r += sharps_get(kSharpRight);
		a++;
		mdelay(10);
	}
	lcdxy(0,1);
	lcdfout("rechts: %d",ir_sum_r/a);
	return ir_sum_r/a;
}
*/

void drive(unsigned long dist)
{
    left_speed = DRIVE_SPEED;
    right_speed = DRIVE_SPEED;
    motors_set(kMotorLeft | kMotorRight, 0);
    encoders_clear();
    mdelay(DELAY);
    sum = 0;
    e = 0;
    e_sum = 0;
    P = 0;
    I = 0;
    D = 0;
    U = 0;

    x = 30;
    unsigned char id_pid = time_cycle_add(&pid,PID_TIME);
    
    while(sum < dist)
    {
        encoders_request(kRequestWait);
        sum += encoders_get(kEncoderLeft);
        encoders_clear();
        mdelay(DELAY);
    }
    time_cycle_clear(id_pid);

    unsigned char id_pid_stop = time_cycle_add(&pid_stop,PID_TIME);
    while(x > 30)
    {
      encoders_clear();
      mdelay(DELAY);
    }
    time_cycle_clear(id_pid_stop);
    motors_set(kMotorRight, -55);
    motors_set(kMotorLeft, -45);
    mdelay(DELAY);
    motors_set(kMotorLeft | kMotorRight, 0);
    mdelay(300);
}

void drive_ir(unsigned long dist)
{
    left_speed = DRIVE_SPEED;
    right_speed = DRIVE_SPEED;
    motors_set(kMotorLeft | kMotorRight, 0);
    encoders_clear();
    mdelay(DELAY);
    sum = 0;
    e_ir = 0;				// IR-Umbenennung eher aus Übersichtsgründen
    e_sum_ir = 0;
    P_ir = 0;
    I_ir = 0;
    D_ir = 0;
    U_ir = 0;

    unsigned char id_pid_ir = time_cycle_add(&pid_ir,PID_TIME_IR);
    
    while(sum < dist)
    {
        encoders_request(kRequestWait);
        sum += encoders_get(kEncoderLeft);
        encoders_clear();
        mdelay(DELAY);
    }

    time_cycle_clear(id_pid_ir);
    
    unsigned char id_pid_stop = time_cycle_add(&pid_stop,PID_TIME);
    while(x > 30)
    {
      encoders_clear();
      mdelay(DELAY);
    }
    time_cycle_clear(id_pid_stop);
    motors_set(kMotorRight, -55);
    motors_set(kMotorLeft, -45);
    mdelay(DELAY);
    motors_set(kMotorLeft | kMotorRight, 0);
    mdelay(300);
}

void turn(unsigned long angle, int dir)
{
    e = 0;
    e_sum = 0;
    P = 0;
    I = 0;
    D = 0;
    U = 0;
    
    if (dir == 0) // Linksdrehung
    {
      t_right = 1;
      t_left = -1;
    }
    else	// Rechtsdrehung
    {
      t_right = -1;
      t_left = 1;
    }
    left_speed = DRIVE_SPEED;
    right_speed = DRIVE_SPEED;
    motors_set(kMotorLeft | kMotorRight, 0);
    encoders_clear();
    mdelay(DELAY);
    sum = 0;
    
    unsigned char id_pid_turn = time_cycle_add(&pid_turn,PID_TIME);
    while(sum < angle)
    {
        encoders_request(kRequestWait);
        sum += t_right*encoders_get(kEncoderRight);
        encoders_clear();
        mdelay(DELAY);
    }
    time_cycle_clear(id_pid_turn);
    mdelay(DELAY);
    motors_set(kMotorRight, -55*t_right);
    motors_set(kMotorLeft, -45*t_left);
    mdelay(DELAY);
    motors_set(kMotorLeft | kMotorRight, 0);
    mdelay(300);
}

void pid()
{
    
    encoders_request(kRequestWait);
    left = encoders_get(kEncoderLeft);
    right = encoders_get(kEncoderRight);
    
    // links und rechts geregelt
    e_old = e;
    e = left-right;
    e_sum += e;
    P = 1.0f * e;			// P = Kp*e			Kp = 6.75/2
    I = 0.25f*e_sum;		// I = Ki*Ta*e_sum		Ki = 2.1
    D = 0.25f*(e-e_old);	// D = (Kd*(e-e_old))/Ta	Kd = 1
    U = P + I + D;
	
    right_speed = DRIVE_SPEED + U;
    left_speed = 1.02f * left_speed - U;

    if(x<=DRIVE_SPEED){
       
    motors_set(kMotorRight, right_speed * x / DRIVE_SPEED);
    motors_set(kMotorLeft, left_speed * x / DRIVE_SPEED);
    x+=5; 
    }
    
    else{

    motors_set(kMotorRight, right_speed);
    motors_set(kMotorLeft, left_speed);
    }
}

void pid_stop()
{
    
    encoders_request(kRequestWait);
    left = encoders_get(kEncoderLeft);
    right = encoders_get(kEncoderRight);
    
    // links und rechts geregelt
    e_old = e;
    e = left-right;
    e_sum += e;
    P = 1.0f * e;			// P = Kp*e			Kp = 6.75/2
    I = 0.25f*e_sum;		// I = Ki*Ta*e_sum		Ki = 2.1
    D = 0.25f*(e-e_old);	// D = (Kd*(e-e_old))/Ta	Kd = 1
    U = P + I + D;
	
    right_speed = DRIVE_SPEED + U;
    left_speed = 1.02f * left_speed - U;

    if(x>=0){
       
    motors_set(kMotorRight, right_speed * x / (DRIVE_SPEED));
    motors_set(kMotorLeft, left_speed * x / (DRIVE_SPEED));
    x-=5; 
    }
}

void pid_turn()
{
    
    encoders_request(kRequestWait);
    left = t_left*encoders_get(kEncoderLeft);
    right = t_right*encoders_get(kEncoderRight);
    
    // links und rechts geregelt
    e_old = e;
    e = left-right;
    e_sum += e;
    P = 1.0f * e;			// P = Kp*e			Kp = 6.75/2
    I = 0.25f*e_sum;		// I = Ki*Ta*e_sum		Ki = 2.1
    D = 0.25f*(e-e_old);	// D = (Kd*(e-e_old))/Ta	Kd = 1
    U = P + I + D;
	
    right_speed = DRIVE_SPEED + U;
    left_speed = 1.02f * left_speed - U;


    motors_set(kMotorRight, t_right*right_speed);
    motors_set(kMotorLeft, t_left*left_speed);

}

void pid_ir()
{ 
    encoders_request(kRequestWait);
    left = encoders_get(kEncoderLeft);
    right = encoders_get(kEncoderRight);
    
    // Encoder
    // links und rechts geregelt
    e_old = e;
    e = left-right;
    e_sum += e;
    P = 1.0f * e;			// P = Kp*e			Kp = 6.75/2
    I = 0.25f*e_sum;		// I = Ki*Ta*e_sum		Ki = 2.1
    D = 0.25f*(e-e_old);	// D = (Kd*(e-e_old))/Ta	Kd = 1
    U = P + I + D;
    

    // IR
    sharps_request(kRequestContinuous);
        
    int ir_sum_l = 0;
    int ir_sum_r = 0;
    int a = 1;

    
    /*
    while (a <= 5)
	{
		ir_sum_l += sharps_get(kSharpLeft);
		ir_sum_r += sharps_get(kSharpRight);
		a++;
		mdelay(10);
	}
*/	
	ir_sum_l = sharps_get(kSharpLeft);
	ir_sum_r = sharps_get(kSharpRight);
	

	if(y==10){
	lcdxy(0,0);
	lcdfout("links: %d",ir_sum_l/a);
	lcdxy(0,1);
	lcdfout("rechts: %d",ir_sum_r/a);
	y=0;
	}
	else{
	  y++;
	}

    
    left_ir = ir_sum_l;
    right_ir = ir_sum_r;
    
    
    e_old_ir = e_ir;
    if(left_ir < 450 && right_ir < 450 && left_ir > 50 && right_ir > 50)
    {
	e_ir = right_ir - left_ir;
    }
    else
    {
	e_ir=0;
    }
    e_sum_ir += e_ir;
    P_ir = 1.0f * e_ir;
    I_ir = 0.25f * e_sum_ir;
    D_ir = 0.25f * (e_ir-e_old_ir);
    U_ir = P_ir + I_ir + D_ir;  
	
    U_ges = U_ir/40.0f + U; // U + U_ir;
    
    right_speed = DRIVE_SPEED + U_ges;
    left_speed = 1.00f * left_speed - U_ges;
    
    if(x<=DRIVE_SPEED){
       
    motors_set(kMotorRight, right_speed * x / DRIVE_SPEED);
    motors_set(kMotorLeft, left_speed * x / DRIVE_SPEED);
    x+=5; 
    }
    
    else{

    motors_set(kMotorRight, right_speed);
    motors_set(kMotorLeft, left_speed);
    }
}
