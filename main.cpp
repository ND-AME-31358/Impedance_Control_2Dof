#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "FastPWM.h"
#define PI 3.14159265358979323846
#define PWMfrequency 5000
#define PWMperiodTicks 12000 // == 60 000 000 / PWMfrequency

/* --------------------------------------------------- 
 * Implementing a workaround to bypass an ISR-related error by substituting 
   the standard AnalogIn with our custom MyAnalogIn class throughout the experiment. 
   This change is crucial but the details are not essential for understanding the overall experiment setup.
   Please ignore the implementation but use MyAnalogIn everytime you want to use AnalogIn
*/
class MyAnalogIn : public AnalogIn {
public:
    MyAnalogIn(PinName inp) : AnalogIn(inp) { }
    virtual void lock() { }
    virtual void unlock() { }
};
/* --------------------------------------------------- */

// Define number of communication parameters with matlab
#define NUM_INPUTS 22
#define NUM_OUTPUTS 21

Serial pc(USBTX, USBRX,115200);     // USB Serial Terminal for debugging
ExperimentServer server;            // Object that lets us communicate with MATLAB
Timer t;                            // Timer to measure elapsed time of experiment
Ticker currentLoopTicker;           // Ticker to call high frequency current loop
DigitalOut led_g(LED_GREEN,1);      // UDP server state indicator

// Assign digital/analog pins for control and sensing
// Motor 1 - Hip
FastPWM    M1PWM(D9);               // Motor PWM output (we are using the "FastPWM" rather than built-in "PwmOut" for higher resolution)
DigitalOut M1INA(D2);               // Motor forward enable
DigitalOut M1INB(D4);               // Motor backward enable
MyAnalogIn   CS1(A2);               // Current sensor
// Motor 2 - Knee
FastPWM    M2PWM(D10);              // Motor PWM output (we are using the "FastPWM" rather than built-in "PwmOut" for higher resolution)
DigitalOut M2INA(D7);               // Motor forward enable
DigitalOut M2INB(D8);               // Motor backward enable
MyAnalogIn   CS2(A3);               // Current sensor

// Create two quadrature encoder
// 64(counts/motor rev)*18.75(gear ratio) = 1200(counts/rev)
// Pins A, B, no index, 1200 counts/rev, Quadrature encoding
// Note: Reversed A & B to match motor direction
QEI encoder1(D5 ,D3 , NC, 1200 , QEI::X4_ENCODING); 
QEI encoder2(D15,D14, NC, 1200 , QEI::X4_ENCODING); 
const float radPerTick = 2.0*PI/1200.0;

// Set motor duty [-1.0f, 1.0f]
void setMotorDuty(float duty, DigitalOut &INA, DigitalOut &INB, FastPWM &PWM);

// Set motor voltage in (V)
void setMotorVoltage(float voltage, DigitalOut &INA, DigitalOut &INB, FastPWM &PWM);

// Declaration of current current control function.
// Please refer to the latter part of this file for its definition (implementation).
void currentLoopFunc();



// Declare global variables

// Variables for q1
float current1;
float current_des1;
float current_error_int1;
float angle1;
float angle_des1;
float velocity1;
float velocity_des1;
float motor_voltate1;
float angle_init1;

// Variables for q2
float current2;
float current_des2;
float current_error_int2;
float angle2;
float angle_des2;
float velocity2;
float velocity_des2;
float motor_voltate2;
float angle_init2;

// Fixed kinematic parameters
const float l_OA=.011; // length between point O and A
const float l_OB=.042; // length between point O and B
const float l_AC=.096; // length between point A and C
const float l_DE=.090; // length between point D and E
              
// Timing parameters
float current_control_period_us;
float impedance_control_period_us;

// Control parameters
float K_xx;
float K_yy;
float K_xy;

float D_xx;
float D_xy;
float D_yy;


float xSetFoot;
float ySetFoot;

float A;
float omega;


// Model parameters
float Rm;
float kb;
float Kp,Ki;
float kv;
float SupplyVoltage;
float duty_max;

/* Main function that would run on the FRDM board
 * Note: unlike Arduino, we do not have a setup function and a loop function
 * The main function would only run once. To have a loop, we have to use while loop by ourselves
*/ 
int main (void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    led_g = 0;  // UDP server is ready, turn on green led

    // Setting PWM frequency. Here is 5k Hz, same as the current loop
    M1PWM.prescaler(1);
    M1PWM.period_ticks(PWMperiodTicks);

    M2PWM.prescaler(1);
    M2PWM.period_ticks(PWMperiodTicks);
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    
    // infinite while loop, analogous to Arduino's loop function
    while(1) {
        if (server.getParams(input_params,NUM_INPUTS)) {
            // Unpack parameters from MATLAB
            
            current_control_period_us   = input_params[0]; // Current control period in micro seconds
            impedance_control_period_us = input_params[1]; // Impedance control period in microseconds
            float ExpTime               = input_params[2]; // Expriement time in second

            Rm                          = input_params[3]; // Terminal resistance (Ohms)
            kb                          = input_params[4]; // Back EMF Constant (V / (rad/s))
            kv                          = input_params[5]; // Friction coefficienct (Nm / (rad/s))
            SupplyVoltage               = input_params[6]; // Power Supply Voltage (V)

            angle_init1                 = input_params[7]; // Initial angle for q1 (rad)
            angle_init2                 = input_params[8];// Initial angle for q2 (rad)

            Kp                          = input_params[9]; // Proportional current gain (V/A)
            Ki                          = input_params[10]; // Ki integration gain of current control
            K_xx                        = input_params[11]; // Foot stiffness N/m
            K_yy                        = input_params[12]; // Foot stiffness N/m
            K_xy                        = input_params[13]; // Foot stiffness N/m

            D_xx                        = input_params[14]; // Foot damping N/(m/s)
            D_yy                        = input_params[15]; // Foot damping N/(m/s)
            D_xy                        = input_params[16]; // Foot damping N/(m/s)
                        
            xSetFoot                    = input_params[17]; // Foot position set point x (m)
            ySetFoot                    = input_params[18]; // Foot position set point y (m)
            A                           = input_params[19]; // Magnitude of oscillation (m)
            omega                       = input_params[20]; // Frequency of oscillation (Hz)

            duty_max                    = input_params[21]; // Maximum duty of PWM


            // Setup experiment
            angle1 = 0.0f;
            angle2 = 0.0f;
            encoder1.reset();
            encoder2.reset();
            current_error_int1 = 0.0f; // Reset integration of current error
            current_error_int2 = 0.0f; // Reset integration of current error
            setMotorVoltage(0,M1INA,M1INB,M1PWM); //Turn off motor just in case
            setMotorVoltage(0,M2INA,M2INB,M2PWM); //Turn off motor just in case
            // Set high-frequency (low-level) current control loop
            // 0.0002 gives the loop period, which means current loop is under 5kHz
            currentLoopTicker.attach_us(&currentLoopFunc,current_control_period_us);

            float softStart = 0.0; // Soft start scalar that prevent the sudden motion at the begining

            t.reset(); // Reset timer
            t.start(); // Start timer so that we have elapsed time of experiment

            // Run experiment
            while( t.read() < ExpTime ) { 
                // Soft start: fully actuation after 2 seconds
                softStart = min(0.5*t.read(),1.0);

                // Control code HERE
                const float th1 = angle1;
                const float th2 = angle2;
                const float dth1= velocity1;
                const float dth2= velocity2;

                
                // Forward kinematics
                // Foot tip position
                float xLeg =   l_AC*sin(th1 + th2) + l_DE*sin(th1) + l_OB*sin(th1);
                float yLeg = - l_AC*cos(th1 + th2) - l_DE*cos(th1) - l_OB*cos(th1);

                // Jacobian
                float Jx_th1 = l_AC*cos(th1 + th2) + l_DE*cos(th1) + l_OB*cos(th1);
                float Jx_th2 = l_AC*cos(th1 + th2);
                float Jy_th1 = l_AC*sin(th1 + th2) + l_DE*sin(th1) + l_OB*sin(th1);
                float Jy_th2 = l_AC*sin(th1 + th2);

                // Foot tip velocity
                float dxLeg = Jx_th1 * dth1 + Jx_th2 * dth2;
                float dyLeg = Jy_th1 * dth1 + Jy_th2 * dth2;

                // Desired foot position
                float xd = A*sin(2.0*PI*omega*t.read())+ xSetFoot;
                float yd = A*cos(2.0*PI*omega*t.read())+ ySetFoot;
                float e_x = ( xLeg - xd);
                float e_y = ( yLeg - yd);
                
                float dxd = 2.0*PI*omega*A*cos(2.0*PI*omega*t.read());
                float dyd = -2.0*PI*omega*A*sin(2.0*PI*omega*t.read());
                float de_x = ( dxLeg - dxd);
                float de_y = ( dyLeg - dyd);

                float fx   = -K_xx * e_x - K_xy * e_y - D_xx * de_x -D_xy * de_y;
                float fy   = -K_yy * e_y - K_xy * e_x - D_yy * de_y -D_xy * de_x;

                
                // Use jacobian to transform virtual force to torques
                float tau_des1 = kv*velocity1 + Jx_th1*fx + Jy_th1*fy;
                float tau_des2 = kv*velocity2 + Jx_th2*fx + Jy_th2*fy;
                
                // Set desired currents                
                current_des1 = softStart*tau_des1/kb;
                current_des2 = softStart*tau_des2/kb;
                
                // Fill the output data to send back to MATLAB
                float output_data[NUM_OUTPUTS];
                output_data[0] = t.read();
                output_data[1] = angle1;
                output_data[2] = velocity1;  
                output_data[3] = current1;
                output_data[4] = current_des1;
                output_data[5] = motor_voltate1;
                
                output_data[6] = angle2;
                output_data[7] = velocity2;
                output_data[8] = current2;
                output_data[9] = current_des2;
                output_data[10]= motor_voltate2;

                output_data[11] = xLeg;
                output_data[12] = yLeg;
                output_data[13] = dxLeg;
                output_data[14] = dyLeg;
                output_data[15] = fx;
                output_data[16] = fy;

                output_data[17] = xd;
                output_data[18] = yd;
                output_data[19] = dxd;
                output_data[20] = dyd;
                
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);
                wait_us(impedance_control_period_us); // Running high-level control loop and sending data in 
                                                      // roughly (1e6/impedance_control_period_us) Hz
            } // end of high-level experiment loop

            // Cleanup after experiment
            currentLoopTicker.detach();
            server.setExperimentComplete();
            setMotorVoltage(0,M1INA,M1INB,M1PWM);
            setMotorVoltage(0,M2INA,M2INB,M2PWM);
        } // end if of "check whether we have parameter"
    } // end while of "infinite while loop"
} // end main

/* Current controller function (low-level control)
 * This function would be called by the Ticker:currentLoopTicker under 5kHz
 * This function reads motor state (e.g. angle, current), and set motor voltage
*/
void currentLoopFunc(){
    
    // Read the current sensor value
    current1 = 36.666667f * (CS1 - 0.5f);
    current2 = 36.666667f * (CS2 - 0.5f);
    
    // Read angle and velocity from encoder
    angle1    = (float)encoder1.getPulses()   * radPerTick + angle_init1;   // in rad
    velocity1 =        encoder1.getVelocity() * radPerTick;                 // in rad/s
    angle2    = (float)encoder2.getPulses()   * radPerTick + angle_init2;   // in rad
    velocity2 =        encoder2.getVelocity() * radPerTick;                 // in rad/s

    // Integrate the current errors
    current_error_int1 += current_des1 - current1;
    current_error_int2 += current_des2 - current2;

    // Current controller: compute command voltage on motor
    motor_voltate1 = Rm * current_des1 + kb * velocity1 + Kp*(current_des1 - current1) + Ki*current_error_int1;
    motor_voltate2 = Rm * current_des2 + kb * velocity2 + Kp*(current_des2 - current2) + Ki*current_error_int2;
    
    setMotorVoltage(motor_voltate1,M1INA,M1INB,M1PWM);
    setMotorVoltage(motor_voltate2,M2INA,M2INB,M2PWM);

}

//Set motor voltage (nagetive means reverse)
void setMotorVoltage(float voltage, DigitalOut &INA, DigitalOut &INB, FastPWM &PWM){
    setMotorDuty(voltage / SupplyVoltage, INA, INB, PWM);
}

// Set motor duty [-1.0f, 1.0f]
void setMotorDuty(float duty, DigitalOut &INA, DigitalOut &INB, FastPWM &PWM)
{
    unsigned char reverse = 0;

    if (duty < 0) {
        duty = -duty;  // Make duty a positive quantity
        reverse = 1;  // Preserve the direction
    }

    if (duty == 0) {
        INA = 0;  // Make the motor coast no
        INB = 0;  // matter which direction it is spinning.
    } else if (reverse) {
        INA = 0;
        INB = 1;
    } else {
        INA = 1;
        INB = 0;
    }

    // The standard way to set built-in PWM is: 
    // PWM.write(duty);
    // However, we are using a high resolution PWM (FastPWM library). 
    // We have to set the PWM is the following way.
    duty = min(duty,duty_max);
    PWM.pulsewidth_ticks((int) (PWMperiodTicks*duty));
}