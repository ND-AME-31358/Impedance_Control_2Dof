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
#define NUM_INPUTS 21
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

const float SupplyVoltage = 12;     // Supply voltage in Volts
// Set motor voltage in (V)
void setMotorVoltage(float voltage, DigitalOut &INA, DigitalOut &INB, FastPWM &PWM);

// Declaration of current current control function.
// Please refer to the latter part of this file for its definition (implementation).
void currentLoopFunc();

// Declare global variables

// Variables for joint 1 (Hip)
float current1;           // Current on motor 1
float current_des1;       // Desired current of motor 1
float current_error_int1; // Integrated current error
float angle1;             // Angle of joint 1
float angle_des1;         // Desired angle of joint 1
float velocity1;          // Angular velocity of joint 1
float velocity_des1;      // Desired angular velocity
float motor_voltage1;     // Command voltage on motor 1
float angle_init1;        // Initial angle of joint 1

// Variables for joint 2 (Knee)
float current2;           // Current on motor 2
float current_des2;       // Desired current of motor 2
float current_error_int2; // Integrated current error
float angle2;             // Angle of joint 2
float angle_des2;         // Desired angle of joint 2
float velocity2;          // Angular velocity of joint 2
float velocity_des2;      // Desired angular velocity
float motor_voltage2;     // Command voltage on motor 2
float angle_init2;        // Initial angle of joint 2

// Fixed kinematic parameters
const float l_OA=.011; // length between point O and A (m)
const float l_OB=.042; // length between point O and B (m)
const float l_AC=.096; // length between point A and C (m)
const float l_DE=.090; // length between point D and E (m)
              
// Timing parameters
float current_control_period_us;
float impedance_control_period_us;

// Control parameters
float K_xx; // Foot stiffness (N/m)
float K_yy; // Foot stiffness (N/m)
float K_xy; // Foot stiffness (N/m)

float D_xx; // Foot damping (N/(m/s))
float D_xy; // Foot damping (N/(m/s))
float D_yy; // Foot damping (N/(m/s))


float xSetFoot; // Foot position set point (m)
float ySetFoot; // Foot position set point (m)

float A;        // Amplitude of oscillation (m) in Part. 4
float omega;    // Frequency of oscillation (Hz) in Part. 4


// Model parameters
float Rm;       // Motor winding resistance (Ohm)
float kb;       // Back EMF constant (V/(rad/s))
float Kp,Ki;    // Current controller gains
float kv;       // Coefficient of viscous friction (Nm/(rad/s))
float duty_max; // Maximum PWM duty in range [0,1] for safety

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
    bool first_run = true;
    
    // infinite while loop, analogous to Arduino's loop function
    while(1) {
        if (server.getParams(input_params,NUM_INPUTS)) {
            // Unpack parameters from MATLAB
            
            current_control_period_us   = input_params[0]; // Current control period in micro seconds
            impedance_control_period_us = input_params[1]; // Impedance control period in microseconds
            float ExpTime               = input_params[2]; // Expriement time in second

            Rm                          = input_params[3]; // Motor winding resistance (Ohms)
            kb                          = input_params[4]; // Back EMF Constant (V / (rad/s))
            kv                          = input_params[5]; // Friction coefficienct (Nm / (rad/s))
            
            float angle_init1_cmd       = input_params[6]; // Initial angle for joint 1 (rad)
            float angle_init2_cmd       = input_params[7]; // Initial angle for joint 2 (rad)

            Kp                          = input_params[8]; // Kp Proportional current gain (V/A)
            Ki                          = input_params[9]; // Ki integration gain (V/As)
            K_xx                        = input_params[10]; // Foot stiffness N/m
            K_yy                        = input_params[11]; // Foot stiffness N/m
            K_xy                        = input_params[12]; // Foot stiffness N/m

            D_xx                        = input_params[13]; // Foot damping N/(m/s)
            D_yy                        = input_params[14]; // Foot damping N/(m/s)
            D_xy                        = input_params[15]; // Foot damping N/(m/s)
                        
            xSetFoot                    = input_params[16]; // Foot position set point x (m)
            ySetFoot                    = input_params[17]; // Foot position set point y (m)
            A                           = input_params[18]; // Magnitude of oscillation (m)
            omega                       = input_params[19]; // Frequency of oscillation (Hz)

            duty_max                    = input_params[20]; // Maximum duty of PWM


            // Setup experiment
            if (first_run){
                encoder1.reset();
                encoder2.reset();
                angle_init1 = angle_init1_cmd;
                angle_init2 = angle_init2_cmd;
            }
            current_error_int1 = 0.0f; // Reset integration of current error
            current_error_int2 = 0.0f; // Reset integration of current error
            current_des1       = 0.0f; // Reset desired current
            current_des2       = 0.0f; // Reset desired current
            setMotorVoltage(0,M1INA,M1INB,M1PWM); //Turn off motor just in case
            setMotorVoltage(0,M2INA,M2INB,M2PWM); //Turn off motor just in case
            // Set high-frequency (low-level) current control loop
            // 0.0002 gives the loop period, which means current loop is under 5kHz
            currentLoopTicker.attach_us(&currentLoopFunc,current_control_period_us);
            currentLoopFunc(); // Run current controller once before report data

            float softStart = 0.0; // Soft start scalar that prevent the sudden motion at the begining

            t.reset(); // Reset timer
            t.start(); // Start timer so that we have elapsed time of experiment

            // Run experiment
            while( t.read() < ExpTime ) { 
                // Soft start: fully actuation after 2 seconds
                softStart = first_run ? min(0.5*t.read(),1.0) : 1.0;
                
                
                /* ===== Complete the code in this block =====
                 * Complate the Cartesian space impedance controller belowe
                 * Note: the variable l_OA, l_OB, l_AC, and l_DE represent 
                 * the length of OA, OB, AC, and DE respectively.
                */
                // Copy the angle and angular velocity for later use. 
                // Please do use th1/th2 and dth1/dth2 rather than angle1/angle2 
                // and velocity1/velocity2 in this code block
                const float th1 = angle1;
                const float th2 = angle2;
                const float dth1= velocity1;
                const float dth2= velocity2;

                // ******TO BE COMPLETED******
                // Forward kinematics Foot tip position 
                // Note: the variable l_OA, l_OB, l_AC, and l_DE represent 
                // the length of OA, OB, AC, and DE respectively.
                float x =   l_AC*sin(0.0);
                float y = - l_AC*cos(0.0);

                // ******TO BE COMPLETED******
                // Fix the computation of Jacobian 
                // Tips: use th1 and th2 as angle of joint 1 and 2
                float Jx_th1 = l_AC*cos(0.0) + l_DE*cos(0.0) + l_OB*cos(0.0);
                float Jx_th2 = l_AC*cos(0.0);
                float Jy_th1 = l_AC*sin(0.0) + l_DE*sin(0.0) + l_OB*sin(0.0);
                float Jy_th2 = l_AC*sin(0.0);

                // ******TO BE COMPLETED******
                // Fix the computation of foot tip velocity 
                float dx = Jx_th1 * dth1 + Jx_th1 * dth2;
                float dy = Jx_th1 * dth1 + Jx_th1 * dth2;

                // ******TO BE COMPLETED IN PART 4******
                // Assign desired foot position
                float xd = xSetFoot;    // Keep this until Part 4.1
                float yd = ySetFoot;    // Keep this until Part 4.2
                // Assign desired foot velocity
                float dxd = 0.0;    // Keep this until Part 4.3
                float dyd = 0.0;    // Keep this until Part 4.3
                
                // Compute the position error
                float e_x = ( xd - x );
                float e_y = ( yd - y );
                // Compute the velocity error
                float de_x = ( dxd - dx );
                float de_y = ( dyd - dy );

                // ******TO BE COMPLETED******
                // Fix the computation of applied virtual force at foot tip 
                float fx   = K_xx * e_x + K_xx * e_y + D_xx * de_x + K_xx * de_y;
                float fy   = K_xx * e_y + K_xx * e_x + K_xx * de_y + K_xx * de_x;

                // ******TO BE COMPLETED******
                // Fix computation of torque command:
                // Use jacobian to transform virtual force to torques 
                float tau_des1 = kv*velocity1 + Jx_th1*fx + Jx_th2*fy;
                float tau_des2 = kv*velocity2 + Jy_th1*fx + Jx_th1*fy;
                
                // Set desired currents
                current_des1 = softStart * tau_des1 / kb;
                current_des2 = softStart * tau_des2 / kb;
                /* ===== End of code block =================== */
                
                // Fill the output data to send back to MATLAB
                float output_data[NUM_OUTPUTS];
                output_data[0] = t.read();
                output_data[1] = angle1;
                output_data[2] = velocity1;  
                output_data[3] = current1;
                output_data[4] = current_des1;
                output_data[5] = motor_voltage1;
                
                output_data[6] = angle2;
                output_data[7] = velocity2;
                output_data[8] = current2;
                output_data[9] = current_des2;
                output_data[10]= motor_voltage2;

                output_data[11] = x;
                output_data[12] = y;
                output_data[13] = dx;
                output_data[14] = dy;
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
            first_run = false;
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
    
    /* ===== Complete the code in this block =====
     * Fix the angle reading from encoders
     * so that it gives the calibrated angles
     * Tip: use variable angle_init1 and angle_init2
    */
    angle1    = (float)encoder1.getPulses()   * radPerTick; // in rad
    velocity1 =        encoder1.getVelocity() * radPerTick; // in rad/s
    angle2    = (float)encoder2.getPulses()   * radPerTick; // in rad
    velocity2 =        encoder2.getVelocity() * radPerTick; // in rad/s
    /* ===== End of code block =================== */

    // Integrate the current errors
    current_error_int1 += current_des1 - current1;
    current_error_int2 += current_des2 - current2;

    // Current controller: compute command voltage on motor
    motor_voltage1 = Rm * current_des1 + kb * velocity1 + Kp*(current_des1 - current1) + Ki*current_error_int1;
    motor_voltage2 = Rm * current_des2 + kb * velocity2 + Kp*(current_des2 - current2) + Ki*current_error_int2;
    
    setMotorVoltage(motor_voltage1,M1INA,M1INB,M1PWM);
    setMotorVoltage(motor_voltage2,M2INA,M2INB,M2PWM);

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
