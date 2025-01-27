#include "robot.h"
#include "main.h"
#include <cmath>
#include <string>

/**
 * PID Implementation for ladybrown for accurate, smooth, macro control
 */

//arm pid constants
const double armkP = 0.5;
const double armkI = 0;
const double armkD = 0;

//parameters
const double maxVoltage = 127;
const double minVoltage = -127;
const double tolerance = 1.0;

double restrict(double value, double min, double max) {
    return fmax(min, fmin(value, max));
}

void arm_control(double targetAngle){
    double error = 0;
    double lastError = 0;
    double integral = 0;
    double derivative = 0;
    double power = 0;

    while (true) {
        // Read the current angle from the rotational sensor
        double currentAngle = armSensor.get_angle()/100.0;

        // Calculate PID components
        error = targetAngle - currentAngle;
        integral += error;
        derivative = error - lastError;

        // Calculate motor power using PID formula
        power = armkP * error + armkI * integral + armkD * derivative;

        // Clamp power to within motor limits
        power = restrict(power, minVoltage, maxVoltage);

        // Set motor power
        ladyBrown.move(power);

        // Break if error is within tolerance
        if (fabs(error) < tolerance) {
            ladyBrown.move(0); // Stop the motor
            break;
        }

        // Update last error
        lastError = error;

        // Delay to prevent overloading the CPU
        pros::delay(20);
    }
}

void intake_forward(){

}

void intake_backward(){

}

void intake_auton(){

}

/**
 * Color sort based on color we want to sort too
 */
std::string detect_color(){
    double hue = color.get_hue();
    double red = 100.0;
    double blue = 100.0;
    if(color.get_proximity() < 100){
        if(hue > blue){
            return "Blue";
        }
        else{
            return "Red";
        }
    }
}

void color_sort(std::string target){
    while(true){
        intake.move(127);

        std::string color = detect_color();
        if(color.compare(target) == 0){
            intake.move(0);

            pros::delay(500);
        }

        pros::delay(50);
    }
}

void intake_piston(){

}

void doinker_control(){

}