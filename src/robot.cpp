#include "robot.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/optical.hpp"

Controller controller(E_CONTROLLER_MASTER);
MotorGroup left({-19, -13, -2}, MotorGearset::blue);
MotorGroup right({-19, -13, -2}, MotorGearset::blue);
Motor intake(10, MotorGearset::red);
Motor ladyBrown(10, MotorGearset::red);
ADIDigitalOut clamp('A');
ADIDigitalOut intakePiston('B');
ADIDigitalOut doinker('C');
Imu imu(10);
Rotation hTracking(10);
Rotation vTracking(8);
Rotation armSensor(8);
Optical color(8);

lemlib::TrackingWheel horizontal_tracking_wheel(&hTracking, lemlib::Omniwheel::NEW_2, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vTracking, lemlib::Omniwheel::NEW_2, -5.75);

Drivetrain drivetrain (
    &left,
    &right,
    12.25,
    Omniwheel::NEW_325,
    360,
    2
);

OdomSensors sensors(
    &vertical_tracking_wheel,
    nullptr,
    &horizontal_tracking_wheel,
    nullptr,
    &imu
);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);