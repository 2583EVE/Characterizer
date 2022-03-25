package frc.robot;

public class Constants {

    public static boolean isTestingTurning = true;

    public static final double C_MAX_SPEED = 6, // arbitrary number afaik
            C_MAX_ANGULAR_SPEED = 1.3 * Math.PI;

    //chassis constant
    public static final double C_DISTANCE_FROM_CENTER_WIDTH = 0.4953/2.0,
            C_DISTANCE_FROM_CENTER_LENGTH = 0.6477/2.0; //meters

    //module constants
    public static final double C_DRIVE_MOTOR_GEAR_RATIO = 6.75,
            C_TURNING_MOTOR_GEAR_RATIO = 12.8,
            C_WHEELS_DIAMETER = 0.1, //meters
            C_MAX_VOLTAGE = 12;

    public static final int C_ENCODER_CPR = 2048;

    public static final double C_DRIVE_ENCODER_DISTANCE_PER_PULSE = (C_WHEELS_DIAMETER * Math.PI) / ((double) C_ENCODER_CPR * C_DRIVE_MOTOR_GEAR_RATIO),
            C_kTURNING_ENCODER_DISTANCE_PER_PULSE = (2.0 * Math.PI) / (C_ENCODER_CPR * C_TURNING_MOTOR_GEAR_RATIO); // Assumes the encoders are on a 1:1 reduction with the module shaft.

    //motor constants
    public static final double C_MAX_MOTOR_ANGULAR_SPEED = 0.02 * 2 * Math.PI, //radians per seconds
            C_MAX_MOTOR_ANGULAR_ACCELERATION = 0.02 * 2 * Math.PI, //radians per seconds sqaured
            C_EDGES_PER_REVOLUTION = 2048; //for use in characterization

    // CAN ports
    public static final int P_FRONT_RIGHT_TURN = 11,  //1
            P_FRONT_RIGHT_DRIVE = 12, //2
            P_FRONT_LEFT_TURN = 13,  //3
            P_FRONT_LEFT_DRIVE = 14,  //4
            P_REAR_LEFT_TURN = 15,    //5
            P_REAR_LEFT_DRIVE = 16,   //6
            P_REAR_RIGHT_TURN = 17,   //7
            P_REAR_RIGHT_DRIVE= 18;   //8

    //CANcoder ports
    public static final int P_FRONT_RIGHT_ENCODER = 1,
            P_FRONT_LEFT_ENCODER = 2,
            P_BACK_RIGHT_ENCODER = 4,
            P_BACK_LEFT_ENCODER = 3;
}
