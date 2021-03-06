// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveDrive;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShuffleboardManager;

import static frc.robot.Constants.*;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  private final AHRS imu = new AHRS();

  private final Translation2d m_frontRightLocation = new Translation2d(C_DISTANCE_FROM_CENTER_WIDTH, C_DISTANCE_FROM_CENTER_LENGTH),
                              m_frontLeftLocation = new Translation2d(-C_DISTANCE_FROM_CENTER_WIDTH, C_DISTANCE_FROM_CENTER_LENGTH),
                              m_rearLeftLocation = new Translation2d(-C_DISTANCE_FROM_CENTER_WIDTH, -C_DISTANCE_FROM_CENTER_LENGTH),
                              m_rearRightLocation = new Translation2d(C_DISTANCE_FROM_CENTER_WIDTH, -C_DISTANCE_FROM_CENTER_LENGTH);

  private final TalonFX frontRightDriveMotor = new TalonFX(P_FRONT_RIGHT_DRIVE),
                        frontRightTurnMotor = new TalonFX(P_FRONT_RIGHT_TURN),
                        frontLeftDriveMotor = new TalonFX(P_FRONT_LEFT_DRIVE),
                        frontLeftTurnMotor = new TalonFX(P_FRONT_LEFT_TURN),
                        rearLeftDriveMotor = new TalonFX(P_REAR_LEFT_DRIVE),
                        rearLeftTurnMotor = new TalonFX(P_REAR_LEFT_TURN),
                        rearRightDriveMotor = new TalonFX(P_REAR_RIGHT_DRIVE),
                        rearRightTurnMotor = new TalonFX(P_REAR_RIGHT_TURN);

  // CANCoders move counter-clockwise from the top.
  private final CANCoder  frontRightEncoder = new CANCoder(P_FRONT_RIGHT_ENCODER),
                          frontLeftEncoder = new CANCoder(P_FRONT_LEFT_ENCODER),
                          backLeftEncoder = new CANCoder(P_BACK_LEFT_ENCODER),
                          backRightEncoder = new CANCoder(P_BACK_RIGHT_ENCODER);

  // Modules arranged in coordinate grid space
  private final SwerveModule  m_frontRight = new SwerveModule(0, frontRightDriveMotor, frontRightTurnMotor, frontRightEncoder, false, false),
                              m_frontLeft = new SwerveModule(1, frontLeftDriveMotor, frontLeftTurnMotor, frontLeftEncoder, false, false),
                              m_rearLeft = new SwerveModule(2, rearLeftDriveMotor, rearLeftTurnMotor, backLeftEncoder, false, true),
                              m_rearRight = new SwerveModule(3, rearRightDriveMotor, rearRightTurnMotor, backRightEncoder, false, false);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontRightLocation,
      m_frontLeftLocation, m_rearLeftLocation, m_rearRightLocation);
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, imu.getRotation2d());

  private double speedMulti = 1;

  public SwerveDrive() {
    imu.reset();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, imu.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, C_MAX_SPEED);
    m_frontRight.setDesiredState(swerveModuleStates[0]);
    m_frontLeft.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

  }

  // Testing isolated turning
  public void turn(int dir, double speed) {
    m_frontRight.setDesiredState(new SwerveModuleState(dir * -speed, Rotation2d.fromDegrees(45)));
    m_frontLeft.setDesiredState(new SwerveModuleState(dir * speed, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(dir * -speed, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(dir * speed, Rotation2d.fromDegrees(45)));
  }

  // Testing more isolated turning
  public void turn(double speed) {
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, speed);
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    m_frontRight.setDesiredState(moduleStates[0]);
    m_frontLeft.setDesiredState(moduleStates[1]);
    m_rearLeft.setDesiredState(moduleStates[2]);
    m_rearRight.setDesiredState(moduleStates[3]);
  }

  // Testing isolated x and y
  public void translate(double x, double y) {
    ChassisSpeeds speeds = new ChassisSpeeds(x, y, 0);
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    m_frontLeft.setDesiredState(moduleStates[1]);
    m_frontRight.setDesiredState(moduleStates[0]);
    m_rearLeft.setDesiredState(moduleStates[2]);
    m_rearRight.setDesiredState(moduleStates[3]);

  }

  // For Dylan's testing code. Sets wheels to zero position (within 180 instead of 360 degrees)
  public void zeroOut() {
    m_frontLeft.setDesiredState(new SwerveModuleState());
    m_frontRight.setDesiredState(new SwerveModuleState());
    m_rearLeft.setDesiredState(new SwerveModuleState());
    m_rearRight.setDesiredState(new SwerveModuleState());
  }

  public void updateOdometry() {
      m_odometry.update(
          imu.getRotation2d(),
          m_frontRight.getState(),
          m_frontLeft.getState(),
          m_rearLeft.getState(),
          m_rearRight.getState());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeedMulti(double num) {
    speedMulti = num;
  }

  public double getSpeedMulti() {
    return speedMulti;
  }

  public void updatePID(double p, double i, double d) {
    m_frontLeft.setPID(p, i, d);
    m_frontRight.setPID(p, i, d);
    m_rearLeft.setPID(p, i, d);
    m_rearRight.setPID(p, i, d);
  }

  public void updateFeedForward(double a, double s, double v) {
    m_frontLeft.setFeedForward(a, s, v);
    m_frontRight.setFeedForward(a, s, v);
    m_rearLeft.setFeedForward(a, s, v);
    m_rearRight.setFeedForward(a, s, v);

  }

}