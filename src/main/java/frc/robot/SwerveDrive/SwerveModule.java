// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShuffleboardManager;

import static frc.robot.Constants.*;


public class SwerveModule extends SubsystemBase 
{ 
  /** Creates a new SwerveModule. */
  private int moduleNum;
  public final TalonFX m_turningMotor;
  public final TalonFX m_driveMotor;
  
  public final CANCoder e_Encoder;
 
  private double driveMotorOutput;
  private double turningMotorOutput;

  private final PIDController driveMotorPID = new PIDController(0, 0, 0);
  private final PIDController turnMotorPID = new PIDController(0, 0, 0);

  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0,0);
  private SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0, 0, 0);

  private boolean drive_inverted;
  private boolean turn_inverted;
  
  Pose2d swerveModulePose = new Pose2d();
  //constructor 
  public SwerveModule(int moduleNum, TalonFX driveMotor, TalonFX turningMotor, CANCoder encoder, boolean invertDrive, boolean invertTurn) 
  {
    this.moduleNum = moduleNum;
    m_driveMotor = driveMotor;
    m_turningMotor = turningMotor;
    e_Encoder = encoder;
    
    //reset encoders
    resetEncoders();

    this.drive_inverted = invertDrive;
    this.turn_inverted = invertTurn;

    driveMotor.setNeutralMode(NeutralMode.Brake);
    turningMotor.setNeutralMode(NeutralMode.Coast);

    driveMotor.clearStickyFaults();
    turningMotor.clearStickyFaults();

    //set the PID controller input range form -pi to pi
    turnMotorPID.enableContinuousInput(-Math.PI, Math.PI);

    // Set I term bounds of drive PID to full motor output range.
    driveMotorPID.setIntegratorRange(-C_MAX_VOLTAGE, C_MAX_VOLTAGE);
  }

  //set encoder position of both motors to 0
  public void resetEncoders()
  {
    m_turningMotor.setSelectedSensorPosition(0);
    m_driveMotor.setSelectedSensorPosition(0);
  }

  public Rotation2d getHeading()
  {
    return new Rotation2d(getTurningRadians());
  }

  public double getTurningRadians() 
  {
    //for without encoders
    //return 2*Math.PI * m_turningMotor.getSelectedSensorPosition()/(Constants.SwerveModuleConstants.C_kENCODER_CPR * Constants.SwerveModuleConstants.C_kTURNING_MOTOR_GEAR_RATIO);
    return Math.toRadians(e_Encoder.getAbsolutePosition());
  }

  public double getTurnAngle() 
  {
    return Units.radiansToDegrees(getTurningRadians());
  }

  public double getVelocity()
  {
    return m_driveMotor.getSelectedSensorVelocity() * C_DRIVE_ENCODER_DISTANCE_PER_PULSE * 10;
  }

  public SwerveModuleState getState()
  {
    return new SwerveModuleState(getVelocity(), new Rotation2d(getTurningRadians()));
  }

  public void setDesiredState(SwerveModuleState state)
  {
    SwerveModuleState outputState = SwerveModuleState.optimize(state, new Rotation2d(getTurningRadians()));

    double drive_vel = getVelocity();
    driveMotorOutput = driveMotorPID.calculate(drive_vel, outputState.speedMetersPerSecond);
    turningMotorOutput = turnMotorPID.calculate(getTurningRadians(), outputState.angle.getRadians());
    
    double driveFeedforward = this.driveFeedforward.calculate(outputState.speedMetersPerSecond);
    double turnFeedforward = this.turnFeedforward.calculate(outputState.angle.getRadians());

    m_driveMotor.set(ControlMode.PercentOutput, (this.drive_inverted ? -1 : 1) * (driveFeedforward + driveMotorOutput) / C_MAX_VOLTAGE);
    m_turningMotor.set(ControlMode.PercentOutput, (this.turn_inverted ? -1 : 1) * (turningMotorOutput + turnFeedforward) / C_MAX_VOLTAGE);

    if (isTestingTurning) {
      double error = outputState.angle.getRadians() - m_turningMotor.getSelectedSensorPosition()/2048.0*2*Math.PI;
      if (Math.abs(error) > 10)
        ShuffleboardManager.updateDouble("error", error);
    }
    else
      ShuffleboardManager.updateDouble("error", outputState.speedMetersPerSecond - drive_vel);


    //testing the correct motor output
//    System.out.println(
//      System.currentTimeMillis() + ", " +
//      outputState.speedMetersPerSecond+ ", " +
//      drive_vel + ", " +
//      driveMotorOutput);
  }

  public void setBrakeMode(boolean mode) 
  { // True is brake, false is coast
    m_driveMotor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);
  }
  
  public Pose2d getPose() 
  {
    return swerveModulePose;
  }

  public void setPose(Pose2d pose) 
  {
    swerveModulePose = pose;
  }

  public void setPID(double p, double i, double d) {
    if (isTestingTurning)
      turnMotorPID.setPID(p, i, d);
    else
      driveMotorPID.setPID(p, i, d);
  }

  public void setFeedForward(double a, double s, double v) {
    if (isTestingTurning)
      turnFeedforward = new SimpleMotorFeedforward(a, s ,v);
    else
      driveFeedforward = new SimpleMotorFeedforward(a, s ,v);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
