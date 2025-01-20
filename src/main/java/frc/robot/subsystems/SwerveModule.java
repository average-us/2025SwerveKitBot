// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  private TalonFX driveMotor;
  private PearadoxSparkMax turnMotor;

  private RelativeEncoder turnEncoder;

  private PIDController turnPIDController;
  private CANcoder absoluteEncoder;

  private boolean absoluteEncoderReversed;
  private double absoluteEncoderOffset;

  private Rotation2d lastAngle;

  private DutyCycleOut driveMotorRequest = new DutyCycleOut(0.0);

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
      this.absoluteEncoderOffset = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;

      absoluteEncoder = new CANcoder(absoluteEncoderId);

      driveMotor = new PearadoxTalonFX(driveMotorId, NeutralModeValue.Brake, 30, driveMotorReversed);
      turnMotor = new PearadoxSparkMax(turnMotorId, MotorType.kBrushless, IdleMode.kCoast, 25, turnMotorReversed);

      turnEncoder = turnMotor.getEncoder();

      //CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
      //currentLimitsConfigs.withSupplyCurrentLimitEnable(true);
      //currentLimitsConfigs.withSupplyCurrentLimit(30);
      //currentLimitsConfigs.withSupplyTimeThreshold(30);
      //currentLimitsConfigs.withSupplyTimeThreshold(1);

      //driveMotor.getConfigurator().apply(currentLimitsConfigs);

      turnPIDController = new PIDController(SwerveConstants.KP_TURNING, 0, 0);
      turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

      resetEncoders();
      lastAngle = getState().angle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setBrake(boolean brake){
    if(brake){
      driveMotor.setNeutralMode(NeutralModeValue.Brake);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
    else{
      driveMotor.setNeutralMode(NeutralModeValue.Coast);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public double getDriveMotorPosition(){
    return driveMotor.getPosition().getValueAsDouble() * SwerveConstants.DRIVE_MOTOR_PCONVERSION;
  }

  public double getDriveMotorVelocity(){
    return driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
  }

  public double getTurnMotorPosition(){
    return turnEncoder.getPosition() * SwerveConstants.TURN_MOTOR_PCONVERSION;
  }

  public double getTurnMotorVelocity(){
    return turnEncoder.getVelocity() * SwerveConstants.TURN_MOTOR_VCONVERSION;
  }

  public double getAbsoluteEncoderAngle(){
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    angle -= absoluteEncoderOffset;
    angle *= (2 * Math.PI);
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    driveMotor.setPosition(0);
    turnEncoder.setPosition(getAbsoluteEncoderAngle() / SwerveConstants.TURN_MOTOR_PCONVERSION);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getTurnMotorPosition()));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getTurnMotorPosition()));
  }
  
  public void stop(){
    driveMotor.setControl(driveMotorRequest.withOutput(0));
    turnMotor.set(0);
  }

  public void setDesiredState(SwerveModuleState desiredState){
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState.optimize(getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState);
  }

  private void setSpeed(SwerveModuleState desiredState){
    driveMotor.setControl(driveMotorRequest.withOutput(desiredState.speedMetersPerSecond / SwerveConstants.DRIVETRAIN_MAX_SPEED));

    // else {
    //     double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
    //     mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
    // }
  }

  private void setAngle(SwerveModuleState desiredState){
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.DRIVETRAIN_MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    
    turnMotor.set(turnPIDController.calculate(getTurnMotorPosition(), desiredState.angle.getRadians()));
    lastAngle = angle;
  }
}
