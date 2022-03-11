/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class Lift extends SubsystemBase {

  private static final CANSparkMax liftSparkMax = new CANSparkMax(Constants.LiftConstants.LiftSparkMax, MotorType.kBrushless);
  private final SparkMaxPIDController pidController;
  private final RelativeEncoder encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private boolean firstLift = true;

  public Lift() {
    
    liftSparkMax.restoreFactoryDefaults();
    pidController = liftSparkMax.getPIDController();
    encoder = liftSparkMax.getEncoder();

    // PID coefficients
    kP = 0.025;
    kI = 0;
    kD = 0.001; 
    kIz = 0; 
    kFF = 0.04; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    liftSparkMax.setSoftLimit(SoftLimitDirection.kForward, Constants.LiftConstants.Home);
    liftSparkMax.setSoftLimit(SoftLimitDirection.kReverse, Constants.LiftConstants.Top);

    liftSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    liftSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);

    liftSparkMax.setSmartCurrentLimit(40, 20, 10);
    liftSparkMax.clearFaults();
    liftSparkMax.enableVoltageCompensation(12);
    liftSparkMax.setIdleMode(IdleMode.kBrake);
    liftSparkMax.setClosedLoopRampRate(.25);
    liftSparkMax.setSecondaryCurrentLimit(95, 250);

    encoder.setPosition(Constants.LiftConstants.Home);
    SmartDashboard.putNumber("Lift position", encoder.getPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lift position", encoder.getPosition());
  }

  public void upPosition() {
    this.setPosition(Constants.LiftConstants.Top);
  }

  public void downPosition() {
    this.setPosition(Constants.LiftConstants.Home);
  }

  private void setPosition(double position) {
    pidController.setReference(position, ControlType.kPosition);
  }

  public void stop() {
    liftSparkMax.set(0.0);
  }

  public void down() {
    liftSparkMax.set(1);
  }

  public void up() {
    liftSparkMax.set(-1);
    firstLift = false;

  }

}