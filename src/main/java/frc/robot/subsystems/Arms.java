// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arms extends SubsystemBase {

  private static final CANSparkMax sparkMaxArms = new CANSparkMax(Constants.ArmsConstants.ArmsSparkMax, MotorType.kBrushless); 
  private final SparkMaxPIDController pidController;
  private final RelativeEncoder encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  
  public Arms() {

    sparkMaxArms.restoreFactoryDefaults();
    pidController = sparkMaxArms.getPIDController();
    encoder = sparkMaxArms.getEncoder();
    
    // PID coefficients
    kP = 0.025;
    kI = 0;
    kD = 0.0001; 
    kIz = 0; 
    kFF = 0.025; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    sparkMaxArms.setSoftLimit(SoftLimitDirection.kForward, Constants.ArmsConstants.Back);
    sparkMaxArms.setSoftLimit(SoftLimitDirection.kReverse, Constants.ArmsConstants.Forward);

    sparkMaxArms.enableSoftLimit(SoftLimitDirection.kForward, true);
    sparkMaxArms.enableSoftLimit(SoftLimitDirection.kReverse, true);

    sparkMaxArms.setSmartCurrentLimit(40, 20, 10);
    sparkMaxArms.clearFaults();
    sparkMaxArms.enableVoltageCompensation(12);
    sparkMaxArms.setIdleMode(IdleMode.kBrake);
    sparkMaxArms.setClosedLoopRampRate(.25);
    sparkMaxArms.setSecondaryCurrentLimit(95, 250);

    encoder.setPosition(Constants.ArmsConstants.Home);
    SmartDashboard.putNumber("Arm position", encoder.getPosition());
    SmartDashboard.putNumber("Arm target", Constants.ArmsConstants.Home);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm position", encoder.getPosition());
  }

  public void home(){
    this.setPosition(Constants.ArmsConstants.Home);
    SmartDashboard.putNumber("Arm target", Constants.ArmsConstants.Home);
  }

  public void forward(){
    this.setPosition(Constants.ArmsConstants.Forward);
    SmartDashboard.putNumber("Arm target", Constants.ArmsConstants.Forward);
  }

  public void back(){
    this.setPosition(Constants.ArmsConstants.Back);
    SmartDashboard.putNumber("Arm target", Constants.ArmsConstants.Back);
  }

  private void setPosition(double position) {

    SmartDashboard.putNumber("Arm target", position);
    pidController.setReference(position, ControlType.kPosition);
  }

  public void stop() {
    sparkMaxArms.set(0.0);
  }

}
