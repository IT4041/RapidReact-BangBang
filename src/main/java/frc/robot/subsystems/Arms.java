// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.components.MagneticLimitSwitches;

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
  private MagneticLimitSwitches m_limit;
  private boolean atlimitBack = false;
  private boolean atlimitForward = false;
  private boolean backwards = true;

  public Arms(MagneticLimitSwitches in_MagSwitch) {

    m_limit = in_MagSwitch;
    sparkMaxArms.restoreFactoryDefaults();
    pidController = sparkMaxArms.getPIDController();
    encoder = sparkMaxArms.getEncoder();
    
    // PID coefficients
    kP = 0.0252;
    kI = 0;
    kD = 0.0001; 
    kIz = 0; 
    kFF = 0.025; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    atlimitBack = !m_limit.isTriggeredArmBack();

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    //sparkMaxArms.setSoftLimit(SoftLimitDirection.kForward, Constants.ArmsConstants.Forward);
    sparkMaxArms.setSoftLimit(SoftLimitDirection.kReverse, Constants.ArmsConstants.Reverse);

    //sparkMaxArms.enableSoftLimit(SoftLimitDirection.kForward, true);
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

    atlimitBack = m_limit.isTriggeredArmBack();
    if(atlimitBack && backwards){
      this.stop();
    }

    atlimitForward = m_limit.isTriggeredArmForward();
    if(atlimitForward && !backwards){
      this.stop();
    }
  }

  public void homePosition(){
    this.backwards = false;
    this.setPosition(Constants.ArmsConstants.Home);
    SmartDashboard.putNumber("Arm target", Constants.ArmsConstants.Home);
  }

  private void setPosition(double position) {
    SmartDashboard.putNumber("Arm target", position);
    pidController.setReference(position, ControlType.kPosition);
  }

  public void stop() {
    sparkMaxArms.set(0.0);
  }

  public void back() {
    backwards = true;

    atlimitBack = m_limit.isTriggeredArmBack();
    if(atlimitBack){
      this.stop();
    }else{
      sparkMaxArms.set(1);
    } 
  }

  public void forward() {
    backwards = false;

    atlimitForward = m_limit.isTriggeredArmForward();
    if(atlimitForward){
      this.stop();
    }else{
      sparkMaxArms.set(-1);
    }
  }

}
