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
import frc.robot.subsystems.components.MagneticLimitSwitch;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeElbow extends SubsystemBase {

  private CANSparkMax sparkMax;
  private SparkMaxPIDController pidController;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private RelativeEncoder encoder;
  private MagneticLimitSwitch limit;

  public IntakeElbow() {

    sparkMax = new CANSparkMax(Constants.IntakeConstants.IntakeElbowSparkMax, MotorType.kBrushless);
    sparkMax.restoreFactoryDefaults();
    pidController = sparkMax.getPIDController();
    encoder = sparkMax.getEncoder();
    limit = new MagneticLimitSwitch();

    // PID coefficients
    kP = 0.013;
    kI = 0;
    kD = 0.005;
    kIz = 0;
    kFF = 0.001;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    // sparkMax.setSoftLimit(SoftLimitDirection.kForward, kForwardSoftLimit);
    // sparkMax.setSoftLimit(SoftLimitDirection.kReverse, kReverseSoftLimit);

    // sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    // sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);

    sparkMax.setSmartCurrentLimit(15, 50, 10);
    sparkMax.clearFaults();
    sparkMax.enableVoltageCompensation(12);
    sparkMax.setIdleMode(IdleMode.kBrake);
    sparkMax.setClosedLoopRampRate(1.0);
    sparkMax.setSecondaryCurrentLimit(95, 250);
    
    if(limit.isTriggered()){
      encoder.setPosition(Constants.IntakeConstants.Down);
    }else{
      encoder.setPosition(Constants.IntakeConstants.Home);
    }
    
    SmartDashboard.putNumber("elbow position", encoder.getPosition());

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elbow position", encoder.getPosition());
  }

  public void up() {
    this.setPosition(Constants.IntakeConstants.Home);
  }

  public void down() {
    this.setPosition(Constants.IntakeConstants.Down);
  }

  private void setPosition(double position) {
    pidController.setReference(position, ControlType.kPosition);
  }

}
