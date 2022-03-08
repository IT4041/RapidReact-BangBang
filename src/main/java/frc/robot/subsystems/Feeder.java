// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Feeder extends SubsystemBase {

  private static final CANSparkMax sparkMaxFeeder = new CANSparkMax(Constants.IntakeConstants.IntakeFeederSparkMax, MotorType.kBrushless); 

  /** Creates a new Feeder. */
  public Feeder() {

    sparkMaxFeeder.restoreFactoryDefaults();
    sparkMaxFeeder.clearFaults();
    sparkMaxFeeder.setInverted(false);
    //sparkMaxFeeder.setSmartCurrentLimit(20, 95, 10);
    sparkMaxFeeder.enableVoltageCompensation(12);
    sparkMaxFeeder.setIdleMode(IdleMode.kCoast);
    sparkMaxFeeder.setClosedLoopRampRate(1.0);
    //sparkMaxFeeder.setSecondaryCurrentLimit(95, 250);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void on(){
    sparkMaxFeeder.set(0.7);
  }

  public void reverse(){
    sparkMaxFeeder.set(-0.7);
  }

  public void off(){
    sparkMaxFeeder.set(0.0);
  }
}
