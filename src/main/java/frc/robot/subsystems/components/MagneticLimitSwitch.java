// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class MagneticLimitSwitch extends SubsystemBase {
  /** Creates a new MagneticLimitSwitch. */

  private DigitalInput limitSwitch = new DigitalInput(Constants.LimitSwitchConstants.ElbowLimitSwitch);
  private boolean triggered = false;

  public MagneticLimitSwitch() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.triggered = this.getLimitSwitchState();
  }

  private boolean getLimitSwitchState() {
    return !limitSwitch.get();
  }

  public boolean isTriggered(){
    return triggered;
  }

}