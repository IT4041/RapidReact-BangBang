// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MagneticLimitSwitches extends SubsystemBase {
  /** Creates a new MagneticLimitSwitch. */

  private DigitalInput ElbowlimitSwitch = new DigitalInput(Constants.LimitSwitchConstants.ElbowLimitSwitch);
  private DigitalInput ArmlimitSwitch = new DigitalInput(Constants.LimitSwitchConstants.ArmLimitSwitch);

  private boolean elbowTriggered = this.getElbowLimitSwitchState();
  private boolean armTriggered = this.getArmLimitSwitchState();

  public MagneticLimitSwitches() {
    SmartDashboard.putBoolean("Elbow Limit triggered", this.elbowTriggered);
    SmartDashboard.putBoolean("Arm Limit triggered", this.armTriggered);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.elbowTriggered = this.getElbowLimitSwitchState();
    SmartDashboard.putBoolean("Elbow Limit triggered", this.elbowTriggered);

    this.armTriggered = this.getArmLimitSwitchState();
    SmartDashboard.putBoolean("Arm Limit triggered", this.armTriggered);
  }

  private boolean getElbowLimitSwitchState() {
    return !ElbowlimitSwitch.get();
  }

  public boolean isTriggeredElbow(){
    return elbowTriggered;
  }

  private boolean getArmLimitSwitchState() {
    return !ArmlimitSwitch.get();
  }

  public boolean isTriggeredArm(){
    return armTriggered;
  }

}
