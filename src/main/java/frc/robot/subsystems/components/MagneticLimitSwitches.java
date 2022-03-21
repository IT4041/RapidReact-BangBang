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
  private DigitalInput ArmlimitSwitchBack = new DigitalInput(Constants.LimitSwitchConstants.ArmLimitSwitchBack);
  private DigitalInput ArmlimitSwitchForward = new DigitalInput(Constants.LimitSwitchConstants.ArmLimitSwitchForward);

  private boolean elbowTriggered = this.getElbowLimitSwitchState();
  private boolean armTriggeredBack = this.getArmBackLimitSwitchState();
  private boolean armTriggeredForward = this.getArmBackLimitSwitchState();
  

  public MagneticLimitSwitches() {
    SmartDashboard.putBoolean("Elbow Limit triggered", this.elbowTriggered);
    SmartDashboard.putBoolean("Arm Limit Back triggered", this.armTriggeredBack);
    SmartDashboard.putBoolean("Arm Limit Forward triggered", this.armTriggeredForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.elbowTriggered = this.getElbowLimitSwitchState();
    SmartDashboard.putBoolean("Elbow Limit triggered", this.elbowTriggered);

    this.armTriggeredBack = this.getArmBackLimitSwitchState();
    SmartDashboard.putBoolean("Arm Limit triggered", this.armTriggeredBack);

    this.armTriggeredForward = this.getArmForwardLimitSwitchState();
    SmartDashboard.putBoolean("Arm Limit triggered", this.armTriggeredForward);
  }

  private boolean getElbowLimitSwitchState() {
    return !ElbowlimitSwitch.get();
  }

  public boolean isTriggeredElbow(){
    return elbowTriggered;
  }

  private boolean getArmBackLimitSwitchState() {
    return !ArmlimitSwitchBack.get();
  }

  public boolean isTriggeredArmBack(){
    return armTriggeredBack;
  }

  private boolean getArmForwardLimitSwitchState() {
    return !ArmlimitSwitchForward.get();
  }

  public boolean isTriggeredArmForward(){
    return armTriggeredForward;
  }

}
