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

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;

public class Lift_BrushedDual extends SubsystemBase {

  private final TalonSRX LiftTalonMaster = new TalonSRX(Constants.LiftConstants.LiftTalonSRX); 
  private final TalonSRX LiftTalonMFollower = new TalonSRX(Constants.LiftConstants.LiftTalonFollowerSRX); 

  private double kMaxOutput, kMinOutput;
  private boolean firstLift = true;
  private boolean up = false;
  private boolean down = false;

  public Lift_BrushedDual() {

    kMaxOutput = 1; 
    kMinOutput = -1;

    LiftTalonMaster.configFactoryDefault();
    LiftTalonMaster.set(ControlMode.Position, 0);
    LiftTalonMaster.setNeutralMode(NeutralMode.Brake);

    LiftTalonMaster.setSensorPhase(false);
    LiftTalonMaster.setInverted(false);
    LiftTalonMaster.configVoltageCompSaturation(12);
    LiftTalonMaster.enableVoltageCompensation(true);

    LiftTalonMaster.configNominalOutputForward(0,30);
    LiftTalonMaster.configNominalOutputReverse(0,30);
    LiftTalonMaster.configPeakOutputForward(kMaxOutput, 30);
    LiftTalonMaster.configPeakOutputReverse(kMinOutput, 30);

    LiftTalonMaster.configPeakCurrentLimit(65, 30);
    LiftTalonMaster.configPeakCurrentDuration(75, 30);
    LiftTalonMaster.configContinuousCurrentLimit(40, 30);
    LiftTalonMaster.enableCurrentLimit(true);

    //----config follower---------------------------------
    LiftTalonMFollower.configFactoryDefault();
    LiftTalonMFollower.set(ControlMode.Follower, 0);
    LiftTalonMFollower.follow(LiftTalonMaster);
    LiftTalonMFollower.setNeutralMode(NeutralMode.Brake);
    LiftTalonMFollower.setInverted(InvertType.FollowMaster);//InvertType.OpposeMaster

    LiftTalonMFollower.configVoltageCompSaturation(12);
    LiftTalonMFollower.enableVoltageCompensation(true);

    LiftTalonMFollower.configNominalOutputForward(0,30);
    LiftTalonMFollower.configNominalOutputReverse(0,30);
    LiftTalonMFollower.configPeakOutputForward(kMaxOutput, 30);

    LiftTalonMFollower.configPeakCurrentLimit(65, 30);
    LiftTalonMFollower.configPeakCurrentDuration(75, 30);
    LiftTalonMFollower.configContinuousCurrentLimit(40, 30);
    LiftTalonMFollower.enableCurrentLimit(true);
    //----config follower---------------------------------

    LiftTalonMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

    LiftTalonMaster.configForwardSoftLimitEnable(true);
    LiftTalonMaster.configReverseSoftLimitEnable(true);

    LiftTalonMaster.configForwardSoftLimitThreshold(Constants.LiftConstants.Top_SRX);
    LiftTalonMaster.configReverseSoftLimitThreshold(Constants.LiftConstants.Home_SRX);

    LiftTalonMaster.setSelectedSensorPosition(0,0,30);
    SmartDashboard.putNumber("Lift position", LiftTalonMaster.getSelectedSensorPosition() );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lift position", LiftTalonMaster.getSelectedSensorPosition());

    if(this.down){
      if(LiftTalonMaster.getSelectedSensorPosition() > Constants.LiftConstants.bottom_brake_threshold) {
        // we're not close to the bottom yet, so go fast
        LiftTalonMaster.set(ControlMode.PercentOutput, -1);
      }
      else{
        // we're close, so slow down
        LiftTalonMaster.set(ControlMode.PercentOutput, -0.35);
      }
    }
    else if(this.up){
      if(LiftTalonMaster.getSelectedSensorPosition() < Constants.LiftConstants.top_brake_threshold) {
        //we're not close to the top yet, so go fast
        LiftTalonMaster.set(ControlMode.PercentOutput, 1);
      }
      else{
        // we're close, so slow down
        LiftTalonMaster.set(ControlMode.PercentOutput, 0.35);
      }
    }
    else{
      this.stop();
    }
  }

  public void stop() {
    LiftTalonMaster.set(ControlMode.PercentOutput, 0.0);
    down = false;
    up = false;
  }

  public void down() {
    down = true;
    up = false;
  }

  public void up() {
    down = false;
    up = true;
    firstLift = false;
  }

  public boolean isFirstLift(){
    return firstLift;
  }

  public void resetFirstLift(){
    this.firstLift = true;
  }

}