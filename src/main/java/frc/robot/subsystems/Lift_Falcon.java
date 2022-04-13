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

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Lift_Falcon extends SubsystemBase {

  private final TalonFX LiftTalonFX = new TalonFX(Constants.LiftConstants.LiftTalonFX); 
  // private final SupplyCurrentLimitConfiguration SupplyConfig = new SupplyCurrentLimitConfiguration(true, 100, 150, 100);
  // private final StatorCurrentLimitConfiguration StatorConfig = new StatorCurrentLimitConfiguration(true, 100, 150, 100);

  private double kMaxOutput, kMinOutput;
  private boolean firstLift = true;
  private boolean up = false;
  private boolean down = false;

  public Lift_Falcon() {

    kMaxOutput = 1; 
    kMinOutput = -1;

    LiftTalonFX.configFactoryDefault();
    LiftTalonFX.set(ControlMode.Position, 0);
    LiftTalonFX.setNeutralMode(NeutralMode.Brake);

    LiftTalonFX.setSensorPhase(false);
    LiftTalonFX.setInverted(true);
    LiftTalonFX.configVoltageCompSaturation(12);
    LiftTalonFX.enableVoltageCompensation(true);

    LiftTalonFX.configForwardSoftLimitEnable(true);
    LiftTalonFX.configReverseSoftLimitEnable(true);

    LiftTalonFX.configForwardSoftLimitThreshold(Constants.LiftConstants.Top_SRX);
    LiftTalonFX.configReverseSoftLimitThreshold(Constants.LiftConstants.Home_SRX);

    LiftTalonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);

    LiftTalonFX.configNominalOutputForward(0,30);
    LiftTalonFX.configNominalOutputReverse(0,30);
    LiftTalonFX.configPeakOutputForward(kMaxOutput, 30);
    LiftTalonFX.configPeakOutputReverse(kMinOutput, 30);

    // LiftTalonFX.configSupplyCurrentLimit(SupplyConfig, 0);
    // LiftTalonFX.configStatorCurrentLimit(StatorConfig, 0);

    LiftTalonFX.setSelectedSensorPosition(0,0,30);
    SmartDashboard.putNumber("Lift position", LiftTalonFX.getSelectedSensorPosition() );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lift position", LiftTalonFX.getSelectedSensorPosition());

    if(this.down){
      if(LiftTalonFX.getSelectedSensorPosition() > Constants.LiftConstants.bottom_brake_threshold) {
        // we're not close to the bottom yet, so go fast
        LiftTalonFX.set(ControlMode.PercentOutput, -1);
      }
      else{
        // we're close, so slow down
        LiftTalonFX.set(ControlMode.PercentOutput, -0.35);
      }
    }
    else if(this.up){
      if(LiftTalonFX.getSelectedSensorPosition() < Constants.LiftConstants.top_brake_threshold) {
        //we're not close to the top yet, so go fast
        LiftTalonFX.set(ControlMode.PercentOutput, 1);
      }
      else{
        // we're close, so slow down
        LiftTalonFX.set(ControlMode.PercentOutput, 0.35);
      }
    }
    else{
      this.stop();
    }
  }

  public void stop() {
    LiftTalonFX.set(ControlMode.PercentOutput, 0.0);
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