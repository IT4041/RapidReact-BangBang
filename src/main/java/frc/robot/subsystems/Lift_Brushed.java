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

public class Lift_Brushed extends SubsystemBase {

  private final TalonSRX LiftTalon = new TalonSRX(Constants.LiftConstants.LiftTalonSRX); 

  private double kMaxOutput, kMinOutput;
  private boolean firstLift = true;
  private boolean up = false;
  private boolean down = false;

  public Lift_Brushed() {

    kMaxOutput = 1; 
    kMinOutput = -1;

    LiftTalon.configFactoryDefault();
    LiftTalon.set(ControlMode.Position, 0);
    LiftTalon.setNeutralMode(NeutralMode.Brake);

    LiftTalon.setSensorPhase(false);
    LiftTalon.setInverted(false);
    LiftTalon.configVoltageCompSaturation(12);
    LiftTalon.enableVoltageCompensation(true);

    LiftTalon.configForwardSoftLimitEnable(true);
    LiftTalon.configReverseSoftLimitEnable(true);

    LiftTalon.configForwardSoftLimitThreshold(Constants.LiftConstants.Top_SRX);
    LiftTalon.configReverseSoftLimitThreshold(Constants.LiftConstants.Home_SRX);

    LiftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

    LiftTalon.configNominalOutputForward(0,30);
    LiftTalon.configNominalOutputReverse(0,30);
    LiftTalon.configPeakOutputForward(kMaxOutput, 30);
    LiftTalon.configPeakOutputReverse(kMinOutput, 30);

    LiftTalon.configPeakCurrentLimit(65, 30);
    LiftTalon.configPeakCurrentDuration(75, 30);
    LiftTalon.configContinuousCurrentLimit(40, 30);
    LiftTalon.enableCurrentLimit(true);

    LiftTalon.setSelectedSensorPosition(0,0,30);
    SmartDashboard.putNumber("Lift position", LiftTalon.getSelectedSensorPosition() );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lift position", LiftTalon.getSelectedSensorPosition());

    if(this.down){
      if(LiftTalon.getSelectedSensorPosition() > 60000) {
        // we're not close to the bottom yet, so go fast
        LiftTalon.set(ControlMode.PercentOutput, -1);
      }
      else{
        // we're close, so slow down
        LiftTalon.set(ControlMode.PercentOutput, -0.6);
      }
    }
    else if(this.up){
      if(LiftTalon.getSelectedSensorPosition() < 950000) {
        //we're not close to the top yet, so go fast
        LiftTalon.set(ControlMode.PercentOutput, 1);
      }
      else{
        // we're close, so slow down
        LiftTalon.set(ControlMode.PercentOutput, 0.6);
      }
    }
    else{
      this.stop();
    }
  }

  public void stop() {
    LiftTalon.set(ControlMode.PercentOutput, 0.0);
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