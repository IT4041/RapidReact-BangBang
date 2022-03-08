/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.components.*;

public class MasterContoller extends SubsystemBase {

  private final Indexer m_Indexer;
  private final Turret m_Turret;
  private final BangBangShooter m_Shooter;
  private final LimeLight m_LimeLight;
  private final IntakeWheels m_IntakeWheels;
  private final Feeder m_Feeder;

  private boolean m_failSafe = false;
  private boolean reversed = false;
  private boolean intakeOn = false;
  private boolean m_target = false;

  public MasterContoller(Indexer in_Indexer, Turret in_Turret, BangBangShooter in_Shooter, LimeLight in_LimeLight,
    IntakeWheels in_IntakeWheels, Feeder in_feeder) {
    m_Indexer = in_Indexer;
    m_Turret = in_Turret;
    m_Shooter = in_Shooter;
    m_LimeLight = in_LimeLight;
    m_IntakeWheels = in_IntakeWheels;
    m_Feeder = in_feeder;

    SmartDashboard.putBoolean("FailSafe", m_failSafe);
  }

  @Override
  public void periodic() {

  // This method will be called once per scheduler run
    if (m_target) {
      this.doTargeting();
    } else {
      this.stopTargeting();
    }
    SmartDashboard.putBoolean("FailSafe", m_failSafe);

    if(!m_Indexer.IsOn() && !intakeOn){
      m_Feeder.off();
    }else{
      if(reversed){
        m_Feeder.reverse();
      }else{
        m_Feeder.on();
      }
    }
  }
  // parameterless functions for inlining ********************
  public void togglFailSafe() {
    m_failSafe = !m_failSafe;
    m_Shooter.setFailSafe(m_failSafe);
  }

  public void targetNoParams() {

    m_Indexer.setAutoIndexOff();
    if (m_failSafe) {
      //lock turret straight forward
      m_Turret.targetingDisabled(true);
      // set shooter head to 1/2 throttle
      m_Shooter.setFailSafe(true);
      m_Shooter.failSafeShoot();
      //turn on indexer
      m_Indexer.shoot();
    
    } else {
      // do turret targeting,
      // distance calculation,
      // and shooter head rpm adjustment
      m_target = true;
      m_Turret.targetingEnabled(0);
      m_Shooter.setFailSafe(false);
    }
  }

  public void stopTargetNoParams() {
    m_target = false;
    m_Indexer.setAutoIndexOn();
    if (m_failSafe) {
      m_Shooter.setRPM(0);
    }
  }
  // ***********************************************************

  // internal processing functions *********************************
  private void doTargeting() {
    m_LimeLight.ledOn();
    m_Turret.targetingEnabled(m_LimeLight.getXOffset());
    if(m_Turret.onTarget() && m_LimeLight.hasValidTarget()){
      m_Shooter.enable();
      m_Shooter.on(m_LimeLight.getDistance());
      if(m_Shooter.readyToShoot()){   
        m_Indexer.shoot();
      }
    }
  }

  private void stopTargeting() {
    m_LimeLight.ledOff();

    boolean reset = DriverStation.isAutonomous()?false:true;
    m_Turret.targetingDisabled(reset);
    m_Shooter.disable();
  }
  // *************************************************
  public void reverseAll(){
    reversed = true;
    m_Indexer.setAutoIndexOff();
    m_Feeder.reverse();
    m_IntakeWheels.reverse();
    m_Indexer.reverse();
  }

  public void returnToPrevState(){
    reversed = false;
    m_Indexer.setAutoIndexOn();
    m_IntakeWheels.returnToPrevState();
  }

  public void intakeWheelsOn(){
    m_IntakeWheels.on();
    intakeOn = true;
  }
  
  public void intakeWheelsOff(){
    m_IntakeWheels.off();
    intakeOn = false;
  }

}