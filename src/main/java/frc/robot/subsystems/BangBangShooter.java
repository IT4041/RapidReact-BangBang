// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class BangBangShooter extends SubsystemBase {

  private final BangBangController bangBangController;
  private final SimpleMotorFeedforward feedforward;
  private final CANSparkMax sparkMax1;
  private final CANSparkMax sparkMax2;
  private final RelativeEncoder encoder;

  private int accumulator = 0;
  private double RPM_Target = 0;
  private double rpmTolerance = 50;
  private double RPM_multiplication_factor = 0.001;
  private double scaledVelo = 0;

  private double kstatic = 0.0008;
  private double kvelocity = 0.19;
  private double kacceleraton = 0.13;

  private boolean enabled = false;
  private boolean isTele = false;

  /** Creates a new BangBangShooter. */
  public BangBangShooter() {

    bangBangController = new BangBangController();
    feedforward = new SimpleMotorFeedforward(kstatic, kvelocity, kacceleraton);

    sparkMax1 = new CANSparkMax(Constants.ShooterConstants.ShooterSparkMax1, MotorType.kBrushless);
    sparkMax2 = new CANSparkMax(Constants.ShooterConstants.ShooterSparkMax2, MotorType.kBrushless);
    encoder = sparkMax1.getEncoder();

    sparkMax1.restoreFactoryDefaults();
    sparkMax2.restoreFactoryDefaults();

    sparkMax1.setIdleMode(IdleMode.kCoast);
    sparkMax2.setIdleMode(IdleMode.kCoast);

    sparkMax1.follow(sparkMax2, true);

    sparkMax1.enableVoltageCompensation(12);
    sparkMax2.enableVoltageCompensation(12);

    SmartDashboard.putNumber("Calculated RPMS", 0);
    SmartDashboard.putBoolean("Ready to Shoot", false);
    SmartDashboard.putNumber("Actual RPMS", 0);
    SmartDashboard.putNumber("RPM diff", 0);
    SmartDashboard.putNumber("Target RPM", RPM_Target);
    SmartDashboard.putNumber("BangBangOutPut", 0);
    SmartDashboard.putNumber("FeedFowardOutput", 0);
    SmartDashboard.putBoolean("Shooter Enabled", enabled);
    SmartDashboard.putNumber("kstatic", kstatic);
    SmartDashboard.putNumber("RPM_multiplication_factor", RPM_multiplication_factor);
    SmartDashboard.putNumber("scaledVelo", scaledVelo);
    SmartDashboard.putNumber("rpmTolerance", rpmTolerance);
    SmartDashboard.putNumber("accumulator", 0);
    SmartDashboard.putBoolean("atSpeed", false);
    SmartDashboard.putBoolean("isTele", isTele);

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Actual RPMS", encoder.getVelocity());
    SmartDashboard.putNumber("scaledVelo", scaledVelo);

    //TODO: remove this test code prior to competition
    // double targetRPM = SmartDashboard.getNumber("Target RPM", RPM_Target);
    // if (targetRPM != RPM_Target) {
    //   RPM_Target = targetRPM;
    // }

    // double newRPM_multiplication_factor = SmartDashboard.getNumber("RPM_multiplication_factor", RPM_multiplication_factor);
    // if (newRPM_multiplication_factor != RPM_multiplication_factor) {
    //   RPM_multiplication_factor = newRPM_multiplication_factor;
    // }

    // double newrpmTolerance = SmartDashboard.getNumber("rpmTolerance", rpmTolerance);
    // if (newrpmTolerance != rpmTolerance) {
    //   rpmTolerance = newrpmTolerance;
    // }

    SmartDashboard.putNumber("Target RPM", RPM_Target);
    double BangBangOutPut = bangBangController.calculate(encoder.getVelocity(), RPM_Target) + 0.9 * feedforward.calculate(RPM_Target);
    double FeedFowardOutput = feedforward.calculate(RPM_Target);
    SmartDashboard.putNumber("BangBangOutPut", BangBangOutPut);
    SmartDashboard.putNumber("FeedFowardOutput", FeedFowardOutput);

    SmartDashboard.putBoolean("Shooter Enabled", enabled);
    SmartDashboard.putNumber("RPM diff", (RPM_Target * RPM_multiplication_factor) - encoder.getVelocity());

    //if(false){
    if(isTele){
      if(enabled){
        sparkMax2.set(bangBangController.calculate(encoder.getVelocity(), RPM_Target) + (0.9 * feedforward.calculate(RPM_Target)));  
        this.readyToShoot();
      }else{//disabled
        sparkMax2.set(0.0);
      }
    }
    else{
      sparkMax2.set(RPM_Target);
    }
  }

  private double calculateRPMs(double distance) {

    double bbControllerValue;

    //Tommy's formula: y= -4808.15*.99^x+5800
    //Tommy formula #2: y= -3558.53 *.975^x + 3750
    //double temp = (-4808.15 * Math.pow(.99,distance)) + 5800;
    double temp = (-3558.53 * Math.pow(.975,distance)) + 3775;
    bbControllerValue = temp * RPM_multiplication_factor;
    
    SmartDashboard.putNumber("Calculated RPMS", bbControllerValue);
    return bbControllerValue;
  }

  public boolean readyToShoot() {

    boolean atSpeed = false;
    double measuredVelo = encoder.getVelocity();
    this.scaledVelo = this.RPM_Target * 1000;
    SmartDashboard.putNumber("scaledVelo", this.scaledVelo);
    SmartDashboard.putNumber("Actual RPMS", encoder.getVelocity());

    if(Math.abs(measuredVelo) <= (Math.abs(scaledVelo) + Math.abs(rpmTolerance)) && Math.abs(measuredVelo) >= (Math.abs(scaledVelo) - Math.abs(rpmTolerance))){
      atSpeed = true;
      accumulator++;
    }

    SmartDashboard.putNumber("RPM diff", scaledVelo - measuredVelo);
    SmartDashboard.putBoolean("Ready to Shoot", atSpeed && accumulator > 2);
    SmartDashboard.putNumber("accumulator", accumulator);
    SmartDashboard.putBoolean("atSpeed", atSpeed);
    
    //return bangBangController.atSetpoint();
    return true;//atSpeed && accumulator > 2;
  }

  public void on(double distance) {
    RPM_Target = this.calculateRPMs(distance);
  }

  public void failSafeShoot(){
    this.setRPM(0.56);
  }
  
  public void enable(){
    enabled = true;
  }

  public void disable(){
    enabled = false;
  }

  public void setIsTele(){
    isTele = true;
  }

  public void setShotRpmFar(){
    this.setRPM(0.58);
  }

  public void setShotRpmClose(){
    this.setRPM(0.56);
  }

  public void setRPM(double rpm){
    this.RPM_Target = rpm;
    SmartDashboard.putNumber("Target RPM", this.RPM_Target);
  }

}
