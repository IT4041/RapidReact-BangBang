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

  private double minRPM = 1000;
  private int accumulator = 0;
  private double RPM_Target = 12;
  private double rpmTolerance = 50;

  private double kstatic = 0.0008;
  private double kvelocity = 0.0597;
  private double kacceleraton = 0.07;

  private boolean enabled = false;

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

    //encoder.setVelocityConversionFactor(0.036);

    SmartDashboard.putNumber("Calculated RPMS", 0);
    SmartDashboard.putBoolean("Ready to Shoot", false);
    SmartDashboard.putNumber("Actual RPMS", 0);
    SmartDashboard.putNumber("RPM diff", 0);
    SmartDashboard.putNumber("Shooter RPM", RPM_Target);
    SmartDashboard.putNumber("BangBangOutPut", 0);
    SmartDashboard.putNumber("FeedFowardOutput", 0);
    SmartDashboard.putBoolean("Shooter Enabled", enabled);

    SmartDashboard.putNumber("kstatic", kstatic);

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Actual RPMS", encoder.getVelocity());

    double targetRPM = SmartDashboard.getNumber("Shooter RPM", RPM_Target);
    if (targetRPM != RPM_Target) {
      RPM_Target = targetRPM;
    }

    double BangBangOutPut = bangBangController.calculate(encoder.getVelocity(), RPM_Target) + 0.9 * feedforward.calculate(RPM_Target);
    double FeedFowardOutput = feedforward.calculate(RPM_Target);
    SmartDashboard.putNumber("BangBangOutPut", BangBangOutPut);
    SmartDashboard.putNumber("FeedFowardOutput", FeedFowardOutput);

    SmartDashboard.putBoolean("Shooter Enabled", enabled);
    SmartDashboard.putNumber("RPM diff", (RPM_Target*309) - encoder.getVelocity());

    // TODO: check all conversions and unit and everything else before trying this
    // check relative encoder get velocity to make sure it's reporting the same things as encoder.getrate()
    if(enabled){
      sparkMax2.set(bangBangController.calculate(encoder.getVelocity(), RPM_Target) + (0.9 * feedforward.calculate(RPM_Target)));  
      this.readyToShoot();
    }else{//disabled
      sparkMax2.set(0.0);
    }
      

  }

  // private double calculateRPMs(double distance) {

  //   double finalRPMS;

  //   // calculate rpms
  //   finalRPMS = distance * RPM_Target;

  //   // use min rpms if calculated value is below min threshold
  //   finalRPMS = (finalRPMS < minRPM) ? minRPM : finalRPMS;
  //   SmartDashboard.putNumber("Calculated RPMS", -finalRPMS);
  //   return -finalRPMS;
  // }

  public boolean readyToShoot() {
    boolean atSpeed = false;
    double measuredVelo = encoder.getVelocity();
    double scaledVelo = this.RPM_Target * 309;

    if (measuredVelo <= (scaledVelo + rpmTolerance) && measuredVelo >= (scaledVelo - this.rpmTolerance) && measuredVelo > this.minRPM) {
      atSpeed = true;
      accumulator++;
    }
    SmartDashboard.putNumber("RPM diff", scaledVelo - measuredVelo);
    SmartDashboard.putBoolean("Ready to Shoot", atSpeed && accumulator > 2);

    return atSpeed && accumulator > 2;
  }

  public void on(double distance) {
    // velocity = this.calculateRPMs(distance);
  }

  public void off() {
    // accumulator = 0;
    // velocity = this.calculateRPMs(0);
  }

  public void failSafeShoot(){
    // this.off();
    // sparkMax1.set(.5);
  }

  public void autoShoot(){
    // this.off();
    // sparkMax1.set(-.65);
  }
  
  public void enable(){
    enabled = true;
  }

  public void disable(){
    enabled = false;
  }

  public void setShotRpmFar(){
    this.setRPM(7);
  }

  public void setShotRpmClose(){
    this.setRPM(5);
  }

  public void setRPM(double rpm){
    this.RPM_Target = rpm;
    SmartDashboard.putNumber("Shooter RPM", this.RPM_Target);
  }

}
