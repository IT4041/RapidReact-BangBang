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

  private double RPM_Target = 0;
  private double RPM_multiplication_factor = 0.001;

  private double kstatic = 0.0008;
  private double kvelocity = 0.19;
  private double kacceleraton = 0.13;

  private boolean enabled = false;
  private boolean failSafe = false;

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
    SmartDashboard.putNumber("Actual RPMS", 0);
    SmartDashboard.putNumber("RPM diff", 0);
    SmartDashboard.putNumber("Target RPM", RPM_Target);
    SmartDashboard.putNumber("BangBangOutPut", 0);
    SmartDashboard.putNumber("FeedFowardOutput", 0);
    SmartDashboard.putBoolean("Shooter Enabled", enabled);
    SmartDashboard.putNumber("kstatic", kstatic);
    SmartDashboard.putNumber("RPM_multiplication_factor", RPM_multiplication_factor);

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Actual RPMS", encoder.getVelocity());

    // TODO: remove this test code prior to competition
    // double targetRPM = SmartDashboard.getNumber("Target RPM", RPM_Target);
    // if (targetRPM != RPM_Target) {
    // RPM_Target = targetRPM;
    // }

    double BangBangOutPut = bangBangController.calculate(encoder.getVelocity(), RPM_Target) + 0.9 * feedforward.calculate(RPM_Target);
    double FeedFowardOutput = feedforward.calculate(RPM_Target);

    SmartDashboard.putNumber("Target RPM", RPM_Target);
    SmartDashboard.putNumber("BangBangOutPut", BangBangOutPut);
    SmartDashboard.putNumber("FeedFowardOutput", FeedFowardOutput);
    SmartDashboard.putNumber("RPM diff", (RPM_Target * RPM_multiplication_factor) - encoder.getVelocity());
    SmartDashboard.putBoolean("Shooter Enabled", enabled);

    if (enabled) {
      sparkMax2.set(bangBangController.calculate(encoder.getVelocity(), RPM_Target) + (0.9 * feedforward.calculate(RPM_Target)));
    } else if (failSafe) {
      sparkMax2.set(RPM_Target);
    } else {// disabled
      sparkMax2.set(0.0);
      RPM_Target = 0.0;
    }

  }

  private double calculateRPMs(double distance) {

    double bbControllerValue;

    // Tommy's formula: y= -4808.15*.99^x+5800
    // Tommy's formula #2: y= -3558.53 *.975^x + 3750
    // Tommy's formula #3: y = -20.8333 * 1.03105^x + 3160-- not correct
    // Tommy's formula #4: y= 21.8274 * 1.03076 ^x + 3150

    // formula #4
    double temp = (21.8274 * Math.pow(1.03076, distance)) + 3150;

    bbControllerValue = temp * RPM_multiplication_factor;

    SmartDashboard.putNumber("Calculated RPMS", bbControllerValue);
    return bbControllerValue;
  }

  public void spin(double distance) {
    RPM_Target = this.calculateRPMs(distance);
  }

  public void failSafeShoot() {
    this.setRPM(0.335);
  }

  public void setFailSafe(boolean inFailSafe) {
    this.failSafe = inFailSafe;
  }

  public void enable() {
    enabled = true;
  }

  public void disable() {
    enabled = false;
  }

  public void setShotRpmFar() {
    this.setRPM(0.60);
  }

  public void setShotRpmClose() {
    this.setRPM(0.58);
  }

  public void setRPM(double rpm) {
    this.RPM_Target = rpm;
    SmartDashboard.putNumber("Target RPM", this.RPM_Target);
  }

}
