/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase {
  /**
   * Creates a new limeLight. class handles everything relate to the limelight and
   * vision
   */

  // LimeLight table values **********************************
  // tv Whether the limelight has any valid targets (0 or 1)
  // tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  // ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
  // ta Target Area (0% of image to 100% of image)
  private final NetworkTable limelightNT = NetworkTableInstance.getDefault().getTable("limelight-it");
  private NetworkTableEntry tvEntry;
  private NetworkTableEntry txEntry;
  private NetworkTableEntry taEntry;
  private NetworkTableEntry tyEntry;

  private double tv, tx, ta, ty;
  public final int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
  public final int camMode = 0; // 0 - vision processing, 1 - driver camera
  public final int pipeline = 0; // 0 - 9
  public final int stream = 2; // sets stream layout if another webcam is attached
  public final int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz

  private double cameraAngle = 43; // degrees
  private double cameraHeight = 37; // inches(get correct value from build team)
  private double targetHeight = 103; // inches (center of targets; targets are 2" tall from 102" to 104")

  public LimeLight() {
    // make sure led is off when lime light is initialized
    limelightNT.getEntry("ledMode").setNumber(ledMode);
    limelightNT.getEntry("camMode").setNumber(camMode);
    limelightNT.getEntry("pipeline").setNumber(pipeline);
    limelightNT.getEntry("stream").setNumber(stream);
    limelightNT.getEntry("snapshot").setNumber(snapshot);

    SmartDashboard.putNumber("limelight angle", cameraAngle);
    SmartDashboard.putNumber("limelight height", cameraHeight);

  }

  @Override
  public void periodic() {

    //TODO: remove this test code prior to competition
    // double cameraAngleNew = SmartDashboard.getNumber("limelight angle", cameraAngle);
    // if (cameraAngleNew != cameraAngle) {
    //   cameraAngle = cameraAngleNew;
    // }

    // double cameraHeightNew = SmartDashboard.getNumber("limelight height", cameraHeight);
    // if (cameraHeightNew != cameraHeight) {
    //   cameraHeight = cameraHeightNew;
    // }
    //**************************************************** */


    // This method will be called once per scheduler run
    tvEntry = limelightNT.getEntry("tv");
    txEntry = limelightNT.getEntry("tx");
    taEntry = limelightNT.getEntry("ta");
    tyEntry = limelightNT.getEntry("ty");

    tv = tvEntry.getDouble(0.0);
    tx = txEntry.getDouble(0.0);
    ta = taEntry.getDouble(0.0);
    ty = tyEntry.getDouble(0.0);

    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ta", ta);
    SmartDashboard.putNumber("ty", ty);

    SmartDashboard.putBoolean("HasValidTarget", hasValidTarget());
    SmartDashboard.putNumber("Distance to Target: ", this.getDistance());

  }

  public void ledOn() {
    limelightNT.getEntry("ledMode").setNumber(3);
  }

  public void ledOff() {
    limelightNT.getEntry("ledMode").setNumber(1);
  }

  public boolean hasValidTarget() {
    return tv >= 1.0;
  }

  public double getXOffset() {
    return tx;
  }

  public double getYOffset() {
    return ty;
  }

  /* distance calculation base on angle (supplied by 4909) */
  public double getDistance() {
    double distance;
    double angle;

    // double distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(CameraAngle + YOffset))
    angle = Math.tan(Math.toRadians(cameraAngle + ty));
    distance = (targetHeight - cameraHeight) / angle;

    return distance;
  }

}
