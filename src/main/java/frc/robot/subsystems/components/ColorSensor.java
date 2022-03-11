/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ColorSensor extends SubsystemBase {

  private final NetworkTable FMSInfo = NetworkTableInstance.getDefault().getTable("FMSInfo");
  private NetworkTableEntry isRedAlliance;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  //private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  //private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  private Boolean isRed;
  private boolean wrongColor = false;

  /**
   * Creates a new colorSensor.
   */
  public ColorSensor() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    //m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    //m_colorMatcher.addColorMatch(kYellowTarget);

    isRedAlliance = FMSInfo.getEntry("IsRedAlliance");
    this.isRed = isRedAlliance.getBoolean(true);

    SmartDashboard.putBoolean("IsRed",  this.isRed);
    SmartDashboard.putBoolean("wrongColor",  wrongColor);
  }

  @Override
  public void periodic() {

    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();

    SmartDashboard.putNumber("Red", detectedColor.red);
    //SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    int proximity = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Proximity", proximity);

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "blue";
    } else if (match.color == kRedTarget) {
      colorString = "red";
    // } else if (match.color == kGreenTarget) {
    //   colorString = "green";
    //} else if (match.color == kYellowTarget) {
      //colorString = "yellow";
    } else {
      colorString = "unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    //SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    isRedAlliance = FMSInfo.getEntry("IsRedAlliance");
    this.isRed = isRedAlliance.getBoolean(true);
    SmartDashboard.putBoolean("IsRed", this.isRed);

    String AllianceColor = this.isRed ? "red" : "blue";
    SmartDashboard.putString("AllianceColor", AllianceColor);

    this.wrongColor = !colorString.equalsIgnoreCase(AllianceColor);
    SmartDashboard.putBoolean("wrongColor",  wrongColor);

  }

  public boolean BallIsWrongColor(){
    return this.wrongColor;
  }
}
