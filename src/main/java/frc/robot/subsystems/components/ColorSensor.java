/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensor extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private Boolean isRed;
  private boolean wrongColor = false;
  private String AllianceColor;
  private DriverStation.Alliance dsAlliance;

  /**
   * Creates a new colorSensor.
   */
  public ColorSensor() {

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);

    this.dsAlliance = DriverStation.getAlliance();

    this.isRed = dsAlliance == DriverStation.Alliance.Red ? true : false;
    this.AllianceColor = this.isRed ? "red" : "blue";

    SmartDashboard.putBoolean("IsRed", this.isRed);
    SmartDashboard.putBoolean("wrongColor", wrongColor);

  }

  @Override
  public void periodic() {

    String colorString;

    this.dsAlliance = DriverStation.getAlliance();
    Color detectedColor = m_colorSensor.getColor();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Blue", detectedColor.blue);

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    SmartDashboard.putNumber("Confidence", match.confidence);
    
    this.isRed = dsAlliance == DriverStation.Alliance.Red ? true : false;
    SmartDashboard.putBoolean("IsRed", this.isRed);
    this.AllianceColor = this.isRed ? "red" : "blue";
    SmartDashboard.putString("AllianceColor", AllianceColor);

    if (match.color == kBlueTarget) {
      colorString = "blue";
    } else if (match.color == kRedTarget) {
      colorString = "red";
    } else {
      //if we can't determine ball color assume it's the correct color
      colorString = this.isRed ? "red" : "blue";
    }
    SmartDashboard.putString("Detected Color", colorString);

    this.wrongColor = !colorString.equalsIgnoreCase(AllianceColor);
    SmartDashboard.putBoolean("wrongColor", wrongColor);

  }

  public boolean BallIsWrongColor() {
    return this.wrongColor;
  }
}
