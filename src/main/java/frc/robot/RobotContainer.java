/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.auto.groups.SingleBallAuto;
import frc.robot.commands.auto.groups.SingleBallAutoWithTargeting;
//import frc.robot.commands.auto.groups.ThreeBallAuto;
import frc.robot.commands.auto.groups.TwoBallAuto;
import frc.robot.controllers.AxisJoystickButton;
import frc.robot.controllers.AxisJoystickButton.ThresholdType;
import frc.robot.subsystems.*;
import frc.robot.subsystems.components.*;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  // components
  public final XboxController driver = new XboxController(Constants.OIConstants.xboxControllerDriver);
  private final XboxController assist = new XboxController(Constants.OIConstants.xboxControllerAssist);

  private final ColorSensor colorSensor = new ColorSensor();
  private final RangeSensors rangeSensors = new RangeSensors();
  private final LimeLight limeLight = new LimeLight();
  private final NavX navX = new NavX();
  private final MagneticLimitSwitches magSwitches = new MagneticLimitSwitches();

  public final DriveTrain driveTrain = new DriveTrain(navX);
  private final Feeder feeder = new Feeder();
  private final Lift lift = new Lift();
  private final Arms arms = new Arms(magSwitches);
  private final Indexer indexer = new Indexer(rangeSensors);
  private final IntakeElbow intakeElbow = new IntakeElbow(magSwitches);
  private final IntakeWheels intakeWheels = new IntakeWheels();
  private final Turret turret = new Turret();
  private final BangBangShooter bbshooter = new BangBangShooter();
  private final MasterContoller masterController = new MasterContoller(indexer, turret, bbshooter, limeLight, intakeWheels, feeder, colorSensor, lift, arms, intakeElbow);

  private SendableChooser<Command> m_chooser;
  private Trajectory[] m_trajectories;
  private SingleBallAuto OneB1;
  private TwoBallAuto TwoB1;
  private SingleBallAuto StraightBack;
  private SingleBallAutoWithTargeting OneB1WithTargeting;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Trajectory[] in_trajectories) {
    m_trajectories = in_trajectories;

    // Configure the button bindings
    configureButtonBindings();

    this.OneB1 = new SingleBallAuto(turret,bbshooter,intakeElbow,indexer,intakeWheels,driveTrain,m_trajectories[0]);
    this.OneB1WithTargeting = new SingleBallAutoWithTargeting(masterController,driveTrain,m_trajectories[0]);
    this.TwoB1 = new TwoBallAuto(turret,bbshooter,intakeElbow,indexer,intakeWheels,driveTrain,m_trajectories[1]);
    this.StraightBack = new SingleBallAuto(turret,bbshooter,intakeElbow,indexer,intakeWheels,driveTrain,m_trajectories[2]);

    this.m_chooser = new SendableChooser<Command>();

    this.m_chooser.setDefaultOption("One Ball", this.OneB1);
    this.m_chooser.addOption("Two Ball", this.TwoB1);
    this.m_chooser.addOption("Straight Back", this.StraightBack);
    this.m_chooser.addOption("One Ball with Targeting", this.OneB1WithTargeting);

    // Put the chooser on the dashboard
    SmartDashboard.putData(this.m_chooser);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    driveTrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
                                                          //FWD                   ROT
        new RunCommand(() -> driveTrain.arcadeDrive(-driver.getRightY(),driver.getLeftX()) , driveTrain));

  }

  private void configureButtonBindings() {

    JoystickButton buttonA_dr = new JoystickButton(driver, Constants.OIConstants.buttonA);
    JoystickButton buttonY_dr = new JoystickButton(driver, Constants.OIConstants.buttonY);
    JoystickButton buttonX_dr = new JoystickButton(driver, Constants.OIConstants.buttonX);
    JoystickButton buttonB_dr = new JoystickButton(driver, Constants.OIConstants.buttonB);
    JoystickButton buttonBumperLeft_dr = new JoystickButton(driver, Constants.OIConstants.buttonBumperLeft);
    JoystickButton buttonSelect_dr = new JoystickButton(driver, Constants.OIConstants.buttonSelect);

    buttonA_dr.whenPressed(new InstantCommand(masterController::liftDown,masterController));
    buttonA_dr.whenReleased(new InstantCommand(masterController::liftStop,masterController));
    buttonY_dr.whenPressed(new InstantCommand(masterController::liftUp,masterController));
    buttonY_dr.whenReleased(new InstantCommand(masterController::liftStop,masterController));

    buttonX_dr.whenPressed(new InstantCommand(masterController::armsForward,masterController));
    buttonX_dr.whenReleased(new InstantCommand(masterController::armsStop,masterController));
    buttonB_dr.whenPressed(new InstantCommand(masterController::armsBack,masterController));
    buttonB_dr.whenReleased(new InstantCommand(masterController::armsStop,masterController));

    buttonBumperLeft_dr.whenPressed(new InstantCommand(arms::homePosition, arms));
    
    // in an emergency allow user to take over control
    buttonSelect_dr.whenPressed(new InstantCommand(masterController::togglFailSafe,masterController));

    JoystickButton buttonA_as = new JoystickButton(assist, Constants.OIConstants.buttonA);
    JoystickButton buttonY_as = new JoystickButton(assist, Constants.OIConstants.buttonY);
    JoystickButton buttonX_as = new JoystickButton(assist, Constants.OIConstants.buttonX);
    JoystickButton buttonB_as = new JoystickButton(assist, Constants.OIConstants.buttonB);
    JoystickButton buttonBumperRight_as = new JoystickButton(assist, Constants.OIConstants.buttonBumperRight);
    AxisJoystickButton triggerRight_as = new AxisJoystickButton(assist, Constants.OIConstants.rightTrigger, 0.5, ThresholdType.GREATER_THAN);

    triggerRight_as.whenPressed(new InstantCommand(masterController::enabledTargetingAndShooting,masterController));
    triggerRight_as.whenReleased(new InstantCommand(masterController::disabledTargetingAndShooting,masterController));

    buttonX_as.whenPressed(new InstantCommand(masterController::intakeWheelsOn,masterController));
    buttonB_as.whenPressed(new InstantCommand(masterController::intakeWheelsOff,masterController));

    buttonA_as.whenPressed(new InstantCommand(masterController::elbowDown,masterController));
    buttonY_as.whenPressed(new InstantCommand(masterController::elbowUp,masterController));

    buttonBumperRight_as.whenPressed(new InstantCommand(masterController::reverseAll,masterController));
    buttonBumperRight_as.whenReleased(new InstantCommand(masterController::returnToPrevState,masterController));

  }

  public Command getAutonomousCommand() {
    System.out.println("getAutonomousCommand");
    return m_chooser.getSelected();
  }

  public void disabledLEDS() {
    limeLight.ledOff();
  }

  public void enableAutoIndexing() {
    indexer.setAutoIndexOn();
  }

  public void enableShooter() {
    bbshooter.enable();
  }

  public void isTele() {
    bbshooter.setIsTele();
  }

  public void resetFirstLift() {
    masterController.resetFirstLift();
  }
}
