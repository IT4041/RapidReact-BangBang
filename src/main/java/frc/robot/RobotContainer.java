/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.auto.groups.SingleBallAuto;
import frc.robot.commands.auto.groups.ThreeBallAuto;
import frc.robot.commands.auto.groups.TrajectoryOnly;
import frc.robot.commands.auto.groups.TwoBallAuto;
import frc.robot.controllers.AxisJoystickButton;
import frc.robot.controllers.AxisJoystickButton.ThresholdType;
import frc.robot.subsystems.*;
import frc.robot.subsystems.components.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  // components
  public final XboxController driver = new XboxController(Constants.OIConstants.xboxControllerDriver);
  private final XboxController assist = new XboxController(Constants.OIConstants.xboxControllerAssist);

  //private final ColorSensor colorSensor = new ColorSensor();
  private final RangeSensors rangeSensors = new RangeSensors();
  private final LimeLight limeLight = new LimeLight();
  private final NavX navX = new NavX();

  public final DriveTrain driveTrain = new DriveTrain(navX);
  private final Feeder feeder = new Feeder();
  private final Lift elevator = new Lift();
  private final Indexer indexer = new Indexer(rangeSensors,feeder);
  private final IntakeElbow intakeElbow = new IntakeElbow();
  private final IntakeWheels intakeWheels = new IntakeWheels(feeder);
  private final Turret turret = new Turret();
  private final BangBangShooter bbshooter = new BangBangShooter();
  private final Bombardier bombardier = new Bombardier(indexer, turret, bbshooter, limeLight, intakeWheels);

  private SendableChooser<Command> m_chooser;
  private Trajectory[] m_trajectories;
  private SingleBallAuto OneB1, OneB2, OneB3;
  private TwoBallAuto TwoB1, TwoB2;
  private ThreeBallAuto ThreeB1;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Trajectory[] in_trajectories) {
    m_trajectories = in_trajectories;

    // Configure the button bindings
    configureButtonBindings();

    this.OneB1 = new SingleBallAuto(turret,bbshooter,intakeElbow,indexer,intakeWheels,driveTrain,m_trajectories[0]);
    this.OneB2 = new SingleBallAuto(turret,bbshooter,intakeElbow,indexer,intakeWheels,driveTrain,m_trajectories[1]);
    this.OneB3 = new SingleBallAuto(turret,bbshooter,intakeElbow,indexer,intakeWheels,driveTrain,m_trajectories[2]);
    this.TwoB1 = new TwoBallAuto(turret,bbshooter,intakeElbow,indexer,intakeWheels,driveTrain,m_trajectories[3]);
    this.TwoB2 = new TwoBallAuto(turret,bbshooter,intakeElbow,indexer,intakeWheels,driveTrain,m_trajectories[4]);
    this.ThreeB1 = new ThreeBallAuto(turret,bbshooter,intakeElbow,indexer,intakeWheels,driveTrain,m_trajectories[5],m_trajectories[6]);

    this.m_chooser = new SendableChooser<Command>();

    this.m_chooser.setDefaultOption("One Ball 1", this.OneB1);
    this.m_chooser.addOption("One Ball 2", this.OneB2);
    this.m_chooser.addOption("One Ball 3", this.OneB3);
    this.m_chooser.addOption("Two Ball 1", this.TwoB1);
    this.m_chooser.addOption("Two Ball 2", this.TwoB2);
    this.m_chooser.addOption("Three Ball", this.ThreeB1);

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
    JoystickButton buttonSelect_dr = new JoystickButton(driver, Constants.OIConstants.buttonSelect);
    AxisJoystickButton triggerRight = new AxisJoystickButton(driver, Constants.OIConstants.rightTrigger, 0.5, ThresholdType.GREATER_THAN);
    
    triggerRight.whenPressed(new InstantCommand(bombardier::targetNoParams,bombardier));
    triggerRight.whenReleased(new InstantCommand(bombardier::stopTargetNoParams,bombardier));

    buttonA_dr.whenPressed(new InstantCommand(elevator::up,elevator));
    buttonY_dr.whenPressed(new InstantCommand(elevator::down, elevator));

    // in an emergency allow user to take over control
    buttonSelect_dr.whenPressed(new InstantCommand(bombardier::togglFailSafe,bombardier));

    JoystickButton buttonA_as = new JoystickButton(assist, Constants.OIConstants.buttonA);
    JoystickButton buttonY_as = new JoystickButton(assist, Constants.OIConstants.buttonY);
    JoystickButton buttonX_as = new JoystickButton(assist, Constants.OIConstants.buttonX);
    JoystickButton buttonB_as = new JoystickButton(assist, Constants.OIConstants.buttonB);

    JoystickButton buttonBumperRight_as = new JoystickButton(assist, Constants.OIConstants.buttonBumperRight);
    JoystickButton buttonBumperLeft_as = new JoystickButton(assist, Constants.OIConstants.buttonBumperLeft);

    buttonX_as.whenPressed(new InstantCommand(intakeWheels::on,intakeWheels));
    buttonB_as.whenPressed(new InstantCommand(intakeWheels::off,intakeWheels));

    buttonA_as.whenPressed(new InstantCommand(intakeElbow::down,intakeElbow));
    buttonY_as.whenPressed(new InstantCommand(intakeElbow::home,intakeElbow));

    buttonBumperRight_as.whenPressed(new InstantCommand(intakeWheels::reverse,intakeWheels));
    buttonBumperRight_as.whenReleased(new InstantCommand(intakeWheels::returnToPrevState,intakeWheels));

    buttonBumperLeft_as.whenPressed(new InstantCommand(indexer::reverseIndexer, indexer));
    buttonBumperLeft_as.whenReleased(new InstantCommand(indexer::off, indexer));

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
}
