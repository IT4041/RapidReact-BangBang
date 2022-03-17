/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public RobotContainer m_robotContainer;

  String basePath = "pathplanner/generatedJSON/";
  String OneB1_dir = "OneBall1_Path.wpilib.json";
  String TwoB1_dir = "TwoBall1_Path.wpilib.json";
  String straightback_dir = "StraightBackPath.wpilib.json";

  Trajectory Traj_OneB1; 
  Trajectory Traj_TwoB1; 
  Trajectory traj_StraightBack;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    System.out.println("Robot Init");
    m_robotContainer = new RobotContainer(this.loadTrajectories());
    m_robotContainer.disabledLEDS();

    CameraServer.startAutomaticCapture();
    SmartDashboard.putString("Auto", "none");

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    m_robotContainer.disabledLEDS();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledLEDS();
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    SmartDashboard.putString("Auto", m_autonomousCommand.getName());

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.enableAutoIndexing();
    m_robotContainer.isTele();
                      
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    m_robotContainer.isTele();
    }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {}

  private Trajectory[] loadTrajectories(){

    Trajectory[] trajectories = new Trajectory[3];
    
    try {
      Path Path_OneB1 = Filesystem.getDeployDirectory().toPath().resolve(basePath + OneB1_dir );
      Traj_OneB1 = TrajectoryUtil.fromPathweaverJson(Path_OneB1);
      System.out.println("One ball 1 open");
      trajectories[0] = Traj_OneB1;

      Path Path_TwoB1 = Filesystem.getDeployDirectory().toPath().resolve(basePath + TwoB1_dir );
      Traj_TwoB1 = TrajectoryUtil.fromPathweaverJson(Path_TwoB1);
      System.out.println("One ball 1 open");
      trajectories[1] = Traj_TwoB1;

      Path Path_straightback = Filesystem.getDeployDirectory().toPath().resolve(basePath + straightback_dir );
      traj_StraightBack = TrajectoryUtil.fromPathweaverJson(Path_straightback);
      System.out.println("Straight Back open");
      trajectories[2] = traj_StraightBack;

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
      System.err.println("Unable to open trajectory\n"+ ex.getStackTrace());
    }
    return trajectories;
  }
}
