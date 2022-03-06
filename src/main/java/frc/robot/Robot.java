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
  String OneB1_dir = "SingleBall_Path1.wpilib.json";
  String OneB2_dir = "SingleBall_Path2.wpilib.json";
  String OneB3_dir = "SingleBall_Path3.wpilib.json";
  String TwoB1_dir = "TwoBall1_Path.wpilib.json";
  String TwoB2_dir = "TwoBall2_Path.wpilib.json";
  String ThreeB1A_dir = "ThreeBall1A_Path.wpilib.json";
  String ThreeB1B_dir = "ThreeBall1B_Path.wpilib.json";

  Trajectory Traj_OneB1, Traj_OneB2, Traj_OneB3 , Traj_TwoB1, Traj_TwoB2 ,Traj_ThreeB1A ,Traj_ThreeB1B;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    System.out.println("Robot Init");
    m_robotContainer = new RobotContainer(this.loadTrajectories());
    m_robotContainer.disabledLEDS();

    CameraServer.startAutomaticCapture();

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

    Trajectory[] trajectories = new Trajectory[7];
    
    try {
      Path Path_OneB1 = Filesystem.getDeployDirectory().toPath().resolve(basePath + OneB1_dir );
      Traj_OneB1 = TrajectoryUtil.fromPathweaverJson(Path_OneB1);
      System.out.println("One ball 1 open");
      trajectories[0] = Traj_OneB1;

      Path Path_OneB2 = Filesystem.getDeployDirectory().toPath().resolve(basePath + OneB2_dir );
      Traj_OneB2 = TrajectoryUtil.fromPathweaverJson(Path_OneB2);
      System.out.println("One ball 2 open");
      trajectories[1] = Traj_OneB2;

      Path Path_OneB3 = Filesystem.getDeployDirectory().toPath().resolve(basePath + OneB3_dir );
      Traj_OneB3 = TrajectoryUtil.fromPathweaverJson(Path_OneB3);
      System.out.println("Two ball 3 open");
      trajectories[2] = Traj_OneB3;

      Path Path_TwoB1 = Filesystem.getDeployDirectory().toPath().resolve(basePath + TwoB1_dir );
      Traj_TwoB1 = TrajectoryUtil.fromPathweaverJson(Path_TwoB1);
      System.out.println("One ball 1 open");
      trajectories[3] = Traj_TwoB1;

      Path Path_TwoB2 = Filesystem.getDeployDirectory().toPath().resolve(basePath + TwoB2_dir );
      Traj_TwoB2 = TrajectoryUtil.fromPathweaverJson(Path_TwoB2);
      System.out.println("Two ball 2 open");
      trajectories[4] = Traj_TwoB2;

      Path Path_ThreeB1A = Filesystem.getDeployDirectory().toPath().resolve(basePath + ThreeB1A_dir );
      Traj_ThreeB1A = TrajectoryUtil.fromPathweaverJson(Path_ThreeB1A);
      System.out.println("Three ball 1A open");
      trajectories[5] = Traj_ThreeB1A;

      Path Path_ThreeB1B = Filesystem.getDeployDirectory().toPath().resolve(basePath + ThreeB1B_dir );
      Traj_ThreeB1B = TrajectoryUtil.fromPathweaverJson(Path_ThreeB1B);
      System.out.println("Three ball 1B open");
      trajectories[6] = Traj_ThreeB1B;

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
      System.err.println("Unable to open trajectory\n"+ ex.getStackTrace());
    }
    return trajectories;
  }
}
