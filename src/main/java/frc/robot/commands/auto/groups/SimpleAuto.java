// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.BangBangShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleAuto extends SequentialCommandGroup {
  /** Creates a new SimpleAuto. */

  private DriveTrain m_drivetrain;
  private BangBangShooter m_shooter;
  private Indexer m_Indexer;

  public SimpleAuto(Indexer in_Indexer, BangBangShooter in_Shooter, DriveTrain in_drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drivetrain = in_drivetrain;
    m_shooter = in_Shooter;
    m_Indexer = in_Indexer;

    addCommands(
      // new InstantCommand(m_intakeElbow::down,m_intakeElbow),
      new InstantCommand(m_Indexer::setAutoIndexOff,m_Indexer),
      new RunCommand(m_shooter::autoShoot, m_shooter).withTimeout(3),
      new ParallelCommandGroup(     
        new RunCommand(m_shooter::autoShoot, m_shooter).withTimeout(2),
        new InstantCommand(m_Indexer::shoot, m_Indexer)),
      new WaitCommand(2),
      new InstantCommand(m_Indexer::off, m_Indexer),
      new RunCommand(m_drivetrain::tankDriveVoltsNoParams, m_drivetrain).withTimeout(2),
      new InstantCommand(m_drivetrain::tankDriveVoltageStop, m_drivetrain), 
      new InstantCommand(m_drivetrain::setBrake, m_drivetrain),     
      new InstantCommand(m_Indexer::setAutoIndexOn,m_Indexer)
    );
  }
}
