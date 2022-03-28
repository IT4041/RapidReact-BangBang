package frc.robot.commands.auto.groups;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.MasterContoller;


import frc.robot.Constants.DriveConstants;

public class OneBallOldSchoolWithTargeting extends SequentialCommandGroup {

  private MasterContoller m_masterController;
  private DriveTrain m_drivetrain;

  public OneBallOldSchoolWithTargeting(MasterContoller in_masterController, DriveTrain in_drivetrain) {

    m_masterController = in_masterController;
    m_drivetrain = in_drivetrain;

    addCommands(
      // enabling Targetting and shooting puts intake elbow down
      new InstantCommand(m_masterController::enabledTargetingAndShooting,m_masterController),
      new WaitCommand(3),
      new InstantCommand(m_masterController::disabledTargetingAndShooting,m_masterController),
      new InstantCommand(m_masterController::intakeWheelsOn,m_masterController),
      //start old school drive backwards
      new InstantCommand(m_drivetrain::autoDriveBack, m_drivetrain),
      new WaitCommand(3),
      new InstantCommand(m_drivetrain::setBrake, m_drivetrain).andThen(new InstantCommand(m_drivetrain::setCoast, m_drivetrain)),
      new InstantCommand(m_drivetrain::autoDriveForward, m_drivetrain),
      new WaitCommand(3),
      new InstantCommand(m_drivetrain::setBrake, m_drivetrain),
      new InstantCommand(m_drivetrain::autoDriveStop, m_drivetrain),
      new InstantCommand(m_drivetrain::setCoast, m_drivetrain),
      //end old school drive backwards
      new InstantCommand(m_masterController::intakeWheelsOff,m_masterController),
      new InstantCommand(m_masterController::enabledTargetingAndShooting,m_masterController),
      new WaitCommand(3),
      new InstantCommand(m_masterController::disabledTargetingAndShooting,m_masterController)

    );
  }
}
