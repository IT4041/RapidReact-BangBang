package frc.robot.commands.auto.groups;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.MasterContoller;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class SingleBallAutoWithTargeting extends SequentialCommandGroup {

  private MasterContoller m_masterController;
  private DriveTrain m_drivetrain;
  private Trajectory m_trajectory;

  public SingleBallAutoWithTargeting(MasterContoller in_masterController, DriveTrain in_drivetrain, Trajectory in_trajectory) {

    m_masterController = in_masterController;
    m_drivetrain = in_drivetrain;
    m_trajectory = in_trajectory;


    var leftController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
    var rightController = new PIDController(DriveConstants.kPDriveVel, 0, 0);

    var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
    var leftReference = table.getEntry("left_reference");
    var leftMeasurement = table.getEntry("left_measurement");
    var rightReference = table.getEntry("right_reference");
    var rightMeasurement = table.getEntry("right_measurement");

    RamseteCommand ramseteCommand = new RamseteCommand(
      m_trajectory,
      m_drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
                      DriveConstants.ksVolts,
                      DriveConstants.kvVoltSecondsPerMeter,
                      DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      m_drivetrain::getWheelSpeeds,
      leftController,
      rightController,
      // RamseteCommand passes volts to the callback
      (leftVolts, rightVolts) -> {
              m_drivetrain.tankDriveVolts(leftVolts, rightVolts);

              leftMeasurement.setNumber(m_drivetrain.getWheelSpeeds().leftMetersPerSecond);
              leftReference.setNumber(leftController.getSetpoint());

              rightMeasurement.setNumber(m_drivetrain.getWheelSpeeds().rightMetersPerSecond);
              rightReference.setNumber(rightController.getSetpoint());
      },
      m_drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(m_trajectory.getInitialPose());
    

    addCommands(
      new InstantCommand(m_masterController::enabledTargetingAndShooting,m_masterController),
      new WaitCommand(.5),
      new InstantCommand(m_masterController::disabledTargetingAndShooting,m_masterController),
      new InstantCommand(m_masterController::intakeWheelsOn,m_masterController),

      ramseteCommand.andThen(new InstantCommand(m_drivetrain::setBrake,m_drivetrain).andThen(new InstantCommand(m_drivetrain::tankDriveVoltageStop,m_drivetrain))),

      new InstantCommand(m_masterController::intakeWheelsOff,m_masterController),
      new InstantCommand(m_masterController::enabledTargetingAndShooting,m_masterController),
      new WaitCommand(.5),
      new InstantCommand(m_masterController::disabledTargetingAndShooting,m_masterController)

    );
  }
}
