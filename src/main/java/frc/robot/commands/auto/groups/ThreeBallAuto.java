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
import frc.robot.subsystems.BangBangShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeElbow;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Turret;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class ThreeBallAuto extends SequentialCommandGroup {

  private BangBangShooter m_shooter;
  private IntakeElbow m_intakeElbow;
  private Indexer m_Indexer;
  private IntakeWheels m_intakeWheels;
  private DriveTrain m_drivetrain;
  private Trajectory m_trajectory1;
  private Trajectory m_trajectory2;
  private Turret m_turret;

  /** Creates a new AutoTest3. */
  public ThreeBallAuto(Turret in_turret, BangBangShooter in_shooter, IntakeElbow in_intakeElbow, Indexer in_Indexer, IntakeWheels in_intakeWheels, DriveTrain in_drivetrain, Trajectory in_trajectory1, Trajectory in_trajectory2) {

    m_shooter = in_shooter;
    m_intakeElbow = in_intakeElbow;
    m_Indexer = in_Indexer;
    m_intakeWheels = in_intakeWheels;
    m_drivetrain = in_drivetrain;
    m_trajectory1 = in_trajectory1;
    m_trajectory2 = in_trajectory2;
    m_turret = in_turret;

    var leftController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
    var rightController = new PIDController(DriveConstants.kPDriveVel, 0, 0);

    var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
    var leftReference = table.getEntry("left_reference");
    var leftMeasurement = table.getEntry("left_measurement");
    var rightReference = table.getEntry("right_reference");
    var rightMeasurement = table.getEntry("right_measurement");

    RamseteCommand ramseteCommand1 = new RamseteCommand(
      m_trajectory1,
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

      RamseteCommand ramseteCommand2 = new RamseteCommand(
        m_trajectory2,
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
    m_drivetrain.resetOdometry(m_trajectory1.getInitialPose());
    

    addCommands(
      new InstantCommand(m_intakeElbow::down,m_intakeElbow),
      new InstantCommand(m_Indexer::setAutoIndexOff, m_Indexer),
      new InstantCommand(m_shooter::setShotRpmClose,m_shooter),
      new InstantCommand(m_shooter::enable,m_shooter),
      new WaitCommand(.5),
      new InstantCommand(m_Indexer::shoot, m_Indexer),
      new WaitCommand(1.5),
      new InstantCommand(m_shooter::disable,m_shooter),
      new InstantCommand(m_Indexer::off, m_Indexer),
      // new InstantCommand(m_turret::turn30degreesPositive,m_turret),
      // new InstantCommand(m_turret::turn5degreesPositive,m_turret),
      new InstantCommand(m_Indexer::setAutoIndexOn, m_Indexer),
      new InstantCommand(m_intakeWheels::on,m_intakeWheels),

      ramseteCommand1.andThen(new InstantCommand(m_drivetrain::setBrake,m_drivetrain).andThen(new InstantCommand(m_drivetrain::tankDriveVoltageStop,m_drivetrain))),

      new InstantCommand(m_shooter::enable,m_shooter),
      new WaitCommand(.5),
      new InstantCommand(m_intakeWheels::off,m_intakeWheels),
      new InstantCommand(m_Indexer::setAutoIndexOff, m_Indexer),
      new InstantCommand(m_Indexer::shoot, m_Indexer),
      new WaitCommand(1.5),
      new InstantCommand(m_shooter::disable,m_shooter),
      new InstantCommand(m_Indexer::off, m_Indexer),
      // new InstantCommand(m_turret::turn5degreesNegative,m_turret),
      // new InstantCommand(m_turret::turn30degreesNegative,m_turret),
      new InstantCommand(m_Indexer::setAutoIndexOn, m_Indexer),
      new InstantCommand(m_intakeWheels::on,m_intakeWheels),

      ramseteCommand2.andThen(new InstantCommand(m_drivetrain::setBrake,m_drivetrain).andThen(new InstantCommand(m_drivetrain::tankDriveVoltageStop,m_drivetrain))),

      new InstantCommand(m_shooter::enable,m_shooter),
      new WaitCommand(.5),
      new InstantCommand(m_intakeWheels::off,m_intakeWheels),
      new InstantCommand(m_Indexer::setAutoIndexOff, m_Indexer),
      new InstantCommand(m_Indexer::shoot, m_Indexer),
      new WaitCommand(1.5),
      new InstantCommand(m_Indexer::off, m_Indexer),
      new InstantCommand(m_shooter::disable,m_shooter),
      new InstantCommand(m_shooter::setIsTele,m_shooter),

      // TODO: remove this prior to comp
      new InstantCommand(m_intakeElbow::up,m_intakeElbow)
    );
  }
}
