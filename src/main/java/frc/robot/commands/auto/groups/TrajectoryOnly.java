// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class TrajectoryOnly extends SequentialCommandGroup {

        private Trajectory m_Trajectory2;
        private DriveTrain m_drivetrain;

        /** Creates a new AutoTest2. */
        public TrajectoryOnly(DriveTrain in_drivetrain, Trajectory in_trajectory2) {
                
                m_Trajectory2 = in_trajectory2;
                m_drivetrain = in_drivetrain;

                var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
                var leftReference = table.getEntry("left_reference");
                var leftMeasurement = table.getEntry("left_measurement");
                var rightReference = table.getEntry("right_reference");
                var rightMeasurement = table.getEntry("right_measurement");

                var leftController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
                var rightController = new PIDController(DriveConstants.kPDriveVel, 0, 0);

                RamseteCommand ramseteCommand2 = new RamseteCommand(
                                m_Trajectory2,
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
                m_drivetrain.resetOdometry(m_Trajectory2.getInitialPose());

                addCommands(
                        ramseteCommand2
                        );
        }
}
