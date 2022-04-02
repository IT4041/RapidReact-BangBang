// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.MasterContoller;

public class OneBallOldSchoolWithTargeting extends SequentialCommandGroup {

        private MasterContoller m_masterController;
        private DriveTrain m_drivetrain;

        public OneBallOldSchoolWithTargeting(MasterContoller in_masterController, DriveTrain in_drivetrain) {

                m_drivetrain = in_drivetrain;
                m_masterController = in_masterController;

                // drives backwards for 1 seconds at .15 speed,
                // drives forwards for 1 seconds at .15 speed, stops.

                addCommands(
                        new InstantCommand(m_masterController::enabledTargetingAndShooting, m_masterController),
                        new WaitCommand(3),
                        new InstantCommand(m_masterController::disabledTargetingAndShooting, m_masterController),
                        new InstantCommand(m_drivetrain::autoDriveBack, m_drivetrain),
                        new WaitCommand(2),
                        new InstantCommand(m_drivetrain::setBrake, m_drivetrain),
                        new InstantCommand(m_drivetrain::autoDriveStop, m_drivetrain),
                        new WaitCommand(2)
                );
        }
}
