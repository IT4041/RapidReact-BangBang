// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.DriveTrain;

public class OneBallOldSchool extends SequentialCommandGroup {

        private DriveTrain m_drivetrain;

        public OneBallOldSchool(DriveTrain in_drivetrain) {
                
                m_drivetrain = in_drivetrain;
                

                //drives backwards for 3 seconds at .25 speed, 
                //drives forwards for 3 seconds at .25 speed, stops.

                addCommands( new InstantCommand(m_drivetrain::autoDriveBack, m_drivetrain),
                new WaitCommand(3),
                new InstantCommand(m_drivetrain::setBrake, m_drivetrain).andThen(new InstantCommand(m_drivetrain::setCoast, m_drivetrain)),
                new InstantCommand(m_drivetrain::autoDriveForward, m_drivetrain),
                new WaitCommand(3),
                new InstantCommand(m_drivetrain::setBrake, m_drivetrain),
                new InstantCommand(m_drivetrain::autoDriveStop, m_drivetrain),
                new InstantCommand(m_drivetrain::setCoast, m_drivetrain)
                        );
        }
}
