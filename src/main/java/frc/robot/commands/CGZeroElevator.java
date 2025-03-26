// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/**
 * CGZeroElevator command.
 */
public class CGZeroElevator extends SequentialCommandGroup {
    /** Creates a new CGZeroElevator. */
    public CGZeroElevator(ElevatorSubsystem elevator, WindmillSubsystem windmill) {
        super(
            //Force the Arm to the Lollipop Position
            new RunCommand(() -> windmill.updateSetpoint(WindmillCalibrations.kLollipopPosition, false)),
            //now zero the elevator
            new ZeroElevator(elevator)
        );
    }
}
