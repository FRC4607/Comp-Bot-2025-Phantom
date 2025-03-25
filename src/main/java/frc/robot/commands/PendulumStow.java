// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * Pendulum Stow Command.
 */
public class PendulumStow extends SequentialCommandGroup {

    /**
     * Pendulum Stow Command Constructor.
     */
    public PendulumStow(ElevatorSubsystem elevator, WindmillSubsystem windmill) {
        super(
            new InstantCommand(() -> elevator.setServoAngle(ElevatorCalibrations.kservoUnlockAngle)),
            new ParallelCommandGroup(
                new MoveElevatorToPosition(
                    ElevatorCalibrations.kPendulumPosition, ElevatorCalibrations.kPendulumTolerance, false, elevator),
                new MoveWindmillToPosition(
                    WindmillCalibrations.kPendulumPrepPosition, WindmillCalibrations.kPendulumPrepTolerance, false, windmill)
            ),
            new MoveWindmillToPosition(
                WindmillCalibrations.kPendulumPosition, WindmillCalibrations.kPendulumTolerance, false, windmill)
        );
    }
}
