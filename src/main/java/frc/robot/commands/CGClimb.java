// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * Deep climb pullup command group.
 */
public class CGClimb extends SequentialCommandGroup {

    /** Deep climb command group constructor.
     *
     * @param elevator the elevator
     * @param windmill the windmill
     */
    public CGClimb(WindmillSubsystem windmill, ElevatorSubsystem elevator) {
        super(
            // Make sure the servo is unlocked
            new InstantCommand(() -> elevator.setServoAngle(ElevatorCalibrations.kservoUnlockAngle)),
            // Lift to the climb position
            new MoveElevatorToPosition(
                ElevatorCalibrations.kClimbUpSetpoint, ElevatorCalibrations.kClimbUpTolerance, true, elevator),
            new MoveWindmillToPosition(
                WindmillCalibrations.kClimbPosition, WindmillCalibrations.kClimbTolerance, true, windmill),
            // Make sure the servo is unlocked
            new InstantCommand(() -> elevator.setServoAngle(ElevatorCalibrations.kservoUnlockAngle)),
            // Finish the climb
            new ParallelRaceGroup(
                new MoveElevatorToPosition(
                    ElevatorCalibrations.kClimbDownSetpoint, ElevatorCalibrations.kClimbDownTolerance, true, elevator),
                new LockElevatorWhenDown(elevator)
            ),
            // Hold the climb position for a few seconds
            new WaitCommand(3),
            // turn off the motors
            new InstantCommand(elevator::setElevatorZeroDutyCycle)

        );
    }
}
