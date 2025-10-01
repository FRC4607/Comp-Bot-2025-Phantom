// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * AlgaeFloorPickup command.
 */
public class AlgaeL2Pickup extends SequentialCommandGroup {

    /**
     * AlgaeFloorPickup command constructor.
     */
    public AlgaeL2Pickup(ElevatorSubsystem elevator, WindmillSubsystem windmill, ManipulatorSubsystem manipulator) {
        super(
            // TODO: Add tolerances
            new ConditionalCommand(
                new SequentialCommandGroup(
                    //Under L2
                    //First go to the stow position if not already there
                    new ConditionalCommand(
                        //If the elevator is not already at the stow position go to the stow position
                        new PendulumStow(elevator, windmill),
                        //If the elevator is already at the stow position Do Nothing
                        new InstantCommand(),
                        //If the elevator is above the stow position - Tolerance
                        () -> (elevator.getPosition()
                                < ElevatorCalibrations.kPendulumPosition - ElevatorCalibrations.kPendulumTolerance)
                    ),
                    //Then go to the under L2 position
                    new MoveWindmillToPosition(
                        WindmillCalibrations.kAlgaeUnderL2Position, 
                        ElevatorCalibrations.kDefaultTolerance, 
                        false, windmill),
                    new MoveElevatorToPosition(
                        ElevatorCalibrations.kAlgaeUnderL2Position, 
                        ElevatorCalibrations.kDefaultTolerance, 
                        false, elevator)),
                new SequentialCommandGroup(
                    //Over L2
                    new MoveElevatorToPosition(
                        ElevatorCalibrations.kAlgaeOverL2Position, ElevatorCalibrations.kDefaultTolerance, false, elevator),
                    new MoveWindmillToPosition(
                        WindmillCalibrations.kAlgaeOverL2Position, ElevatorCalibrations.kDefaultTolerance, false, windmill)),
                    () -> Math.abs(windmill.getPosition() - WindmillCalibrations.kPendulumPosition) < 90),

            new RunAlgaeIntake(manipulator)
        );
    }
}
