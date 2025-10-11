// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * AlgaeFloorPickup command.
 */
public class AlgaeFloorPickup extends SequentialCommandGroup {

    /**
     * AlgaeFloorPickup command constructor.
     */
    public AlgaeFloorPickup(ElevatorSubsystem elevator, WindmillSubsystem windmill, ManipulatorSubsystem manipulator) {
        super(
            // TODO: Swap the elevator and windmill move commands - Resulted in a fatal error
            new MoveElevatorToPosition(
                ElevatorCalibrations.kAlgaePickupPosition, ElevatorCalibrations.kDefaultTolerance, false, elevator),
            new MoveWindmillToPosition(
                WindmillCalibrations.kAlgaePickupPosition, WindmillCalibrations.kDefaultTolerance, false, windmill),
            new RunAlgaeIntake(manipulator)//, 
        // Eli request not to automate this 1/12/2025
        //new LollipopStow(elevator, windmill)
        );
    }
}
