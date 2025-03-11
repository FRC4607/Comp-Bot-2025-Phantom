// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.Utils;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * L4 command.
 */
public class L4 extends SequentialCommandGroup {
  
    private WindmillSubsystem m_windmill;
    private ElevatorSubsystem m_elevator;

    /**
     * CoralStation command constructor.
     */
    public L4(ElevatorSubsystem elevator, WindmillSubsystem windmill) {
        super(
            new MoveElevatorToPosition(ElevatorCalibrations.kL4Position, false, elevator),
            new MoveWindmillToPosition(WindmillCalibrations.kL4Position, false, windmill)
        );
    }

}
