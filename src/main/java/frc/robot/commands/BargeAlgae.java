// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * BargeAlgae command.
 */
public class BargeAlgae extends Command {
  
    private WindmillSubsystem m_windmill;
    private ElevatorSubsystem m_elevator;

    /**
     * BargeAlgae command constructor.
     */
    public BargeAlgae(ElevatorSubsystem elevator, WindmillSubsystem windmill) {
        m_windmill = windmill;
        m_elevator = elevator;
        addRequirements(m_windmill, m_elevator);
    }

    @Override
    public void initialize() {
        m_windmill.updateSetpoint(WindmillCalibrations.kBargePosition, false);
        if (m_windmill.isWithinTolerance(WindmillCalibrations.kBargeTolerance)) {
            m_elevator.updateSetpoint(ElevatorCalibrations.kBargePosition, false);
        }
    }
  
    @Override
     public void execute() {
        if (m_windmill.isWithinTolerance(WindmillCalibrations.kBargeTolerance)) {
            m_elevator.updateSetpoint(ElevatorCalibrations.kBargePosition, false);
        }
    }
  
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_elevator.isWithinTolerance(ElevatorCalibrations.kBargePositionTolerance)
            && m_windmill.isWithinTolerance(WindmillCalibrations.kBargeTolerance);
    }

}
