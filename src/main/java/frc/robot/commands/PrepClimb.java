// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.Utils;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * Deep climb placement command group.
 */
public class PrepClimb extends Command {

    private final ElevatorSubsystem m_elevator;
    private final WindmillSubsystem m_windmill;

    /** Deep climb placement command group constructor.
     *
     * @param elevator the elevator
     * @param windmill the windmill
     */
    public PrepClimb(ElevatorSubsystem elevator, WindmillSubsystem windmill) {
        // TODO: Make this a sequential command group
        m_elevator = elevator;
        m_windmill = windmill;
        addRequirements(m_windmill, m_elevator);
    }

    @Override
    public void initialize() {
        Shuffleboard.selectTab("CLIMB CAM");
        
        NetworkTableInstance.getDefault().getTable("limelight-two").getEntry("pipeline").setDouble(1);
        m_windmill.updateSetpoint(WindmillCalibrations.kPrepClimbPosition, false);
        if (m_windmill.isWithinTolerance(WindmillCalibrations.kPrepClimbTolerance)) {
            m_elevator.updateSetpoint(ElevatorCalibrations.kPrepClimbPosition, false);
        }

        new InstantCommand(() -> m_elevator.setServoAngle(ElevatorCalibrations.kservoUnlockAngle));
    }
  
    @Override
     public void execute() {
        if (m_windmill.isWithinTolerance(WindmillCalibrations.kPrepClimbTolerance)) {
            m_elevator.updateSetpoint(ElevatorCalibrations.kPrepClimbPosition, false);
        }
    }
  
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return Utils.isDoubleEqual(m_windmill.getSetpoint(), WindmillCalibrations.kPrepClimbPosition, 0.01)
               && Utils.isDoubleEqual(m_elevator.getSetpoint(), ElevatorCalibrations.kPrepClimbPosition, 0.01);
    }
}
