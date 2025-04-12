// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * RunIntake.
 */
public class RunAlgaeIntake extends Command {

    private ManipulatorSubsystem m_manipulator;
    private boolean m_isAtVelocity = false;
    
    /**
     * RunIntake command constructor.
     */
    public RunAlgaeIntake(ManipulatorSubsystem manipulator) {
        m_manipulator = manipulator;
        addRequirements(m_manipulator);
    }

    @Override
    public void initialize() {
        m_isAtVelocity = false;
        m_manipulator.updateSetpoint(
            ManipulatorCalibrations.kAlgaeIntakeVelocity, ManipulatorCalibrations.kCoralAcceleration);
    }

    @Override
    public void execute() {
        if (Math.abs(m_manipulator.getVelocity()) > 15.0) {
            m_isAtVelocity = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        LEDSubsystem.setManipulatorReady();
        m_manipulator.setOpenLoopDutyCycle(ManipulatorCalibrations.kAlgaeHoldDutyCycle);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (m_isAtVelocity 
            && (Math.abs(m_manipulator.getVelocity()) < ManipulatorCalibrations.kIntakeAlgaeZeroTolerance));
    }

}
