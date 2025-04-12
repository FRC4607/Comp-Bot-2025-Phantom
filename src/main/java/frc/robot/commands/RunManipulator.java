// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * RunOutake.
 */
public class RunManipulator extends Command {

    private ManipulatorSubsystem m_manipulator;
    private double m_outtakeSpeed;
    private double m_acceleration;
    
    /**
     * RunIntake command constructor.
     */
    public RunManipulator(double outtakeSpeed, double acceleration, ManipulatorSubsystem manipulator) {
        m_manipulator = manipulator;
        m_outtakeSpeed = outtakeSpeed;
        m_acceleration = acceleration;
        addRequirements(m_manipulator);
    }

    @Override
    public void initialize() {
        m_manipulator.updateSetpoint(m_outtakeSpeed, m_acceleration);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        LEDSubsystem.setNeutral();
        m_manipulator.updateSetpoint(0, m_acceleration);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
