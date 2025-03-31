// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * RunIntake.
 */
public class RunAlgaeIntake extends Command {

    private ManipulatorSubsystem m_manipulator;
    private LinearFilter m_filter = LinearFilter.movingAverage(10);
    // TODO: add calibration values for the linear filter input buffer
    private double[] m_inputBuffer = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0};
    private double[] m_outputBuffer = {};
    
    /**
     * RunIntake command constructor.
     */
    public RunAlgaeIntake(ManipulatorSubsystem manipulator) {
        m_manipulator = manipulator;
        addRequirements(m_manipulator);
    }

    @Override
    public void initialize() {
        m_manipulator.updateSetpoint(
            ManipulatorCalibrations.kAlgaeIntakeVelocity, ManipulatorCalibrations.kCoralAcceleration);
        m_filter.reset(m_inputBuffer, m_outputBuffer);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulator.setOpenLoopDutyCycle(-.03);
        //m_manipulator.updateSetpoint(
        //    ManipulatorCalibrations.kAlgaeHoldingVelocity, ManipulatorCalibrations.kCoralAcceleration);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        var deltaCurrent = m_manipulator.getStatorCurrent() - m_filter.lastValue();
        m_filter.calculate(m_manipulator.getStatorCurrent());
        
        return false; //deltaCurrent > ManipulatorCalibrations.kAlgaeIntakeThreshold;
        
    }

}
