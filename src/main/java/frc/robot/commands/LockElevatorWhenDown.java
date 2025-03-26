// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Lock elevator when down command.
 */
public class LockElevatorWhenDown extends Command {
    private ElevatorSubsystem m_elevator;

    public LockElevatorWhenDown(ElevatorSubsystem elevator) {
        m_elevator = elevator;
        
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        m_elevator.setServoAngle(ElevatorCalibrations.kservoLockAngle);
        LEDSubsystem.setClimb_Complete();
        NetworkTableInstance.getDefault().getTable("limelight-two").getEntry("pipeline").setDouble(0);
    }

    @Override
    public boolean isFinished() {
        return m_elevator.getCANdiState();
    }
}
