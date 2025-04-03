// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.subsystems.LEDSubsystem;
import java.util.Optional;

/**
 * Robot.
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {

        m_robotContainer.m_elevator.disableServo();
        DriverStation.reportError("m_robotContainer.m_leds.getState().toString(): ", false);
        DriverStation.reportError(m_robotContainer.m_leds.getState().toString(), false);
       
        if (m_robotContainer.m_leds.getState() == LEDSubsystem.LEDSubsystemState.CLIMB_COMPLETE) {
            LEDSubsystem.setClimb_Complete();
        } else {
            LEDSubsystem.setDisabled();
        }
    }

    @Override
    public void disabledPeriodic() {
        Optional<Alliance> value = DriverStation.getAlliance();
        if (m_robotContainer.m_leds.getState() == LEDSubsystem.LEDSubsystemState.CLIMB_COMPLETE) {
            // TODO: What was supposed to be here?  It was left empty.
        } else {
            if (value.isPresent()) {
                LEDSubsystem.setAlliance(value.get());
            }
            if (!m_robotContainer.m_elevator.getCANdiState()) {
                LEDSubsystem.setError();
            } else {
                LEDSubsystem.setDisabled();
            }
        }
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_robotContainer.m_windmill.updateSetpoint(m_robotContainer.m_windmill.getPosition(), false);
        m_robotContainer.m_elevator.updateSetpoint(m_robotContainer.m_elevator.getPosition(), false);
        m_robotContainer.m_manipulator.updateSetpoint(0, ManipulatorCalibrations.kCoralAcceleration);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        LEDSubsystem.setNeutral();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        Shuffleboard.selectTab("Teleoperated");
        m_robotContainer.m_windmill.updateSetpoint(m_robotContainer.m_windmill.getPosition(), false);
        m_robotContainer.m_elevator.updateSetpoint(m_robotContainer.m_elevator.getPosition(), false);
        m_robotContainer.m_manipulator.updateSetpoint(0, ManipulatorCalibrations.kCoralAcceleration);
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        LEDSubsystem.setNeutral();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
