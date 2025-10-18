// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.datalog.DataLog;
import edu.wpi.first.datalog.IntegerLogEntry;
import edu.wpi.first.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.Utility.batteryCAN;
import frc.robot.subsystems.LEDSubsystem;
import java.util.Optional;
import edu.wpi.first.wpilibj.PowerDistribution;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.datalog.StringLogEntry;


/**
 * Robot.
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;
    StringLogEntry eventName;
    IntegerLogEntry matchNumber;
    private batteryCAN batteryCan;

    public Robot() {

        batteryCan = new batteryCAN(0,33, 0 , PowerDistribution.ModuleType.kRev);

        m_robotContainer = new RobotContainer();
        //CTRE Logger Setup
        SignalLogger.setPath("/home/systemcore/logs/ctre-logs/");
        SignalLogger.start();

        // Starts recording to data log
        DataLogManager.start();
        DataLog wpilog = DataLogManager.getLog();
        // Record both DS control and joystick data
        DriverStation.startDataLog(wpilog);
        // (alternatively) Record only DS control data
        DriverStation.startDataLog(wpilog, false);

        eventName = new StringLogEntry(wpilog, "EVENT/Event Name");
        matchNumber = new IntegerLogEntry(wpilog, "EVENT/Match Number");

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (batteryCan != null && batteryCan.valid) {
            SmartDashboard.putString("Battery SN", batteryCan.serialNumber);
            SmartDashboard.putString("Battery First Use (UTC)", 
                String.format("%04d-%02d-%02d",
                    batteryCan.firstUseYear,
                    batteryCan.firstUseMonth,
                    batteryCan.firstUseDay
                )
            );
            SmartDashboard.putString("Battery Note", batteryCan.noteText);
            SmartDashboard.putNumber("Battery Cycle Count", batteryCan.cycleCount);
        } else {
            SmartDashboard.putString("Battery SN", "INVALID");
            SmartDashboard.putString("Battery First Use (UTC)", "0000-00-00");
            SmartDashboard.putString("Battery Note", "INVALID");
            SmartDashboard.putNumber("Battery Cycle Count", 0);
        }

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
        SignalLogger.writeString("Event Name", DriverStation.getEventName());
        SignalLogger.writeInteger("Match Number", DriverStation.getMatchNumber());
        eventName.append(DriverStation.getEventName());
        matchNumber.append(DriverStation.getMatchNumber());

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
        Elastic.selectTab("Teleoperated");
        //Shuffleboard.selectTab("Teleoperated");
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
