// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.DriverCalibrations;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * AlignToTag command.
 */
public class TranslatationYRobotCentric extends Command {

    private CommandSwerveDrivetrain m_drivetrain;
    private RobotCentric m_swerveRequest = new RobotCentric().withRotationalDeadband(DriverCalibrations.kmaxSpeed * 0.1);

    /**
     * TranslatationYRobotCentric Constructor.
     *
     * @param drivetrain The drivetrain
     */
    public TranslatationYRobotCentric(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // Apply the robot-centric translation speeds
        m_drivetrain.setControl(m_swerveRequest.withVelocityX(0.0)
                                            .withVelocityY(DriverCalibrations.kRobotCentricTranslationYRate));

    }
    
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(m_swerveRequest.withVelocityX(0.0)
                                            .withVelocityY(0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
