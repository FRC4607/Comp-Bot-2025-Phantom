// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.DriverCalibrations;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;


/**
 * AlignToTag command.
 */
public class TranslationAlignToTag extends Command {

    private CommandSwerveDrivetrain m_drivetrain;
    private double m_xspeed;
    private int m_pipeline;
    private double m_txValue;

    private final ProfiledPIDController m_profiledPid = 
        new ProfiledPIDController(DriverCalibrations.kAprilTagTranslationXAlignmentKP,
                                0.0, 
                                DriverCalibrations.kAprilTagTranslationXAlignmentKD,
                                new TrapezoidProfile.Constraints(1, .05));

    private RobotCentric m_swerveRequest = new RobotCentric().withRotationalDeadband(DriverCalibrations.kmaxSpeed * 0.1);

    /**
     * AlignToTag Constructor.
     *
     * @param drivetrain The drivetrain
     */
    public TranslationAlignToTag(int pipeline, CommandSwerveDrivetrain drivetrain) {
        m_pipeline = pipeline;
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        NetworkTableInstance.getDefault().getTable("limelight-one").getEntry("pipeline").setDouble(m_pipeline);
    }

    @Override
    public void execute() {
        //Check if there is a target
        double isTarget = NetworkTableInstance.getDefault().getTable("limelight-one").getEntry("tv").getDouble(0);
        //Get the Error
        m_txValue = NetworkTableInstance.getDefault().getTable("limelight-one").getEntry("tx")
                                             .getDouble(DriverCalibrations.kLimelightDefaultKTx);

        //Invert the error and calculate PID
        m_xspeed = m_profiledPid.calculate(-m_txValue);
        
        m_drivetrain.setControl(m_swerveRequest
            .withVelocityX(m_xspeed)
            .withVelocityY(DriverCalibrations.kAprilTagTranslationYRate));

        if (isTarget > 0) {
            if (Math.abs(m_txValue) < 1.75) {
                LEDSubsystem.setCoralOnTarget();
            } else {
                LEDSubsystem.setCoralTargeting();
            }
        } else {
            LEDSubsystem.setNeutral();
        }
        
    }
    
    @Override
    public void end(boolean interrupted) {
        LEDSubsystem.setNeutral();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
