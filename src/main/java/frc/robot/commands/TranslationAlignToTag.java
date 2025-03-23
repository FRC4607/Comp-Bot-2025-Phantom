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
import frc.robot.Calibrations.FieldCalibrations;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;


/**
 * AlignToTag command.
 */
public class TranslationAlignToTag extends Command {

    private CommandSwerveDrivetrain m_drivetrain;
    private int m_branch;
    private double m_xspeed;
    private double m_yspeed;
    private double m_targetTx;
    private double m_currentTx;
    private int m_tagId;
    private int m_lockedTagId;
    private boolean m_validTagId;
    private boolean m_onTarget;
    private RobotCentric m_swerveRequest = new RobotCentric().withRotationalDeadband(DriverCalibrations.kmaxSpeed * 0.1);
    private final ProfiledPIDController m_profiledPid = new ProfiledPIDController(
        DriverCalibrations.kAprilTagTranslationXAlignmentKP,
        0.0, 
        DriverCalibrations.kAprilTagTranslationXAlignmentKD,
        // TODO: Tune the max velocity/acceleration
        new TrapezoidProfile.Constraints(DriverCalibrations.kmaxSpeed, 2.0 * DriverCalibrations.kmaxSpeed));

    /**
     * AlignToTag Constructor.
     *
     * @param drivetrain The drivetrain
     */
    public TranslationAlignToTag(int branch, CommandSwerveDrivetrain drivetrain) {
        m_branch = branch;
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_onTarget = false;
        m_targetTx = 0.0;  // Once a valid target is found, use the hashmap to set this target
        m_lockedTagId = 0;  // Init with an invalid AprilTag ID
    }

    @Override
    public void execute() {
        m_tagId = (int) NetworkTableInstance.getDefault().getTable("limelight-one").getEntry("tid")
                                            .getInteger(0);  // Get the current AprilTag ID
        m_validTagId = FieldCalibrations.m_validTagIds.contains(m_tagId);  // Make sure it's a coral reef AprilTag ID

        // Default to doing nothing
        m_xspeed = 0.0;
        m_yspeed = 0.0;

        // Have a valid AprilTag ID
        if (m_validTagId) {
            
            // Lock onto the first valid AprilTag ID by setting the profiled PID goal
            if (m_lockedTagId == 0) {
                m_lockedTagId = m_tagId;
                m_targetTx = FieldCalibrations.m_coralReefTargets.get(m_lockedTagId).get(m_branch);
                m_profiledPid.setGoal(m_targetTx);
            }
            
            // Only servo to the locked AprilTag ID
            if (m_lockedTagId == m_tagId) {
                LEDSubsystem.setCoralTargeting();
                m_currentTx = NetworkTableInstance.getDefault().getTable("limelight-one").getEntry("tx")
                                                  .getDouble(DriverCalibrations.kLimelightDefaultKTx);
                // TODO: Verify the sign on the x-velocity
                m_xspeed = -m_profiledPid.calculate(m_currentTx);
                m_yspeed = DriverCalibrations.kAprilTagTranslationYRate;
                if (Math.abs(m_currentTx) < DriverCalibrations.kAprilTagTranslationXOnTarget) {
                    LEDSubsystem.setCoralOnTarget();
                    m_onTarget = true;
                }
            }
        }

        // Apply the robot-centric translation speeds
        m_drivetrain.setControl(m_swerveRequest.withVelocityX(m_xspeed).withVelocityY(m_yspeed));

    }
    
    @Override
    public void end(boolean interrupted) {
        // m_drivetrain.setControl(m_swerveRequest.withVelocityX(0.0).withVelocityY(0.0));
        LEDSubsystem.setNeutral();
    }

    @Override
    public boolean isFinished() {
        // return m_onTarget;
        return false;
    }
}
