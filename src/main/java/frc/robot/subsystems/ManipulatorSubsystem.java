// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.Constants.ManipulatorConstants;

/**
 * Manipulator Subsystem.
 */
public class ManipulatorSubsystem extends SubsystemBase {

    private final TalonFX m_motor;
    private final MotionMagicVelocityTorqueCurrentFOC m_request;
    private TalonFXConfiguration m_talonFxConfig;


    /**
     * Manipulator subsystem constructor.
     */
    public ManipulatorSubsystem() {

        /* Create the hardware and configurators */
        m_motor = new TalonFX(ManipulatorConstants.kmotorCanId, "kachow");
        m_request = new MotionMagicVelocityTorqueCurrentFOC(0);
        m_talonFxConfig = new TalonFXConfiguration();

        /* Configure the motor */
        m_talonFxConfig.Slot0.kS = ManipulatorCalibrations.kS;
        m_talonFxConfig.Slot0.kV = ManipulatorCalibrations.kV;
        m_talonFxConfig.Slot0.kA = ManipulatorCalibrations.kA;
        m_talonFxConfig.Slot0.kP = ManipulatorCalibrations.kP;
        m_talonFxConfig.Slot0.kD = ManipulatorCalibrations.kD;

        m_talonFxConfig.MotionMagic.MotionMagicAcceleration = ManipulatorCalibrations.kMaxAcceleration;

        m_talonFxConfig.TorqueCurrent.PeakForwardTorqueCurrent = ManipulatorCalibrations.kMaxStatorCurrent;
        m_talonFxConfig.TorqueCurrent.PeakReverseTorqueCurrent = -ManipulatorCalibrations.kMaxStatorCurrent;

        /* Apply hardware configurations */
        m_motor.getConfigurator().apply(m_talonFxConfig);
    }

    /**
    * Sets the velocity of the Manipulator (rps) using MotionMagic VelocityTorqueCurrentFOC.
    *
    * @param newSetpoint New velocity setpoint for the manipulator
    */
    public void updateSetpoint(double newSetpoint) {
        m_motor.setControl(m_request.withVelocity(newSetpoint));
    }

    /**
     * Returns the velocity of the Manipulator (rps).
     *
     * @return The current velocity of the manipulator (rps).
     */
    public double getVelocity() {
        return m_motor.getVelocity().getValueAsDouble();
    }

    /**
     * Returns the stator current (amps) of the manipulator motor.
     *
     * @return The stator current (amps) of the Manipulator motor.
     */
    public double getStatorCurrent() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
