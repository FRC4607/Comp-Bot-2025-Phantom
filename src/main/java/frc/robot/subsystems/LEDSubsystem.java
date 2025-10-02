// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;

/**
 * LED substem.
 */
public class LEDSubsystem extends SubsystemBase {

    /**
     * LED state enumerations.
     */
    public static enum LEDSubsystemState {
        DISABLED,
        NEUTRAL,
        INTAKE,
        MANIPULATOR_NOT_READY,
        MANIPULATOR_READY,
        CLIMB_COMPLETE,
        CLIMB_ENABLED,
        CLIMB_HOOKED,
        CORAL_CLOSETOTARGET,
        CORAL_TARGETING,
        CORAL_ON_TARGET,
        ERROR
    }

    private final CANdle m_candle;
    CANdleConfiguration m_config;
    private static Alliance m_alliance;
    private static LEDSubsystemState m_currentState;
    private static LEDSubsystemState m_pastState;
    private final StrobeAnimation m_noAlliance;
    private final SingleFadeAnimation m_redDisabled;
    private final SingleFadeAnimation m_blueDisabled;
    private final StrobeAnimation m_intake;
    private final StrobeAnimation m_error;
    private final TwinkleOffAnimation m_manipulatorNotReady;
    private final StrobeAnimation m_manipulatorReady;
    private final LarsonAnimation m_climbLEFT;
    private final LarsonAnimation m_climbTOP;
    private final LarsonAnimation m_climbRIGHT;
    private static boolean colorUpdate = false;
    private final TwinkleOffAnimation m_coralCloseToTarget;
    private final TwinkleOffAnimation m_coralTargeting;
    private final StrobeAnimation m_coralOnTarget;

    /**
     * LED subsystem constructor.
     */
    public LEDSubsystem() {

        /* Create the hardware and configurator */
        m_candle = new CANdle(Constants.LEDConstants.kCANdleID, new CANBus("kachow"));
        m_config = new CANdleConfiguration();

        /* Configure hardware */
        m_config.LED.StripType = StripTypeValue.RGB;
        m_config.LED.BrightnessScalar = 1.0;
        m_config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        m_candle.getConfigurator().apply(m_config);

        /* Clear all animation slots */
        for (int i = 0; i < 8; i++) {
            m_candle.setControl(new EmptyAnimation(i));
        }

        SmartDashboard.putBoolean("Animation", false);

        /* Configure state */
        m_alliance = null;
        m_currentState = LEDSubsystemState.DISABLED;
        m_pastState = null;

        /* Configure animations */
        m_noAlliance = new StrobeAnimation(0, Constants.LEDConstants.kRGBCount)
            .withColor(new RGBWColor(255, 0, 255, 0))
            .withFrameRate(10);

        m_redDisabled = new SingleFadeAnimation(0, Constants.LEDConstants.kRGBCount)
            .withColor(new RGBWColor(255, 0, 0, 0))
            .withFrameRate(10);

        m_blueDisabled = new SingleFadeAnimation(0, Constants.LEDConstants.kRGBCount)
            .withColor(new RGBWColor(0, 0, 255, 0))
            .withFrameRate(10);

        m_intake = new StrobeAnimation(0, Constants.LEDConstants.kRGBCount)
            .withColor(new RGBWColor(255, 255, 0, 0))
            .withFrameRate(10);

        m_error = new StrobeAnimation(0, Constants.LEDConstants.kRGBCount)
            .withColor(new RGBWColor(255, 0, 0, 0))
            .withFrameRate(10);

        m_manipulatorNotReady = new TwinkleOffAnimation(0, Constants.LEDConstants.kRGBCount)
            .withColor(new RGBWColor(0, 255, 0, 0));
            //.withTwinklePercent(0.64);

        m_manipulatorReady = new StrobeAnimation(0, Constants.LEDConstants.kRGBCount)
            .withColor(new RGBWColor(0, 255, 0, 0))
            .withFrameRate(10);

        m_climbLEFT = new LarsonAnimation(Constants.LEDConstants.kRGBSection1.m_start, 
            Constants.LEDConstants.kRGBSection1.m_start + Constants.LEDConstants.kRGBSection1.m_length - 1)
            .withColor(new RGBWColor(255, 0, 255, 0))
            .withFrameRate(10)
            .withSize(4);

        m_climbTOP = new LarsonAnimation(Constants.LEDConstants.kRGBSection2.m_start, 
            Constants.LEDConstants.kRGBSection2.m_start + Constants.LEDConstants.kRGBSection2.m_length - 1)
            .withColor(new RGBWColor(255, 0, 255, 0))
            .withFrameRate(10)
            .withSize(4);

        m_climbRIGHT = new LarsonAnimation(Constants.LEDConstants.kRGBSection3.m_start, 
            Constants.LEDConstants.kRGBSection3.m_start + Constants.LEDConstants.kRGBSection3.m_length - 1)
            .withColor(new RGBWColor(255, 0, 255, 0))
            .withFrameRate(10)
            .withSize(4);

        m_coralTargeting = new TwinkleOffAnimation(0, Constants.LEDConstants.kRGBCount)
            .withColor(new RGBWColor(0, 255, 0, 0))
            .withFrameRate(10);
            //.withTwinklePercent(0.30);

        m_coralCloseToTarget = new TwinkleOffAnimation(0, Constants.LEDConstants.kRGBCount)
            .withColor(new RGBWColor(255, 0, 255, 0))
            .withFrameRate(10);
            //.withTwinklePercent(0.30);

        m_coralOnTarget = new StrobeAnimation(0, Constants.LEDConstants.kRGBCount)
            .withColor(new RGBWColor(255, 0, 255, 0))
            .withFrameRate(10);
    }

    @Override
    public void periodic() {

        if (m_alliance == null) {
            Optional<Alliance> value = DriverStation.getAlliance();
            if (value.isPresent()) {
                m_alliance = value.get();
                colorUpdate = true;
            }
        }

        if ((m_currentState != m_pastState) || colorUpdate) {
            switch (m_currentState) {
                case DISABLED:
                    if (m_alliance == Alliance.Blue) {
                        m_candle.setControl(new EmptyAnimation(2));
                        m_candle.setControl(new EmptyAnimation(1));
                        m_candle.setControl(m_blueDisabled.withSlot(0));
                    } else if (m_alliance == Alliance.Red) {
                        m_candle.setControl(new EmptyAnimation(2));
                        m_candle.setControl(new EmptyAnimation(1));
                        m_candle.setControl(m_redDisabled.withSlot(0));
                    } else {
                        m_candle.setControl(new EmptyAnimation(2));
                        m_candle.setControl(new EmptyAnimation(1));
                        m_candle.setControl(new EmptyAnimation(0));
                        m_candle.setControl(m_noAlliance.withSlot(0));
                    }
                    break;
                case NEUTRAL:
                    if (m_alliance == Alliance.Blue) {
                        m_candle.setControl(new EmptyAnimation(2));
                        m_candle.setControl(new EmptyAnimation(1));
                        m_candle.setControl(new EmptyAnimation(0));
                        m_candle.setControl(new SolidColor(0, Constants.LEDConstants.kRGBCount)
                            .withColor(new RGBWColor(0, 0, 255, 0)));
                    } else if (m_alliance == Alliance.Red) {
                        m_candle.setControl(new EmptyAnimation(2));
                        m_candle.setControl(new EmptyAnimation(1));
                        m_candle.setControl(new EmptyAnimation(0));
                        m_candle.setControl(new SolidColor(0, Constants.LEDConstants.kRGBCount)
                            .withColor(new RGBWColor(255, 0, 0, 0)));
                    } else {
                        m_candle.setControl(new EmptyAnimation(2));
                        m_candle.setControl(new EmptyAnimation(1));
                        m_candle.setControl(m_noAlliance.withSlot(0));
                    }
                    break;
                case INTAKE:
                    m_candle.setControl(new EmptyAnimation(2));
                    m_candle.setControl(new EmptyAnimation(1));
                    m_candle.setControl(m_intake.withSlot(0));
                    break;
                case MANIPULATOR_NOT_READY:
                    m_candle.setControl(new EmptyAnimation(2));
                    m_candle.setControl(new EmptyAnimation(1));
                    m_candle.setControl(m_manipulatorNotReady.withSlot(0));
                    break;
                case MANIPULATOR_READY:
                    m_candle.setControl(new EmptyAnimation(2));
                    m_candle.setControl(new EmptyAnimation(1));
                    m_candle.setControl(m_manipulatorReady.withSlot(0));
                    break;
                case CLIMB_ENABLED:
                    m_candle.setControl(new EmptyAnimation(2));
                    m_candle.setControl(new EmptyAnimation(1));
                    m_candle.setControl(new EmptyAnimation(0));
                    m_candle.setControl(new SolidColor(0, Constants.LEDConstants.kRGBCount)
                        .withColor(new RGBWColor(255, 0, 255, 0)));
                    break;
                case CLIMB_HOOKED:
                    m_candle.setControl(new EmptyAnimation(2));
                    m_candle.setControl(new EmptyAnimation(1));
                    m_candle.setControl(new EmptyAnimation(0));
                    m_candle.setControl(new SolidColor(0, Constants.LEDConstants.kRGBCount)
                        .withColor(new RGBWColor(255, 0, 255, 0)));
                    break;
                case CLIMB_COMPLETE:
                    m_candle.setControl(m_climbRIGHT.withSlot(2));
                    m_candle.setControl(m_climbTOP.withSlot(1));
                    m_candle.setControl(m_climbLEFT.withSlot(0));
                    break;
                case ERROR:
                    m_candle.setControl(new EmptyAnimation(2));
                    m_candle.setControl(new EmptyAnimation(1));
                    m_candle.setControl(m_error.withSlot(0));
                    break;
                case CORAL_ON_TARGET:
                    m_candle.setControl(new EmptyAnimation(2));
                    m_candle.setControl(new EmptyAnimation(1));
                    m_candle.setControl(m_coralOnTarget.withSlot(0));
                    break;
                case CORAL_TARGETING:
                    m_candle.setControl(new EmptyAnimation(2));
                    m_candle.setControl(new EmptyAnimation(1));
                    m_candle.setControl(m_coralTargeting.withSlot(0));
                    break;
                case CORAL_CLOSETOTARGET:
                    m_candle.setControl(new EmptyAnimation(2));
                    m_candle.setControl(new EmptyAnimation(1));
                    m_candle.setControl(m_coralCloseToTarget.withSlot(0));
                    break;
                default:
                    m_candle.setControl(new EmptyAnimation(2));
                    m_candle.setControl(new EmptyAnimation(1));
                    m_candle.setControl(m_error.withSlot(0));
            }
            colorUpdate = false;
        }
        m_pastState = m_currentState;
    }

    public static void setAlliance(Alliance alliance) {
        m_alliance = alliance;
        colorUpdate = true;
    }

    public static void setDisabled() {
        m_currentState = LEDSubsystemState.DISABLED;
    }

    public static void setNeutral() {
        m_currentState = LEDSubsystemState.NEUTRAL;
    }

    public static void setIntake() {
        m_currentState = LEDSubsystemState.INTAKE;
    }

    public static void setClimb_Complete() {
        m_currentState = LEDSubsystemState.CLIMB_COMPLETE;
    }

    public static void setClimb_Enabled() {
        m_currentState = LEDSubsystemState.CLIMB_ENABLED;
    }

    public static void setClimb_Hooked() {
        m_currentState = LEDSubsystemState.CLIMB_HOOKED;
    }

    public static void setManipulatorNotReady() {
        m_currentState = LEDSubsystemState.MANIPULATOR_NOT_READY;
    }

    public static void setManipulatorReady() {
        m_currentState = LEDSubsystemState.MANIPULATOR_READY;
    }

    public static void setError() {
        m_currentState = LEDSubsystemState.ERROR;
    }

    public LEDSubsystemState getState() {
        return m_currentState;
    }

    public static void setCoralCloseToTarget() {
        m_currentState = LEDSubsystemState.CORAL_CLOSETOTARGET;
    }

    public static void setCoralOnTarget() {
        m_currentState = LEDSubsystemState.CORAL_ON_TARGET;
    }

    public static void setCoralTargeting() {
        m_currentState = LEDSubsystemState.CORAL_TARGETING;
    }

}