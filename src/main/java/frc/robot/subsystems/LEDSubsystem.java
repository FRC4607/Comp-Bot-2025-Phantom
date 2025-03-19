// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
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

    private static enum LEDSubsystemState {
        DISABLED,
        NEUTRAL,
        INTAKE,
        MANIPULATOR_NOT_READY,
        MANIPULATOR_READY,
        CLIMB,
        CLIMB_HOOKED,
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
    //private final LarsonAnimation m_intake;
    private final StrobeAnimation m_intake;
    private final StrobeAnimation m_error;
    private final TwinkleOffAnimation m_manipulatorNotReady;
    private final StrobeAnimation m_manipulatorReady;
    private final LarsonAnimation m_climbLEFT;
    private final LarsonAnimation m_climbTOP;
    private final LarsonAnimation m_climbRIGHT;
    private static boolean colorUpdate = false;
        
       
        /**
         * LED subsystem constuctor.
         */
        public LEDSubsystem() {
    
            /* Create the hardware and configurator */
            m_candle = new CANdle(Constants.LEDConstants.kCANdleID, "kachow");
            m_config = new CANdleConfiguration();
    
            /* Configure hardware */
            m_config.stripType = LEDStripType.RGB; // The chips we use seem to be RGB
            m_config.brightnessScalar = 1; // Avoid drawing too much current
            m_candle.configAllSettings(m_config);
            m_candle.configLEDType(LEDStripType.RGB);
            m_candle.clearAnimation(0);
            m_candle.clearAnimation(1);
            m_candle.clearAnimation(2);
            m_candle.clearAnimation(3);
            SmartDashboard.putBoolean("Animation", false);
    
            /* Configure state */
            m_alliance = null;
            m_currentState = LEDSubsystemState.DISABLED;
            m_pastState = null;
     
            /* Configure animatinons */
            m_noAlliance = new StrobeAnimation(255, 0, 255, 0, 0.5, Constants.LEDConstants.kRGBCount);
            m_redDisabled = new SingleFadeAnimation(255, 0, 0, 0, 0.3, Constants.LEDConstants.kRGBCount);
            m_blueDisabled = new SingleFadeAnimation(0, 0, 255, 0, 0.3, Constants.LEDConstants.kRGBCount);
            m_intake = new StrobeAnimation(255, 255, 0, 0, 0.5, Constants.LEDConstants.kRGBCount);
            //m_intake = new LarsonAnimation(165, 255, 0, 0, 0.25, Constants.LEDConstants.kRGBCount, LarsonAnimation.BounceMode.Back, 3);
            m_error = new StrobeAnimation(255, 0, 0, 0, 0.5, Constants.LEDConstants.kRGBCount);
            m_manipulatorNotReady = new TwinkleOffAnimation(0, 255, 0, 0, 1, Constants.LEDConstants.kRGBCount,
                                                            TwinkleOffAnimation.TwinkleOffPercent.Percent64);
            m_manipulatorReady = new StrobeAnimation(0, 255, 0, 0,  1, Constants.LEDConstants.kRGBCount);
            m_climbLEFT = new LarsonAnimation(255, 0, 255, 0, .1, Constants.LEDConstants.kRGBSection1.m_length,
                                              LarsonAnimation.BounceMode.Back, 4, Constants.LEDConstants.kRGBSection1.m_start);
            m_climbTOP = new LarsonAnimation(255, 0, 255, 0, .5, Constants.LEDConstants.kRGBSection2.m_length,
                                             LarsonAnimation.BounceMode.Center, 4, Constants.LEDConstants.kRGBSection2.m_start);
            m_climbRIGHT = new LarsonAnimation(255, 0, 255, 0, .1, Constants.LEDConstants.kRGBSection3.m_length,
                                               LarsonAnimation.BounceMode.Back, 4, Constants.LEDConstants.kRGBSection3.m_start);
        
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
                            m_candle.clearAnimation(2);
                            m_candle.clearAnimation(1);
                            m_candle.animate(m_blueDisabled, 0);
                        } else if (m_alliance == Alliance.Red) {
                            m_candle.clearAnimation(2);
                            m_candle.clearAnimation(1);
                            m_candle.animate(m_redDisabled, 0);
                        } else {
                            m_candle.clearAnimation(2);
                            m_candle.clearAnimation(1);
                            m_candle.clearAnimation(0);
                            m_candle.animate(m_noAlliance, 0);
                        }
                        break;
                    case NEUTRAL:
                        if (m_alliance == Alliance.Blue) {
                            m_candle.clearAnimation(2);
                            m_candle.clearAnimation(1);
                            m_candle.clearAnimation(0);
                            m_candle.setLEDs(0, 0, 255, 0, 0, Constants.LEDConstants.kRGBCount);
                        } else if (m_alliance == Alliance.Red) {
                            m_candle.clearAnimation(2);
                            m_candle.clearAnimation(1);
                            m_candle.clearAnimation(0);
                            m_candle.setLEDs(255, 0, 0, 0, 0, Constants.LEDConstants.kRGBCount);
                        } else {
                            m_candle.clearAnimation(2);
                            m_candle.clearAnimation(1);
                            m_candle.animate(m_noAlliance, 0);
                        }
                        break;
                    case INTAKE:
                        m_candle.clearAnimation(2);
                        m_candle.clearAnimation(1);
                        m_candle.animate(m_intake, 0);
                        break;
                    case MANIPULATOR_NOT_READY:
                        m_candle.clearAnimation(2);
                        m_candle.clearAnimation(1);
                        m_candle.animate(m_manipulatorNotReady, 0);
                        break;
                    case MANIPULATOR_READY:
                        m_candle.clearAnimation(2);
                        m_candle.clearAnimation(1);
                        m_candle.animate(m_manipulatorReady, 0);
                        break;
                    case CLIMB:
                        m_candle.animate(m_climbRIGHT, 2);
                        m_candle.animate(m_climbTOP, 1);
                        m_candle.animate(m_climbLEFT, 0);
                        break;
                    case ERROR:
                        m_candle.clearAnimation(2);
                        m_candle.clearAnimation(1);
                        m_candle.animate(m_error, 0);
                        break;
                    default:
                        m_candle.clearAnimation(2);
                        m_candle.clearAnimation(1);
                        m_candle.animate(m_error, 0);
                }
                // System.out.println(m_currentState);
            }
            m_pastState = m_currentState;
        }
    
        public static void setAlliance(Alliance alliance){
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

    public static void setClimb() {
        m_currentState = LEDSubsystemState.CLIMB;
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

}
