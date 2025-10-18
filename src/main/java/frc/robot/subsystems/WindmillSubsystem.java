
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.Constants.WindmillConstants;

/**
 * Windmill subsystem.
 */
public class WindmillSubsystem extends SubsystemBase {

    private final TalonFX m_motor;
    private final CANcoder m_encoder;
    private TalonFXConfiguration m_talonFxConfig;
    private CANcoderConfiguration m_canCoderConfig;
    private final DynamicMotionMagicTorqueCurrentFOC m_request;

    /**
     * Windmill subsystem constructor.
     */
    public WindmillSubsystem() {

        /* Create the hardware and configurators */
        m_motor = new TalonFX(WindmillConstants.kmotorCanId, new CANBus("kachow"));
        m_encoder = new CANcoder(WindmillConstants.kcanCoderCanId, new CANBus("kachow"));
        m_talonFxConfig = new TalonFXConfiguration();
        m_canCoderConfig = new CANcoderConfiguration();

        m_request = new DynamicMotionMagicTorqueCurrentFOC(
            0, 
            WindmillCalibrations.kMaxSpeedMotionMagic, 
            WindmillCalibrations.kMaxAccelerationMotionMagic, 
            WindmillCalibrations.kMaxJerkMotionMagic);

        /* Configure the motor */
        /* TOOD: Are we sure we want to allow the 360 on windmill?? */
        m_talonFxConfig.ClosedLoopGeneral.ContinuousWrap = true;

        m_talonFxConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        m_talonFxConfig.Feedback.FeedbackRemoteSensorID = WindmillConstants.kcanCoderCanId;
        m_talonFxConfig.Feedback.SensorToMechanismRatio = WindmillConstants.ksensorToMechanismRatio;
        m_talonFxConfig.Feedback.RotorToSensorRatio = WindmillConstants.krotorToSensorRatio;

        m_talonFxConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        m_talonFxConfig.Slot0.kG = WindmillCalibrations.kG;
        m_talonFxConfig.Slot0.kS = WindmillCalibrations.kS;
        m_talonFxConfig.Slot0.kV = WindmillCalibrations.kV;
        m_talonFxConfig.Slot0.kA = WindmillCalibrations.kA;
        m_talonFxConfig.Slot0.kP = WindmillCalibrations.kP;
        m_talonFxConfig.Slot0.kD = WindmillCalibrations.kD;

        m_talonFxConfig.Slot1.kG = WindmillCalibrations.kG;
        m_talonFxConfig.Slot1.kS = WindmillCalibrations.kS;
        m_talonFxConfig.Slot1.kV = WindmillCalibrations.kV;
        m_talonFxConfig.Slot1.kA = WindmillCalibrations.kA;
        m_talonFxConfig.Slot1.kP = WindmillCalibrations.kClimbP;
        m_talonFxConfig.Slot1.kD = WindmillCalibrations.kD;

        m_talonFxConfig.MotionMagic.MotionMagicCruiseVelocity = WindmillCalibrations.kMaxSpeedMotionMagic;
        m_talonFxConfig.MotionMagic.MotionMagicAcceleration = WindmillCalibrations.kMaxAccelerationMotionMagic;
        m_talonFxConfig.MotionMagic.MotionMagicJerk = WindmillCalibrations.kMaxJerkMotionMagic;

        m_talonFxConfig.CurrentLimits.StatorCurrentLimit = WindmillCalibrations.kMaxWindmillStatorCurrentPerMotor;

        m_talonFxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_talonFxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Configure the CANCoder */
        m_canCoderConfig.MagnetSensor.MagnetOffset = WindmillCalibrations.kCanCoderOffset;
        m_canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0;
        m_canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        /* Apply hardware configurations */
        m_motor.getConfigurator().apply(m_talonFxConfig);
        m_encoder.getConfigurator().apply(m_canCoderConfig);
    
    }

    /**
     * Takes in a value in degrees and sets it as the new setpoint.
     *
     * @param newSetpoint The new setpoint in degrees.
     */
    public void updateSetpoint(double newSetpoint, boolean isClimbing) {
        if (isClimbing) {
            m_motor.setControl(m_request.withPosition(newSetpoint / 360)
                                        .withVelocity(WindmillCalibrations.kClimbMaxSpeedMotionMagic)
                                        .withSlot(1));
        } else {
            m_motor.setControl(m_request.withPosition(newSetpoint / 360)
                                        .withVelocity(WindmillCalibrations.kMaxSpeedMotionMagic)
                                        .withSlot(0));
        }
    }

    /**
     * Return postion between [0,360).
     */
    public double getPosition() {
        return (((m_motor.getPosition().getValueAsDouble() * 360) % 360) + 360) % 360;
    }

    /**
     * Return windmill setpoint.
     *
     * @return setpoint as a number between [0,360)
     */
    public double getSetpoint() {
        return (((m_request.Position * 360) % 360) + 360) % 360;
    }

    /**
     * Return if the windmill has reached target position within a default tolerance.
     *
     * @return true is at target within tolerance
     */
    public boolean atTarget() {
        return Math.abs(getPosition() - getSetpoint()) < WindmillCalibrations.kDefaultTolerance;
    }

    public boolean isWithinTolerance(double tolerance) {
        // System.out.println(Math.abs(getPosition() - getSetpoint()));
        return Math.abs(getPosition() - getSetpoint()) < tolerance;
    }

    @Override
    public void periodic() {

        /* Debug vaues */
        SmartDashboard.putNumber("Windmill Position", getPosition());
    }

}
