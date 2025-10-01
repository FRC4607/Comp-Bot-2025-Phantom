// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Constants.ElevatorConstants;



/**
 * Elevator subsystem.
 */
public class ElevatorSubsystem extends SubsystemBase {

    private final TalonFX m_motor1;
    private final TalonFX m_motor2;
    private final TalonFX m_motor3;
    private final TalonFX m_motor4; 
    private final CANdi m_candi;
    private final Servo m_lockServo;
    private TalonFXConfiguration m_talonFxConfig;
    private CANdiConfiguration m_candiConfig;
    private final DynamicMotionMagicTorqueCurrentFOC m_request;

    private boolean m_pastCaNdi = false;

    /**
     * Elevator subsystem constructor.
     */
    public ElevatorSubsystem() {

        /* Create the hardware and configurators */
        m_motor1 = new TalonFX(ElevatorConstants.kmotor1CanId, CANBus.systemCore(0));
        m_motor2 = new TalonFX(ElevatorConstants.kmotor2CanId, CANBus.systemCore(0));
        m_motor3 = new TalonFX(ElevatorConstants.kmotor3CanId, CANBus.systemCore(0));
        m_motor4 = new TalonFX(ElevatorConstants.kmotor4CanId, CANBus.systemCore(0));
        m_candi = new CANdi(ElevatorConstants.kcandiCanId, CANBus.systemCore(0));
        m_lockServo = new Servo(ElevatorConstants.kservoPort);
        m_talonFxConfig = new TalonFXConfiguration();
        m_candiConfig = new CANdiConfiguration();

        m_request = new DynamicMotionMagicTorqueCurrentFOC(
            0, 
            ElevatorCalibrations.kMaxSpeedMotionMagic, 
            ElevatorCalibrations.kMaxAccelerationMotionMagic, 
            0);
      
        /* Configure the motors */
        m_talonFxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_talonFxConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
 
        // Slot gains for going down
        m_talonFxConfig.Slot0.kG = ElevatorCalibrations.kG;
        m_talonFxConfig.Slot0.kS = ElevatorCalibrations.kS;
        m_talonFxConfig.Slot0.kV = ElevatorCalibrations.kV;
        m_talonFxConfig.Slot0.kA = ElevatorCalibrations.kA;
        m_talonFxConfig.Slot0.kP = ElevatorCalibrations.kDownP;
        m_talonFxConfig.Slot0.kD = ElevatorCalibrations.kD;
 
        // Slot gains for climbing
        m_talonFxConfig.Slot1.kG = ElevatorCalibrations.kClimbG;
        m_talonFxConfig.Slot1.kS = ElevatorCalibrations.kS;
        m_talonFxConfig.Slot1.kV = ElevatorCalibrations.kV;
        m_talonFxConfig.Slot1.kA = ElevatorCalibrations.kA;
        m_talonFxConfig.Slot1.kP = ElevatorCalibrations.kClimbP;
        m_talonFxConfig.Slot1.kD = ElevatorCalibrations.kD;

        // Slot gains for going up
        m_talonFxConfig.Slot2.kG = ElevatorCalibrations.kG;
        m_talonFxConfig.Slot2.kS = ElevatorCalibrations.kS;
        m_talonFxConfig.Slot2.kV = ElevatorCalibrations.kV;
        m_talonFxConfig.Slot2.kA = ElevatorCalibrations.kA;
        m_talonFxConfig.Slot2.kP = ElevatorCalibrations.kUpP;
        m_talonFxConfig.Slot2.kD = ElevatorCalibrations.kD;
 
        m_talonFxConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorCalibrations.kMaxSpeedMotionMagic;
        m_talonFxConfig.MotionMagic.MotionMagicAcceleration = ElevatorCalibrations.kMaxAccelerationMotionMagic;

        m_talonFxConfig.TorqueCurrent.PeakForwardTorqueCurrent = ElevatorCalibrations.kMaxCurrentPerMotor;
        m_talonFxConfig.TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorCalibrations.kMaxCurrentPerMotor;
    
        m_talonFxConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
        m_talonFxConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = m_candi.getDeviceID();
        m_talonFxConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
        m_talonFxConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        m_talonFxConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;

        m_talonFxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_talonFxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorCalibrations.kForwardSoftLimitThreshold;

        m_talonFxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Configure the CANdi */
        /* Closed(tripped) and float(open) states. These settings can vary based on the type of sensor */
        m_candiConfig.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
        m_candiConfig.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh;
        m_candiConfig.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;
        m_candiConfig.DigitalInputs.S2FloatState = S2FloatStateValue.PullHigh;

        /* Apply hardware configurations */
        m_motor1.getConfigurator().apply(m_talonFxConfig);  
        m_motor2.getConfigurator().apply(m_talonFxConfig);
        m_motor3.getConfigurator().apply(m_talonFxConfig);
        m_motor4.getConfigurator().apply(m_talonFxConfig);        
        m_candi.getConfigurator().apply(m_candiConfig);

        m_motor2.setControl(new Follower(ElevatorConstants.kmotor1CanId, true));
        m_motor3.setControl(new Follower(ElevatorConstants.kmotor1CanId, false));
        m_motor4.setControl(new Follower(ElevatorConstants.kmotor1CanId, true));

    }

    /**
     * Passes in a value in degrees for the Motion Magic Motion Profiler to use.
     *
     * @param newSetpoint - New setpoint for the elevator in inches.
     */
    public void updateSetpoint(double newSetpoint, boolean isClimbing) {

        if (isClimbing) {
            m_motor1.setControl(m_request.withPosition(newSetpoint * ElevatorConstants.kPulleyGearRatio)
                                         .withVelocity(ElevatorCalibrations.kClimbSpeedMotionMagic)
                                         .withSlot(1));
        } else {
            if (getPosition() >= newSetpoint) {
                m_motor1.setControl(m_request.withPosition(newSetpoint * ElevatorConstants.kPulleyGearRatio)
                                    .withVelocity(ElevatorCalibrations.kMaxSpeedMotionMagic)
                                    .withSlot(0));
            } else {
                m_motor1.setControl(m_request.withPosition(newSetpoint * ElevatorConstants.kPulleyGearRatio)
                                    .withVelocity(ElevatorCalibrations.kMaxSpeedMotionMagic)
                                    .withSlot(2));
            }
        }
    
    }

    /**
     * Return the position of the elevator (in).
     *
     * @return Position of the elevator (in)
     */
    public double getPosition() {
        return m_motor1.getPosition().getValueAsDouble() / ElevatorConstants.kPulleyGearRatio;
    }

    /**
     * Return elevator setpoint in inches.
     *
     * @return setpoint as a number in inches
     */
    public double getSetpoint() {
        return m_request.Position / ElevatorConstants.kPulleyGearRatio;
    }

    public boolean getCANdiState() {
        return m_candi.getS2Closed().getValue();
    }

    public boolean atTarget() {
        return Math.abs(getPosition() - getSetpoint()) < ElevatorCalibrations.kDefaultTolerance;
    }

    public boolean isWithinTolerance(double tolerance) {
        // System.out.println(Math.abs(getPosition() - getSetpoint()));
        return Math.abs(getPosition() - getSetpoint()) < tolerance;
    }

    /**
     * Disable lock servo.
     */
    public void disableServo() {
        m_lockServo.close(); // TODO: unknown replacement method, setDisabled() doesn't exist now
    }
  
    /**
     * Set the lock servo angle.
     *
     * @param angle servo angle in degrees
     */
    public void setServoAngle(double angle) {
        m_lockServo.setAngle(angle);
    }
  
    /**
     * Return the lock servo postion.
     *
     * @return servo position in degrees
     */
    public double getServoPos() {
        return m_lockServo.getAngle();
    }

    @Override
    public void periodic() {

        if ((m_candi.getS2Closed().getValue().booleanValue() != m_pastCaNdi)
            && (m_pastCaNdi == false) && (getPosition() < ElevatorCalibrations.kResetPositionTolerance)) {
            m_motor1.setPosition(0);
        }
        m_pastCaNdi = m_candi.getS2Closed().getValue().booleanValue();
          
        /* Debug values */
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putBoolean("CANdi State", getCANdiState());
    }

    public void setElevatorZeroDutyCycle() {
        m_motor1.set(0);
    }

    public void setElevatorOpenLoopDutyCycle(double dutyCycle) {
        m_motor1.set(dutyCycle);
    }

    public void setZeroPosition() {
        m_motor1.setPosition(0);
    }
}