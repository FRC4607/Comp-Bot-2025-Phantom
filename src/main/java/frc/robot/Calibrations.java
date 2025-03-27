package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.generated.TunerConstants;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Robot calibrations.
 */
public class Calibrations {

    /**
     * Field calibrations.
     */
    public static class FieldCalibrations {

        // The LL AprilTag ID is an integer
        // Encode the left branch as a 0 and the right branch as a 1 to match LL
        // Red alliance reef tags: 6,7,8,9,10,11
        // Blue alliance reef tags: 17,18,19,20,21,22
        
        public static final Map<Integer, Map<Integer, Double>> m_coralReefTargets;

        static {
            Map<Integer, Map<Integer, Double>> tempMap = new HashMap<>();
            tempMap.put(6, createImmutableMap(0, -29.48, 1, 2.20));
            tempMap.put(7, createImmutableMap(0, -28.18, 1, 0.51));
            tempMap.put(8, createImmutableMap(0, -28.72, 1, -0.8));
            tempMap.put(9, createImmutableMap(0, -29.32, 1, -0.09));
            tempMap.put(10, createImmutableMap(0, -27.54, 1, 1.97));
            tempMap.put(11, createImmutableMap(0, -29.03, 1, 2.13));
            tempMap.put(17, createImmutableMap(0, -28.17, 1, 2.07));
            tempMap.put(18, createImmutableMap(0, -29.15, 1, 1.66));
            tempMap.put(19, createImmutableMap(0, -29.3, 1, 3.8));
            tempMap.put(20, createImmutableMap(0, -29.33, 1, 3.95));
            tempMap.put(21, createImmutableMap(0, -29.27, 1, 1.99));
            tempMap.put(22, createImmutableMap(0, -28.80, 1, 1.71));

            m_coralReefTargets = Collections.unmodifiableMap(tempMap);
        }
        
        private static <K, V> Map<K, V> createImmutableMap(K k1, V v1, K k2, V v2) {
            Map<K, V> map = new HashMap<>();
            map.put(k1, v1);
            map.put(k2, v2);
            return Collections.unmodifiableMap(map);
        }

        public static final List<Integer> m_validTagIds = List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    }

    /**
     * Driver calibrations.
     */
    public static class DriverCalibrations {

        /* Max speed, in meters per seocond, of the robot */
        public static final double kmaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        
        /* 3/4 max angular velocity, in rotations per second, of the robot */
        public static final double kmaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        /* P-gain for rotational controller using LL tx as feedback */
        public static final double kAprilTagRotationAlignmentKP = 0.05;
        
        /* When the LL doesn't see a tag, use this value...which essentially sets the error to 0 */
        public static final double kLimelightDefaultKTx = 0;

        /* P-gain for robot-centric X-translational profiled PID controller */
        public static final double kAprilTagTranslationXAlignmentKP = 0.1;

        /* D-gain for robot-centric X-translational profiled PID controller */
        public static final double kAprilTagTranslationXAlignmentKD = 0.005;

        /* Profiled PID controller on-target threshold in degrees*/
        public static final double kAprilTagTranslationXClose = 2.0;        
        public static final double kAprilTagTranslationXOnTarget = 1.75;        
        
        /* Robot-centric X-translational controller - add a little Y-translation to stay flush to the coral reef */
        public static final double kAprilTagTranslationYRate = -0.1;

        /* Amount of rumble (0-1) to apply to the driver controller */
        public static final double kControllerRumbleValue = 1;

        /* Time to apply the controller rumble for before it turns off */
        public static final double kControllerRumblePulseTime = 0.1;
        
    }

    /**
     * Elevator calibrations.
     */
    public static class ElevatorCalibrations {

        /* Gains for the MotionMagic profiler for teleoperated elevator */
        public static final double kG = 7;
        public static final double kS = 4.2;
        public static final double kA = 0;
        public static final double kV = 0;
        public static final double kP = 10;
        public static final double kD = 1;

        /* Gains for the MotionMagic profiler for endgame elevator */
        public static final double kClimbG = -25;
        public static final double kClimbP = 15;

        /* MotionMagic constraints for teleoperated elevator */
        public static final double kMaxSpeedMotionMagic = 100;
        public static final double kMaxAccelerationMotionMagic = 550; //1100;
        public static final double kMaxCurrentPerMotor = 80;

        /* MotionMagic constraints for engame elevator */
        public static final double kClimbSpeedMotionMagic = 18;

        /* Soft limit in rotor rotations */
        public static final double kForwardSoftLimitThreshold = 61.2;

        /* atTarget() returns true if within this tolerance, in inches */
        public static final double kDefaultTolerance = 5;

        /* Limit to apply to the CANdi limit switch resetting the elevator 0 point */
        public static final double kResetPositionTolerance = 1.0;

        public static final double kResetPositionVelocity = -0.15;

        /* Bottom position in inches */
        public static final double kBottomPosition = 2.0; //-0.2;

        /* Bottom position in inches for auto*/
        public static final double kBottomPositionForAuto = 25.0;

        /* Pendulum position in inches */
        public static final double kPendulumPosition = 24;

        /* Pendulum tolerance in inches before any windmill movement */
        public static final double kPendulumTolerance = 2;

        /* Position to go to for a raised lollipop stow. */
        public static final double kRaisedStowPosition = 15;

        /* Coral station position in inches */
        public static final double kCoralStationPosition = 34;

        /* Coral station tolerance in inches before any windmill movement */
        public static final double kCoralStationTolerance = 2;

        /* Coral reef L4 position in inches */
        public static final double kL4Position = 51.5;

        /* Coral reef L4 tolerance in inches before any windmill movement */
        public static final double kL4Tolerance = 2;

        /* Coral reef L3 position in inches */
        public static final double kL3Position = 25;

        /* Coral reef L3 tolerance in inches before any windmill movement */
        public static final double kL3Tolerance = 2;

        /* Coral reef L2 position in inches */
        public static final double kL2Position = 9.5;

        /* Coral reef L2 tolerance in inches before any windmill movement */
        public static final double kL2Tolerance = 2;

        /* Coral station stow position in inches, windmilll moves after elevator */
        public static final double kCoralStationStowPosition = 30;

        /* Climb position in inches */
        public static final double kPrepClimbPosition = 4.5;

        /* Barge position in inches */
        public static final double kBargePosition = 53.5;

        /* Servo lock position in degrees */
        public static final int kservoLockAngle = 100;

        /* Servo unlock position in degrees */
        public static final int kservoUnlockAngle = 28;

        /* Floor pickup position for Algae */
        public static final double kAlgaePickupPosition = 3;

        /* L2 OVER pickup position for Algae */
        public static final double kAlgaeOverL2Position = 13;
        
        /* L2 UNDER pickup position for Algae */
        public static final double kAlgaeUnderL2Position = 22.0;

        /* L3 OVER pickup position for Algae */
        public static final double kAlgaeOverL3Position = 28.5;
        
        /* L3 UNDER pickup position for Algae */
        public static final double kAlgaeUnderL3Position = 30;

        /* Position for algae pickup when the algae is on top of a coral */
        public static final double kAlgaeStandingPosition = 10;

        /* Position to move to when processing algae */
        public static final double kProcessorPosition = -0.2;

        /* Tolerance for the processor position */
        public static final double kProcessorTolerance = 1;

        /* Setpoint to go to when on the upstroke of the climb */
        public static final double kClimbUpSetpoint = 18;

        /* Tolerance for the up stroke of the climb */
        public static final double kClimbUpTolerance = 2;

        /* Position to go to on the down stroke of the climb */
        public static final double kClimbDownSetpoint = -5;

        /* Tolerance for the down stroke of the climb */
        public static final double kClimbDownTolerance = 2;

    }

    /**
     * Windmill calibrations.
     */
    public static class WindmillCalibrations {
        
        /* The encoder offset when the windmill is in lollipop position */
        public static final double kCanCoderOffset = 0.76708984375;

        /* Gains for the MotionMagic profiler for teleoperated */
        public static final double kG = 10.6;
        public static final double kS = 3.0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kP = 800;
        public static final double kD = 200;

        /* Gains for the MotionMagic profiler for endgame */
        public static final double kClimbP = 1200;

        /* MotionMagic constraints for teleoperated */
        public static final double kMaxSpeedMotionMagic = 1;
        public static final double kMaxAccelerationMotionMagic = 4;
        public static final double kMaxJerkMotionMagic = 100;
        public static final double kMaxWindmillStatorCurrentPerMotor = 80;

        /* MotionMagic constraints for endgame */
        public static final double kClimbMaxSpeedMotionMagic = 0.25;

        /* atTarget() returns true if within this tolerance, in degrees */
        public static final double kDefaultTolerance = 10;

        /* Lollipop position in degrees */
        public static final double kLollipopPosition = 90;

        /* Lollipop tolerance in degrees before any elevator movement */
        public static final double kLollipopTolerance = 30;

        /* Position to go to before going to the pendulum position */
        public static final double kPendulumPrepPosition = 0;

        /* Tolerance for the prep position in degrees */
        public static final double kPendulumPrepTolerance = 25;

        /* Pendulum position in degrees */
        public static final double kPendulumPosition = 270;

        /* Tolerance for the pendulum position in degrees */
        public static final double kPendulumTolerance = 2;

        /* Horizontal position for before the windmill goes to the Coral Station */
        public static final double kCoralStationPrepPosition = 180;

        /* Horizontal position tolerance for before the windmill goes to the Coral Station */
        public static final double kCoralStationPrepTolerance = 10;

        /* Tolerance to bypass the prep position of the coral station command group */
        public static final double kBypassCoralPrepTolerance = 20;
        
        /* Coral station position in degrees */
        public static final double kCoralStationPosition = 287;
        
        /* Coral reef L4 position in degrees */
        public static final double kL4Position = 120;

        /* Coral reef L3 position in degrees */
        public static final double kL3Position = 110;

        /* Coral reef L2 position in degrees */
        public static final double kL2Position = 110;

        /* Coral station position in degrees */
        public static final double kCoralStationStowPosition = 287;
        
        /* Coral station tolerance in degrees before any elevator movement */
        public static final double kCoralStationStowTolerance = 5;

        /* Climb position in degrees */
        public static final double kPrepClimbPosition = 113;

        /* Climb tolerance in degrees before any elevator movement */
        public static final double kPrepClimbTolerance = 5;

        /* Barge position in degrees */
        public static final double kBargePosition = 90;

        /* Barge tolerance in degrees before any elevator movement */
        public static final double kBargeTolerance = 10;

        /* Floor Pickup Position for Algae */
        public static final double kAlgaePickupPosition = 345;
        
        /* L2 OVER pickup position for Algae */
        public static final double kAlgaeOverL2Position = 26;
        
        /* L2 UNDERpickup position for Algae */
        public static final double kAlgaeUnderL2Position = 195;

        /* L3 OVER pickup position for Algae */
        public static final double kAlgaeOverL3Position = 26;
        
        /* L3 UNDER pickup position for Algae */
        public static final double kAlgaeUnderL3Position = 175;

        /* Position for algae pickup when the algae is on top of a coral */
        public static final double kAlgaeStandingPosition = 0;

        /* Position to move to when processing algae */
        public static final double kProcessorPosition = 26;

        /* Tolerance for the processor position */
        public static final double kProcessorTolerance = 10;
        
        /* Position to go to after the up stroke of the climb */
        public static final double kClimbPosition = 180;

        /* Tolerance for after the up stroke of the climb */
        public static final double kClimbTolerance = 5;
    }

    /**
     * Manipulator Calibrations.
     */
    public static class ManipulatorCalibrations {
        
        /* Gains for the MotionMagic velocity controller */
        public static final double kS = 6;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kP = 5;
        public static final double kD = 0;

        /* MotionMagic constraints for velocity controller */
        public static final double kMaxAcceleration = 1000;
        public static final double kCoralAcceleration = 1000;
        public static final double kMaxSpeed = 50;
        //TODO: Add Calibration value for max coral speed
        public static final double kMaxStatorCurrent = 40;
 
        /* Stator current delta threshold to stop motors */
        public static final double kCurrentThreshold = 15;

        /* Used to track state of when the intake is running full tilt */
        public static final double kIntakeVelocityTolerance = 10;

        /* If the velocity goes from full tilt (50rps) to 25 assume coral has been fetched */
        public static final double kIntakeZeroTolerance = 25;

        /* Coral reef L4 outtake speed, in rotations per second */
        public static final double kL4OuttakeSpeed = -30;

        /* Coral reef L4 outtake time, in seconds */
        public static final double kL4OuttakeTime = 0.25;

        /* Coral reef L3 outtake speed, in rotations per second */
        public static final double kL3OuttakeSpeed = -30;

        /* Coral reef L3 outtake time, in seconds */
        public static final double kL3OuttakeTime = 0.25;

        /* Coral reef L2 outtake speed, in rotations per second */
        public static final double kL2OuttakeSpeed = -30;

        /* Coral reef L2 outtake time, in seconds */
        public static final double kL2OuttakeTime = 0.25;

        /* Algae intake and holding speed, in rotations per second */
        public static final double kAlgaeHoldingVelocity = -15;

        /* Algae barge outtake speed, in rotations per second */
        public static final double kAlgaeBargingVelocity = 300;
        
        /* Velocity to intake Algae in the floor position at */
        public static final double kAlgaeIntakeVelocity = -75;
        
        /* Current threshold for deciding when we have an algae */
        public static final double kAlgaeIntakeThreshold = 25;

        /* Algae barge outtake speed, in rotations per second */
        public static final double kL1Velocity = -35;

    }
}