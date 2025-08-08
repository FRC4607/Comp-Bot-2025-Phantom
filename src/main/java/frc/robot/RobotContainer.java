// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Calibrations.DriverCalibrations;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.commands.AlgaeFloorPickup;
import frc.robot.commands.AlgaeL2Pickup;
import frc.robot.commands.AlgaeL2PickupPrep;
import frc.robot.commands.AlgaeL3Pickup;
import frc.robot.commands.AlgaeL3PickupPrep;
import frc.robot.commands.AlgaeStandingPickup;
import frc.robot.commands.BargeAlgae;
import frc.robot.commands.CGClimb;
import frc.robot.commands.CGOuttakeThenStow;
import frc.robot.commands.CGZeroElevator;
import frc.robot.commands.CoralStation;
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.commands.L3Stow;
import frc.robot.commands.L4;
import frc.robot.commands.LollipopStow;
import frc.robot.commands.LollipopStowForAuto;
import frc.robot.commands.MoveWindmillToPosition;
import frc.robot.commands.PendulumStow;
import frc.robot.commands.PrepClimb;
import frc.robot.commands.ProcessAlgae;
import frc.robot.commands.RunAlgaeIntake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunManipulator;
import frc.robot.commands.TranslatationYRobotCentric;
import frc.robot.commands.TranslationAlignToTag;
import frc.robot.commands.ZeroElevator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * The robot container.
 */
public class RobotContainer {

    /* Create the robot subsystems */
    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    public final WindmillSubsystem m_windmill = new WindmillSubsystem();
    public final ManipulatorSubsystem m_manipulator = new ManipulatorSubsystem();
    public final LEDSubsystem m_leds = new LEDSubsystem();
    public final CommandXboxController m_joystick = new CommandXboxController(0);
    public final Joystick m_coPilot = new Joystick(1);

    /* Drive request for default drivetrain command */
    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
            .withDeadband(Calibrations.DriverCalibrations.kmaxSpeed * 0.1)
            .withRotationalDeadband(Calibrations.DriverCalibrations.kmaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /* Path follower */
    private final SendableChooser<Command> m_autoChooser;

    /**
     * The robot container constructor.
     */
    public RobotContainer() {

        configureBindings();
        
 
        //new EventTrigger("GoTo L1").onTrue(new L1(m_elevator, m_windmill));
        //new EventTrigger("ET GoTo L2").onTrue(new L2(m_elevator, m_windmill));
        //new EventTrigger("ET GoTo L3").onTrue(new L3(m_elevator, m_windmill));
        //new EventTrigger("ET GoTo L4").onTrue(new L4(m_elevator, m_windmill));
        //new EventTrigger("ET Lollipop Stow").onTrue(new LollipopStow(m_elevator, m_windmill));
        //new EventTrigger("ET Pendulum Stow").onTrue(new PendulumStow(m_elevator, m_windmill));
        //new EventTrigger("ET Intake Coral").onTrue(new CoralStation(m_elevator, m_windmill));
        NamedCommands.registerCommand("CMD Unlock Elevator", new InstantCommand(() -> m_elevator.setServoAngle(ElevatorCalibrations.kservoUnlockAngle)));
        
        NamedCommands.registerCommand("CMD GoTo L2", new L2(m_elevator, m_windmill));
        NamedCommands.registerCommand("CMD GoTo L3", new L3(m_elevator, m_windmill));
        NamedCommands.registerCommand("CMD GoTo L4", new L4(m_elevator, m_windmill));

        NamedCommands.registerCommand("CMD Lollipop Stow", new LollipopStowForAuto(m_elevator, m_windmill));
        NamedCommands.registerCommand("CMD Pendulum Stow", new PendulumStow(m_elevator, m_windmill).withTimeout(0.5));
        
        NamedCommands.registerCommand("CMD Run Intake", new RunIntake(m_manipulator));
        NamedCommands.registerCommand("CMD Intake Coral", new CoralStation(m_elevator, m_windmill)
                                                            .alongWith(new RunIntake(m_manipulator).withTimeout(3)));
        NamedCommands.registerCommand("CMD Score Coral", 
                                        new RunManipulator(ManipulatorCalibrations.kL4OuttakeSpeed,
                                        ManipulatorCalibrations.kCoralAcceleration, m_manipulator)
                                        .withTimeout(ManipulatorCalibrations.kL4OuttakeTime));

        NamedCommands.registerCommand("CMD Align Left", new TranslationAlignToTag(0, m_drivetrain));
        NamedCommands.registerCommand("CMD Align Right", new TranslationAlignToTag(1, m_drivetrain));
        NamedCommands.registerCommand("CMD Align Center", new TranslationAlignToTag(2, m_drivetrain));
        NamedCommands.registerCommand("CMD Translate Y", new TranslatationYRobotCentric(m_drivetrain));

        // Go to position and run the Intake
        NamedCommands.registerCommand("CMD Algae L2 Intake", new AlgaeL2Pickup(m_elevator, m_windmill, m_manipulator));
        // Just go to position
        NamedCommands.registerCommand("CMD Algae L2 Prep", new AlgaeL2PickupPrep(m_elevator, m_windmill));
        // Just run the intake
        NamedCommands.registerCommand("CMD Algae Intake", new RunAlgaeIntake(m_manipulator));
        NamedCommands.registerCommand("CMD Algae L3 Intake", 
            new AlgaeL3Pickup(m_elevator, m_windmill, m_manipulator).withTimeout(2.5));
        // timed sequents for algae L3 pickup from barge 1
        NamedCommands.registerCommand("CMD Algae Stow and L3 Prep", new SequentialCommandGroup(
            new PendulumStow(m_elevator, m_windmill),
            new WaitCommand(1),
            new AlgaeL3PickupPrep(m_elevator, m_windmill)
        ));
        NamedCommands.registerCommand("CMD Algae L3 Under", new AlgaeL3PickupPrep(m_elevator, m_windmill));
        NamedCommands.registerCommand("CMD Barge Algae", new BargeAlgae(m_elevator, m_windmill));
        NamedCommands.registerCommand("CMD Score Algae", new RunManipulator(ManipulatorCalibrations.kAlgaeBargingVelocity,
                                                                        ManipulatorCalibrations.kBargeAlgaeAcceleration, 
                                                                        m_manipulator).withTimeout(.3).deadlineFor(
                                                                            new MoveWindmillToPosition(
                                                                                WindmillCalibrations.kBargeFlickPosition, 
                                                                                WindmillCalibrations.kBargeTolerance,
                                                                                false, m_windmill)));
        
        m_autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");
        
        SmartDashboard.putData("Auto Mode", m_autoChooser);
        SmartDashboard.putData("Unlock", new InstantCommand(
            () -> m_elevator.setServoAngle(ElevatorCalibrations.kservoUnlockAngle))
            .alongWith(new InstantCommand(LEDSubsystem::setNeutral)));
    
        SmartDashboard.putData("Lock", new InstantCommand(
            () -> m_elevator.setServoAngle(ElevatorCalibrations.kservoLockAngle))
            .alongWith(new InstantCommand(LEDSubsystem::setClimb_Complete))); 

        SmartDashboard.putData("Zero Elevator", new ZeroElevator(m_elevator));

        SmartDashboard.putData(m_elevator);
        SmartDashboard.putData(m_windmill);
    }
    
    /**
     * Configure bindings defines how the robot is controlled via user input.
     */
    private void configureBindings() {

        m_drivetrain.setDefaultCommand(
            m_drivetrain.applyRequest(() -> m_drive
                .withVelocityX(-m_joystick.getLeftY() * Calibrations.DriverCalibrations.kmaxSpeed)
                .withVelocityY(-m_joystick.getLeftX() * Calibrations.DriverCalibrations.kmaxSpeed)
                .withRotationalRate(-m_joystick.getRightX() * Calibrations.DriverCalibrations.kmaxAngularRate)
            )
        );
    
        /* This allows the driver to reset the rotation */
        m_joystick.start().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

        m_joystick.b().onTrue(new LollipopStow(m_elevator, m_windmill)
                      .alongWith(new InstantCommand(() -> m_elevator.setServoAngle(
                        ElevatorCalibrations.kservoUnlockAngle))));

        m_joystick.x().onTrue(new PendulumStow(m_elevator, m_windmill)
                      .alongWith(new InstantCommand(() -> m_elevator.setServoAngle(
                        ElevatorCalibrations.kservoUnlockAngle))));
    
        /* Coral station pickup sequence */
        m_joystick.rightBumper().onTrue(new CoralStation(m_elevator, m_windmill)
                                .andThen(new InstantCommand(LEDSubsystem::setIntake)));
        m_joystick.rightBumper().onTrue(new RunIntake(m_manipulator)
            .andThen(new InstantCommand(
                () -> m_joystick.setRumble(RumbleType.kBothRumble, DriverCalibrations.kControllerRumbleValue))
                .alongWith(new InstantCommand(LEDSubsystem::setManipulatorReady)))
            .andThen(new WaitCommand(DriverCalibrations.kControllerRumblePulseTime))
            .andThen(new InstantCommand(
                () -> m_joystick.setRumble(RumbleType.kBothRumble, 0))));
        m_joystick.rightBumper().onFalse(new L3Stow(m_elevator, m_windmill)
                                .alongWith(new InstantCommand(LEDSubsystem::setNeutral)));

        /* Coral reef L4 dropoff sequence */
        /* m_joystick.povUp().and(m_joystick.leftBumper().negate()).onTrue(new L4(m_elevator, m_windmill));
        m_joystick.povUp().and(m_joystick.leftBumper().negate())
            .onFalse(new CGOuttakeThenStow(ManipulatorCalibrations.kL4OuttakeSpeed, 
                                                         ManipulatorCalibrations.kL4OuttakeTime, 
                                                         m_elevator, m_windmill, m_manipulator)); */

        /* Coral reef L3 dropoff sequence */
       /*  m_joystick.povLeft().and(m_joystick.leftBumper().negate()).onTrue(new L3(m_elevator, m_windmill));
        m_joystick.povLeft().and(m_joystick.leftBumper().negate())
            .onFalse(new CGOuttakeThenStow(
                ManipulatorCalibrations.kL3OuttakeSpeed,
                ManipulatorCalibrations.kL3OuttakeTime,
                m_elevator, m_windmill, m_manipulator)); */

        /* Coral reef L2 dropoff sequence */
        /* m_joystick.povRight().and(m_joystick.leftBumper().negate()).onTrue(new L2(m_elevator, m_windmill));
        m_joystick.povRight().and(m_joystick.leftBumper().negate())
            .onFalse(new CGOuttakeThenStow(
                ManipulatorCalibrations.kL2OuttakeSpeed, ManipulatorCalibrations.kL2OuttakeTime, 
                    m_elevator, m_windmill, m_manipulator)); */

        /* Target the left coral reef stick */
        m_joystick.axisGreaterThan(2, 0.1).whileTrue(new TranslationAlignToTag(0, m_drivetrain));
        /* Target the right coral reef stick */
        m_joystick.axisGreaterThan(3, 0.1).whileTrue(new TranslationAlignToTag(1, m_drivetrain));
        
        /* Prep Climb and finish Climb when released */
        m_joystick.y().onTrue(new PrepClimb(m_elevator, m_windmill)).onFalse(new CGClimb(m_windmill, m_elevator));

        /* Algae Floor Pickup */
        m_joystick.a().onTrue(new AlgaeFloorPickup(m_elevator, m_windmill, m_manipulator));

        /* Algae on Coral Pickup */
        // m_joystick.povDown().and(m_joystick.leftBumper())
        //     .onTrue(new AlgaeStandingPickup(m_elevator, m_windmill, m_manipulator));

        //m_joystick.povRight().and(m_joystick.leftBumper()).onTrue(new AlgaeL2Pickup(m_elevator, m_windmill, m_manipulator));

        //m_joystick.povLeft().and(m_joystick.leftBumper()).onTrue(new AlgaeL3Pickup(m_elevator, m_windmill, m_manipulator));

       // m_joystick.povUp().and(m_joystick.leftBumper()).onTrue(new ProcessAlgae(m_elevator, m_windmill));
       
        m_joystick.povUp().and(m_joystick.leftBumper()).onFalse(new RunManipulator(
            ManipulatorCalibrations.kAlgaeProcessorVelocity, 
            ManipulatorCalibrations.kCoralAcceleration, 
            m_manipulator).withTimeout(1));


        // *********************      Configure the co-pilot controller     ***********************************************

        // Zero Elevator
        Trigger coPilotRed1 = new Trigger(() -> m_coPilot.getRawButton(1));
        Trigger coPilotRed2 = new Trigger(() -> m_coPilot.getRawButton(2));
        Trigger coPilotOrange1 = new Trigger(() -> m_coPilot.getRawButton(3));
        Trigger coPilotOrange2 = new Trigger(() -> m_coPilot.getRawButton(4));
        Trigger coPilotOrange3 = new Trigger(() -> m_coPilot.getRawButton(5));
        Trigger coPilotOrange4 = new Trigger(() -> m_coPilot.getRawButton(6));
        Trigger coPilotBlue1 = new Trigger(() -> m_coPilot.getRawButton(7));
        Trigger coPilotBlue2 = new Trigger(() -> m_coPilot.getRawButton(8));
        Trigger coPilotLever1Up = new Trigger(() -> m_coPilot.getRawButton(13));
        Trigger coPilotLever1Down = new Trigger(() -> m_coPilot.getRawButton(14));
        Trigger coPilotLowPower =  new Trigger(() -> m_coPilot.getRawButton(16));

        
        coPilotRed1.onTrue(new L4(m_elevator, m_windmill));
        coPilotRed1.onFalse(new CGOuttakeThenStow(ManipulatorCalibrations.kL4OuttakeSpeed, 
            ManipulatorCalibrations.kL4OuttakeTime, 
            m_elevator, m_windmill, m_manipulator));
        
        coPilotOrange1.onTrue(new L3(m_elevator, m_windmill));
        coPilotOrange1.onFalse(new CGOuttakeThenStow(ManipulatorCalibrations.kL3OuttakeSpeed,
            ManipulatorCalibrations.kL3OuttakeTime,
            m_elevator, m_windmill, m_manipulator));
        
        coPilotOrange2.onTrue(new AlgaeL3Pickup(m_elevator, m_windmill, m_manipulator));

        coPilotOrange3.onTrue(new L2(m_elevator, m_windmill));
        coPilotOrange3.onFalse(new CGOuttakeThenStow(ManipulatorCalibrations.kL2OuttakeSpeed,
            ManipulatorCalibrations.kL2OuttakeTime, 
            m_elevator, m_windmill, m_manipulator));
        
        coPilotOrange4.onTrue(new AlgaeL2Pickup(m_elevator, m_windmill, m_manipulator));

        coPilotBlue1.onTrue(new AlgaeStandingPickup(m_elevator, m_windmill, m_manipulator))
            .onFalse(new LollipopStow(m_elevator, m_windmill));

        coPilotBlue2.onTrue(new ProcessAlgae(m_elevator, m_windmill))
            .onFalse(new RunManipulator(ManipulatorCalibrations.kAlgaeProcessorVelocity, 
                ManipulatorCalibrations.kMaxAcceleration, m_manipulator).withTimeout(3));
        

        coPilotLever1Up.onTrue(new RunManipulator(ManipulatorCalibrations.kL4OuttakeSpeed,
                 ManipulatorCalibrations.kMaxAcceleration, m_manipulator))
            .onFalse(new RunManipulator(0, ManipulatorCalibrations.kMaxAcceleration, m_manipulator));


        coPilotLever1Down.onTrue(new RunManipulator(30, ManipulatorCalibrations.kMaxAcceleration, m_manipulator))
            .onFalse(new RunManipulator(0, ManipulatorCalibrations.kMaxAcceleration, m_manipulator));
        
        

        // **********************      Driver + Copilot bindings      *******************************************************
        
        
        //coPilotRed2.and(coPilotLowPower).onTrue(new BargeAlgae(m_elevator, m_windmill))
        m_joystick.leftBumper().and(coPilotLowPower).onTrue(new BargeAlgae(m_elevator, m_windmill))
            .onFalse(new RunManipulator(ManipulatorCalibrations.kAlgaeBargingLowVelocity,
            ManipulatorCalibrations.kBargeAlgaeAcceleration, 
                                        m_manipulator).withTimeout(1));

        //coPilotRed2.and(coPilotLowPower.negate()).onTrue(new BargeAlgae(m_elevator, m_windmill))
        m_joystick.leftBumper().and(coPilotLowPower.negate()).onTrue(new BargeAlgae(m_elevator, m_windmill))
            .onFalse(new RunManipulator(ManipulatorCalibrations.kAlgaeBargingVelocity,
                                    ManipulatorCalibrations.kBargeAlgaeAcceleration, 
                                    m_manipulator).withTimeout(1).deadlineFor(
                                                        new MoveWindmillToPosition(
                                                            WindmillCalibrations.kBargeFlickPosition, 
                                                            WindmillCalibrations.kBargeTolerance,
                                                            false, m_windmill)));
    }


    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
