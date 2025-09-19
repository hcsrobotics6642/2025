// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.commands.SeedPose;
import java.lang.reflect.WildcardType;
import frc.robot.commands.limelight.AlignToAprilTagWithOffset;
import frc.robot.commands.limelight.AlignToAprilTagWithOffset.AlignmentPosition;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.CANConfig;
import frc.robot.commands.Climb;
import frc.robot.commands.Arm.RunArmClosedLoop;
import frc.robot.commands.Intake.IntakeSUCK;
import frc.robot.commands.Intake.Intakespit;
import frc.robot.commands.Intake.RunIntakeOpenLoopCoral;
import frc.robot.commands.Intake.ShootCoralTimed;
import frc.robot.commands.Intake.ShootCoralTimed;
import frc.robot.commands.IntakeA.RunIntakeOpenLoop;
import frc.robot.commands.limelight.AlignToTag;
import frc.robot.commands.limelight.LimeLight;
import frc.robot.generated.T16000MController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakePivotA;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.intake;

public class RobotContainer {
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Elevator m_Elevator = new Elevator(CANConfig.ELEVATOR_FRONT, CANConfig.ELEVATOR_BACK);
    private final climber m_climber = new climber();
    private final IntakePivotA m_IntakePivotA = new IntakePivotA();
    final IntakePivot m_IntakePivot = new IntakePivot();
    private final intake m_intake = new intake();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final T16000MController joystick = new T16000MController(0);
    private final T16000MController joystick2 = new T16000MController(1);
    private final CommandXboxController gamepad = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

// In RobotContainer.java, add as a field
private final Command pivotToScore = new RunIntakeOpenLoopCoral(m_IntakePivot, ArmConstants.CPivotScore);

// Update command groups
SequentialCommandGroup L1 = new SequentialCommandGroup(
    pivotToScore,
    new WaitCommand(1),
    new RunArmClosedLoop(m_Elevator, ArmConstants.L1pos)
);
SequentialCommandGroup L2 = new SequentialCommandGroup(
    pivotToScore,
    new WaitCommand(1),
    new RunArmClosedLoop(m_Elevator, ArmConstants.L2pos)
);
SequentialCommandGroup L3 = new SequentialCommandGroup(
    pivotToScore,
    new WaitCommand(1),
    new RunArmClosedLoop(m_Elevator, ArmConstants.L3pos)
);
SequentialCommandGroup L4 = new SequentialCommandGroup(
    pivotToScore,
    new WaitCommand(1),
    new RunArmClosedLoop(m_Elevator, ArmConstants.L4pos)
);

    SequentialCommandGroup CStow = new SequentialCommandGroup(
        new RunIntakeOpenLoopCoral(m_IntakePivot, ArmConstants.CPivotStow)
    );

    SequentialCommandGroup goLeft = new SequentialCommandGroup(
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric()),
        drivetrain.applyRequest(() ->
            drive.withVelocityX(0)
                .withVelocityY(-1 * MaxSpeed)
                .withRotationalRate(0)
        ).withTimeout(0.130),
        drivetrain.applyRequest(() ->
            drive.withVelocityX(joystick.getLeftY() * MaxSpeed)
                .withVelocityY(joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(joystick2.getLeftX() * MaxAngularRate * -1)
        )
    );
    SequentialCommandGroup goRight = new SequentialCommandGroup(
        drivetrain.applyRequest(() ->
            drive.withVelocityX(0)
                .withVelocityY(1 * MaxSpeed)
                .withRotationalRate(0)
        ).withTimeout(0.130),
        drivetrain.applyRequest(() ->
            drive.withVelocityX(joystick.getLeftY() * MaxSpeed)
                .withVelocityY(joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(joystick2.getLeftX() * MaxAngularRate * -1)
        )
    );

    public RobotContainer() {
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("L1", L1);
        NamedCommands.registerCommand("L2", L2);
        NamedCommands.registerCommand("L3", L3);
        NamedCommands.registerCommand("L4", L4);
        NamedCommands.registerCommand("IntakeSuck", new IntakeSUCK(m_intake));
        //NamedCommands.registerCommand("TimedIntakeSpit", new ShootCoralTimed(m_intake));
        NamedCommands.registerCommand("IntakeSpit", new Intakespit(m_intake)); // Add for mismatch in paths; use if different from TimedIntakeSpit
        NamedCommands.registerCommand("ShootCoralAuto", new ShootCoralTimed(m_intake)); // Alias for mismatch in "Left Start.path"
        NamedCommands.registerCommand("HumanStation", CStow);
        NamedCommands.registerCommand("WaitCommand_1s", new WaitCommand(1));
        NamedCommands.registerCommand("SeedPose", new SeedPose(drivetrain, constants.VisionConstants.LIMELIGHT_LEFT_NAME));
        // Ensure PathPlanner marker names exactly match these (case-sensitive). Add named commands to markers in GUI via + button.
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(joystick2.getLeftX() * MaxAngularRate * -1)
            )
        );

        joystick2.R().whileTrue(
            new AlignToAprilTagWithOffset(drivetrain, AlignmentPosition.RIGHT)
        );
        joystick2.L().whileTrue(
            new AlignToAprilTagWithOffset(drivetrain, AlignmentPosition.LEFT)
        );

        // NEW: Toggle robot-centric mode with joystick2 trigger (hold to use, release for field-centric)
        joystick2.T().whileTrue(
        drivetrain.applyRequest(() ->
            forwardStraight
                .withVelocityX(joystick.getLeftY() * MaxSpeed)  // Forward/back relative to robot
                .withVelocityY(joystick.getLeftX() * MaxSpeed)  // Strafe relative to robot
                .withRotationalRate(joystick2.getLeftX() * MaxAngularRate)  // Spin relative to robot
        )
    );

        gamepad.rightBumper().whileTrue(new Intakespit(m_intake));
        gamepad.leftBumper().whileTrue(new IntakeSUCK(m_intake));

        gamepad.povRight().onTrue(new RunIntakeOpenLoopCoral(m_IntakePivot, ArmConstants.CPivotStow));
        gamepad.povDown().onTrue(new RunIntakeOpenLoopCoral(m_IntakePivot, ArmConstants.CPivotScore));

        gamepad.povUp().onTrue(L1);
        gamepad.a().onTrue(L2);
        gamepad.x().onTrue(L3);
        gamepad.leftStick().onTrue(L4);

        gamepad.y().whileTrue(new Climb(m_climber, 0.5));
        gamepad.b().whileTrue(new Climb(m_climber, -0.5));

        gamepad.rightTrigger().whileTrue(new RunIntakeOpenLoop(m_IntakePivotA, 9));
        gamepad.leftTrigger().whileTrue(new RunIntakeOpenLoop(m_IntakePivotA, 0));

        joystick.T().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}