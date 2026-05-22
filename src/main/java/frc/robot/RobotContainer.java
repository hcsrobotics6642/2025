package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final CommandXboxController gamepad = new CommandXboxController(0);

    private final PWM blueRelay = new PWM(0);
    private final PWM redRelay = new PWM(1);
    private final PWM greenRelay = new PWM(2);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.05)
        .withRotationalDeadband(MaxAngularRate * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(gamepad.getLeftY() * MaxSpeed)
                    .withVelocityY(gamepad.getLeftX() * MaxSpeed)
                    .withRotationalRate(gamepad.getRightX() * MaxAngularRate * -1)
            )
        );

        // Blue button / X -> PWM port 0
        gamepad.x().whileTrue(
            new StartEndCommand(
                () -> blueRelay.setSpeed(1.0),
                () -> blueRelay.setSpeed(0.0)
            )
        );

        // Red button / B -> PWM port 1
        gamepad.b().whileTrue(
            new StartEndCommand(
                () -> redRelay.setSpeed(1.0),
                () -> redRelay.setSpeed(0.0)
            )
        );

        // Green button / A -> PWM port 2
        gamepad.a().whileTrue(
            new StartEndCommand(
                () -> greenRelay.setSpeed(1.0),
                () -> greenRelay.setSpeed(0.0)
            )
        );

        // Yellow button / Y -> reset field-centric heading
        gamepad.y().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
