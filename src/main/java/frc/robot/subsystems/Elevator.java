package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;

public class Elevator extends SubsystemBase {
    public final SparkMax LeftMotor;
    public final SparkMax RightMotor;
    public final double conf;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController armPID;
    private final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    private final GenericEntry currentEntry = tab.add("position", 0).getEntry();

    public Elevator(int elevatorFront, int elevatorBack) {
        LeftMotor = new SparkMax(CANConfig.ELEVATOR_BACK, MotorType.kBrushless);
        RightMotor = new SparkMax(CANConfig.ELEVATOR_FRONT, MotorType.kBrushless);
        conf = 0.37305; // Inches per motor rotation (2.375" OD pulley, 20:1 gear ratio)
        SparkMaxConfig Leftconfig = new SparkMaxConfig();
        Leftconfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        Leftconfig.encoder
            .positionConversionFactor(conf) // Scales to inches
            .velocityConversionFactor(conf);
        Leftconfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.05)
            .d(0);

        SparkMaxConfig Rightconfig = new SparkMaxConfig();
        Rightconfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .follow(LeftMotor, true);

        LeftMotor.configure(Leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RightMotor.configure(Rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = LeftMotor.getEncoder();
        armPID = LeftMotor.getClosedLoopController();
    }

    public void runOpenLoop(double supplier) {
        if (getPos() >= ArmConstants.kElevatorUpperLimit) {
            LeftMotor.set(0);
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
        } else if (getPos() <= ArmConstants.kElevatorLowerLimit) {
            LeftMotor.set(0);
            System.out.println("¡TOO LOW! ¡LOWER LIMIT!");
        } else {
            LeftMotor.set(supplier);
        }
    }

    public void hold() {
        if (getPos() < ArmConstants.kElevatorUpperLimit) {
            LeftMotor.set(0.0);
        }
    }

    public void stopArm(double speed) {
        LeftMotor.set(0);
    }

    public void runToPosition(double setpoint) {
        System.out.println("Arm Current Position: " + getPos() + " inches");
        System.out.println("Setpoint: " + setpoint + " inches");
        if (getPos() >= ArmConstants.kElevatorUpperLimit) {
            LeftMotor.set(-1);
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
            System.out.println("Position: " + getPos() + " inches");
        } else if (getPos() <= ArmConstants.kElevatorLowerLimit) {
            LeftMotor.set(1);
            System.out.println("¡TOO LOW! ¡LOWER LIMIT!");
            System.out.println("Position: " + getPos() + " inches");
        } else {
            armPID.setReference(setpoint, ControlType.kPosition); // Setpoint in inches
        }
    }

    public double getPos() {
        return encoder.getPosition(); // Returns inches
    }

    public void periodic() {
        double current = getPos();
        System.out.println("Elevator: " + current + " inches, " + (current / conf) + " rotations");
        SmartDashboard.putNumber("Position (inches)", current);
        currentEntry.setDouble(current);
    }
}