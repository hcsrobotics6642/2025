package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConfig;
import frc.robot.constants.SystemConfig;
import static frc.robot.constants.CANConfig.LaserId;

public class intake extends SubsystemBase {
    private final SparkMax left;
    private final SparkMax right;
    private final RelativeEncoder encoder;
    private final LaserCan lc;  // LaserCAN on CAN bus (e.g., ID 0)
    private final ShuffleboardTab tab = Shuffleboard.getTab("optic");

    public intake(int canId) {
        left = new SparkMax(CANConfig.CORAL_RUN_LEFT, MotorType.kBrushless);
        right = new SparkMax(CANConfig.CORAL_RUN_RIGHT, MotorType.kBrushless);
        lc = new LaserCan(LaserId);  // Initialize LaserCAN with CAN ID
        encoder = left.getEncoder();

        SparkMaxConfig Leftconfig = new SparkMaxConfig();
        Leftconfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);

        SparkMaxConfig Rightconfig = new SparkMaxConfig();
        Rightconfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .follow(left, true);

        left.configure(Leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        right.configure(Rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure LaserCAN (short mode for intake, 33ms timing for fast response)
        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT);  // Short mode (up to 1.3m)
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);  // Fast updates
            lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
        } catch (ConfigurationFailedException e) {
            DriverStation.reportError("LaserCAN configuration failed: " + e.getMessage(), false);
        }
    }

    public void stopIntake(double speed) {
        left.set(0);
    }

    public void runAtVelocity(double setpoint) {
        left.set(setpoint);
    }

    public void runOpenLoop(double supplier) {
        left.set(supplier);
    }

    public void autoIntake() {
        if (!isCoral()) {
            left.set(SystemConfig.CORAL_INTAKE_SPEED);
        } else {
            left.set(0);
            SmartDashboard.putBoolean("NoteDetected", true);
        }
    }

    public boolean isCoral() {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            SmartDashboard.putNumber("Intake Distance (mm)", measurement.distance_mm);
            return measurement.distance_mm < 30;  // Detect game piece when < 30mm
        } else {
            DriverStation.reportWarning("LaserCAN invalid measurement", false);
            return false;  // Default to false if invalid
        }
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getOutputCurrent() {
        return left.getOutputCurrent();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getTemp() {
        return left.getMotorTemperature();
    }    

    public void periodic() {
        SmartDashboard.putBoolean("OpticSensor", isCoral());
        if (getOutputCurrent() > 50) {
            left.set(0);
            DriverStation.reportWarning("Intake jam detected", false);
        }
    }
}