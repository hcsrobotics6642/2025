package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConfig;

public class intake extends SubsystemBase {
    private final SparkFlex left;
    private final SparkFlex right;
    private final RelativeEncoder encoder;
    private final LaserCan exitOptic; // Renamed for clarity (existing sensor at chute exit)
    private final LaserCan entryOptic; // New sensor at chute entry
    private final SparkClosedLoopController intakePID;
    private final ShuffleboardTab tab = Shuffleboard.getTab("optic");
    private final GenericEntry exitDistanceEntry;
    private final GenericEntry exitStatusEntry;
    private final GenericEntry exitDetectionEntry;
    private final GenericEntry entryDistanceEntry;
    private final GenericEntry entryStatusEntry;
    private final GenericEntry entryDetectionEntry;

    public intake() {
        left = new SparkFlex(CANConfig.CORAL_RUN_LEFT, MotorType.kBrushless);
        right = new SparkFlex(CANConfig.CORAL_RUN_RIGHT, MotorType.kBrushless);
        exitOptic = new LaserCan(CANConfig.INTAKE_LASER_SENSOR); // Existing exit sensor
        entryOptic = new LaserCan(CANConfig.INTAKE_LASER_ENTRY_SENSOR); // New entry sensor (add to CANConfig)

        intakePID = left.getClosedLoopController();
        encoder = left.getEncoder();
        SparkMaxConfig Leftconfig = new SparkMaxConfig();
        Leftconfig
            .idleMode(IdleMode.kBrake);
        Leftconfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        Leftconfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(.05)
            .d(0);
        
        SparkMaxConfig Rightconfig = new SparkMaxConfig();
        Rightconfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .follow(left, true);

        left.configure(Leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        right.configure(Rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure exit sensor
        try {
            exitOptic.setRangingMode(LaserCan.RangingMode.SHORT); // Optimized for close-range
            exitOptic.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS); // Reliable updates
            exitOptic.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
        } catch (ConfigurationFailedException e) {
            System.out.println("Exit LaserCAN configuration failed: " + e);
        }

        // Configure entry sensor
        try {
            entryOptic.setRangingMode(LaserCan.RangingMode.SHORT);
            entryOptic.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
            entryOptic.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
        } catch (ConfigurationFailedException e) {
            System.out.println("Entry LaserCAN configuration failed: " + e);
        }

        // Initialize Shuffleboard entries for debugging
        exitDistanceEntry = tab.add("Exit LaserCAN Distance", 0)
                               .withWidget(BuiltInWidgets.kTextView)
                               .getEntry();
        exitStatusEntry = tab.add("Exit LaserCAN Status", "Unknown")
                             .withWidget(BuiltInWidgets.kTextView)
                             .getEntry();
        exitDetectionEntry = tab.add("Exit Object Detected", false)
                                .withWidget(BuiltInWidgets.kBooleanBox)
                                .getEntry();
        entryDistanceEntry = tab.add("Entry LaserCAN Distance", 0)
                                .withWidget(BuiltInWidgets.kTextView)
                                .getEntry();
        entryStatusEntry = tab.add("Entry LaserCAN Status", "Unknown")
                              .withWidget(BuiltInWidgets.kTextView)
                              .getEntry();
        entryDetectionEntry = tab.add("Entry Object Detected", false)
                                 .withWidget(BuiltInWidgets.kBooleanBox)
                                 .getEntry();
    }

    public void hold(double pos) {
        intakePID.setReference(pos, ControlType.kPosition);
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
        boolean entryDetected = isEntry();
        boolean exitDetected = isCoral();
        if (exitDetected) {
            left.set(0); // Stop immediately when Coral reaches exit
        } else if (entryDetected) {
            left.set(0.1); // Slow speed when Coral enters chute
        } else {
            left.set(0.2); // Normal speed when chute is empty
        }
    }

    public boolean isCoral() { // Exit sensor detection
        LaserCan.Measurement measurement = exitOptic.getMeasurement();
        boolean detected = false;
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            exitDistanceEntry.setDouble(measurement.distance_mm);
            System.out.println("Exit LaserCAN Distance: " + measurement.distance_mm + "mm");
            if (measurement.distance_mm < 20) { // Tight threshold for exit
                detected = true;
            }
        } else {
            exitDistanceEntry.setDouble(-1);
            System.out.println("Exit LaserCAN Invalid: " + (measurement != null ? measurement.status : "No measurement"));
        }
        exitDetectionEntry.setBoolean(detected);
        return detected;
    }

    public boolean isEntry() { // Entry sensor detection
        LaserCan.Measurement measurement = entryOptic.getMeasurement();
        boolean detected = false;
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            entryDistanceEntry.setDouble(measurement.distance_mm);
            System.out.println("Entry LaserCAN Distance: " + measurement.distance_mm + "mm");
            if (measurement.distance_mm < 50) { // Wider threshold for entry
                detected = true;
            }
        } else {
            entryDistanceEntry.setDouble(-1);
            System.out.println("Entry LaserCAN Invalid: " + (measurement != null ? measurement.status : "No measurement"));
        }
        entryDetectionEntry.setBoolean(detected);
        return detected;
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
        // Update exit sensor
        LaserCan.Measurement exitMeas = exitOptic.getMeasurement();
        if (exitMeas != null && exitMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            exitDistanceEntry.setDouble(exitMeas.distance_mm);
            exitStatusEntry.setString("Valid");
        } else {
            exitDistanceEntry.setDouble(-1);
            exitStatusEntry.setString("Invalid: " + (exitMeas != null ? exitMeas.status : "No measurement"));
        }

        // Update entry sensor
        LaserCan.Measurement entryMeas = entryOptic.getMeasurement();
        if (entryMeas != null && entryMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            entryDistanceEntry.setDouble(entryMeas.distance_mm);
            entryStatusEntry.setString("Valid");
        } else {
            entryDistanceEntry.setDouble(-1);
            entryStatusEntry.setString("Invalid: " + (entryMeas != null ? entryMeas.status : "No measurement"));
        }
    }
}