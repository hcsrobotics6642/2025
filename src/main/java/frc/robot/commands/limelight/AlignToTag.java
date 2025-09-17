package frc.robot.commands.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;

/**
 * A command that aligns the robot to a specified distance from an AprilTag.
 * This command uses the Limelight's 3D pose estimation relative to the target,
 * which is more reliable than distance calculations based on target area.
 */
public class AlignToTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController distancePID = new PIDController(1, 0, 0);  // PID for forward/backward movement
    private final PIDController lateralPID = new PIDController(1, 0, 0);  // PID for left/right movement
    private final PIDController rotationPID = new PIDController(1, 0, 0); // PID for rotation
    private final String limelightName;
    private final double desiredDistanceMeters;
    private final Rotation2d desiredRotation;

    /**
     * Creates a new AlignToTag command.
     * @param drivetrain The swerve drivetrain subsystem.
     * @param desiredDistanceMeters The desired distance from the AprilTag in meters.
     * @param desiredRotation The desired final rotation of the robot relative to the tag.
     * @param limelightName The name of the Limelight camera to use.
     */
    public AlignToTag(CommandSwerveDrivetrain drivetrain, double desiredDistanceMeters, Rotation2d desiredRotation, String limelightName) {
        this.drivetrain = drivetrain;
        this.desiredDistanceMeters = desiredDistanceMeters;
        this.desiredRotation = desiredRotation;
        this.limelightName = limelightName;
        
        // Add the drivetrain subsystem as a requirement.
        // This ensures no other command can control the drivetrain at the same time.
        addRequirements(drivetrain);

        // Set the setpoints and tolerances for the PID controllers.
        // The setpoint is the desired value (e.g., 0 for lateral error).
        distancePID.setSetpoint(0);
        lateralPID.setSetpoint(0);
        rotationPID.setSetpoint(desiredRotation.getRadians());
        
        // Set tolerances to determine when the command is finished.
        // These values should be tuned for your robot.
        distancePID.setTolerance(Units.inchesToMeters(2)); // Within 2 inches of the target distance
        lateralPID.setTolerance(Units.inchesToMeters(2)); // Within 2 inches of the center line
        rotationPID.setTolerance(Units.degreesToRadians(2)); // Within 2 degrees of the target rotation
    }

    @Override
    public void execute() {
        // Get the robot's pose relative to the AprilTag.
        // The x, y, z coordinates are the distance from the tag.
        Pose3d botPoseTargetSpace = LimelightHelpers.getBotPose3d_TargetSpace(limelightName);
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
    
        // If no target is visible, stop the robot and exit the command.
        if (!hasTarget || botPoseTargetSpace == null) {
            drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
            return;
        }
        
        // Calculate the error for each dimension.
        // The X component of the pose is the distance from the tag.
        double distanceError = botPoseTargetSpace.getX() - desiredDistanceMeters;
        // The Y component is the lateral (sideways) error.
        double lateralError = botPoseTargetSpace.getY();
        
        // Get the robot's rotation relative to the tag and calculate the error.
        double rotationError = botPoseTargetSpace.getRotation().getZ() - desiredRotation.getRadians();
        
        // Use the PID controllers to calculate the speeds needed to correct the errors.
        double forwardSpeed = distancePID.calculate(distanceError);
        double lateralSpeed = lateralPID.calculate(lateralError);
        double rotationSpeed = rotationPID.calculate(rotationError);

        // Send the new speeds to the drivetrain.
        drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(
            new ChassisSpeeds(forwardSpeed, lateralSpeed, rotationSpeed)
        ));
    }
    
    @Override
    public boolean isFinished() {
        // The command is finished when all three PID controllers are at their setpoints.
        return distancePID.atSetpoint() && lateralPID.atSetpoint() && rotationPID.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        // When the command ends, stop the drivetrain.
        drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }
}
