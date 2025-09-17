
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.constants;

public class SeedPose extends InstantCommand {
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;

    public SeedPose(CommandSwerveDrivetrain drivetrain, String limelightName) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (poseEstimate.tagCount >= 1) {
            drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
            System.out.println("Seeded pose: " + poseEstimate.pose);
        } else {
            System.out.println("No valid AprilTag data from " + limelightName);
        }
    }
}