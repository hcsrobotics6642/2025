
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Optional;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;
    private final boolean kUseLimelight = true;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // NEW: Log current robot pose to SmartDashboard for troubleshooting
        Pose2d currentPose = m_robotContainer.drivetrain.getState().Pose;
        SmartDashboard.putNumber("Robot X (m)", currentPose.getX());
        SmartDashboard.putNumber("Robot Y (m)", currentPose.getY());
        SmartDashboard.putNumber("Robot Rotation (deg)", currentPose.getRotation().getDegrees());

        if (kUseLimelight) {
            var driveState = m_robotContainer.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            LimelightHelpers.SetRobotOrientation(constants.VisionConstants.LIMELIGHT_LEFT_NAME, headingDeg, 0, 0, 0, 0, 0);
            LimelightHelpers.SetRobotOrientation(constants.VisionConstants.LIMELIGHT_RIGHT_NAME, headingDeg, 0, 0, 0, 0, 0);

            var llMeasurement1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(constants.VisionConstants.LIMELIGHT_LEFT_NAME);
            if (llMeasurement1 != null && llMeasurement1.tagCount > 0 && omegaRps < 2.0) {
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement1.pose, Utils.fpgaToCurrentTime(llMeasurement1.timestampSeconds));
            }

            var llMeasurement2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(constants.VisionConstants.LIMELIGHT_RIGHT_NAME);
            if (llMeasurement2 != null && llMeasurement2.tagCount > 0 && omegaRps < 2.0) {
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement2.pose, Utils.fpgaToCurrentTime(llMeasurement2.timestampSeconds));
            }
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            NetworkTableEntry allianceEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("alliance");
            if (alliance.get() == Alliance.Blue) {
                allianceEntry.setString("blue");
            } else if (alliance.get() == Alliance.Red) {
                allianceEntry.setString("red");
            }
        }

        Pose2d startPose;
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) {
                startPose = new Pose2d(7.51, 5.08, Rotation2d.fromDegrees(180));
            } else {
                startPose = new Pose2d(0.72, 5.08, Rotation2d.fromDegrees(0));
            }
            m_robotContainer.drivetrain.resetPose(startPose);
        }

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
