// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakePivotA;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_roboOn;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = true;
  //private final IntakePivot m_IntakePivot;
  //private final IntakePivotA m_IntakePivotA;
  public Robot() {
    m_robotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) {
      // Get the current robot state for both Limelights
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
  
      // Set the robot's orientation for both Limelights
      LimelightHelpers.SetRobotOrientation("limelight-left", headingDeg, 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation("limelight-right", headingDeg, 0, 0, 0, 0, 0);
  
      // Get the pose from the first Limelight and add to odometry
      var llMeasurement1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
      if (llMeasurement1 != null && llMeasurement1.tagCount > 0 && omegaRps < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement1.pose, Utils.fpgaToCurrentTime(llMeasurement1.timestampSeconds));
      }
  
      // Get the pose from the second Limelight and add to odometry
      var llMeasurement2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
      if (llMeasurement2 != null && llMeasurement2.tagCount > 0 && omegaRps < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement2.pose, Utils.fpgaToCurrentTime(llMeasurement2.timestampSeconds));
      }
    }
  }
  


  @Override
  public void disabledInit() {

}




  

  @Override
  public void disabledPeriodic() {


  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
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
   // Starts recording to data log
DataLogManager.start();
// Record both DS control and joystick data
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
