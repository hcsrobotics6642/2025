package frc.robot.commands.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class LimeLight extends Command { 

    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController xPID = new PIDController(1, 0, 0);  // Lateral alignment PID
    private final PIDController yPID = new PIDController(1, 0, 0);  // Forward/backward alignment PID
    private final PIDController rotationPID = new PIDController(1, 0, 0); // Rotation PID
    private final String limelightName;
    private final Pose2d desiredPose;

    public LimeLight(CommandSwerveDrivetrain drivetrain, Pose2d desiredPose, String limelight) {
        this.drivetrain = drivetrain;
        this.desiredPose = desiredPose;
        this.limelightName = limelight;

        addRequirements(drivetrain);
        
        yPID.setSetpoint(desiredPose.getY());
        yPID.setTolerance(0.1); 

        xPID.setSetpoint(desiredPose.getX()); 
        xPID.setTolerance(0.1); 

        rotationPID.setSetpoint(desiredPose.getRotation().getRadians());
        rotationPID.setTolerance(Units.degreesToRadians(2));
    }

    @Override
    public void execute() {
        Pose2d botPose = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
    
        if (!hasTarget || botPose == null) {
            drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
            return;
        }
        
        double xError = botPose.getX() - desiredPose.getX();
        double yError = botPose.getY() - desiredPose.getY();
        double rotationError = botPose.getRotation().getRadians() - desiredPose.getRotation().getRadians();
        
        double forwardSpeed = yPID.calculate(botPose.getY());
        double lateralSpeed = xPID.calculate(botPose.getX());
        double rotationSpeed = rotationPID.calculate(botPose.getRotation().getRadians());

        drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(
            new ChassisSpeeds(forwardSpeed, lateralSpeed, rotationSpeed)
        ));
    }
    
    @Override
    public boolean isFinished() {
        return xPID.atSetpoint() && yPID.atSetpoint() && rotationPID.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
        xPID.reset();
        yPID.reset();
        rotationPID.reset();
    }
}