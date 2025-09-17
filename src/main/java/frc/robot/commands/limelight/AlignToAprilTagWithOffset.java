package frc.robot.commands.limelight;

import com.ctre.phoenix6.swerve.SwerveRequest; 

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.*;

public class AlignToAprilTagWithOffset extends Command {

    public enum AlignmentPosition {
        LEFT(-6.0),  // Manipulator 6 inches to the left of center
        RIGHT(6.0), // Manipulator 6 inches to the right of center
        CENTER(0.0);

        public final double offsetInches;

        AlignmentPosition(double offsetInches) {
            this.offsetInches = offsetInches;
        }
    }

    private final CommandSwerveDrivetrain m_drivetrain;
    private final AlignmentPosition m_alignmentPosition;
    
    private final PIDController m_strafeController;
    private final PIDController m_distanceController;
    private final PIDController m_rotationController;

    private double m_targetAngle; // The target heading for the robot to hold

    // Setpoint for ty (angle). Adjust for desired scoring distance (e.g., -15° ~1m for reef)
    private static final double DISTANCE_SETPOINT_TY = -15.0; 

    private final Timer timer = new Timer();

    public AlignToAprilTagWithOffset(CommandSwerveDrivetrain drivetrain, AlignmentPosition alignmentPosition) {
        m_drivetrain = drivetrain;
        m_alignmentPosition = alignmentPosition;

        // Tune these PID values for your robot
        m_strafeController = new PIDController(0.04, 0.0, 0.004);
        m_distanceController = new PIDController(0.05, 0.0, 0.005);
        m_rotationController = new PIDController(0.02, 0.0, 0.0);

        m_strafeController.setTolerance(0.5); 
        m_distanceController.setTolerance(0.5);
        m_rotationController.setTolerance(1.0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_targetAngle = m_drivetrain.getState().Pose.getRotation().getDegrees();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        boolean leftHasTarget = LimelightHelpers.getTV("limelight-left");
        boolean rightHasTarget = LimelightHelpers.getTV("limelight-right");

        if (!leftHasTarget && !rightHasTarget) {
            timer.start();
            if (timer.hasElapsed(2.0)) {
                m_drivetrain.setControl(new SwerveRequest.FieldCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            }
            return;
        }

        int tagID = leftHasTarget ? (int) LimelightHelpers.getFiducialID("limelight-left") : (int) LimelightHelpers.getFiducialID("limelight-right");
        if (tagID < 6 || tagID > 11) {  // Reef tags only (6-11)
            m_drivetrain.setControl(new SwerveRequest.FieldCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        double tx_L = leftHasTarget ? LimelightHelpers.getTX("limelight-left") : 0.0;
        double ty_L = leftHasTarget ? LimelightHelpers.getTY("limelight-left") : 0.0;
        double tx_R = rightHasTarget ? LimelightHelpers.getTX("limelight-right") : 0.0;
        double ty_R = rightHasTarget ? LimelightHelpers.getTY("limelight-right") : 0.0;

        double avg_tx = leftHasTarget && rightHasTarget ? (tx_L + tx_R) / 2.0 : (leftHasTarget ? tx_L : tx_R);
        double avg_ty = leftHasTarget && rightHasTarget ? (ty_L + ty_R) / 2.0 : (leftHasTarget ? ty_L : ty_R);
        
        // Dynamic height for tag ID (reef 12.13, processor 51.25, coral 58.5)
        double tagHeight = VisionConstants.REEF_APRILTAG_HEIGHT.in(Inches);  // Default reef; adjust per ID if needed
        double distanceInches = (tagHeight - VisionConstants.LIMELIGHT_LENS_HEIGHT.in(Inches))
                / Math.tan(Math.toRadians(VisionConstants.LIMELIGHT_ANGLE.in(Degrees) + avg_ty));

        double strafeSetpointDegrees = Math.toDegrees(Math.atan2(m_alignmentPosition.offsetInches, distanceInches));

        double strafeOutput = m_strafeController.calculate(avg_tx, strafeSetpointDegrees);
        double distanceOutput = m_distanceController.calculate(avg_ty, DISTANCE_SETPOINT_TY);  // Removed negative sign
        double rotationOutput = m_rotationController.calculate(m_drivetrain.getState().Pose.getRotation().getDegrees(), m_targetAngle);

        SmartDashboard.putNumber("DistanceInches", distanceInches);
        SmartDashboard.putNumber("StrafeSetpointDeg", strafeSetpointDegrees);
        SmartDashboard.putNumber("DistanceError", avg_ty - DISTANCE_SETPOINT_TY);
        SmartDashboard.putNumber("StrafeError", avg_tx - strafeSetpointDegrees);
        SmartDashboard.putNumber("RotationError", m_drivetrain.getState().Pose.getRotation().getDegrees() - m_targetAngle);

        m_drivetrain.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(distanceOutput)
                .withVelocityY(strafeOutput)
                .withRotationalRate(rotationOutput));
    }

    @Override
    public boolean isFinished() {
        return m_strafeController.atSetpoint() && m_distanceController.atSetpoint() && m_rotationController.atSetpoint() || timer.hasElapsed(2.0);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(new SwerveRequest.FieldCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
}