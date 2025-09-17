package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;

public class HoldIntake extends Command {
  private final intake intake;
  private double setpoint;

  /** Creates a new HoldIntake. */
  public HoldIntake(intake in) {
    intake = in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(in);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = 0.1;  // Fixed: Use a constant low voltage for open-loop holding
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runOpenLoop(setpoint);  // Fixed: Use runOpenLoop instead of hold
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake(0);  // Stop intake when command ends
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  // Run until interrupted
  }
}