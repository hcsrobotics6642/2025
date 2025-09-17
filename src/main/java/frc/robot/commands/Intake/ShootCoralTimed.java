// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.SystemConfig;
import frc.robot.subsystems.intake;

/** A command to run the intake motors for a specific duration to eject a game piece. */
public class ShootCoralTimed extends SequentialCommandGroup {
  /**
   * Creates a new SpitNoteTimed command.
   *
   * @param intake The intake subsystem to control.
   */
  public ShootCoralTimed(intake intake) {
    // Add your commands in the sequential command group here.
    addCommands(
        // Command to start running the intake motors in reverse
        new InstantCommand(() -> intake.runOpenLoop(-SystemConfig.CORAL_INTAKE_SPEED), intake),

        // Command to wait for a short duration to ensure the game piece is ejected
        new WaitCommand(1), // Run motors for 0.5 seconds

        // Command to stop the intake motors
        new InstantCommand(() -> intake.stopIntake(0), intake)
    );
  }
}
