// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral_Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Coral_Outake_Cmd extends Command {
  /** Creates a new coralIntakeInCmd. */
  CoralIntakeSubsystem m_coralIntakeSubsystem;
  public Coral_Outake_Cmd(CoralIntakeSubsystem coralIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_coralIntakeSubsystem = coralIntakeSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_coralIntakeSubsystem.intakeOut();
    System.out.println("yes it out");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coralIntakeSubsystem.intakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
