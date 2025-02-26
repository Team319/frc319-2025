// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralRollerConstants;
import frc.robot.subsystems.superstructure.Superstructure;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoral extends Command {

  Superstructure m_superstructure;

  /** Creates a new ScoreCoral. */
  public ScoreCoral(Superstructure superstructure) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(superstructure);

    m_superstructure = superstructure;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_superstructure.coralRoller.setPO(CoralRollerConstants.Speeds.score);
    
    // If elevator and coral pivot drop when called. try uncommenting these lines below

    //m_superstructure.coralPivot.runPosition(m_superstructure.coralPivot.getPosition());
    //m_superstructure.elevator.runPosition(m_superstructure.elevator.getPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_superstructure.coralRoller.setPO(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
