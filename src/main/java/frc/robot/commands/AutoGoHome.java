// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.CoralRollerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.EqualsUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoGoHome extends Command {

  Superstructure m_superstructure;
  int passedCycles = 0;
  double pivotThreshold;


  /** Creates a new CollectCoral. */
  public AutoGoHome(Superstructure superstructure) {
    // Use addRequirements() here to declare subsystem dependencies.
    pivotThreshold = 5;
    

    addRequirements(superstructure);

    m_superstructure = superstructure;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_superstructure.coralPivot.runPosition(CoralPivotConstants.Setpoints.home);
      passedCycles = 0;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (m_superstructure.coralPivot.getPosition() > CoralPivotConstants.Setpoints.collect-pivotThreshold && m_superstructure.coralPivot.getPosition() < CoralPivotConstants.Setpoints.collect+pivotThreshold)
    {
        System.out.println("[AutoGoHome]: Coral Pivot in Threshold");

        m_superstructure.elevator.runPosition(ElevatorConstants.Setpoints.collect_flush);
      
    }
    passedCycles++;

  }  // do nothing new... just let the coral roller run

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    //stop the rollers!
    System.out.println("[AutoGoHome]: Hit end logic");
    // Hold whatever position I'm at now...
    m_superstructure.coralPivot.runPosition(m_superstructure.coralPivot.getPosition());
    m_superstructure.elevator.runPosition(m_superstructure.elevator.getPosition());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return passedCycles >= 75; // was 50
  }
}
