// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.EqualsUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SafelyMoveToScoringPosition extends Command {

  Superstructure m_superstructure;
  int m_level = 0;

  double desiredElevatorPosition = 0;
  double desiredCoralPivotPosition = 0;

  double elevatorTolerance = 1;
  double coralPivotTolerance = 1;

  boolean isElevatorAtPosition = false;
  boolean isCoralPivotAtPosition = false;

  /** Creates a new SafelyMoveToScoringPosition. */
  public SafelyMoveToScoringPosition(Superstructure superstructure, int level) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(superstructure);

    m_superstructure = superstructure;
    m_level = level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     isElevatorAtPosition = false;
     isCoralPivotAtPosition = false;
    m_superstructure.climber.runPosition(0); // just make sure i'm still trying to  hold my position straight up, and out of the way.

    //Check if the coral pivot is too far forward, and if so, move it back
    if(m_superstructure.coralPivot.getPosition() > CoralPivotConstants.Setpoints.home) {  // Tune this to some safe position...
      m_superstructure.coralPivot.runPosition(CoralPivotConstants.Setpoints.home-2);
    }

    // calculate the desired setpoints from the level
    switch (m_level) {
      case 4:
        desiredElevatorPosition = ElevatorConstants.Setpoints.level4;
        desiredCoralPivotPosition = CoralPivotConstants.Setpoints.level4;        
        break;
      case 3:
        desiredElevatorPosition = ElevatorConstants.Setpoints.level3;
        desiredCoralPivotPosition = CoralPivotConstants.Setpoints.level3;        
        break;
      case 2:
        desiredElevatorPosition = ElevatorConstants.Setpoints.level2;
        desiredCoralPivotPosition = CoralPivotConstants.Setpoints.level2;        
        break;
      case 1:
        desiredElevatorPosition = ElevatorConstants.Setpoints.level1;
        desiredCoralPivotPosition = CoralPivotConstants.Setpoints.level1;
        break;
    
      default:
        desiredElevatorPosition = m_superstructure.elevator.getPosition();
        desiredCoralPivotPosition = m_superstructure.coralPivot.getPosition();
        break;
    }



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //check if the elevator is at the correct position
    if (EqualsUtil.epsilonEquals(m_superstructure.elevator.getPosition(), desiredElevatorPosition, elevatorTolerance) ) {
      isElevatorAtPosition = true;
    }
    else {
      m_superstructure.elevator.runPosition(desiredElevatorPosition);
    }

    //check if the coral pivot is at the correct position
    if (EqualsUtil.epsilonEquals(m_superstructure.coralPivot.getPosition(), desiredCoralPivotPosition, coralPivotTolerance) ) {
      isCoralPivotAtPosition = true;
    }
    else {
      if(m_superstructure.elevator.getPosition() >= 10)
        m_superstructure.coralPivot.runPosition(desiredCoralPivotPosition);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isElevatorAtPosition && isCoralPivotAtPosition;
  }
}
