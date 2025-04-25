// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.CoralRollerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.EqualsUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoHome extends Command {

  Superstructure m_superstructure;
  double pivotThreshold = 5;
  double elevatorThreshold = 5;
  boolean isElevatorAtPosition = false;
  boolean isCoralPivotAtPosition = false;

  /** Creates a new CollectCoral. */
  public GoHome(Superstructure superstructure) {
    // Use addRequirements() here to declare subsystem dependencies.    

   // addRequirements(superstructure);

    m_superstructure = superstructure;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_superstructure.coralPivot.runPosition(CoralPivotConstants.Setpoints.home);
      
      //m_superstructure.climber.runPosition(ClimberConstants.Setpoints.ready);
      
     // m_superstructure.coralRoller.setPO(CoralRollerConstants.Speeds.pick);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if ( EqualsUtil.epsilonEquals(m_superstructure.coralPivot.getPosition(), CoralPivotConstants.Setpoints.home, pivotThreshold) ){
        m_superstructure.elevator.runPosition(ElevatorConstants.Setpoints.home);
        isCoralPivotAtPosition = true;
      }

      if(EqualsUtil.epsilonEquals(m_superstructure.elevator.getPosition(),ElevatorConstants.Setpoints.home, elevatorThreshold)){
        isElevatorAtPosition = true;
      }
    }
    // do nothing new... just let the coral roller run

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    //stop the rollers!

    // Hold whatever position I'm at now...
    m_superstructure.coralPivot.runPosition(m_superstructure.coralPivot.getPosition());
    m_superstructure.elevator.runPosition(m_superstructure.elevator.getPosition());

    m_superstructure.coralRoller.setPO(0.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isElevatorAtPosition && isCoralPivotAtPosition; 
  }
}
