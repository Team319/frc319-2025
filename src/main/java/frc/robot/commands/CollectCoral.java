// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.Superstructure;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectCoral extends Command {

  Superstructure m_superstructure;
  boolean isElevatorAtPosition = false;
  boolean isCoralPivotAtPosition = false;
  double pivotThreshold = 5;
  int passedCycles = 0;

  double currentDebounceCounter = 0;
  double detectCurrent = 13; // Tune this current limit number with Advantagescope looking at RealOutputs/CoralRoller/MotorStatorCurrent
  double currentTolerance = 0.1;

  /** Creates a new CollectCoral. */
  public CollectCoral(Superstructure superstructure) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(superstructure);
    m_superstructure = superstructure;


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_superstructure.climber.runPosition(ClimberConstants.Setpoints.ready);
    m_superstructure.coralPivot.runPosition(CoralPivotConstants.Setpoints.collect);
    m_superstructure.coralRoller.setPO(0.1);

    passedCycles = 0;
    currentDebounceCounter = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_superstructure.climber.getPosition() > ClimberConstants.Setpoints.ready-pivotThreshold && m_superstructure.climber.getPosition() < ClimberConstants.Setpoints.ready+pivotThreshold){      
      
      if(passedCycles >= 10){
        m_superstructure.elevator.runPosition(ElevatorConstants.Setpoints.collect_flush);
      }
    }

    if(m_superstructure.coralRoller.getStatorCurrent() >= detectCurrent){
      currentDebounceCounter++;
    }else{
      currentDebounceCounter = 0;
    }

    
    passedCycles++;
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    //stop the rollers!
    // Hold whatever position I'm at now...
    m_superstructure.coralRoller.stop();
    m_superstructure.coralPivot.runPosition(CoralPivotConstants.Setpoints.home);
    m_superstructure.elevator.runPosition(ElevatorConstants.Setpoints.home);
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return passedCycles >= 50 && currentDebounceCounter >= 1;
  }}
    //passedCycles >= 10; }}


