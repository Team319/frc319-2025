// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.CoralRollerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.EqualsUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectCoral extends Command {

  Superstructure m_superstructure;
  private final Timer timer = new Timer();
  private static final double DEBOUNCE_TIME = 0.5; // Adjust debounce time as needed



  double detectCurrent = 8; // Tune this current limit number with Advantagescope looking at RealOutputs/CoralRoller/MotorStatorCurrent
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
    m_superstructure.coralRoller.setPO(CoralRollerConstants.Speeds.collect);
    m_superstructure.coralPivot.runPosition(CoralPivotConstants.Setpoints.collect);
    m_superstructure.elevator.runPosition(ElevatorConstants.Setpoints.collect_flush);
    timer.reset();
    timer.start();


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}  // do nothing new... just let the coral roller run

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    //stop the rollers!
    m_superstructure.coralRoller.setPO(CoralRollerConstants.Speeds.stop);
    // Hold whatever position I'm at now...
    m_superstructure.coralPivot.runPosition(m_superstructure.coralPivot.getPosition());
    m_superstructure.elevator.runPosition(m_superstructure.elevator.getPosition());
    timer.stop();

  

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_superstructure.coralRoller.getStatorCurrent() < 10) {
      if (timer.hasElapsed(DEBOUNCE_TIME)) {
          return true;
      }
  } else {
      timer.reset();
  }
  return false;
}
}

