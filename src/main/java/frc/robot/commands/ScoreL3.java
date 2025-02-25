package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.coralPivot.CoralPivot;
import frc.robot.subsystems.coralRoller.CoralRoller;
import frc.robot.subsystems.elevator.Elevator;

public class ScoreL3 extends Command{
    private final CoralPivot coralPivot; 
    private final Elevator elevator;
    double elevatorThreshold;
    int passedCycles;

    public ScoreL3(CoralPivot coralPivot, Elevator elevator){
        this.coralPivot = coralPivot;
        this.elevator = elevator;
        elevatorThreshold = 5;
        passedCycles = 0;
        addRequirements(coralPivot, elevator);
    }

    @Override
    public void initialize() {
        passedCycles = 0;
        elevator.runPosition(ElevatorConstants.Setpoints.scoreL3);;
    }

    @Override
    public void execute () {
        if (elevator.getPosition() > ElevatorConstants.Setpoints.scoreL3-elevatorThreshold && elevator.getPosition() < ElevatorConstants.Setpoints.scoreL3+elevatorThreshold);
        coralPivot.runPosition(CoralPivotConstants.Setpoints.scoreL3);
        
        passedCycles++;
    }

    @Override
    public void end(boolean interrupted) {
        coralPivot.stop();
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return passedCycles >= 5; //TODO: Tune
    }
}

