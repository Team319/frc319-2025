package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.coralPivot.CoralPivot;
import frc.robot.subsystems.coralRoller.CoralRoller;
import frc.robot.subsystems.elevator.Elevator;

public class ScoreL4 extends Command{
    private final CoralPivot coralPivot; 
    private final Elevator elevator;
    private final CoralRoller coralRoller;
    double pivotThreshold;
    double elevatorThreshold;
    int passedCycles;

    public ScoreL4(CoralPivot coralPivot, Elevator elevator, CoralRoller coralRoller){
        this.coralPivot = coralPivot;
        this.elevator = elevator;
        this.coralRoller = coralRoller;
        pivotThreshold = 5;
        elevatorThreshold = 5;
        passedCycles = 0;
        addRequirements(coralPivot, elevator, coralRoller);
    }

    @Override
    public void initialize() {
        passedCycles = 0;
        coralPivot.runPosition(CoralPivotConstants.Setpoints.scoreL4);
    }

    @Override
    public void execute () {
        if (coralPivot.getPosition() > CoralPivotConstants.Setpoints.scoreL4-pivotThreshold && coralPivot.getPosition() < CoralPivotConstants.Setpoints.scoreL4+pivotThreshold){
            elevator.runPosition(ElevatorConstants.Setpoints.scoreL4);
        }
        if (elevator.getPosition() > ElevatorConstants.Setpoints.collect_flush-pivotThreshold && elevator.getPosition() < ElevatorConstants.Setpoints.collect_flush+pivotThreshold){
            coralRoller.setPO(-0.2);
        }
        passedCycles++;
    }

    @Override
    public void end(boolean interrupted) {
        coralPivot.stop();
        elevator.stop();
        coralRoller.stop();
    }

    @Override
    public boolean isFinished() {
        return passedCycles >= 10; //TODO: Tune
    }
}

