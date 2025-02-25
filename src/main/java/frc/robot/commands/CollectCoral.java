package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.coralPivot.CoralPivot;
import frc.robot.subsystems.coralRoller.CoralRoller;
import frc.robot.subsystems.elevator.Elevator;

public class CollectCoral extends Command{
    private final CoralPivot coralPivot;
    private final CoralRoller coralRoller;
    private final Elevator elevator;
    double pivotThreshold;
    double elevatorThreshold;
    int passedCycles;

    public CollectCoral(CoralPivot coralPivot, CoralRoller coralRoller, Elevator elevator){
        this.coralPivot = coralPivot;
        this.coralRoller = coralRoller;
        this.elevator = elevator;
        pivotThreshold = 5;
        elevatorThreshold = 5;
        passedCycles = 0;
        addRequirements(coralPivot, elevator, coralRoller);
    }

    @Override
    public void initialize() {
        passedCycles = 0;
        coralPivot.runPosition(CoralPivotConstants.Setpoints.collect);
    }
    @Override
    public void execute() {
        if (coralPivot.getPosition() > CoralPivotConstants.Setpoints.collect-pivotThreshold && coralPivot.getPosition() < CoralPivotConstants.Setpoints.collect+pivotThreshold){
            elevator.runPosition(ElevatorConstants.Setpoints.collect_flush);
        }
        if (elevator.getPosition() > ElevatorConstants.Setpoints.collect_flush-pivotThreshold && elevator.getPosition() < ElevatorConstants.Setpoints.collect_flush+pivotThreshold){
            coralRoller.setPO(0.5);
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
