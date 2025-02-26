package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.coralPivot.CoralPivot;
import frc.robot.subsystems.coralRoller.CoralRoller;
import frc.robot.subsystems.elevator.Elevator;

public class CollectCoralObstructed extends Command{
    private final CoralPivot coralPivot;
    private final CoralRoller coralRoller;
    private final Elevator elevator;
    double pivotThreshold;
    double elevatorThreshold;
    int passedCycles;

    public CollectCoralObstructed(CoralPivot coralPivot, CoralRoller coralRoller, Elevator elevator){
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
        elevator.runPosition(ElevatorConstants.Setpoints.collect_obstructed);
    }
    @Override
    public void execute() {
        if (elevator.getPosition() > ElevatorConstants.Setpoints.collect_obstructed-pivotThreshold && elevator.getPosition() < ElevatorConstants.Setpoints.collect_obstructed+pivotThreshold){
            coralPivot.runPosition(CoralPivotConstants.Setpoints.collect_obstructed);
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
