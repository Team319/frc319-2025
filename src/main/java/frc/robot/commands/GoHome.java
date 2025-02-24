package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.algaePivot.AlgaePivot;
import frc.robot.subsystems.coralPivot.CoralPivot;
import frc.robot.subsystems.elevator.Elevator;


public class GoHome extends Command {
    private final CoralPivot coralPivot;
    private final Elevator elevator;
    private final AlgaePivot algaePivot;
    double pivotThreshold;
    int passedCycles;


    public GoHome(CoralPivot coralPivot, Elevator elevator, AlgaePivot algaePivot) {
        this.coralPivot = coralPivot;
        this.elevator = elevator;
        this.algaePivot = algaePivot;
        pivotThreshold = 5;
        passedCycles = 0;
        addRequirements(coralPivot, elevator);
    }

    @Override
    public void initialize() {
        passedCycles = 0;
        coralPivot.runPosition(CoralPivotConstants.Setpoints.home);
        algaePivot.runPosition(AlgaePivotConstants.Setpoints.home);
    }

    @Override
    public void execute() {
        if (algaePivot.getPosition() > AlgaePivotConstants.Setpoints.home-pivotThreshold && algaePivot.getPosition() < AlgaePivotConstants.Setpoints.home+pivotThreshold){
        if (coralPivot.getPosition() > CoralPivotConstants.Setpoints.home-pivotThreshold && coralPivot.getPosition() < CoralPivotConstants.Setpoints.home+pivotThreshold){
            elevator.runPosition(ElevatorConstants.Setpoints.home);
        }
    }
        passedCycles++;
    }

    @Override
    public void end(boolean interrupted) {
        coralPivot.stop();
        algaePivot.stop();
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return passedCycles >=5; //TODO: Tune
    }
}

