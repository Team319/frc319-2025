package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.algaePivot.AlgaePivot;
import frc.robot.subsystems.coralPivot.CoralPivot;
import frc.robot.subsystems.elevator.Elevator;


public class ClimbingPrep extends Command {
    private final CoralPivot coralPivot;
    private final Elevator elevator;
    private final AlgaePivot algaePivot;
    double elevatorThreshold;
    double pivotThreshold;
    int passedCycles;


    public ClimbingPrep(CoralPivot coralPivot, Elevator elevator, AlgaePivot algaePivot) {
        this.coralPivot = coralPivot;
        this.elevator = elevator;
        this.algaePivot = algaePivot;
        elevatorThreshold = 5;
        pivotThreshold = 5;
        passedCycles = 0;
        addRequirements(coralPivot, elevator);
    }

    @Override
    public void initialize() {
        passedCycles = 0;
        elevator.runPosition(ElevatorConstants.Setpoints.readyToClimb);
    }

    @Override
    public void execute() {
        if (algaePivot.getPosition() > AlgaePivotConstants.Setpoints.home-pivotThreshold && algaePivot.getPosition() < AlgaePivotConstants.Setpoints.home+pivotThreshold){
        if(elevator.getPosition() > ElevatorConstants.Setpoints.readyToClimb-elevatorThreshold && elevator.getPosition() < ElevatorConstants.Setpoints.readyToClimb+elevatorThreshold);{
            coralPivot.runPosition(CoralPivotConstants.Setpoints.readyToClimb);
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

