package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.subsystems.algaePivot.AlgaePivot;
import frc.robot.subsystems.algaeRoller.AlgaeRoller;

public class ScoreProcessor extends Command{
    private final AlgaePivot algaePivot;
    private final AlgaeRoller algaeRoller;
    double pivotThreshold;
    double elevatorThreshold;
    int passedCycles;

    public ScoreProcessor(AlgaePivot algaePivot, AlgaeRoller algaeRoller){
        this.algaePivot = algaePivot;
        this.algaeRoller = algaeRoller;
        pivotThreshold = 5;
        elevatorThreshold = 5;
        passedCycles = 0;
        addRequirements(algaePivot, algaeRoller);
    }

    @Override
    public void initialize() {
        passedCycles = 0;
        algaePivot.runPosition(AlgaePivotConstants.Setpoints.home);
    }
    @Override
    public void execute() {
        if (algaePivot.getPosition() > AlgaePivotConstants.Setpoints.home-pivotThreshold && algaePivot.getPosition() < AlgaePivotConstants.Setpoints.home+pivotThreshold){
            algaeRoller.setPO(-0.5);
        }
        passedCycles++;
    }

    @Override
    public void end(boolean interrupted) {
        algaePivot.stop();
        algaeRoller.stop();
    }

    @Override
    public boolean isFinished() {
        return passedCycles >= 10; //TODO: Tune
    }
}
