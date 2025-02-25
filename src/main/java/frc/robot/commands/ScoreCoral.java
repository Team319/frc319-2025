package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.subsystems.algaePivot.AlgaePivot;
import frc.robot.subsystems.algaeRoller.AlgaeRoller;
import frc.robot.subsystems.coralPivot.CoralPivot;
import frc.robot.subsystems.coralRoller.CoralRoller;

public class ScoreCoral extends Command{
    private final CoralRoller coralRoller;
    double pivotThreshold;
    double elevatorThreshold;
    int passedCycles;

    public ScoreCoral(CoralRoller coralRoller){
        this.coralRoller = coralRoller;
        pivotThreshold = 5;
        passedCycles = 0;
        addRequirements(coralRoller);
    }

    @Override
    public void initialize() {
        passedCycles = 0;
    }
    @Override
    public void execute() {
        coralRoller.setPO(-0.5);
            passedCycles++;
    }

    @Override
    public void end(boolean interrupted) {
        coralRoller.stop();
    }

    @Override
    public boolean isFinished() {
        return passedCycles >= 10; //TODO: Tune
    }
}
