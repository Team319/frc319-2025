package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class JoystickClimb extends Command{
    private static final double DEADBAND = 0.5;
    private final Climber climber;
    DoubleSupplier rightSupplier;
    double climberPO = 0.0;


    public JoystickClimb(Climber climber, DoubleSupplier rightSupplier){
       this.climber = climber;
       this.rightSupplier = rightSupplier;
       addRequirements(climber);
    }

     @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    climberPO = MathUtil.applyDeadband(rightSupplier.getAsDouble(), DEADBAND);

    this.climber.setPO(climberPO);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
