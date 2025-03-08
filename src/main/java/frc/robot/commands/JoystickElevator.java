package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class JoystickElevator extends Command{
    private static final double DEADBAND = 0.5;
    private final Elevator elevator;
    DoubleSupplier leftSupplier;
    double elevatorPO = 0.0;


    public JoystickElevator(Elevator elevator, DoubleSupplier leftSupplier){
       this.elevator = elevator;
       this.leftSupplier = leftSupplier;
       addRequirements(elevator);
    }

     @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    elevatorPO = MathUtil.applyDeadband(leftSupplier.getAsDouble(), DEADBAND);

    this.elevator.setPO(elevatorPO);

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
