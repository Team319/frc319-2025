package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;



public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS");
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA");

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD");
    
    // private static final LoggedTunableNumber maxVelocityMetersPerSec =
    //   new LoggedTunableNumber("Elevator/MaxVelocityMetersPerSec", 2.0);
    // private static final LoggedTunableNumber maxAccelerationMetersPerSec2 =
    //   new LoggedTunableNumber("Elevator/MaxAccelerationMetersPerSec2", 10);

    // private TrapezoidProfile profile = new TrapezoidProfile(
    //                                         new TrapezoidProfile.Constraints(
    //                                         maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get()));
    
    // @ AutoLogOutput private State setpoint = new State();
    // private Supplier<State> goal = State::new;
    // @ AutoLogOutput private volatile boolean atGoal = false;

    public Elevator(ElevatorIO io) {
        this.io = io;
        
        switch (Constants.getRobot()) {
            case COMPBOT:
            case DEVBOT:
              kP.initDefault(Constants.ElevatorConstants.Gains.kPUp); // was 1200
              kI.initDefault(Constants.ElevatorConstants.Gains.kIUp);
              kD.initDefault(Constants.ElevatorConstants.Gains.kDUp);
              
              kS.initDefault(Constants.ElevatorConstants.Gains.kS); // was 0.5
              kG.initDefault(Constants.ElevatorConstants.Gains.kG); // was 5
              kA.initDefault(Constants.ElevatorConstants.Gains.kA);
                break;
            case SIMBOT:
              // Defaults taken from 6328's Public Codebase
              kP.initDefault(5000);
              kI.initDefault(0);
              kD.initDefault(2000);
              
              kS.initDefault(5);
              kG.initDefault(50);
              kA.initDefault(0);
                break;
            default:
                break;
        } 
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("/RealOutputs/Elevator", inputs);


        // If Tunable Values are enabled, update them
        if ( ( kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode()) ) ) {
          io.configurePID(kP.get(), kI.get(), kD.get() );
        }

        // // Periodic Elevator Control
        // var goalState =
        //   new State(
        //       MathUtil.clamp(goal.get().position, Constants.ElevatorConstants.SoftLimits.reverseSoftLimit, Constants.ElevatorConstants.SoftLimits.forwardSoftLimit),
        //       goal.get().velocity);
        
        // Logger.recordOutput("Elevator/Goal/position", goalState.position);
        // Logger.recordOutput("Elevator/Goal/velocity", goalState.velocity);

        // setpoint = profile.calculate(0.02, setpoint, goalState);

        // Logger.recordOutput("Elevator/Setpoint/position", setpoint.position);
        // Logger.recordOutput("Elevator/Setpoint/velocity", setpoint.velocity);

        // goToPosition(setpoint.position);
        
        // // Check at goal
        // atGoal =
        //   EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
        //       && EqualsUtil.epsilonEquals(setpoint.velocity, goalState.velocity);

    }

    public void stop() {
        io.stop();
      }
    
      public void setPO(double PO) {
        io.setPO(PO);
      }
    
      public void setPosition(double position) {
        io.setPosition(position);
      }
    
      public void setVoltage(double voltage) {
        io.setVoltage(voltage);
      }
    
      public void configurePID(double kP, double kI, double kD) {
        io.configurePID(kP, kI, kD);
      }
    
      public double getPosition() {
        return io.getPosition();
      }
    
      public double getVelocity() {
        return io.getVelocity();
      }

      public void runPosition(double positionRad) {
        io.runPosition(positionRad);
      }

      // =============================================================
      // Trapezoid Profile helpers
      // =============================================================

      // private void goToPosition(double positionRad) {
        
      //   double feedforward = kS.get() * Math.signum(setpoint.velocity) + kG.get() ;
      //   Logger.recordOutput("/RealOutputs/Elevator/Feedforward", feedforward);
      //   io.runPosition(positionRad, feedforward);

      // }

      // public void setGoal(DoubleSupplier goal) {
      //   setGoal(() -> new State(goal.getAsDouble(), 0.0));
      // }

      // public void setGoal(Supplier<State> goal) {
      //   atGoal = false;
      //   this.goal = goal;
      // }
    }
    

