package frc.robot.subsystems.coralPivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;



public class CoralPivot extends SubsystemBase {
    private final CoralPivotIO io;
    private final CoralPivotIOInputsAutoLogged inputs = new CoralPivotIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("CoralPivot/kP");
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("CoralPivot/kI");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("CoralPivot/kD");

    public CoralPivot(CoralPivotIO io) {
        this.io = io;
        
        switch (Constants.getRobot()) {
            case COMPBOT:
            case DEVBOT:
                kP.initDefault(Constants.CoralPivotConstants.Gains.kPUp);
                kI.initDefault(Constants.CoralPivotConstants.Gains.kIUp);
                kD.initDefault(Constants.CoralPivotConstants.Gains.kDUp);
                break;

            case SIMBOT:
                break;

            default:
                break;
        } 
    }

    @Override
    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("/RealOutputs/CoralPivot", inputs);


        // If Tunable Values are enabled, update them
        if ( ( kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode()) ) ) {
          io.configurePID(kP.get(), kI.get(), kD.get() );
        }
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
    
      public void configurePID(double kP, double kI, double kD, double kFF) {
        io.configurePID(kP, kI, kD);
      }
    
      public double getPosition() {
        return io.getPosition();
      }
    
      public double getVelocity() {
        return io.getVelocity();
      }

      public void runPosition(double position){
        io.runPosition(position);
      }
    }
    

