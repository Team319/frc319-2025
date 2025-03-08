package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Climber/kP");
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Climber/kI");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Climber/kD");


    public Climber(ClimberIO io) {
        this.io = io;
        
        switch (Constants.getRobot()) {
            case COMPBOT:
            case DEVBOT:
              kP.initDefault(Constants.ClimberConstants.Gains.kPUp);
              kI.initDefault(Constants.ClimberConstants.Gains.kIUp);
              kD.initDefault(Constants.ClimberConstants.Gains.kDUp);

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
        Logger.processInputs("/RealOutputs/Climber", inputs);

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
    
      public void configurePID(double kP, double kI, double kD) {
        io.configurePID(kP, kI, kD);
      }
    
      public double getPosition() {
        return io.getPosition();
      }
    
      public double getVelocity() {
        return io.getVelocity();
      }

      public void runPosition(double position) {
        io.runPosition(position);
      }
    }
    

