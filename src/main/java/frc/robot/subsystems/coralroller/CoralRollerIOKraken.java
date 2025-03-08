package frc.robot.subsystems.coralRoller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;


public class CoralRollerIOKraken implements CoralRollerIO {
        private TalonFX coralRollerMotor;
        private StatusSignal<Current> motorStatorCurrent;
        private StatusSignal<Angle> motorPosition;
        
        public CoralRollerIOKraken(){
            setup();
        }
    
        public void setup(){
            coralRollerMotor = new TalonFX(36);
    
            TalonFXConfiguration coralRollerConfigs = new TalonFXConfiguration();
            coralRollerMotor.getConfigurator().apply(coralRollerConfigs);
    
            coralRollerConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            coralRollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
            coralRollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
            coralRollerConfigs.CurrentLimits.StatorCurrentLimit = 40;
    
            coralRollerMotor.getConfigurator().apply(coralRollerConfigs);

            motorStatorCurrent = coralRollerMotor.getStatorCurrent();
            motorPosition = coralRollerMotor.getPosition();
            BaseStatusSignal.setUpdateFrequencyForAll(50, motorPosition, motorStatorCurrent);
            coralRollerMotor.optimizeBusUtilization();
        }
    
        @Override
        public void updateInputs(CoralRollerIOInputs inputs) {

            BaseStatusSignal.refreshAll(motorStatorCurrent, motorPosition);
            // Updates all of the inputs/data points being monitored about the motor
            inputs.coralRollerMotorStatorCurrent = motorStatorCurrent.getValueAsDouble();
            inputs.coralRollerMotorPosition = motorPosition.getValueAsDouble();
    }

    public void stop(){
        coralRollerMotor.stopMotor();
    }

    @Override
    public void setPO(double PO) {
      coralRollerMotor.set(PO);
    }

    @Override
    public void setPosition(double position) {
      coralRollerMotor.setPosition(position);
    }

    @Override
    public double getPosition() {
      return motorPosition.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
      return coralRollerMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getStatorCurrent() {
      return motorStatorCurrent.refresh().getValueAsDouble();
    }
}
