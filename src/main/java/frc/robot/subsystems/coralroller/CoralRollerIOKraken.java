package frc.robot.subsystems.coralRoller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.CoralRollerConstants;

public class CoralRollerIOKraken implements CoralRollerIO {
        private TalonFX coralRollerMotor;
        private StatusSignal<Current> motorStatorCurrent;
        private StatusSignal<Angle> motorPosition;
        
        public CoralRollerIOKraken(){
            setup();
        }
    
        public void setup(){
            coralRollerMotor = new TalonFX(15);
    
            TalonFXConfiguration coralRollerConfigs = new TalonFXConfiguration();
            coralRollerMotor.getConfigurator().apply(coralRollerConfigs);
    
            coralRollerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            coralRollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
            coralRollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
            coralRollerConfigs.CurrentLimits.StatorCurrentLimit = 40;

            configurePID(CoralRollerConstants.PID.kPUp,CoralRollerConstants.PID.kIUp,CoralRollerConstants.PID.kDUp,CoralRollerConstants.PID.kFFUp);
    
            coralRollerConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            coralRollerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = CoralRollerConstants.Setpoints.topLimit;
            coralRollerConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            coralRollerConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = CoralRollerConstants.Setpoints.bottomLimit;
    
            coralRollerMotor.getConfigurator().apply(coralRollerConfigs);

            motorStatorCurrent = coralRollerMotor.getStatorCurrent();
            motorPosition = coralRollerMotor.getPosition();
            BaseStatusSignal.setUpdateFrequencyForAll(50, motorPosition, motorStatorCurrent);
            coralRollerMotor.optimizeBusUtilization();
        }
    
        @Override
        public void updateInputs(CoralRollerIOInputs inputs) {
            inputs.kPUp = CoralRollerConstants.PID.kPUp;
            inputs.kIUp = CoralRollerConstants.PID.kIUp;
            inputs.kDUp = CoralRollerConstants.PID.kDUp;
            inputs.kFFUp = CoralRollerConstants.PID.kFFUp;
    
            inputs.kPDown = CoralRollerConstants.PID.kPDown;
            inputs.kIDown = CoralRollerConstants.PID.kIDown;
            inputs.kDDown = CoralRollerConstants.PID.kDDown;
            inputs.kFFDown = CoralRollerConstants.PID.kFFDown;

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
}
