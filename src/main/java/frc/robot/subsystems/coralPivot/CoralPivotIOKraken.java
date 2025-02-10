package frc.robot.subsystems.coralPivot;

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

public class CoralPivotIOKraken implements CoralPivotIO {
        private TalonFX coralPivotMotor;
        private StatusSignal<Current> motorStatorCurrent;
        private StatusSignal<Angle> motorPosition;
        
        public CoralPivotIOKraken(){
            setup();
        }
    
        public void setup(){
            coralPivotMotor = new TalonFX(15);
    
            TalonFXConfiguration coralPivotConfigs = new TalonFXConfiguration();
            coralPivotMotor.getConfigurator().apply(coralPivotConfigs);
    
            coralPivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            coralPivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
            coralPivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
            coralPivotConfigs.CurrentLimits.StatorCurrentLimit = 40;

            configurePID(CoralPivotConstants.PID.kPUp,CoralPivotConstants.PID.kIUp,CoralPivotConstants.PID.kDUp,CoralPivotConstants.PID.kFFUp);
    
            coralPivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            coralPivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = CoralPivotConstants.Setpoints.topLimit;
            coralPivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            coralPivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = CoralPivotConstants.Setpoints.bottomLimit;
    
            coralPivotMotor.getConfigurator().apply(coralPivotConfigs);

            motorStatorCurrent = coralPivotMotor.getStatorCurrent();
            motorPosition = coralPivotMotor.getPosition();
            BaseStatusSignal.setUpdateFrequencyForAll(50, motorPosition, motorStatorCurrent);
            coralPivotMotor.optimizeBusUtilization();
        }
    
        @Override
        public void updateInputs(CoralPivotIOInputs inputs) {
            inputs.kPUp = CoralPivotConstants.PID.kPUp;
            inputs.kIUp = CoralPivotConstants.PID.kIUp;
            inputs.kDUp = CoralPivotConstants.PID.kDUp;
            inputs.kFFUp = CoralPivotConstants.PID.kFFUp;
    
            inputs.kPDown = CoralPivotConstants.PID.kPDown;
            inputs.kIDown = CoralPivotConstants.PID.kIDown;
            inputs.kDDown = CoralPivotConstants.PID.kDDown;
            inputs.kFFDown = CoralPivotConstants.PID.kFFDown;

            BaseStatusSignal.refreshAll(motorStatorCurrent, motorPosition);
            // Updates all of the inputs/data points being monitored about the motor
            inputs.coralPivotMotorStatorCurrent = motorStatorCurrent.getValueAsDouble();
            inputs.coralPivotMotorPosition = motorPosition.getValueAsDouble();
    }

    public void stop(){
        coralPivotMotor.stopMotor();
    }

    @Override
    public void setPO(double PO) {
      coralPivotMotor.set(PO);
    }

    @Override
    public void setPosition(double position) {
      coralPivotMotor.setPosition(position);
    }

    @Override
    public double getPosition() {
      return motorPosition.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
      return coralPivotMotor.getVelocity().getValueAsDouble();
    }
}
