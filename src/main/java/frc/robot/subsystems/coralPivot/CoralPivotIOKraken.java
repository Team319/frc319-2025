package frc.robot.subsystems.coralPivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CoralPivotConstants;

public class CoralPivotIOKraken implements CoralPivotIO {
        private TalonFX coralPivotMotor;
        private StatusSignal<Current> motorStatorCurrent;
        private StatusSignal<Angle> motorPosition;


        Slot0Configs slot0Configs = new Slot0Configs();
        private final PositionVoltage positionVoltage = new PositionVoltage(0.0);

        public CoralPivotIOKraken(){
            setup();
        }
    
        public void setup(){
            coralPivotMotor = new TalonFX(15);
    
            TalonFXConfiguration coralPivotConfigs = new TalonFXConfiguration();
            coralPivotMotor.getConfigurator().apply(coralPivotConfigs);
    
            coralPivotConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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

    @Override
    public void configurePID(double kP, double kI, double kD){

      System.out.println("Applying PID Values: kP=" + kP + " kI=" + kI + " kD=" + kD);
      
      // Feedback gains
      slot0Configs.kP = kP;
      slot0Configs.kI = kI;
      slot0Configs.kD = kD;

      // Update the motors with the new Gains
      coralPivotMotor.getConfigurator().apply(slot0Configs, 0.050);
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

        @Override
    public void runPosition(double positionRad) {

      coralPivotMotor.setControl(
        positionVoltage
              .withPosition((positionRad)));
    }

    @Override
    public void runPosition(double positionRad, double feedforward) {

      coralPivotMotor.setControl(
        positionVoltage
              .withPosition((positionRad))
              .withFeedForward(feedforward));
    }
}
