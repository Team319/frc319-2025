package frc.robot.subsystems.algaePivot;

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
import frc.robot.Constants.AlgaePivotConstants;

public class AlgaePivotIOKraken implements AlgaePivotIO {
        private TalonFX algaePivotMotor;
        private StatusSignal<Current> motorStatorCurrent;
        private StatusSignal<Angle> motorPosition;

        private final PositionVoltage positionVoltage = new PositionVoltage(0.0);
            Slot0Configs slot0Configs = new Slot0Configs();


        
        public AlgaePivotIOKraken(){
            setup();
        }
    
        public void setup(){
            algaePivotMotor = new TalonFX(20);
    
            TalonFXConfiguration algaePivotConfigs = new TalonFXConfiguration();
            algaePivotMotor.getConfigurator().apply(algaePivotConfigs);
    
            algaePivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            algaePivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
            algaePivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
            algaePivotConfigs.CurrentLimits.StatorCurrentLimit = 40;

            configurePID(AlgaePivotConstants.Gains.kPUp,AlgaePivotConstants.Gains.kIUp,AlgaePivotConstants.Gains.kDUp);
    
            algaePivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            algaePivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = AlgaePivotConstants.Setpoints.topLimit;
            algaePivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            algaePivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = AlgaePivotConstants.Setpoints.bottomLimit;
    
            algaePivotMotor.getConfigurator().apply(algaePivotConfigs);

            motorStatorCurrent = algaePivotMotor.getStatorCurrent();
            motorPosition = algaePivotMotor.getPosition();
            BaseStatusSignal.setUpdateFrequencyForAll(50, motorPosition, motorStatorCurrent);
            algaePivotMotor.optimizeBusUtilization();
        }
    
        @Override
        public void updateInputs(AlgaePivotIOInputs inputs) {
            inputs.kPUp = AlgaePivotConstants.Gains.kPUp;
            inputs.kIUp = AlgaePivotConstants.Gains.kIUp;
            inputs.kDUp = AlgaePivotConstants.Gains.kDUp;
            inputs.kFFUp = AlgaePivotConstants.Gains.kFFUp;
    
            inputs.kPDown = AlgaePivotConstants.Gains.kPDown;
            inputs.kIDown = AlgaePivotConstants.Gains.kIDown;
            inputs.kDDown = AlgaePivotConstants.Gains.kDDown;
            inputs.kFFDown = AlgaePivotConstants.Gains.kFFDown;

            BaseStatusSignal.refreshAll(motorStatorCurrent, motorPosition);
            // Updates all of the inputs/data points being monitored about the motor
            inputs.algaePivotMotorStatorCurrent = motorStatorCurrent.getValueAsDouble();
            inputs.algaePivotMotorPosition = motorPosition.getValueAsDouble();
    }

    @Override
    public void configurePID(double kP, double kI, double kD){

      System.out.println("[Elevator] Applying PID Values: kP=" + kP + " kI=" + kI + " kD=" + kD);
      // Feedback gains
      slot0Configs.kP = kP;
      slot0Configs.kI = kI;
      slot0Configs.kD = kD;

      // Update the motors with the new Gains
      algaePivotMotor.getConfigurator().apply(slot0Configs, 0.050);
    }

    public void stop(){
        algaePivotMotor.stopMotor();
    }

    @Override
    public void setPO(double PO) {
      algaePivotMotor.set(PO);
    }

    @Override
    public void setPosition(double position) {
      algaePivotMotor.setPosition(position);
    }

    @Override
    public double getPosition() {
      return motorPosition.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
      return algaePivotMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void runPosition(double positionRad) {

      algaePivotMotor.setControl(
        positionVoltage
              .withPosition(positionRad));
    }

    @Override
    public void runPosition(double positionRad, double feedforward) {

      algaePivotMotor.setControl(
        positionVoltage
              .withPosition(positionRad)
              .withFeedForward(feedforward));
    }
}