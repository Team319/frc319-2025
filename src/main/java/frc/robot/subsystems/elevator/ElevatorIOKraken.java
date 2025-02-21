package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOKraken implements ElevatorIO {
        private TalonFX elevatorLead;
        private TalonFX elevatorFollow;

        TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration();

        private StatusSignal<Current> motorStatorCurrent;
        private StatusSignal<Angle> motorPosition;


        public ElevatorIOKraken(){

          elevatorLead = new TalonFX(13);
          elevatorFollow = new TalonFX(14);

          //int eleID = elevatorLead.getDeviceID();
          //System.out.println(eleID);

          elevatorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
          elevatorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
          configurePID(ElevatorConstants.PID.kPUp,ElevatorConstants.PID.kIUp,ElevatorConstants.PID.kDUp,ElevatorConstants.PID.kFFUp);
  
          elevatorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
          elevatorConfigs.CurrentLimits.StatorCurrentLimit = 40;
  
          elevatorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
          elevatorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.Setpoints.topLimit;
          elevatorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
          elevatorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.Setpoints.bottomLimit;

          elevatorLead.getConfigurator().apply(elevatorConfigs);
          //elevatorFollow.getConfigurator().apply(elevatorConfigs);

          elevatorLead.setControl(new Follower(elevatorFollow.getDeviceID(), true)); //TODO: Make sure this is correct
          //elevatorFollow.setControl(new Follower(elevatorLead.getDeviceID(), true));

          motorStatorCurrent = elevatorLead.getStatorCurrent();
          motorPosition = elevatorLead.getPosition();
          //BaseStatusSignal.setUpdateFrequencyForAll(50, motorPosition, motorStatorCurrent);
          //elevatorLead.optimizeBusUtilization();
        }


        @Override
        public void updateInputs(ElevatorIOInputs inputs) {
            inputs.kPUp = ElevatorConstants.PID.kPUp;
            inputs.kIUp = ElevatorConstants.PID.kIUp;
            inputs.kDUp = ElevatorConstants.PID.kDUp;
            inputs.kFFUp = ElevatorConstants.PID.kFFUp;
    
            inputs.kPDown = ElevatorConstants.PID.kPDown;
            inputs.kIDown = ElevatorConstants.PID.kIDown;
            inputs.kDDown = ElevatorConstants.PID.kDDown;
            inputs.kFFDown = ElevatorConstants.PID.kFFDown;

            BaseStatusSignal.refreshAll(motorStatorCurrent, motorPosition);
            // Updates all of the inputs/data points being monitored about the motor
            inputs.elevatorMotorStatorCurrent = motorStatorCurrent.getValueAsDouble();
            inputs.elevatorMotorPosition = motorPosition.getValueAsDouble();
    }

    @Override
    public void stop(){
        elevatorLead.stopMotor();
    }

    @Override
    public void setPO(double PO) {
      DutyCycleOut m_request = new DutyCycleOut(PO);

      //elevatorLead.setControl(new Follower(elevatorFollow.getDeviceID(), true)); //TODO: Make sure this is correct

      //elevatorLead.setControl(m_request);
      elevatorFollow.setControl(m_request);
    }

    @Override
    public void setPosition(double position) {
      elevatorLead.setPosition(position);
    }

    @Override
    public double getPosition() {
      return motorPosition.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
      return elevatorLead.getVelocity().getValueAsDouble();
    }
}