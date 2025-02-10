package frc.robot.subsystems.coralPivot;

public interface CoralPivotIO {

    public static class CoralPivotIOInputs {
        public double kPUp = 0.0;  // Power applied to motor
        public double kIUp = 0.0;  // margin of error in motor
        public double kDUp = 0.0;  // Makes the graph line smooth from point A to point B
        public double kFFUp = 0.0; // Feedforward value

        public double kPDown = 0.0;  // Power applied to motor
        public double kIDown = 0.0;  // margin of error in motor
        public double kDDown = 0.0;  // Makes the graph line smooth from point A to point B
        public double kFFDown = 0.0; // Feedforward value

        public double targetPosition = 0.0; // Target position of the coralPivot
        public double appliedVoltage = 0.0; // Voltage applied to the motor
        public double outputCurrentAmps = 0.0; // Current applied to the motor
        public double position = 0.0; // Position of the coralPivot
        public double velocity = 0.0; // Velocity of the coralPivot
        public double coralPivotMotorStatorCurrent;
        public double coralPivotMotorPosition;
    }

    public default void updateInputs(CoralPivotIOInputs inputs) {}

    public default void stop() {}

    public default void configurePID(double kP, double kI, double kD, double kFF) {}

    public default void setPosition(double targetPosition) {}

    public default double getPosition() {return 0.0;}

    public default double getVelocity() {return 0.0;}

    public default void setVoltage(double voltage) {}

    public default void setPO(double PO) {}

    
}

    
