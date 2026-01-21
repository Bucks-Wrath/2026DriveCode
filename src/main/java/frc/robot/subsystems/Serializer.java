package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIds;

public class Serializer extends SubsystemBase {

	public TalonFX SerializerKraken = new TalonFX(DeviceIds.Serializer.MotorId);
    public TalonFXConfiguration SerializerFXConfig = new TalonFXConfiguration();


	public Serializer() {
        /** Serializer Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		SerializerFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        SerializerFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Current Limiting */
        //SerializerFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        //SerializerFXConfig.CurrentLimits.SupplyCurrentLimit = 10;
        //SerializerFXConfig.CurrentLimits.SupplyCurrentThreshold = 20;
        //SerializerFXConfig.CurrentLimits.SupplyTimeThreshold = 0.01;
        SerializerFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        SerializerFXConfig.CurrentLimits.StatorCurrentLimit = 30;

        /* PID Config */
        SerializerFXConfig.Slot0.kP = 0.2;
        SerializerFXConfig.Slot0.kI = 0;
        SerializerFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
         SerializerFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.05;
         SerializerFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.05;

        SerializerFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.05;
        SerializerFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;

        // Config Motor
       SerializerKraken.getConfigurator().apply(SerializerFXConfig);
        SerializerKraken.getConfigurator().setPosition(0.0);
	}

	public void setSpeed(double speed) {
        this.SerializerKraken.set(speed);
	}

	public double getCurrentDraw() {
		return this.SerializerKraken.getSupplyCurrent().getValueAsDouble();
	}

	public void resetSerializerEncoder() {
        try {
			SerializerKraken.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Serializer.resetSerializerEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Serializer Current", this.getCurrentDraw());
	}
}