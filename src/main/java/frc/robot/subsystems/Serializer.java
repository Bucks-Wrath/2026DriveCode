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

	public TalonFX FeederKraken = new TalonFX(DeviceIds.Feeder.MotorId);
    public TalonFXConfiguration FeederFXConfig = new TalonFXConfiguration();


	public Serializer() {
        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		FeederFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        FeederFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Current Limiting */
        //FeederFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        //FeederFXConfig.CurrentLimits.SupplyCurrentLimit = 10;
        //FeederFXConfig.CurrentLimits.SupplyCurrentThreshold = 20;
        //FeederFXConfig.CurrentLimits.SupplyTimeThreshold = 0.01;
        FeederFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        FeederFXConfig.CurrentLimits.StatorCurrentLimit = 30;

        /* PID Config */
        FeederFXConfig.Slot0.kP = 0.2;
        FeederFXConfig.Slot0.kI = 0;
        FeederFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
        FeederFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.05;
        FeederFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.05;

        FeederFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.05;
        FeederFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;

        // Config Motor
        FeederKraken.getConfigurator().apply(FeederFXConfig);
        FeederKraken.getConfigurator().setPosition(0.0);
	}

	public void setSpeed(double speed) {
        this.FeederKraken.set(speed);
	}

	public double getCurrentDraw() {
		return this.FeederKraken.getSupplyCurrent().getValueAsDouble();
	}

	public void resetShooterEncoder() {
        try {
			FeederKraken.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Shooter.resetShooterEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Feeder Current", this.getCurrentDraw());
	}
}