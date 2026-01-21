package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIds;

public class Upkicker extends SubsystemBase {

	public TalonFX UpkickerKraken = new TalonFX(DeviceIds.Upkicker.MotorId);
    public TalonFXConfiguration UpkickerFXConfig = new TalonFXConfiguration();


	public Upkicker() {
        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		UpkickerFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        UpkickerFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Current Limiting */
        //UpkickerFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        //UpkickerFXConfig.CurrentLimits.SupplyCurrentLimit = 10;
        //UpkickerFXConfig.CurrentLimits.SupplyCurrentThreshold = 20;
        //UpkickerFXConfig.CurrentLimits.SupplyTimeThreshold = 0.01;
        UpkickerFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        UpkickerFXConfig.CurrentLimits.StatorCurrentLimit = 30;

        /* PID Config */
        UpkickerFXConfig.Slot0.kP = 0.2;
        UpkickerFXConfig.Slot0.kI = 0;
        UpkickerFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
        UpkickerFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.05;
        UpkickerFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.05;

        UpkickerFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.05;
        UpkickerFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;

        // Config Motor
        UpkickerKraken.getConfigurator().apply(UpkickerFXConfig);
        UpkickerKraken.getConfigurator().setPosition(0.0);
	}

	public void setSpeed(double speed) {
        this.UpkickerKraken.set(speed);
	}

	public double getCurrentDraw() {
		return this.UpkickerKraken.getSupplyCurrent().getValueAsDouble();
	}

	public void resetShooterEncoder() {
        try {
			UpkickerKraken.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Shooter.resetShooterEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Upkicker Current", this.getCurrentDraw());
	}
}