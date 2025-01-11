package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
    private final Intake intake;
    private final DoubleSupplier speedSupplier;
    public RunIntake(Intake intake, DoubleSupplier speedSupplier) {
        this.intake = intake;
        this.speedSupplier = speedSupplier;
        addRequirements(intake);
    }
    @Override
    public void execute() {
        intake.setMotors(speedSupplier.getAsDouble());
    }
    @Override
    public void end(boolean interrupted) {
        intake.stopMotors();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}


