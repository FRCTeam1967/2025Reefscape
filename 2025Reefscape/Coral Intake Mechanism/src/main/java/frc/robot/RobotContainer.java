package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class RobotContainer {
    // create instances of the subsystems
    private final Intake m_intake = new Intake();
    private final Pivot m_pivot = new Pivot();
    private final XboxController m_controller = new XboxController(0);

    public RobotContainer() {
        configureButtonBindings();
    }

    public void maintainPivotPosition() {
        m_pivot.setRelToAbs();
    }

    private void configureButtonBindings() {
        // bind left joystick to run intake
        new Trigger(() -> Math.abs(m_controller.getLeftY()) > 0.1)
            .whileTrue(new RunIntake(m_intake, () -> m_controller.getLeftY()));
    }
}