package first.robot.opmode;

import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

import first.robot.Robot;
import first.robot.commands.Autos;
import first.robot.subsystems.Drivetrain;

@Autonomous(name = "Do Nothing", group = "Group 1")
public class RobotAutoBox extends PeriodicOpMode {
    public RobotAutoBox(Robot robot) {

        RobotModeTriggers.autonomous().onTrue(Autos.boxPattern(robot.drivetrain) ); 

    }
    @Override
    public void periodic() {
        
    }
}
