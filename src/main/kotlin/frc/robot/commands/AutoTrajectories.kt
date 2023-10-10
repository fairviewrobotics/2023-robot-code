package frc.robot.commands

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.constants.DrivetrainConstants
import frc.robot.constants.CommandValues
import frc.robot.constants.TrajectoryConstants
import frc.robot.subsystems.PickAndPlaceSubsystem
import frc.robot.subsystems.SwerveSubsystem
import java.nio.file.Path


class AutoTrajectories(val pnp: PickAndPlaceSubsystem, val swerveSubsystem: SwerveSubsystem) {
    fun base(pathName: String, firstConstraint: PathConstraints, vararg pathConstraints: PathConstraints): Command {
        var thetaController = PIDController(
            TrajectoryConstants.kPThetaController, 0.0, TrajectoryConstants.kDThetaController
        )
        thetaController.enableContinuousInput(-Math.PI, Math.PI)

        val pathGroup: List<PathPlannerTrajectory> = PathPlanner.loadPathGroup(
            pathName, firstConstraint, *pathConstraints
        )
        //println("Auto Running")
        //It might be this for the line above: val pathGroup: ArrayList<PathPlannerTrajectory> = arrayListOf()
        // PathPlanner.loadPathGroup("Red Top 1 Get Balance", PathConstraints(1.0, 0.5)).toCollection(pathGroup)

        //The way these are called may need to change
        val eventMap: HashMap<String, Command> = hashMapOf(
            "MidPlace" to AutoPlaceCubeMid(pnp).withTimeout(3.0),
            "Base" to AutoBase(pnp).withTimeout(3.0),
            "PickUpCube" to AutoPickCube(pnp),
            "HighPlace" to AutoPlaceCubeHigh(pnp).withTimeout(4.0),
            "MidConeGetThere" to AutoPlaceConeMidGetThere(pnp).withTimeout(2.0),
            "MidConePlace" to AutoPlaceConeMidPlace(pnp).withTimeout(2.0),
            "HighCone" to AutoPlaceConeHigh(pnp).withTimeout(4.0),
            "Base2" to AutoBase2(pnp),
            "Balance" to Balancer(swerveSubsystem),
            "PickUpConeUp" to AutoPickCone(pnp)
        )

        val autoBuilder = SwerveAutoBuilder(
            swerveSubsystem::pose,
            swerveSubsystem::resetOdometry,
            DrivetrainConstants.driveKinematics,
            PIDConstants(TrajectoryConstants.kPXController, 0.0, 0.0),
            PIDConstants(TrajectoryConstants.kPThetaController, 0.0, TrajectoryConstants.kDThetaController),
            swerveSubsystem::setModuleStates,
            eventMap,
            false,
            swerveSubsystem
        )
        var fullAuto: Command = autoBuilder.fullAuto(pathGroup)
        return fullAuto
    }
    fun TestPath(): Command {
        return base("Test Path",
            PathConstraints(0.50,0.50)
        )
    }
    fun BlueTop1(): Command {
        return base("Blue Top 1",
            PathConstraints(3.50,2.00)
        )
    }
    fun BlueBottom1(): Command {
        return base("Blue Bottom 1",
            PathConstraints(3.50,2.00)
        )
    }
    fun RedTop1(): Command {
        return base("Red Top 1",
            PathConstraints(3.50,2.00)
        )
    }
    fun RedBottom1(): Command {
        return base("Red Bottom 1",
            PathConstraints(3.50,2.00)
        )
    }
    fun RedLeftConeRightPlaceLeave(): Command {
        return base("Red Left Cone Right Place Leave",
            PathConstraints(3.50,2.00)
        )
    }
    fun RedLeftConeLeftPlaceLeave(): Command {
        return base("Red Left Cone Left Place Leave",
            PathConstraints(3.50,2.00)
        )
    }
    fun RedRightConeLeftPlaceLeave(): Command {
        return base("Red Right Cone Left Place Leave",
            PathConstraints(3.50,2.00)
        )
    }
    fun BlueRightConeRightPlaceLeave(): Command {
        return base("Blue Right Cone Right Place Leave",
            PathConstraints(3.50,2.00)
        )
    }
    fun BlueLeftConeRightPlaceLeave(): Command {
        return base("Blue Left Cone Right Place Leave",
            PathConstraints(3.50,2.00)
        )
    }
    fun BlueLeftConeLeftPlaceLeave(): Command {
        return base("Blue Left Cone Left Place Leave",
            PathConstraints(3.50,2.00)
        )
    }
    fun BlueRightConeLeftPlaceLeave(): Command {
        return base("Blue Right Cone Left Place Leave",
            PathConstraints(3.50,2.00)
        )
    }
    fun RedRightConeRightPlaceLeave(): Command {
        return base("Red Right Cone Right Place Leave",
            PathConstraints(3.50,2.00)
        )
    }
    fun BlueTop1Balance(): Command {
        return base("Blue Top 1 Balance",
            PathConstraints(3.00,1.50),
            PathConstraints(2.00, 1.00),
            PathConstraints(1.00, 0.50)
        )
    }
    fun BlueBottom1Balance(): Command {
        return base("Blue Bottom 1 Balance",
            PathConstraints(3.00,1.50),
            PathConstraints(2.00, 1.00),
            PathConstraints(1.00, 0.50)
        )
    }
    fun RedTop1Balance(): Command {
        return base("Red Top 1 Balance",
            PathConstraints(3.00,1.50),
            PathConstraints(3.00, 1.50),
            PathConstraints(2.00, 1.00)
        )
    }
    fun RedBottom1Balance(): Command {
        println("Running The Path")
        return base("Red Bottom 1 Balance",
            PathConstraints(4.00,2.50),
            PathConstraints(3.50, 1.75),
            PathConstraints(1.20, 0.70)
        )
    }
}
