package frc.robot.commands

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.constants.DrivetrainConstants
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
        println("Running")
        //It might be this for the line above: val pathGroup: ArrayList<PathPlannerTrajectory> = arrayListOf()
        // PathPlanner.loadPathGroup("Red Top 1 Get Balance", PathConstraints(1.0, 0.5)).toCollection(pathGroup)

        //The way these are called may need to change
        val eventMap: HashMap<String, Command> = hashMapOf(
            "MidPlace" to AutoPlaceCubeMid(pnp),
            "Base" to AutoBase(pnp),
            "PickUpCube" to AutoPickCube(pnp),
            "HighPlace" to AutoPlaceCubeHigh(pnp),
            "MidCone" to AutoPlaceConeMid(pnp),
            "Balance" to Balancer(swerveSubsystem)
        )
//        val eventMap = hashMapOf<String, Command>()
//        eventMap["MidPlace"] = MidPlaceCube(pnp)
//        eventMap["Base"] = Base(pnp)
//        eventMap["PickUpCube"] = LowPickCube(pnp)
//        eventMap["HighPlace"] = HighPlaceCube(pnp)
//        eventMap["Balance"] = Balance(swerveSubsystem)

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
            PathConstraints(0.50,0.50),
            PathConstraints(1.00, 1.00)
        )
    }
    fun BlueTop1(): Command {
        return base("Blue Top 1",
            PathConstraints(2.00,1.00)
        )
    }
    fun BlueBottom1(): Command {
        return base("Blue Bottom 1",
            PathConstraints(2.00,1.00)
        )
    }
    fun RedTop1(): Command {
        return base("Red Top 1",
            PathConstraints(2.00,1.00)
        )
    }
    fun RedBottom1(): Command {
        return base("Red Bottom 1",
            PathConstraints(2.00,1.00)
        )
    }
    fun BlueTop2(): Command {
        return base("Blue Top 2",
            PathConstraints(1.00,0.50),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00)
        )
    }
    fun BlueBottom2(): Command {
        return base("Blue Bottom 2",
            PathConstraints(1.00,0.50),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00)
        )
    }
    fun RedTop2(): Command {
        return base("Red Top 2",
            PathConstraints(1.00,0.50),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00)
        )
    }
    fun RedBottom2(): Command {
        return base("Red Bottom 2",
            PathConstraints(1.00,0.50),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00)
        )
    }
    fun BlueCenter1Balance(): Command {
        return base("Blue Center 1 Balance",
            PathConstraints(1.00,0.50),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00)
        )
    }
    fun RedCenter1Balance(): Command {
        return base("Red Center 1 Balance",
            PathConstraints(1.00,0.50),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00)
        )
    }
    fun BlueTop1Balance(): Command {
        return base("Blue Top 1 Balance",
            PathConstraints(1.00,0.50),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00)
        )
    }
    fun BlueBottom1Balance(): Command {
        return base("Blue Bottom 1 Balance",
            PathConstraints(1.00,0.50),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00)
        )
    }
    fun RedTop1Balance(): Command {
        return base("Red Top 1 Balance",
            PathConstraints(1.00,0.50),
            PathConstraints(1.00, 0.50),
            PathConstraints(1.00, 0.50)
        )
    }
    fun RedBottom1Balance(): Command {
        println("Running The Path")
        return base("Red Bottom 1 Balance",
            PathConstraints(1.00,0.50),
            PathConstraints(2.00, 1.00),
            PathConstraints(1.00, 0.50)
        )
    }
    fun BlueTop1GetBalance(): Command {
        return base("Blue Top 1 Get Balance",
            PathConstraints(1.00,0.50),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00)
        )
    }
    fun BlueBottom1GetBalance(): Command {
        return base("Blue Bottom 1 Get Balance",
            PathConstraints(1.00,0.50),
            PathConstraints(2.00, 1.00),
            PathConstraints(3.00, 2.00),
            PathConstraints(1.00, 1.00)
        )
    }
    fun RedTop1GetBalance(): Command {
        return base("Red Top 1 Get Balance",
            PathConstraints(1.00,0.50),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00)
        )
    }
    fun RedBottom1GetBalance(): Command {
        return base("Red Bottom 1 Get Balance",
            PathConstraints(1.00,0.50),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00),
            PathConstraints(4.00, 3.00)
        )
    }
}
