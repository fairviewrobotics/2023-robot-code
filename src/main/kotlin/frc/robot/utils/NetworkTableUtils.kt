package frc.robot.utils

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import java.io.Serializable

class NetworkTableUtils(table: String) {

    private val table: NetworkTable;


    init {
        this.table = NetworkTableInstance.getDefault().getTable(table)
    }


    fun getTable(): NetworkTable {
        return this.table
    }

    private fun getInt(key: String, value: Int): Serializable {
        return this.table.getEntry(key).getInteger(value.toLong())
    }
    private fun getDouble(key: String, defaultValue: Double) : Serializable {
        return this.table.getEntry(key).getDouble(defaultValue)
    }

    private fun getString(key: String, defaultValue: String) : Serializable {
        return this.table.getEntry(key).getString(defaultValue)
    }

     fun getDoubleArray(key: String, doubles: DoubleArray): DoubleArray {
        return this.table.getEntry(key).getDoubleArray(doubles)
    }

    fun setInt(key: String, value: Int) {
        this.table.getEntry(key).setInteger(value.toLong())
    }

    private fun setDouble(key: String, value: Double) {
        this.table.getEntry(key).setDouble(value)
    }

    private fun setString(key: String, value: String) {
        this.table.getEntry(key).setString(value)
    }


    fun <T: Any> getEntry(key: String, value: T): Any {
        return if (this.table.getEntry(key).exists()) {
            when(value) {
                Double -> getDouble(key, value as Double)
                String -> getString(key, value as String)
                Int -> getInt(key, value as Int)
                else -> {
                    IllegalArgumentException("Invalid value type")
                }
            }
        } else {
            IllegalArgumentException("Invalid key")
        }
    }



    fun <T: Any> setEntry(key: String, value: T) {
        if (this.table.getEntry(key).exists()) {
            when(value) {
                Double -> setDouble(key, value as Double)
                String -> setString(key, value as String)
                Int -> setInt(key, value as Int)
                else -> {
                    throw IllegalArgumentException("Invalid value type")
                }
            }
        } else {
            throw IllegalArgumentException("Invalid key")
        }
    }
}