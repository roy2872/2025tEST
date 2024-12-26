package frc.robot.controllers.interfaces;

import java.util.function.DoubleSupplier;

public interface DriverInterface {
    
    public DoubleSupplier getLeftX();

    public DoubleSupplier getLeftY();

    public DoubleSupplier getRightX();
}
