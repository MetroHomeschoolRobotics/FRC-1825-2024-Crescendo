package frc.robot.subsystems.tagtracker.io;

import frc.robot.logging.AutoLoggedInputs;

public interface TagTrackerEnvironmentIO {
    void updateInputs(Inputs inputs);

    final class Inputs extends AutoLoggedInputs {
        public boolean dataChanged = false;
        public double[] packedData = new double[0];
    }
}
