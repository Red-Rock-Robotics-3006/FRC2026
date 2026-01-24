package redrocklib.util;

import redrocklib.logging.SmartDashboardNumber;

/**
 * Represents a linear function between inputs and outputs
 * Typically used for converting units to motor positions
 */
public class LerpingSmartDashboardNumber {
    private SmartDashboardNumber inputLowerBound;
    private SmartDashboardNumber inputUpperBound;
    private SmartDashboardNumber outputLowerBound;
    private SmartDashboardNumber outputUpperBound;

    public LerpingSmartDashboardNumber(double inputLower, double outputLower, double inputUpper, double outputUpper, 
                String inputIdentifier, String outputIdentifier, boolean tuneable) {
        inputLowerBound = new SmartDashboardNumber(inputIdentifier + "-lower", inputLower, tuneable);
        inputUpperBound = new SmartDashboardNumber(inputIdentifier + "-upper", inputUpper, tuneable);
        outputLowerBound = new SmartDashboardNumber(outputIdentifier + "-lower", outputLower, tuneable);
        outputUpperBound = new SmartDashboardNumber(outputIdentifier + "-upper", outputUpper, tuneable);
    }

    public double getValue(double input) {
        return ((outputUpperBound.getNumber() - outputLowerBound.getNumber()) / (inputUpperBound.getNumber() - inputLowerBound.getNumber()))
            * (input - inputLowerBound.getNumber()) + outputLowerBound.getNumber();
    }

    public double convertOutputByRate(double val) {
        return ((inputUpperBound.getNumber() - inputLowerBound.getNumber()) / (outputUpperBound.getNumber() - outputLowerBound.getNumber())) * val;
    }

    public double getMaxOutput() {
        return outputUpperBound.getNumber();
    }

    public double getMinOutput() {
        return outputLowerBound.getNumber();
    }
}
