//import se.lth.control.realtime.*;
import se.lth.control.realtime.AnalogIn;
import se.lth.control.realtime.AnalogOut;
import se.lth.control.realtime.IOChannelException;


public class Regul extends Thread {

    private PI inner = new PI("PI");
    private PID outer = new PID("PID");

    private ReferenceGenerator refGen;
    private OpCom opCom;

    private AnalogIn analogInAngle;
    private AnalogIn analogInPosition;
    private AnalogOut analogOut;

    private int priority;
    private boolean shouldRun = true;
    private long startTime;

    private ModeMonitor modeMon;

    public Regul(int pri, ModeMonitor modeMon) {
        this.priority = pri;
        setPriority(priority);
        this.modeMon = modeMon;
        inner = new PI("PI");
        outer = new PID("PID");

        try {
            this.analogOut = new AnalogOut(0);
            this.analogInAngle = new AnalogIn(0);
            this.analogInPosition = new AnalogIn(1);
            
        } catch (Exception e) {
            System.out.print("Error: IOChannelException: ");
            System.out.println(e.getMessage());
        }
    }

    /** Sets OpCom (called from main) */
    public void setOpCom(OpCom opCom) {
        this.opCom = opCom;
    }

    /** Sets ReferenceGenerator (called from main) */
    public void setRefGen(ReferenceGenerator refGen) {
        this.refGen = refGen;
    }

    // Called in every sample in order to send plot data to OpCom
    private void sendDataToOpCom(double yRef, double y, double u) {
        double x = (double) (System.currentTimeMillis() - startTime) / 1000.0;
        opCom.putControlData(x, u);
        opCom.putMeasurementData(x, yRef, y);
    }

    // Sets the inner controller's parameters
    public void setInnerParameters(PIParameters p) {
        inner.setParameters((PIParameters) p.clone());
    }

    // Gets the inner controller's parameters
    public PIParameters getInnerParameters() {
        return inner.getParameters();
    }

    // Sets the outer controller's parameters
    public void setOuterParameters(PIDParameters p) {
        outer.setParameters((PIDParameters) p.clone());
    }

    // Gets the outer controller's parameters
    public PIDParameters getOuterParameters(){
        return outer.getParameters();
    }

    // Called from OpCom when shutting down
    public void shutDown() {
        shouldRun = false;
    }

    // Saturation function
    private double limit(double v) {
        return limit(v, -10, 10);
    }

    // Saturation function
    private double limit(double v, double min, double max) {
        if (v < min) v = min;
        else if (v > max) v = max;
        return v;
    }

    public void run() {

        long duration;
        long t = System.currentTimeMillis();
        startTime = t;
        double u = 0.0;

        while (shouldRun) {
            double yAng = readInput(analogInAngle);
            double yPos = readInput(analogInPosition);
            double posRef = refGen.getRef();

            switch (modeMon.getMode()) {
                case OFF: {
                    yPos = 0.0;
                    posRef = 0.0;
                    break;
                }
                case BEAM: {
                    synchronized (inner) {
                        u = limit(inner.calculateOutput(yPos, posRef));
                        writeOutput(u);
                        inner.updateState(u);
                    }
                    break;
                }
                case BALL: {
                    double angU = 0.0;
                    synchronized(outer) {
                        angU = limit(outer.calculateOutput(yPos, posRef));
                        outer.updateState(angU);
                    }

                    synchronized (inner) {
                        u = limit(inner.calculateOutput(yAng, angU));
                        writeOutput(u);
                        inner.updateState(angU);
                    }

                    break;
                }
                default: {
                    System.out.println("Error: Illegal mode.");
                    break;
                }
            }

            sendDataToOpCom(posRef, yPos, u);

            // sleep
            t = t + inner.getHMillis();
            duration = t - System.currentTimeMillis();
            if (duration > 0) {
                try {
                    sleep(duration);
                } catch (InterruptedException x) {}
            } else {
                System.out.println("Lagging behind...");
            }
        }
        u = 0.0;
        /** Written by you: Set control signal to zero before exiting run loop */
    }

    // Writes the control signal u to the output channel: analogOut
    // @throws: IOChannelException
    private void writeOutput(double u) {
        try {
            analogOut.set(u);
        } catch (IOChannelException e) {
            e.printStackTrace();
        }
    }

    // Reads the measurement value from the input channel: in
    // @throws: IOChannelException
    private double readInput(AnalogIn in) {
        try {
            return in.get();
        } catch (IOChannelException e) {
            e.printStackTrace();
            return 0.0;
        }
    }
}
