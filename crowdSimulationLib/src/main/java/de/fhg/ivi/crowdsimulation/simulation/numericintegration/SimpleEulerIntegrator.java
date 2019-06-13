package de.fhg.ivi.crowdsimulation.simulation.numericintegration;

import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import de.fhg.ivi.crowdsimulation.simulation.forcemodel.ForceModel;
import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import math.geom2d.Vector2D;

/**
 * Simple Euler or Forward Euler is one of many possible algorithms of numerical integration which
 * can dissolve ordinary differential equations, like the Social Force Model {@link ForceModel}. The
 * difference between this class and the {@link SemiImplicitEulerIntegrator} is, that the position
 * of the {@link Pedestrian} is updated in the next time step in this class. For further explanation
 * see the links below.
 * <p>
 * Euler explanation: <br>
 * http://people.physik.hu-berlin.de/~mitdank/dist/scriptenm/Eulerintegration.htm<br>
 * https://en.wikipedia.org/wiki/Euler_method<br>
 * http://tutorial.math.lamar.edu/Classes/DE/EulersMethod.aspx<br>
 * also look into the PedSim library<br>
 * https://scicomp.stackexchange.com/questions/20172/why-are-runge-kutta-and-eulers-method-so-different<br>
 *
 * @author hahmann/meinert
 *
 */
public class SimpleEulerIntegrator extends NumericIntegrator
{

    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger = LoggerFactory.getLogger(SimpleEulerIntegrator.class);

    /**
     * Overrides the abstract method in {@link NumericIntegrator} and sets this
     * {@link SimpleEulerIntegrator} as algorithm of numeric integration, which dissolves the
     * {@link ForceModel}.
     * <p>
     * Basically this class calculates the movement of a specific {@link Pedestrian}. This means
     * his/her new position and velocity, in dependence to his/her old velocity and the terms of the
     * Social Force Model (see Helbing et al. 2005), is computed.
     *
     * @param currentTimeMillis the current unix time stamp in simulated time, given in milliseconds
     * @param simulationInterval the time between this method invocation and the last one
     * @param pedestrian the one {@link Pedestrian}, whose movement is calculated
     * @param pedestrians represents a list of all {@link Pedestrian}s
     * @param boundaries represents a list of all {@link Boundary}s
     * @param forceModel an {@link Object} of {@link ForceModel}
     *
     *
     * @see de.fhg.ivi.crowdsimulation.simulation.numericintegration.NumericIntegrator#move(long,
     *      double, Pedestrian, List, List, ForceModel)
     */
    @Override
    public void move(long currentTimeMillis, double simulationInterval, Pedestrian pedestrian,
        List<Pedestrian> pedestrians, List<Boundary> boundaries, ForceModel forceModel)
    {
        // old position
        Vector2D currentPosition = pedestrian.getCurrentPosition();

        // update position = x(n+1)
        Vector2D updatedPosition = pedestrian.getCurrentPosition()
            .plus(pedestrian.getCurrentVelocity().times(simulationInterval));

        // validated updated position - guaranteed not to go through a boundary
        updatedPosition = NumericIntegrationTools.validateMove(pedestrian, boundaries,
            currentPosition, updatedPosition);

        // update current Position
        pedestrian.setCurrentPosition(updatedPosition);

        // test whether a WayPoint has been passed by the Pedestrian
        pedestrian.getMentalModel().checkWayPointPassing(pedestrian, currentPosition,
            updatedPosition);

        // check if WayPoint has been passed
        pedestrian.getMentalModel().checkCourse(pedestrian, currentTimeMillis);

        // compute resulting force
        Vector2D resultingForce = pedestrian.getForces(currentTimeMillis, pedestrians, boundaries,
            forceModel);

        logger.trace("SimpleEulerIntegrator.move(), resultingForce: " + resultingForce);

        // updatedVelocity = v(n+1) (t+1)
        Vector2D updatedVelocity = pedestrian.getCurrentVelocity()
            .plus(resultingForce.times(simulationInterval));

        // check, if velocity is not too high
        updatedVelocity = NumericIntegrationTools.getValidatedVelocity(pedestrian, updatedVelocity);

        // update current Velocity
        pedestrian.setCurrentVelocity(updatedVelocity);
    }
}
