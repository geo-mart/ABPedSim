package de.fhg.ivi.crowdsimulation.simulation.numericintegration;

import java.util.List;

import de.fhg.ivi.crowdsimulation.simulation.forcemodel.ForceModel;
import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import math.geom2d.Vector2D;

/**
 * Runge-Kutta is one of many possible algorithms of numerical integration which can dissolve
 * ordinary differential equations, like the Social Force Model {@link ForceModel}. In this class
 * the Runge-Kutta algorithm of 4. order is used. For explanation see the links below.
 * <p>
 * Runge-Kutta explanation:<br>
 * http://pubsonline.informs.org/doi/pdf/10.1287/trsc.1040.0108<br>
 * http://itp.uni-frankfurt.de/~gros/StudentProjects/Applets_2014_PedestrianCrowdDynamics/PedestrianApplet.html<br>
 * https://scicomp.stackexchange.com/questions/20172/why-are-runge-kutta-and-eulers-method-so-different<br>
 *
 * @author hahmann/meinert
 */
public class RungeKuttaIntegrator extends NumericIntegrator
{

    /**
     * Overrides the abstract method in {@link NumericIntegrator} and sets this
     * {@link RungeKuttaIntegrator} as algorithm of numeric integration, which dissolves the
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
     * @see de.fhg.ivi.crowdsimulation.simulation.numericintegration.NumericIntegrator#move(long,
     *      double, Pedestrian, List, List, ForceModel)
     */
    @Override
    public void move(long currentTimeMillis, double simulationInterval, Pedestrian pedestrian,
        List<Pedestrian> pedestrians, List<Boundary> boundaries, ForceModel forceModel)
    {
        double[] integrationIntervals = { 0, simulationInterval / 2, simulationInterval / 2,
            simulationInterval };
        Vector2D[] temporaryForces = new Vector2D[integrationIntervals.length];
        Vector2D[] temporaryVelocities = new Vector2D[integrationIntervals.length];

        // current position
        Vector2D currentPosition = pedestrian.getCurrentPosition();
        // current velocity
        Vector2D currentVelocity = pedestrian.getCurrentVelocity();

        // iterate over 4 Runge-Kutta steps
        for (int k = 0; k < 4; k++ )
        {
            // temporary position during this Runge-Kutta step
            Vector2D temporaryPosition;
            // temporary velocity during this Runge-Kutta step
            Vector2D temporaryVelocity = pedestrian.getCurrentVelocity();

            if (k == 0)
            {
                temporaryPosition = currentPosition;
            }
            // k > 0
            else
            {
                temporaryPosition = currentPosition
                    .plus(temporaryVelocity.times(integrationIntervals[k]))
                    .plus(temporaryForces[k - 1].times(0.5)
                        .times(integrationIntervals[k])
                        .times(integrationIntervals[k]));
                temporaryVelocity = temporaryVelocity
                    .plus(temporaryForces[k - 1].times(integrationIntervals[k]));
            }

            // total acceleration on current pedestrian
            Vector2D resultingForce = pedestrian.getForces(temporaryPosition, temporaryVelocity,
                currentTimeMillis, pedestrians, boundaries, forceModel);

            // add resulting temporary forces to array
            temporaryForces[k] = resultingForce;
            temporaryVelocities[k] = temporaryVelocity
                .plus(resultingForce.times(integrationIntervals[k]));
        }

        // updatedVelocity = v(n+1)
        Vector2D updatedVelocity = currentVelocity
            .plus((temporaryForces[0].plus(temporaryForces[1].times(2d))
                .plus(temporaryForces[2].times(2d))
                .plus(temporaryForces[3])).times(1d / 6d).times(simulationInterval));

        // validate velocity
        updatedVelocity = NumericIntegrationTools.getValidatedVelocity(pedestrian, updatedVelocity);

        // update velocity
        pedestrian.setCurrentVelocity(updatedVelocity);

        // updatedPosition = x(n+1)
        Vector2D updatedPosition = currentPosition
            .plus((temporaryVelocities[0].plus(temporaryVelocities[1].times(2d))
                .plus(temporaryVelocities[2].times(2d))
                .plus(temporaryVelocities[3])).times(simulationInterval).times(1d / 6d));

        // validated updated position - guaranteed not to go through a boundary
        updatedPosition = NumericIntegrationTools.validateMove(pedestrian, boundaries,
            currentPosition, updatedPosition);

        // update position
        pedestrian.setCurrentPosition(updatedPosition);

        // check if the current WayPoint has been passed
        pedestrian.getMentalModel().checkWayPointPassing(pedestrian, currentPosition,
            updatedPosition);

        // check if WayPoint has been passed
        pedestrian.getMentalModel().checkCourse(pedestrian, currentTimeMillis);
    }
}
