package de.fhg.ivi.crowdsimulation.ui.gui.control.actions.io;

import java.awt.event.ActionEvent;

import javax.swing.Action;
import javax.swing.JOptionPane;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import de.fhg.ivi.crowdsimulation.simulation.CrowdSimulator;
import de.fhg.ivi.crowdsimulation.simulation.CrowdSimulatorNotValidException;
import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.ui.CrowdSimulation;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.AbstractAction;
import jiconfont.icons.FontAwesome;

/**
 * Action, which allows to import data for {@link Boundary}s as Shapefile or Well-known text.
 *
 * @author meinert
 */
public class BoundaryLoadAction extends AbstractAction
{
    /**
     * default serial version uid
     */
    private static final long   serialVersionUID = 1L;

    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger           = LoggerFactory
        .getLogger(BoundaryLoadAction.class);

    /**
     * the name and the short description of this action
     */
    private final static String name             = "Import Boundaries...";

    /**
     * Creates a new {@link BoundaryLoadAction}, which imports {@link Boundary} data.
     *
     * @param crowdSimulation object of the main class of the simulation {@link CrowdSimulation}
     */
    public BoundaryLoadAction(CrowdSimulation crowdSimulation)
    {
        super(name, crowdSimulation, FontAwesome.GLOBE, blackColor);
        putValue(Action.SHORT_DESCRIPTION, name);
    }

    /**
     * Overrides the same method in {@link AbstractAction} and invokes a specific {@link Action}.
     *
     * @see de.fhg.ivi.crowdsimulation.ui.gui.control.actions.AbstractAction#actionPerformed(ActionEvent)
     */
    @Override
    public void actionPerformed(ActionEvent e)
    {
        try
        {
            CrowdSimulator cs = crowdSimulation.getCrowdSimulator();
            boolean validateStrict = cs.getBoundaries() != null && !cs.getBoundaries().isEmpty()
                && cs.getWayPoints() != null && !cs.getWayPoints().isEmpty()
                && cs.getCrowds() != null && !cs.getCrowds().isEmpty();
            crowdSimulation.loadBoundaries(null, validateStrict);
        }
        catch (CrowdSimulatorNotValidException ex)
        {
            JOptionPane.showMessageDialog(null, ex.getMessage(), "Warning",
                JOptionPane.WARNING_MESSAGE);
            logger.debug("BoundaryLoadAction.actionPerformed(), ", e);
        }
    }
}
