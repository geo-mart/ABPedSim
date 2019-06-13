package de.fhg.ivi.crowdsimulation.ui.gui.control.actions.io;

import java.awt.event.ActionEvent;

import javax.swing.Action;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import de.fhg.ivi.crowdsimulation.simulation.objects.WayPoint;
import de.fhg.ivi.crowdsimulation.ui.CrowdSimulation;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.AbstractAction;
import de.fhg.ivi.crowdsimulation.ui.gui.visualisation.MapPanel;
import jiconfont.icons.FontAwesome;

/**
 * Action, which allows to import data for {@link WayPoint}s as Shapefile or Well-known text.
 *
 * @author meinert
 */
public class WayPointLoadAction extends AbstractAction
{
    /**
     * default serial version uid
     */
    private static final long   serialVersionUID = 1L;

    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger           = LoggerFactory
        .getLogger(WayPointLoadAction.class);

    /**
     * the name and the short description of this action
     */
    private final static String name             = "Import WayPoints...";

    /**
     * Creates a new {@link WayPointLoadAction}, which imports {@link WayPoint} data.
     *
     * @param crowdSimulation object of the main class of the simulation {@link CrowdSimulation}
     */
    public WayPointLoadAction(CrowdSimulation crowdSimulation)
    {
        super(name, crowdSimulation, FontAwesome.MAP_MARKER, blackColor);
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
        MapPanel mapPanel = crowdSimulation.getPanelManager().getPanel(MapPanel.class);

        // try
        // {
        // CrowdSimulator cs = crowdSimulation.getCrowdSimulator();
        // boolean validateStrict = cs.getBoundaries() != null && !cs.getBoundaries().isEmpty()
        // && cs.getWayPoints() != null && !cs.getWayPoints().isEmpty()
        // && cs.getCrowds() != null && !cs.getCrowds().isEmpty();
        // crowdSimulation.loadWayPoints(null, validateStrict);
        // }
        // catch (CrowdSimulatorNotValidException ex)
        // {
        // JOptionPane.showMessageDialog(null, ex.getMessage(), "Warning",
        // JOptionPane.WARNING_MESSAGE);
        // logger.debug("WayPointLoadAction.actionPerformed(), ", e);
        // }
        mapPanel.repaint();
    }
}
