package de.fhg.ivi.crowdsimulation.ui.gui.control.actions.io;

import java.awt.event.ActionEvent;

import javax.swing.Action;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import de.fhg.ivi.crowdsimulation.ui.CrowdSimulation;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.AbstractAction;
import de.fhg.ivi.crowdsimulation.ui.gui.visualisation.MapPanel;
import jiconfont.icons.FontAwesome;

/**
 * Action, which allows to import data for {@link Pedestrian}s as Shapefile or Well-known text.
 *
 * @author meinert
 */
public class CrowdLoadAction extends AbstractAction
{
    /**
     * default serial version uid
     */
    private static final long   serialVersionUID = 1L;

    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger           = LoggerFactory.getLogger(CrowdLoadAction.class);

    /**
     * the name and the short description of this action
     */
    private final static String name             = "Import Pedestrians...";

    /**
     * Creates a new {@link CrowdLoadAction}, which imports {@link Pedestrian} data.
     *
     * @param crowdSimulation object of the main class of the simulation {@link CrowdSimulation}
     */
    public CrowdLoadAction(CrowdSimulation crowdSimulation)
    {
        super(name, crowdSimulation, FontAwesome.USERS, blackColor);
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
        // crowdSimulation.loadCrowd(null, validateStrict);
        // }
        // catch (CrowdSimulatorNotValidException ex)
        // {
        // JOptionPane.showMessageDialog(null, ex.getMessage(), "Warning",
        // JOptionPane.WARNING_MESSAGE);
        // logger.debug("actionPerformed(), ", e);
        // }
        mapPanel.repaint();
    }
}
