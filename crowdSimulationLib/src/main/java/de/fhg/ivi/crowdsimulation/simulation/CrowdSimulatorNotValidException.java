package de.fhg.ivi.crowdsimulation.simulation;

/**
 * The class {@code SceneNotValidException} inherits from {@code Exception} and is with it a form of
 * {@code Throwable} that indicates conditions that a reasonable application might want to catch.
 *
 * @author Hahmann
 */
public class CrowdSimulatorNotValidException extends Exception
{
    /**
     * default serial version ID
     */
    private static final long serialVersionUID = 1L;

    /**
     * Constructs a new exception with the specified detail message.
     *
     * @param message the detail message. The detail message is saved for later retrieval by the
     *            {@link #getMessage()} method.
     */
    public CrowdSimulatorNotValidException(String message)
    {
        super(message);
    }
}
