package org.firstinspires.ftc.teamcode.util;

import java.util.Hashtable;
import java.util.Objects;
import java.util.function.Function;

/**
 * A utility class that allows switch statement-like operations to be performed on reference types.  Equality is
 *  determined via the objects' hashCode() methods
 * @param <SWITCH_ON> The type of the cases and object to switch on
 * @param <RETURN> The type that will be returned by every case and, by extension, {@link #execute(Object)}
 */
public class SwitchStatement<SWITCH_ON, RETURN> {
    /**
     * The set of cases mapped to the code to run for each case
     */
    private final Hashtable<SWITCH_ON, Function<SWITCH_ON, RETURN>> cases;

    /**
     * The default case that will be run if the given object matches no other cases
     */
    private final Function<SWITCH_ON, RETURN> defaultCase;

    /**
     * Initializes the switch statement with a given default case
     * @param defaultCase The default case that will be run if no match is found
     */
    public SwitchStatement(Function<SWITCH_ON, RETURN> defaultCase) {
        this.defaultCase = defaultCase;
        cases = new Hashtable<>();
    }

    /**
     * Adds a case to this switch statement and returns itself
     * @param caseObj The case
     * @param whatToDo What to run if the case is a match
     * @return The switch statement, after the case has been added
     */
    public SwitchStatement<SWITCH_ON, RETURN> addCase(SWITCH_ON caseObj, Function<SWITCH_ON, RETURN> whatToDo) {
        Objects.requireNonNull(caseObj, "Attempted to add a null case!  Null cases are not permitted; use the " +
                "default case instead!");
        Objects.requireNonNull(whatToDo, "Attempted to add a case without a body!");
        cases.put(caseObj, whatToDo);
        return this;
    }

    /**
     * Executes the switch statement
     * @param switchOn The object to compare with each case
     * @return The result of the matching case, or the result of the default case if no match is found
     */
    public RETURN execute(SWITCH_ON switchOn) {
        if (Objects.isNull(switchOn)) {
            return defaultCase.apply(switchOn);
        }

        final Function<SWITCH_ON, RETURN> matchingCase = cases.getOrDefault(switchOn, defaultCase);
        return matchingCase.apply(switchOn);
    }
}
