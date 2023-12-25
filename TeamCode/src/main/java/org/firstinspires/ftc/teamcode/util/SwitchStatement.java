package org.firstinspires.ftc.teamcode.util;

import java.util.Hashtable;
import java.util.Objects;
import java.util.function.Function;

public final class SwitchStatement<SWITCH_ON, RETURN> {
    private final Hashtable<SWITCH_ON, Function<SWITCH_ON, RETURN>> cases;

    private final Function<SWITCH_ON, RETURN> defaultCase;

    public SwitchStatement(Function<SWITCH_ON, RETURN> defaultCase) {
        this.defaultCase = defaultCase;
        cases = new Hashtable<>();
    }

    public SwitchStatement<SWITCH_ON, RETURN> addCase(SWITCH_ON caseObj, Function<SWITCH_ON, RETURN> whatToDo) {
        Objects.requireNonNull(caseObj, "Attempted to add a null case!  Null cases are not permitted; use the " +
                "default case instead!");
        Objects.requireNonNull(whatToDo, "Attempted to add a case without a body!");
        cases.put(caseObj, whatToDo);
        return this;
    }

    public RETURN execute(SWITCH_ON switchOn) {
        if (Objects.isNull(switchOn)) {
            return defaultCase.apply(switchOn);
        }

        final Function<SWITCH_ON, RETURN> matchingCase = cases.getOrDefault(switchOn, defaultCase);
        return matchingCase.apply(switchOn);
    }
}
