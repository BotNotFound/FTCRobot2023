package org.firstinspires.ftc.teamcode.util;

import android.util.Pair;

import java.util.LinkedList;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

public final class SwitchStatement<SWITCH_ON, RETURN> {
    private final LinkedList<Pair<SWITCH_ON, Function<SWITCH_ON, RETURN>>> cases;

    private final Function<SWITCH_ON, RETURN> defaultCase;

    public SwitchStatement(Function<SWITCH_ON, RETURN> defaultCase) {
        this.defaultCase = defaultCase;
        cases = new LinkedList<>();
    }

    public SwitchStatement<SWITCH_ON, RETURN> addCase(SWITCH_ON caseObj, Function<SWITCH_ON, RETURN> whatToDo) {
        Objects.requireNonNull(caseObj, "Attempted to add a null case!  Null cases are not permitted; use the " +
                "default case instead!");
        Objects.requireNonNull(whatToDo, "Attempted to add a case without a body!");
        cases.add(new Pair<>(caseObj, whatToDo));
        return this;
    }

    public RETURN execute(SWITCH_ON switchOn) {
        if (Objects.isNull(switchOn)) {
            return defaultCase.apply(switchOn);
        }

        final AtomicReference<Function<SWITCH_ON, RETURN>> matchingCase = new AtomicReference<>(null);
        cases.parallelStream().forEach(caseStatement -> {
            if (switchOn.equals(caseStatement.first)) {
                matchingCase.compareAndSet(null, caseStatement.second);
            }
        });
        final Function<SWITCH_ON, RETURN> toRun = matchingCase.get();
        if (toRun != null) {
            return toRun.apply(switchOn);
        }
        return defaultCase.apply(switchOn);
    }
}
