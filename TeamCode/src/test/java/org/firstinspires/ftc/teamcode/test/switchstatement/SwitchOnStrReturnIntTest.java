package org.firstinspires.ftc.teamcode.test.switchstatement;

import org.firstinspires.ftc.teamcode.util.SwitchStatement;
import org.junit.Assert;
import org.junit.Test;

public class SwitchOnStrReturnIntTest {
    private static SwitchStatement<String, Integer> getSwitchStatement() {
        return new SwitchStatement<>(String::hashCode)
                .addCase("hello", str -> 1)
                .addCase("whee", str -> 15)
                .addCase("yeet", str -> 36);
    }

    @Test
    public void helloTest() {
        Assert.assertEquals(getSwitchStatement().execute("hello").intValue(), 1);
    }

    @Test
    public void wheeTest() {
        Assert.assertEquals(getSwitchStatement().execute("whee").intValue(), 15);
    }

    @Test
    public void yeetTest() {
        Assert.assertEquals(getSwitchStatement().execute("yeet").intValue(), 36);
    }

    @Test
    public void defaultCaseTest() {
        final String randomStr = "fddsiofpe232";
        Assert.assertEquals(getSwitchStatement().execute(randomStr).intValue(), randomStr.hashCode());
    }
}
