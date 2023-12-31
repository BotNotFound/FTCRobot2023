package org.firstinspires.ftc.teamcode.test.switchstatement;

import org.firstinspires.ftc.teamcode.util.SwitchStatement;
import org.junit.Assert;
import org.junit.Test;

public class ErrorTests {
    @Test
    public void nullDefaultCaseThrowsException() {
        Assert.assertThrows(NullPointerException.class, () -> new SwitchStatement<>(null));
    }

    @Test
    public void addingNullCaseThrowsException() {
        final SwitchStatement<Object, Object> statement = new SwitchStatement<>();
        Assert.assertThrows(NullPointerException.class, () -> statement.addCase(null, obj -> null));
    }

    @Test
    public void addingNullHandlerThrowsException() {
        final SwitchStatement<Object, Object> statement = new SwitchStatement<>();
        Assert.assertThrows(NullPointerException.class, () -> statement.addCase(new Object(), null));
    }

    @Test
    public void addingExistingCaseThrowsException() {
        final SwitchStatement<Object, Object> statement = new SwitchStatement<>();
        final Object obj = new Object();
        statement.addCase(obj, s -> null);
        Assert.assertThrows(IllegalArgumentException.class, () -> statement.addCase(obj, s -> null));
    }

    private static class ObjectWithConstantHashCode {
        @Override
        public int hashCode() {
            return 5;
        }
    }

    @Test
    public void addingCaseWithSameHashThrowsNothing() {
        final SwitchStatement<Object, Object> statement = new SwitchStatement<>();
        final Object obj1 = new ObjectWithConstantHashCode();
        final Object obj2 = new ObjectWithConstantHashCode();

        final Object ret1 = new Object();
        final Object ret2 = new Object();

        statement.addCase(obj1, s -> ret1);
        statement.addCase(obj2, s -> ret2);

        final Object randomObject = new Object();

        Assert.assertEquals(statement.execute(obj1), ret1);
        Assert.assertEquals(statement.execute(obj2), ret2);
        Assert.assertNull(statement.execute(randomObject));
    }
}
