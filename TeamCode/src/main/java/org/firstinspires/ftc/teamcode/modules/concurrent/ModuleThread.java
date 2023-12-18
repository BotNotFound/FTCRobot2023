package org.firstinspires.ftc.teamcode.modules.concurrent;

public class ModuleThread<T extends ConcurrentModule> extends Thread {
    protected final T host;

    public ModuleThread(T host, String threadName) {
        super(host.moduleThreadGroup, threadName);
        this.host = host;
    }
}