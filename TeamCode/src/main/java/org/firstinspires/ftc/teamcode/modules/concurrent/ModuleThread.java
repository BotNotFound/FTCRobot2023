package org.firstinspires.ftc.teamcode.modules.concurrent;

public abstract class ModuleThread<T extends ConcurrentModule> extends Thread {
    protected final T host;

    public ModuleThread(T host, String threadName) {
        super(host.moduleThreadGroup, threadName);
        this.host = host;
    }

    public abstract void execute() throws InterruptedException;

    @Override
    public final void run() {
        try {
            // wait for host to exit constructor
            final Object hostThreadStartNotifier = host.getThreadStartNotifier();
            synchronized (hostThreadStartNotifier) {
                while (host.getState().isInConstructor()) {
                    hostThreadStartNotifier.wait();
                }
            }

            execute();
        }
        catch (InterruptedException e) {
            // thread interrupts are our signal to stop running
        }
    }
}