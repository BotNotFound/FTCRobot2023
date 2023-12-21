package org.firstinspires.ftc.teamcode.annotation.processor;

import com.google.auto.service.AutoService;
import org.firstinspires.ftc.teamcode.annotations.ModuleInitializer;

import javax.lang.model.SourceVersion;
import javax.lang.model.element.Element;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.TypeElement;
import javax.lang.model.element.VariableElement;
import javax.lang.model.type.NoType;
import javax.tools.Diagnostic;
import javax.tools.JavaFileObject;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

@SupportedAnnotationTypes("org.firstinspires.ftc.teamcode.annotations.ModuleInitializer")
@SupportedSourceVersion(SourceVersion.RELEASE_7)
@AutoService(Processor.class)
public class ModuleProcessor extends AbstractProcessor {
    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnvironment) {
        annotations.forEach(annotation -> {
            Set<? extends Element> annotatedElements
                    = roundEnvironment.getElementsAnnotatedWith(annotation);
            Map<Boolean, List<Element>> testedElements = annotatedElements.parallelStream().collect(
                    Collectors.partitioningBy(
                            element -> {
                                ExecutableElement constructor = (ExecutableElement) element;
                                List<? extends VariableElement> parameters = constructor.getParameters();
                                for (
                                        TypeElement clazz = (TypeElement) constructor.getEnclosingElement();
                                        clazz != null && !(clazz.asType() instanceof NoType);
                                        clazz = (TypeElement) processingEnv.getTypeUtils().asElement(clazz.getSuperclass())
                                )
                                {
                                    // If the class declaring this constructor is not a module, there
                                    if (clazz.getSimpleName().contentEquals("Module")) {
                                        if (parameters.size() == 1
                                                &&
                                                processingEnv
                                                        .getTypeUtils()
                                                        .asElement(
                                                                parameters.get(0)
                                                                        .asType()
                                                        )
                                                        .getSimpleName()
                                                        .contentEquals(
                                                                "OpMode"
                                                        )
                                        ) {
                                            return true;
                                        }

                                        processingEnv.getMessager().printMessage(
                                                Diagnostic.Kind.ERROR,
                                                "Constructor with annotation "
                                                        + ModuleInitializer.class.getSimpleName()
                                                        + " is defined with an invalid signature",
                                                element
                                        );
                                        return false;
                                    }
                                }

                                processingEnv.getMessager().printMessage(
                                        Diagnostic.Kind.ERROR,
                                        "Defining class of constructor with annotation "
                                                + ModuleInitializer.class.getSimpleName()
                                                + " does not extend from class 'Module'",
                                        element
                                );
                                return false;
                            }
                    )
            );

            List<Element> validElements = testedElements.get(true),
                    invalidElements = testedElements.get(false);

            if (!invalidElements.isEmpty()) {
                return;
            }

            try {
                JavaFileObject sourceFile = processingEnv.getFiler().createSourceFile("RegisteredModules.java");
                try (PrintWriter writer = new PrintWriter(sourceFile.openWriter())) {
                    writer.println("package org.firstinspires.ftc.teamcode.modules.core;");
                    writer.println();
                    writer.println("import com.qualcomm.robotcore.eventloop.opmode.OpMode;");
                    writer.println();
                    writer.println("final class RegisteredModules {");
                    writer.println("public static final <T extends Module> T initModule(Class<? extends T> moduleClass, OpMode initializer) {");
                    validElements.forEach(element -> {
                        TypeElement clazz = (TypeElement)element.getEnclosingElement();
                        if (clazz.getAnnotation(Deprecated.class) != null) {
                            return;
                        }
                        String className = clazz.getQualifiedName().toString();
                        writer.println("if (moduleClass.isAssignableFrom(" + className + ".class))");
                        writer.println("return (T) new " + className + "(initializer);");
                    });
                    writer.println("return null;");
                    writer.println("}");
                    writer.println("}");
                }
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        });
        return true;
    }
}
