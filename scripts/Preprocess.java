/**
 * 文件职责：把我们自己的PDE标签页转成可编译Java，并复制草图的Java组件。
 * 输入：草图目录；输出：.cache/generated。仅由scripts/build.ps1调用。
 */
import java.nio.file.*;
import java.io.*;
import java.util.*;
import processing.mode.java.preproc.PdePreprocessor;

/** Uses the installed Processing preprocessor without IDE preferences or downloads. */
public class Preprocess {
  public static void main(String[] args) throws Exception {
    processing.app.Preferences.skipInit();
    processing.app.Preferences.setBoolean("preproc.substitute_unicode", true);
    Path sketch = Paths.get(args[0]), out = Paths.get(args[1]);
    String name = sketch.getFileName().toString();
    List<Path> tabs = new ArrayList<>();
    tabs.add(sketch.resolve(name + ".pde"));
    try (java.util.stream.Stream<Path> files = Files.list(sketch)) {
      files.filter(p -> p.toString().endsWith(".pde") && !tabs.contains(p))
        .sorted().forEach(tabs::add);
    }
    StringBuilder source = new StringBuilder();
    for (Path tab : tabs) source.append(Files.readString(tab)).append('\n');
    Files.createDirectories(out);
    try (Writer writer = Files.newBufferedWriter(out.resolve(name + ".java"))) {
      PdePreprocessor.builderFor(name).setTabSize(2).setIsTesting(true)
        .build().write(writer, source.toString());
    }
    try (java.util.stream.Stream<Path> files = Files.list(sketch)) {
      for (Path p : (Iterable<Path>) files.filter(p -> p.toString().endsWith(".java"))::iterator)
        Files.copy(p, out.resolve(p.getFileName()), StandardCopyOption.REPLACE_EXISTING);
    }
  }
}
