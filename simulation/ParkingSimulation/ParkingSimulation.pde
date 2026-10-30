/**
 * 文件职责：Processing 原生停车仿真窗口；这是直接打开的主文件。
 * 输入：configs/CURRENT_SEARCH.json 和固定模型；输出：动画、停车结果和回放。
 * 调用关系：只调用共用 SimulationRunner，运行三个当前控制模块，不带其他窗口包装。
 */
import java.nio.file.*;
import java.util.*;
import processing.data.*;

SimulationRunner session;
MapEditor_c editor;
ParkingCore.MapDescription marker = new ParkingCore.MapDescription();
ArrayList<double[]> trail = new ArrayList<double[]>();
String projectRoot;
boolean paused = false;
int stepsPerFrame = 20;
long sceneSeed = 20261014L;
JSONObject selection;

void settings() { size(1180, 960); }

void setup() {
  surface.setTitle("重复停车 · Processing 仿真");
  textFont(createFont("Microsoft YaHei", 16));
  projectRoot = System.getProperty("parking.root");
  // 启动器已传入根目录时直接使用；只在Processing IDE运行时查找祖先目录。
  if (projectRoot == null || projectRoot.isBlank()) {
    Path candidate = Paths.get(sketchPath()).toAbsolutePath().normalize();
    while (candidate != null && !Files.isRegularFile(candidate.resolve("configs/CURRENT_SEARCH.json"))) candidate = candidate.getParent();
    if (candidate == null) throw new IllegalStateException("请通过停车控制台.cmd启动，或在Processing中打开本项目的ParkingSimulation.pde。");
    projectRoot = candidate.toString();
  }
  sceneSeed = Long.getLong("parking.demoSeed", 20261014L);
  try {
    selection = SimulationRunner.configureProject(Paths.get(projectRoot));
    editor = new MapEditor_c(Integer.getInteger("parking.arenaMetres", selection.getInt("default_size_m")));
    startScene();
  } catch (Exception e) { throw new RuntimeException(e); }
  frameRate(60);
}

// 场景尺寸仅给模拟环境；控制器只收到反射率、编码器和时间。
void startScene() throws Exception {
  Random random = new Random(sceneSeed);
  JSONObject c = editor.geometry();
  editor.place(c, random); // 先验证草稿及起点，失败时保留原运行。
  c.setLong("encoder_seed", sceneSeed); c.setLong("policy_seed", sceneSeed);
  c.setInt("memory_side", 192); c.setString("stop_mode", "continue"); c.setDouble("margin_mm", 0);
  c.setDouble("reserve_gb", 2); c.setDouble("quota_gb", 5);
  if (session != null && !session.finished) session.finish("user_stopped");
  Path out = Paths.get(projectRoot, "runs", "processing", "scene_" + System.currentTimeMillis());
  Files.createDirectories(out);
  // 每次运行冻结两份权重，回放不受后续模型变化影响。
  Files.copy(Paths.get(System.getProperty("parking.policy")), out.resolve("policy.bin"));
  Files.copy(Paths.get(System.getProperty("parking.motionPolicy")), out.resolve("motion_policy.bin"));
  c.setString("policy", out.resolve("policy.bin").toString());
  c.setString("motion_policy", out.resolve("motion_policy.bin").toString());
  c.setString("policy_sha256", selection.getString("sha256"));
  c.setString("motion_sha256", selection.getJSONObject("boundary_motion").getString("sha256"));
  c.setString("motion_controller", selection.getString("wheel_motion_model"));
  c.setString("boundary_evidence",editor.enabled?"local_obstacle_points":"straight_edges_convex_hull");
  Files.writeString(out.resolve("config.json"), c.toString());
  System.setProperty("parking.policy", c.getString("policy"));
  System.setProperty("parking.motionPolicy", c.getString("motion_policy"));
  System.setProperty("parking.policySeed", String.valueOf(sceneSeed));
  session = new SimulationRunner(c, out); trail.clear();
}

void draw() {
  if (!paused && !editor.editing && !session.finished) {
    try {
      JSONArray frames = session.step(stepsPerFrame).getJSONArray("frames");
      for (int i = 0; i < frames.size(); i++) {
        JSONArray f = frames.getJSONArray(i); trail.add(new double[]{f.getDouble(1), f.getDouble(2)});
      }
    } catch (Exception e) { throw new RuntimeException(e); }
  }
  background(235, 239, 244);
  double width = editor.editing ? editor.metres*1000.0 : session.plant.arena.w;
  float zoom = mapZoom(width);
  pushMatrix(); translate(400, 415); scale(zoom, -zoom);
  rectMode(CENTER); fill(255); stroke(90); strokeWeight(2 / zoom);
  if (editor.editing) { rectMode(CORNER); editor.renderDraft(); }
  else {
    SimulationRunner.Arena arena = session.plant.arena;
    if (arena.shape.equals("rectangle")) rect(0, 0, (float)arena.w, (float)arena.h);
    else { beginShape();for(double[] point:arena.polygon)vertex((float)point[0],(float)point[1]);endShape(CLOSE); }
    noFill();stroke(125,75,55);strokeWeight((float)(2/zoom+10));
    for(double[][] wall:arena.walls) { beginShape();for(double[] point:wall)vertex((float)point[0],(float)point[1]);endShape(); }
  }
  rectMode(CORNER);
  if (!editor.editing || !editor.randomFrame) {
  pushMatrix();
  if (!editor.editing) { translate((float)session.plant.tx,(float)session.plant.ty);rotate((float)session.plant.ta); }
  fill(0); noStroke();
  for (ParkingCore.Rect r : marker.ink) rect((float)r.x0, (float)r.y0, (float)(r.x1-r.x0), (float)(r.y1-r.y0));
  popMatrix();
  }
  if (!editor.editing) {
  noFill(); stroke(40, 135, 210); strokeWeight(1.4 / zoom); beginShape();
  for (double[] p : trail) vertex((float)p[0], (float)p[1]); endShape();
  ParkingCore.Pose p = new ParkingCore.Pose(session.plant.robot.odometry.preciseX,session.plant.robot.odometry.preciseY,session.plant.robot.odometry.preciseTheta);
  pushMatrix(); translate((float)p.x, (float)p.y); rotate((float)p.a);
  fill(session.trial.success ? color(60, 180, 110) : color(100, 155, 215)); stroke(25, 55, 90);
  ellipse(0, 0, 100, 100); line(0, 0, 45, 0);
  strokeWeight(6 / zoom); line(0, -45, 0, -35); line(0, 35, 0, 45);
  popMatrix();
  }
  popMatrix();
  fill(25, 40, 60); textSize(22); text((editor.editing?"地图编辑 · ":"重复停车 · ") + (int)(width/1000) + "米", 805, 60); textSize(16);
  text("探索：heading_seed4", 805, 110);
  text("触框停车：early_fit", 805, 140);
  text("触边：小夹角单轮调整，其余刹停转向", 805, 170);
  text(editor.editing?"边界：应用后使用局部触边":session.plant.usesLocalBoundaryEvidence()?"边界：局部触边，不合并整条直边":"边界：直边记忆＋桌内约束",805,200);
  text("状态：" + (editor.editing?"编辑暂停，应用后生效":session.trial.controller.state.toString()), 805, 235);
  text("动作：" + session.trial.controller.guardState, 805, 265);
  text(String.format(Locale.US, "模拟时间：%.2f 秒", session.plant.robot.getMillis()*.001), 805, 320);
  text("触边次数：" + session.trial.controller.edgeEvents, 805, 350);
  text(String.format(Locale.US, "当前轮心余量：%.2f 毫米", session.plant.currentWheel), 805, 380);
  // 真实几何只用于画面与验收，不能传给触边控制器。
  boolean crossed = session.plant.crossings > 0;
  fill(crossed ? color(170, 65, 35) : color(25, 40, 60));
  text(String.format(Locale.US, "历史最大轮心越线：%.2f 毫米", Math.max(0, -session.plant.minWheel)), 805, 515);
  text("轮心越线：" + session.plant.crossings + " 次；严重越界：" + session.plant.risk.criticalEvents + " 次", 805, 545);
  if (session.plant.arena.custom) text("触墙/边界碰撞：" + session.plant.collisions + " 次",805,575);
  fill(25, 40, 60);
  if (session.finished) {
    text(session.trial.success ? "停车成功" : "运行结束：" + session.result, 805, 440);
    text(String.format(Locale.US, "中心误差：%.2f 毫米", session.trial.finalError), 805, 470);
  }
  text("空格：暂停 / 继续", 805, 620); text("R：下一个随机起点", 805, 650);
  text("＋ / －：仿真倍速", 805, 680); text("S：保存画面", 805, 710);
  textSize(13); text(editor.enabled?"趣味实验模式；复杂地图尚未验证停车能力":"停车成功单独验收；途中越线见上方记录", 805, 760);
  editor.drawControls();
  // 窗口验证入口：实际渲染后保存截图、退出，不修改控制逻辑。
  if (Integer.getInteger("parking.smokeFrames", 0) > 0 && frameCount >= Integer.getInteger("parking.smokeFrames")) {
    saveFrame(Paths.get(projectRoot, ".cache", "processing-window.png").toString()); exit();
  }
}

void keyPressed() {
  if (editor.editing) { if (key=='z'||key=='Z')editor.undoDrawing();if(key==ESC){key=0;editor.strokePoints.clear();}return; }
  if (key == ' ') paused = !paused;
  if (key == '+' || key == '=') stepsPerFrame = Math.min(500, stepsPerFrame * 2);
  if (key == '-') stepsPerFrame = Math.max(1, stepsPerFrame / 2);
  if (key == 'r' || key == 'R') { sceneSeed++; editor.restart(); }
  if (key == 's' || key == 'S') saveFrame(Paths.get(projectRoot, "runs", "processing", "view-####.png").toString());
}

float mapZoom(double mapWidth) { return (float)Math.min(710/(mapWidth+100),700/(mapWidth+100)); }
void mousePressed() { editor.press(); }
void mouseDragged() { editor.drag(); }
void mouseReleased() { editor.release(); }

public void dispose() {
  try { if (session != null && !session.finished) session.finish("window_closed"); }
  catch (Exception e) { System.err.println(e); }
  super.dispose();
}
