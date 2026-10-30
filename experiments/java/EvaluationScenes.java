/**
 * 文件职责：从保存的种子和场景表重建仿真，提供统一实验设置
 * 输入：CSV场景的一行；可选第13列为保存的配置JSON；输出：对应的完整场景。
 * 调用关系：ParkingEvaluation 和 SimulationChecks 使用。
 */
import java.util.*;import java.nio.file.*;import processing.data.*;
public final class EvaluationScenes {
 static JSONObject config(double size,double x,double y,double yaw,long enc,double tx,double ty,double ta){
  JSONObject c=new JSONObject();c.setString("shape","rectangle");c.setDouble("width_mm",size*1000);c.setDouble("height_mm",size*1000);c.setJSONArray("polygon",new JSONArray());
  c.setDouble("target_x",tx);c.setDouble("target_y",ty);c.setDouble("target_yaw",ta);c.setDouble("robot_x",x);c.setDouble("robot_y",y);c.setDouble("robot_yaw",yaw);c.setLong("encoder_seed",enc);c.setDouble("margin_mm",0);c.setString("stop_mode","continue");return c;
 }
 static SimulationRunner.Plant scene(String[] c)throws java.io.IOException{
  // 可选第13列指向保存的完整场景，含手绘边界和墙；普通12列场景保持原样。
  if(c.length>12&&!c[12].isBlank())return new SimulationRunner.Plant(JSONObject.parse(Files.readString(Paths.get(c[12]))));
  return new SimulationRunner.Plant(config(Double.parseDouble(c[0]),Double.parseDouble(c[3]),Double.parseDouble(c[4]),Double.parseDouble(c[5]),Long.parseLong(c[6]),Double.parseDouble(c[7]),Double.parseDouble(c[8]),Double.parseDouble(c[9])));
 }
 public static void settings(){System.setProperty("parking.memory","lines_corridor_arc");System.setProperty("parking.reverseArc","true");System.setProperty("parking.learnBoundary","true");System.setProperty("parking.hardHeading","true");System.setProperty("parking.policyTemperature",".5");}
}
