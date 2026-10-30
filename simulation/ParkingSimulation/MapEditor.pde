/**
 * 文件职责：窗口下方的尺寸、随机停车框和独立趣味地图开关，以及鼠标画边界/墙线。
 * 输入：按钮与鼠标笔画；输出：场景几何、可用起点及 configs/FUN_MAP.json。
 * 调用关系：ParkingSimulation 调用；几何只交给 SimulationRunner 的环境，不传给模型。
 */
class MapEditor_c {
  int metres, tool = 0, brush = 0; // tool: 外边界/墙；brush: 手绘/圆/椭圆/矩形/直线
  boolean randomFrame = false, enabled = false, editing = false;
  ArrayList<double[]> boundary = new ArrayList<double[]>();
  ArrayList<ArrayList<double[]>> walls = new ArrayList<ArrayList<double[]>>();
  ArrayList<double[]> strokePoints = new ArrayList<double[]>();
  ArrayDeque<String> undo = new ArrayDeque<String>();
  Path draftFile;
  String notice = "选择尺寸或打开开关；关闭趣味地图时使用普通矩形场地。";

  MapEditor_c(int size) {
    metres = Math.max(1, Math.min(4, size)); resetBoundary();
    draftFile=Paths.get(System.getProperty("parking.mapDraft",Paths.get(projectRoot,"configs","FUN_MAP.json").toString()));
    Path saved = draftFile;
    if (Files.isRegularFile(saved)) try {
      JSONObject data = JSONObject.parse(Files.readString(saved)); restore(data);
      enabled=true;new SimulationRunner.Arena(geometry());enabled=false;
    } catch (Exception e) { enabled=false;resetBoundary(); walls.clear(); notice = "保存地图无法读取，已恢复默认草稿。"; }
  }

  void resetBoundary() {
    boundary.clear();
    boundary.add(new double[]{-.5,-.5}); boundary.add(new double[]{.5,-.5});
    boundary.add(new double[]{.5,.5}); boundary.add(new double[]{-.5,.5});
  }
  JSONArray points(ArrayList<double[]> line, double scale) {
    JSONArray a = new JSONArray();
    for (double[] p : line) { JSONArray q = new JSONArray(); q.append(p[0]*scale); q.append(p[1]*scale); a.append(q); }
    return a;
  }
  JSONObject snapshot() {
    JSONObject c = new JSONObject(); c.setString("format", "normalized_map_v1");
    c.setJSONArray("boundary", points(boundary,1)); JSONArray a = new JSONArray();
    for (ArrayList<double[]> line : walls) a.append(points(line,1)); c.setJSONArray("walls",a); return c;
  }
  ArrayList<double[]> readPoints(JSONArray a) {
    ArrayList<double[]> result = new ArrayList<double[]>();
    for (double[] p : SimulationRunner.Arena.points(a)) {
      if (Math.abs(p[0])>.50001 || Math.abs(p[1])>.50001) throw new IllegalArgumentException("地图点超出画布");
      result.add(p);
    } return result;
  }
  void restore(JSONObject c) {
    if (!c.getString("format").equals("normalized_map_v1")) throw new IllegalArgumentException("地图格式无效");
    boundary = readPoints(c.getJSONArray("boundary")); walls.clear(); JSONArray a = c.getJSONArray("walls");
    if (a.size()>64) throw new IllegalArgumentException("最多64道墙");
    for (int i=0;i<a.size();i++) walls.add(readPoints(a.getJSONArray(i)));
  }
  void remember() { if (undo.size()>=20) undo.removeFirst(); undo.addLast(snapshot().toString()); }
  void undoDrawing() {
    strokePoints.clear();
    if (!undo.isEmpty()) { restore(JSONObject.parse(undo.removeLast())); notice = "已撤销上一笔。"; }
  }
  JSONObject geometry() {
    double width = metres*1000.0; JSONObject c = new JSONObject();
    c.setDouble("width_mm",width); c.setDouble("height_mm",width);
    c.setString("shape",enabled?"polygon":"rectangle");
    c.setJSONArray("polygon",enabled?points(boundary,width):new JSONArray());
    JSONArray a = new JSONArray(); if (enabled) for (ArrayList<double[]> line : walls) a.append(points(line,width));
    c.setJSONArray("walls",a); c.setBoolean("custom_map",enabled); c.setBoolean("random_frame",randomFrame);
    return c;
  }

  // 选起点只属于场景生成；不让控制器借此读取地图或停车框位置。
  void place(JSONObject c, Random random) {
    SimulationRunner.Arena arena = new SimulationRunner.Arena(c);
    double tx=0,ty=0,width=arena.w;
    if (randomFrame) {
      boolean found=false;
      for (int k=0;k<5000;k++) {
        tx=(random.nextDouble()*2-1)*(width/2-160); ty=(random.nextDouble()*2-1)*(width/2-160);
        if (arena.clearance(tx,ty)>=160) { found=true; break; }
      }
      if (!found) throw new IllegalArgumentException("地图没有足够空间放停车框，请扩大外边界或移开墙线。");
    } else if (arena.clearance(0,0)<160) {
      throw new IllegalArgumentException("中心停车框被边界或墙挡住；请打开随机停车框，或修改地图。");
    }
    c.setDouble("target_x",tx); c.setDouble("target_y",ty); c.setDouble("target_yaw",0);
    double x=0,y=0;
    if (!enabled) {
      // 两个新开关关闭时，随机数调用顺序与原版完全相同。
      do { x=(random.nextDouble()*2-1)*(width/2-70); y=(random.nextDouble()*2-1)*(width/2-70); }
      while (Math.hypot(x-tx,y-ty)<230);
    } else {
      ArrayList<double[]> candidates = reachableStarts(arena,tx,ty);
      if (candidates.isEmpty()) throw new IllegalArgumentException("与停车框连通的空间太小，无法放小车；请扩大地图或留出通道。");
      double[] p=candidates.get(random.nextInt(candidates.size())); x=p[0];y=p[1];
      for (int k=0;k<30;k++) {
        double xx=p[0]+(random.nextDouble()*2-1)*10,yy=p[1]+(random.nextDouble()*2-1)*10;
        if (arena.clearance(xx,yy)>=70 && Math.hypot(xx-tx,yy-ty)>=230) { x=xx;y=yy;break; }
      }
    }
    c.setDouble("robot_x",x); c.setDouble("robot_y",y); c.setDouble("robot_yaw",random.nextDouble()*2*Math.PI-Math.PI);
  }
  ArrayList<double[]> reachableStarts(SimulationRunner.Arena arena,double tx,double ty) {
    // 50毫米网格只用于避免将起点放在被墙完全隔开的另一间房。
    int n=(int)Math.ceil(arena.w/50);double step=arena.w/n;
    boolean[] free=new boolean[n*n],seen=new boolean[n*n];int start=-1;double nearest=Double.POSITIVE_INFINITY;
    for (int j=0;j<n;j++) for (int i=0;i<n;i++) {
      int k=j*n+i;double x=-arena.w/2+(i+.5)*step,y=-arena.h/2+(j+.5)*step;
      free[k]=arena.clearance(x,y)>=60;double d=Math.hypot(x-tx,y-ty);
      if (free[k]&&d<nearest) { nearest=d;start=k; }
    }
    ArrayList<double[]> result=new ArrayList<double[]>(); if (start<0||nearest>70) return result;
    ArrayDeque<Integer> queue=new ArrayDeque<Integer>();queue.add(start);seen[start]=true;
    while (!queue.isEmpty()) {
      int k=queue.removeFirst(),i=k%n,j=k/n;
      double x=-arena.w/2+(i+.5)*step,y=-arena.h/2+(j+.5)*step;
      if (arena.clearance(x,y)>=85&&Math.hypot(x-tx,y-ty)>=245) result.add(new double[]{x,y});
      for (int[] d:new int[][]{{-1,0},{1,0},{0,-1},{0,1}}) {
        int ii=i+d[0],jj=j+d[1]; if (ii<0||jj<0||ii>=n||jj>=n) continue;int next=jj*n+ii;
        if (free[next]&&!seen[next]) { seen[next]=true;queue.add(next); }
      }
    } return result;
  }

  boolean restart() { try { startScene(); notice="新场景已开始。";return true; } catch(Exception e) { notice=e.getMessage();return false; } }
  void toggleEditor() {
    strokePoints.clear();
    if (!enabled) { enabled=true;editing=true;paused=true;notice="先选外边界或内部墙，再在地图上拖动；画完点击应用运行。"; }
    else { enabled=false;editing=false;
      restart();paused=false;
      notice="趣味地图已关闭，普通矩形模式。草稿已保留。";
    }
  }
  void applyMap() {
    try { startScene(); Files.writeString(draftFile,snapshot().toString());
      editing=false;paused=false;notice="地图已应用并保存；再次点击编辑可修改，关闭开关恢复普通地图。";
    } catch(Exception e) { notice=e.getMessage(); }
  }
  void button(float x,float y,float w,String label,boolean selected) {
    strokeWeight(1);
    fill(selected?color(45,109,170):color(246,248,251));stroke(selected?color(45,109,170):color(180,191,204));
    rect(x,y,w,34,5);noStroke();fill(selected?255:color(25,40,60));textAlign(CENTER,CENTER);textSize(14);text(label,x+w/2,y+17);textAlign(LEFT,BASELINE);
  }
  boolean hit(float x,float y,float w) { return mouseX>=x&&mouseX<=x+w&&mouseY>=y&&mouseY<=y+34; }
  void drawControls() {
    rectMode(CORNER);fill(221,228,237);noStroke();rect(0,790,width,height-790);fill(25,40,60);textSize(15);text("地图大小",22,826);
    for (int i=1;i<=4;i++) button(94+(i-1)*56,804,48,i+"米",metres==i);
    button(334,804,170,"随机停车框："+(randomFrame?"开":"关"),randomFrame);
    button(514,804,170,"趣味地图："+(enabled?"开":"关"),enabled);
    button(704,804,120,"新随机起点",false);button(834,804,120,paused?"继续":"暂停",false);
    if (enabled) {
      button(22,848,90,"外边界",tool==0);button(120,848,90,"内部墙",tool==1);
      String[] names={"手绘","圆形","椭圆","矩形","直线"};for (int i=0;i<5;i++) button(224+i*70,848,62,names[i],brush==i);
      button(582,848,62,"撤销",false);button(654,848,72,"清空墙",false);button(736,848,72,"重置",false);
      button(824,848,160,editing?"应用地图并运行":"编辑当前地图",false);
      fill(25,40,60);textSize(14);text(editing?"拖动鼠标：手绘外边界自动闭合；手绘墙线保持开放。编辑期间暂停。":"当前使用自定义地图；修改草稿后需重新应用。",22,908);
    } else { fill(25,40,60);textSize(14);text("打开趣味地图可画圆形、椭圆、异形外边界和内部墙；两个开关默认关闭。",22,876); }
    fill(100,60,35);textSize(14);text(notice==null?"":notice,22,936);
  }
  void press() {
    if (mouseY>=790) {
      for (int i=1;i<=4;i++) if (hit(94+(i-1)*56,804,48)) { int previous=metres;metres=i;if (!editing) { if(!restart())metres=previous; }else notice="地图尺寸已改为"+i+"米；应用后生效。";return; }
      if (hit(334,804,170)) { randomFrame=!randomFrame;if (!editing&&!restart())randomFrame=!randomFrame;return; }
      if (hit(514,804,170)) { toggleEditor();return; }
      if (hit(704,804,120)&&!editing) { sceneSeed++;restart();return; }
      if (hit(834,804,120)&&!editing) { paused=!paused;return; }
      if (!enabled) return;
      if (hit(824,848,160)) { if (editing) applyMap();else { editing=true;paused=true;notice="编辑期间暂停；修改完后点击应用地图并运行。"; }return; }
      if (!editing) return;
      if (hit(22,848,90)) tool=0;else if (hit(120,848,90)) tool=1;
      else if (hit(582,848,62)) undoDrawing();
      else if (hit(654,848,72)) { remember();walls.clear();notice="已清空内部墙。"; }
      else if (hit(736,848,72)) { remember();resetBoundary();walls.clear();notice="草稿已重置为矩形。"; }
      else for (int i=0;i<5;i++) if (hit(224+i*70,848,62)) brush=i;
      return;
    }
    if (!editing||!canvasHit(mouseX,mouseY)) return;
    if (tool==0&&brush==4) { notice="直线用于内部墙；外边界请用手绘或形状工具。";return; }
    strokePoints.clear();strokePoints.add(pointer());
  }
  float zoom() { return mapZoom(metres*1000.0); }
  boolean canvasHit(int x,int y) { double half=metres*1000.0*zoom()/2;return Math.abs(x-400)<=half&&Math.abs(y-415)<=half; }
  double[] pointer() {
    double scale=metres*1000.0*zoom();return new double[]{ParkingCore.clip((mouseX-400)/scale,-.5,.5),ParkingCore.clip((415-mouseY)/scale,-.5,.5)};
  }
  void drag() {
    if (!editing||strokePoints.isEmpty()) return;double[] p=pointer(),last=strokePoints.get(strokePoints.size()-1);
    if (brush==0) { if (Math.hypot(p[0]-last[0],p[1]-last[1])*metres*1000*zoom()>=4&&strokePoints.size()<512) strokePoints.add(p); }
    else { if (strokePoints.size()==1) strokePoints.add(p);else strokePoints.set(1,p); }
  }
  ArrayList<double[]> finishedStroke() {
    ArrayList<double[]> line=new ArrayList<double[]>();
    if (strokePoints.size()<2) return line;
    double[] a=strokePoints.get(0),b=strokePoints.get(strokePoints.size()-1);
    if (brush==0||brush==4) { line.addAll(strokePoints); }
    else if (brush==3) {
      double x0=Math.min(a[0],b[0]),x1=Math.max(a[0],b[0]),y0=Math.min(a[1],b[1]),y1=Math.max(a[1],b[1]);
      line.add(new double[]{x0,y0});line.add(new double[]{x1,y0});line.add(new double[]{x1,y1});line.add(new double[]{x0,y1});
    } else {
      double cx=(a[0]+b[0])/2,cy=(a[1]+b[1])/2,rx=Math.abs(b[0]-a[0])/2,ry=Math.abs(b[1]-a[1])/2;
      if (brush==1) rx=ry=Math.min(rx,ry);
      if (rx*metres*1000<20||ry*metres*1000<20) return line;
      for (int i=0;i<96;i++) { double angle=i*2*Math.PI/96;line.add(new double[]{cx+rx*Math.cos(angle),cy+ry*Math.sin(angle)}); }
    }
    if (tool==0&&line.size()>3&&Math.hypot(line.get(0)[0]-line.get(line.size()-1)[0],line.get(0)[1]-line.get(line.size()-1)[1])<.005) line.remove(line.size()-1);
    if (tool==1&&brush>=1&&brush<=3&&!line.isEmpty()) line.add(line.get(0).clone());
    return line;
  }
  void release() {
    if (!editing||strokePoints.isEmpty()) return;drag();ArrayList<double[]> line=finishedStroke();strokePoints.clear();
    if (line.size()<(tool==0?3:2)) { notice="这一笔太短，请重新画。";return; }
    double length=0;for(int i=1;i<line.size();i++)length+=Math.hypot(line.get(i)[0]-line.get(i-1)[0],line.get(i)[1]-line.get(i-1)[1]);
    if(length*metres*1000<1){notice="这一笔太短，请重新画。";return;}
    if (tool==1&&walls.size()>=64) { notice="最多64道墙；可先撤销或清空。";return; }
    String before=snapshot().toString();remember();
    if (tool==0) boundary=line;else walls.add(line);
    try { new SimulationRunner.Arena(geometry());notice=tool==0?"外边界已更新；画完点击应用运行。":"墙线已加入；画完点击应用运行。"; }
    catch(Exception e) { restore(JSONObject.parse(before));undo.removeLast();notice=e.getMessage(); }
  }
  void renderLine(ArrayList<double[]> line,double scale,boolean closed) {
    beginShape();for(double[] p:line) vertex((float)(p[0]*scale),(float)(p[1]*scale));if(closed)endShape(CLOSE);else endShape();
  }
  void renderDraft() {
    double scale=metres*1000.0;fill(255);stroke(85);strokeWeight(2/zoom());renderLine(boundary,scale,true);
    noFill();stroke(125,75,55);strokeWeight((float)(2/zoom()+10));for(ArrayList<double[]> line:walls)renderLine(line,scale,false);
    if (!strokePoints.isEmpty()) {
      stroke(40,125,190);strokeWeight(2/zoom());ArrayList<double[]> line=finishedStroke();
      if(line.isEmpty()) line=strokePoints;renderLine(line,scale,tool==0);
    }
  }
}
