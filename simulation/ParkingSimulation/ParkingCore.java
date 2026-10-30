/**
 * 文件职责：当前停车控制器与共用数据结构：提前定位、方向确认、按需横扫、停稳验收
 * 输入：反射率、编码器和停车标记的已知图案；输出：左右轮指令、停车状态和验收结果。
 * 调用关系：SimulationRunner 调用；触框前交给 CompactSearch，触框后使用 ParkingFrameProbe。
 */
import java.util.*;
import java.io.*;
import java.nio.file.*;

/** Platform-independent behaviour. No Processing, image, plant or true-pose reference. */
public final class ParkingCore {
  public static final double B = 90, MM_COUNT = 2 * Math.PI * 16 / 358.3;
  public static final double[] SENSOR_ANGLES = {45, 14, 0, -14, -45};
  public static double wrap(double a) { return Math.atan2(Math.sin(a), Math.cos(a)); }
  public static double clip(double a, double low, double high) { return Math.max(low, Math.min(high, a)); }
  public static boolean independentlyParked(Pose p,long stationaryMs,boolean reportedComplete) {
    return reportedComplete && Math.min(70-Math.abs(p.x),85-Math.abs(p.y))-50>=5
      && Math.abs(wrap(p.a-Math.PI/2))<=Math.toRadians(5) && stationaryMs>=1000;
  }

  public static final class Rect {
    public final double x0, y0, x1, y1;
    Rect(double a, double b, double c, double d) { x0=a; y0=b; x1=c; y1=d; }
    double outside(double x, double y) {
      return Math.max(Math.max(Math.max(x0-x, x-x1), Math.max(y0-y, y-y1)), 0);
    }
    double inside(double x, double y) { return Math.max(0, Math.min(Math.min(x-x0,x1-x), Math.min(y-y0,y1-y))); }
  }

  /** Known printed landmark dimensions, not the environment's raster image. */
  public static final class MapDescription {
    public final List<Rect> ink = new ArrayList<>();
    public MapDescription() {
      ink.add(new Rect(-76,-91,-70,91)); ink.add(new Rect(70,-91,76,91));
      ink.add(new Rect(-76,-91,76,-85)); ink.add(new Rect(-76,85,76,91));
      {
        ink.add(new Rect(-8,-91,8,65));
        ink.add(new Rect(-50,-54,50,-46));
        ink.add(new Rect(-50,-23,50,-7));
        ink.add(new Rect(-50,41,50,49));
      }
    }
    public boolean black(double x, double y) {
      for (Rect r : ink) if (x>=r.x0 && x<r.x1 && y>r.y0 && y<=r.y1) return true;
      return false;
    }
    double outside(double x, double y) {
      if (Math.abs(x)>101 || Math.abs(y)>116) return Math.max(Math.abs(x)-76,Math.abs(y)-91);
      double d=1e9; for(Rect r:ink) d=Math.min(d,r.outside(x,y)); return d;
    }
    double inside(double x, double y) { double d=0; for(Rect r:ink) d=Math.max(d,r.inside(x,y)); return d; }
    List<double[]> anchorLocations() {
      List<double[]> a=new ArrayList<>();
      for(Rect r:ink) {
        if(r.x1-r.x0 < r.y1-r.y0) {
          for(double y=r.y0;y<=r.y1;y+=6) a.add(new double[]{(r.x0+r.x1)/2,y});
        } else {
          for(double x=r.x0;x<=r.x1;x+=6) a.add(new double[]{x,(r.y0+r.y1)/2});
        }
      }
      return a;
    }
  }

  /** The complete input capability granted to the controller. */
  public static final class Observation {
    public final long millis, leftCount, rightCount;
    private final float[] values;
    public Observation(long t, long l, long r, float[] sensors) {
      if(sensors.length!=5) throw new IllegalArgumentException("Five sensors required");
      millis=t;leftCount=l;rightCount=r;values=sensors.clone();
    }
    public float sensor(int i) { return values[i]; }
  }
  public static final class Command {
    public final float left, right;
    // Robot.cpp casts to int16_t before transmitting PWM: truncate identically.
    Command(double l,double r) { left=(int)clip(l,-200,200);right=(int)clip(r,-200,200); }
  }
  /** Sensor bands are calibrated interface units, never a conversion to distance.
   * The defaults describe synthetic, separable readings; hardware needs five measured bands.
   */
  public static final class Calibration {
    public final double[][] white,ink,edge;
    public Calibration(double edgeValue) {
      white=new double[5][2];ink=new double[5][2];edge=new double[5][2];
      for(int i=0;i<5;i++){white[i]=new double[]{300,500};ink[i]=new double[]{1850,2150};edge[i]=new double[]{edgeValue-100,edgeValue+100};}
    }
    public boolean separable(){
      for(int i=0;i<5;i++)if(overlap(white[i],ink[i])||overlap(white[i],edge[i])||overlap(ink[i],edge[i]))return false;
      return true;
    }
    private boolean overlap(double[] a,double[] b){return Math.max(a[0],b[0])<=Math.min(a[1],b[1]);}
    public int kind(int i,double v){
      if(v>=white[i][0] && v<=white[i][1])return 0;
      if(v>=ink[i][0] && v<=ink[i][1])return 1;
      if(v>=edge[i][0] && v<=edge[i][1])return 2;
      return 3;
    }
  }
  public static final class Pose {
    public double x,y,a;
    public Pose(double x,double y,double a) { this.x=x;this.y=y;this.a=a; }
  }
  static final class Point {
    final double x,y; final boolean black;
    Point(double x,double y,boolean b){this.x=x;this.y=y;black=b;}
  }
  static final class Fit {
    double x,y,a,score;
    Fit(double x,double y,double a,double score){this.x=x;this.y=y;this.a=a;this.score=score;}
    Pose apply(Pose p){double c=Math.cos(a),s=Math.sin(a);return new Pose(x+c*p.x-s*p.y,y+s*p.x+c*p.y,wrap(a+p.a));}
    boolean near(Fit f,double position,double angle){return Math.hypot(x-f.x,y-f.y)<position && Math.abs(wrap(a-f.a))<angle;}
  }

  /** Correlative geometric landmark matching with subsequent independent code verification. */
  static final class Localizer {
    final MapDescription map; final List<double[]> anchors;
    final List<Point> black=new ArrayList<>(), white=new ArrayList<>();
    int fits; double bestScore=1e9, alternateScore=1e9;
    Localizer(MapDescription m){map=m;anchors=m.anchorLocations();}
    void add(Observation o,Pose p) {
      double c=Math.cos(p.a),s=Math.sin(p.a);
      for(int i=0;i<5;i++){
        double ang=Math.toRadians(SENSOR_ANGLES[i]),sx=45*Math.cos(ang),sy=45*Math.sin(ang);
        if(o.sensor(i)!=400 && o.sensor(i)!=2000)continue;
        Point pt=new Point(p.x+c*sx-s*sy,p.y+s*sx+c*sy,o.sensor(i)==2000);
        (pt.black?black:white).add(pt);
      }
    }
    List<Point> sample(List<Point> src,int n) {
      List<Point> result=new ArrayList<>();
      if(src.size()<=n) return src;
      for(int i=0;i<n;i++) result.add(src.get((int)((long)i*(src.size()-1)/(n-1))));
      return result;
    }
    double cost(Fit f,List<Point> bp,List<Point> wp,double tolerance,double limit) {
      double c=Math.cos(f.a),s=Math.sin(f.a),sum=0;
      for(Point p:bp) {
        double d=Math.max(0,map.outside(f.x+c*p.x-s*p.y,f.y+s*p.x+c*p.y)-tolerance);
        sum+=Math.min(625,d*d);
        if(sum>limit) return sum;
      }
      double weight=(double)bp.size()/Math.max(1,wp.size());
      for(Point p:wp) {
        double d=Math.max(0,map.inside(f.x+c*p.x-s*p.y,f.y+s*p.x+c*p.y)-tolerance);
        sum+=weight*Math.min(625,d*d);
        if(sum>limit) return sum;
      }
      return sum;
    }
    Fit refine(Fit f,List<Point> bp,List<Point> wp) {
      f=new Fit(f.x,f.y,f.a,0);
      double[] ds={8,4,2,1,.5,.25};
      double[] da={Math.toRadians(3),Math.toRadians(1.5),Math.toRadians(.75),Math.toRadians(.3),Math.toRadians(.15),Math.toRadians(.06)};
      f.score=cost(f,bp,wp,1,Double.POSITIVE_INFINITY);
      for(int k=0;k<ds.length;k++){
        for(int iteration=0;iteration<25;iteration++) {
          boolean changed=false;
          for(int axis=0;axis<3;axis++) for(int sign:new int[]{-1,1}) {
            Fit g=new Fit(f.x+(axis==0?ds[k]*sign:0),f.y+(axis==1?ds[k]*sign:0),f.a+(axis==2?da[k]*sign:0),0);
            g.score=cost(g,bp,wp,1,f.score);
            if(g.score+1e-8<f.score){f=g;changed=true;}
          }
          if(!changed) break;
        }
      }
      f.a=wrap(f.a);f.score/=Math.max(1,bp.size());return f;
    }
    Fit solve() {
      if(black.size()<35) return null;
      Point first=black.get(0);
      double span=0; for(Point p:black) span=Math.max(span,Math.hypot(p.x-first.x,p.y-first.y));
      if(span<90) return null;
      fits++;
      List<Point> bp=sample(black,32),wp=sample(white,48);
      List<Fit> best=new ArrayList<>(); double worst=Double.POSITIVE_INFINITY;
      for(int angle=0;angle<120;angle++) {
        double a=angle*Math.PI/60,c=Math.cos(a),s=Math.sin(a);
        for(double[] anchor:anchors) {
          Fit f=new Fit(anchor[0]-c*first.x+s*first.y,anchor[1]-s*first.x-c*first.y,a,0);
          f.score=cost(f,bp,wp,7,worst);
          if(f.score>worst) continue;
          int near=-1;
          for(int k=0;k<best.size();k++)if(f.near(best.get(k),18,Math.toRadians(8))){near=k;break;}
          if(near>=0) {if(f.score<best.get(near).score)best.set(near,f);}
          else best.add(f);
          best.sort(Comparator.comparingDouble(g->g.score));
          if(best.size()>96)best.remove(best.size()-1);
          if(best.size()==96)worst=best.get(95).score;
        }
      }
      bp=sample(black,160);wp=sample(white,240);
      List<Fit> fine=new ArrayList<>();
      for(Fit f:best) fine.add(refine(f,bp,wp));
      fine.sort(Comparator.comparingDouble(f->f.score));
      Fit f=fine.get(0);bestScore=f.score;alternateScore=1e9;
      for(Fit g:fine)if(!f.near(g,30,Math.toRadians(10))){alternateScore=g.score;break;}
      // A fit remains provisional until the unequal-width three-bar code is measured.
      return f.score<2.5 ? f : null;
    }
  }

  public static final class Controller {
    public enum State { SEARCH, TRACE_EDGE, GO_ENTRY, ALIGN_ENTRY, VERIFY_CODE, SCAN_APPROACH, SCAN_ALIGN, SCAN_CROSS, GO_TARGET, ALIGN_TARGET, SETTLE, SUCCESS, FAILED }
    public State state=State.SEARCH;
    public final Pose local=new Pose(0,0,0);
    public boolean localized=false;
    public int attempts=0, rejects=0, codeEvents=0;
    public double travelled=0, score=1e9, alternate=1e9;
    public long localizationMs=-1, completionMs=-1;
    public long firstBlackMs=-1, firstFitMs=-1;
    public double lastBarWidth=Double.NaN, lastBarY=Double.NaN;
    public boolean headingCalibrated=false;
    public boolean lateralCalibrated=false;
    public double lateralCorrection=Double.NaN;
    public int edgeEvents=0;
    public String guardState="CLEAR";
    public final Calibration calibration;
    public String failure="", event="reset_unknown_pose";
    final Localizer locator;
    private Fit transform;
    public double searchPathMm=0,searchRepeatedMm=0;
    public int searchUnique=0;
    private CompactSearch compact;private CompactSearch.Network compactNetwork;
    private final boolean localBoundary;
    private final Random policyRandom=new Random(Long.getLong("parking.policySeed",730001L));
    private long previousTime=-1,leftBase,rightBase,lastFit=-10000,settleStart=-1;
    private double lastSampleX=1e9,lastSampleY=1e9,lastSampleA=1e9;
    private boolean bar=false;
    private double barStart, lastBarCentre=Double.NaN, lastCodeTravel, verifyStartTravel;
    private final List<Double> widths=new ArrayList<>();
    private final List<double[]> codeEdges=new ArrayList<>();
    private final boolean[] previousBlack=new boolean[5];
    private Pose previousCodePose;
    private int avoidance=0;
    private double turnGoal,scanStart=0;
    private boolean scanBlack=false;
    private Pose scanEnter;
    private final List<Command> commandHistory=new ArrayList<>();
    private int replayIndex=-1;
    private ParkingFrameProbe tracer;
    public boolean skippedLateralScan=false;
    private long lastEncoderMotionMs=0;
    boolean lateralEvidenceConfident(){
      // After independently validating the asymmetric code, retain a lateral
      // scan only when shifted poses are still compatible with collected ink.
      List<Point> bp=locator.sample(locator.black,160),wp=locator.sample(locator.white,240);
      double base=locator.cost(transform,bp,wp,1,Double.POSITIVE_INFINITY)/Math.max(1,bp.size());
      for(int sign:new int[]{-1,1}){
        Fit shifted=new Fit(transform.x+12*sign,transform.y,transform.a,0);
        double cost=locator.cost(shifted,bp,wp,1,Double.POSITIVE_INFINITY)/Math.max(1,bp.size());
        if(cost<base+2.5)return false;
      }
      return true;
    }
    public int cornerEvents=0,observedCells=0;
    public long localScanStart=-1;
    // 入口点在框外115毫米。框靠桌边且入口一端朝边时，小车无法站到入口点，原逻辑会无限重试同一入口。
    // 方向条纹间距不对称，从两端都能读出方向：入口或横扫途中触边，下次改从另一端进入/横扫。
    final boolean adaptiveEntry=Boolean.parseBoolean(System.getProperty("parking.adaptiveEntry","false"));
    public int entrySide=1,scanSide=1,sideSwitches=0;
    // 直接停车：拟合出框后不去固定入口，直接开到停车点。沿途每个新采样按当前拟合预测黑白并计数矛盾；
    // 每走30毫米用拟合后的黑白跳变对齐真实墨迹边界修正位置和航向；到点后横向证据不足、或转正后跳变太少/残差大，
    // 退回入口+条纹验证；最后查五路传感器特征（可区分180°倒转）。
    final boolean directApproach=Boolean.parseBoolean(System.getProperty("parking.directApproach","false"));
    final int directMismatch=Integer.getInteger("parking.directMismatch",3);
    public boolean direct=false;
    public int directAttempts=0,directRejects=0,directFallbacks=0,directRefits=0,directChecks=0,directBad=0;
    // 8条/1.5毫米：调参集192次按事先规则（0误报停车，平均局部用时最短）在 12/1.0、10/1.5、8/1.5 中选出。
    final int directMinEdges=Integer.getInteger("parking.directMinEdges",8);
    final double directMaxRms=Double.parseDouble(System.getProperty("parking.directMaxRms","1.5"));
    public int edgeMatched=0,edgeUnmatched=0;public double edgeRms=1e9;
    private double lastRefineTravel;
    private final List<double[]> directEdges=new ArrayList<>();
    private Pose previousDirectPose;
    private final boolean[] directPreviousBlack=new boolean[5];

    public Controller(MapDescription knownMap,Calibration calibration) { this(knownMap,calibration,false); }
    public Controller(MapDescription knownMap,Calibration calibration,boolean localBoundary) { locator=new Localizer(knownMap);this.calibration=calibration;this.localBoundary=localBoundary; }
    public Pose searchOdometryPose(){return compact==null?null:compact.pose;}
    public int retainedHistorySize(){return commandHistory.size()+locator.black.size()+locator.white.size();}
    public Pose estimatedMapPose(){return transform==null?null:transform.apply(local);}
    public int blackSamples(){return locator.black.size();}
    public boolean done(){return state==State.SUCCESS || state==State.FAILED;}
    public void fail(String reason){state=State.FAILED;failure=reason;event="failure:"+reason;}
    private Command velocity(double v,double w){return new Command((v-B*w/2)/1.25,(v+B*w/2)/1.25);}
    private Command stop(){return new Command(0,0);}
    private void odometry(Observation o) {
      if(previousTime<0){leftBase=o.leftCount;rightBase=o.rightCount;previousTime=o.millis;return;}
      if(o.leftCount!=leftBase||o.rightCount!=rightBase)lastEncoderMotionMs=o.millis;
      double l=(o.leftCount-leftBase)*MM_COUNT,r=(o.rightCount-rightBase)*MM_COUNT;
      double distance=(l+r)/2;
      local.x+=distance*Math.cos(local.a);local.y+=distance*Math.sin(local.a);
      local.a=wrap(local.a+(r-l)/B);travelled+=Math.abs(distance);
      leftBase=o.leftCount;rightBase=o.rightCount;previousTime=o.millis;
    }
    private Command point(Pose p,double x,double y,double speed,boolean reverse) {
      double d=Math.hypot(x-p.x,y-p.y),bearing=Math.atan2(y-p.y,x-p.x);
      int sign=1;
      if(reverse && Math.abs(wrap(bearing-p.a))>Math.PI/2){sign=-1;bearing=wrap(bearing+Math.PI);}
      double e=wrap(bearing-p.a);
      if(Math.abs(e)>.075) return velocity(0,clip(2.5*e,-1.2,1.2));
      return velocity(sign*Math.min(speed,Math.max(5,1.8*d)),clip(2*e,-.5,.5));
    }
    private Command guard(Observation raw){
      if(!calibration.separable()){fail("edge_ink_calibration_overlap");return stop();}
      boolean edge=false;
      for(int i=0;i<5;i++){
        int k=calibration.kind(i,raw.sensor(i));
        if(k==3){fail("uncertain_reflectance_stop");return stop();}
        if(k==2)edge=true;
      }
      if(avoidance==0 && edge){
        edgeEvents++;avoidance=1;replayIndex=commandHistory.size()-1;guardState="BRAKE";
        if(state!=State.SEARCH && state!=State.TRACE_EDGE){
          if(adaptiveEntry){
            if(state==State.GO_ENTRY||state==State.ALIGN_ENTRY||state==State.VERIFY_CODE){entrySide=-entrySide;sideSwitches++;}
            else if(state==State.SCAN_APPROACH||state==State.SCAN_ALIGN||state==State.SCAN_CROSS){scanSide=-scanSide;sideSwitches++;}
          }
          reject(raw.millis,"edge_interrupted_approach");
        }
        event="edge_brake";return stop();
      }
      if(avoidance==1){
        guardState="RETRACE";
        // Reverse the actual command history to the episode's clear start.
        // A fixed straight reverse can hit another side of a small table.
        // This relies on the initial placement being clear and reversible dynamics.
        if(replayIndex<0){commandHistory.clear();avoidance=2;turnGoal=wrap(local.a+Math.toRadians(137.5));return stop();}
        Command previous=commandHistory.get(replayIndex--);return new Command(-previous.left,-previous.right);
      }
      if(avoidance==2){
        guardState="TURN_INWARD";double e=wrap(turnGoal-local.a);
        if(Math.abs(e)<.04){avoidance=0;guardState="CLEAR";event="edge_avoidance_complete";return stop();}
        return velocity(0,clip(2.5*e,-1.2,1.2));
      }
      return null;
    }
    private void reject(long now,String why) {
      rejects++;localized=false;state=tracer!=null?State.TRACE_EDGE:State.SEARCH;transform=null;lastFit=now;direct=false;
      bar=false;codeEvents=0;headingCalibrated=false;lateralCalibrated=false;event="rejected:"+why;
    }
    private void recordEdges(Observation o,Pose mapPose) {
      if(previousCodePose!=null){
        Pose midpoint=new Pose((previousCodePose.x+local.x)/2,(previousCodePose.y+local.y)/2,
          previousCodePose.a+wrap(local.a-previousCodePose.a)/2);
        double c=Math.cos(midpoint.a),s=Math.sin(midpoint.a);
        for(int i:new int[]{0,1,3,4}) {
          boolean black=o.sensor(i)>=1400;
          if(black!=previousBlack[i]){
            double angle=Math.toRadians(SENSOR_ANGLES[i]),sx=45*Math.cos(angle),sy=45*Math.sin(angle);
            Pose sensorLocal=new Pose(midpoint.x+c*sx-s*sy,midpoint.y+s*sx+c*sy,0);
            Pose sensorMap=transform.apply(sensorLocal);
            // 沿+y行驶时进入条纹的是低y边；从另一端（沿-y）进入时是高y边。
            double[] edges=black==(entrySide>0)?new double[]{-54,-23,41}:new double[]{-46,-7,49};
            double best=1e9,edge=0;
            for(double e:edges)if(Math.abs(sensorMap.y-e)<best){best=Math.abs(sensorMap.y-e);edge=e;}
            if(best<8 && Math.abs(sensorMap.x)<47)codeEdges.add(new double[]{sensorLocal.x,sensorLocal.y,edge});
          }
        }
      }
      previousCodePose=new Pose(local.x,local.y,local.a);
      for(int i=0;i<5;i++)previousBlack[i]=o.sensor(i)>=1400;
    }
    private void calibrateFromEdges() {
      if(codeEdges.size()<12)return;
      double x=0,y=0,z=0;
      for(double[] p:codeEdges){x+=p[0];y+=p[1];z+=p[2];}
      x/=codeEdges.size();y/=codeEdges.size();z/=codeEdges.size();
      double xx=0,xy=0,yy=0,xz=0,yz=0;
      for(double[] p:codeEdges){double dx=p[0]-x,dy=p[1]-y,dz=p[2]-z;xx+=dx*dx;xy+=dx*dy;yy+=dy*dy;xz+=dx*dz;yz+=dy*dz;}
      double determinant=xx*yy-xy*xy;
      if(determinant<1)return;
      double a=(xz*yy-yz*xy)/determinant,b=(yz*xx-xz*xy)/determinant;
      double angle=Math.atan2(a,b);
      if(Math.abs(Math.hypot(a,b)-1)>.04 || Math.abs(wrap(angle-transform.a))>Math.toRadians(12))return;
      double residual=0;
      for(double[] p:codeEdges){double e=a*(p[0]-x)+b*(p[1]-y)-(p[2]-z);residual+=e*e;}
      if(Math.sqrt(residual/codeEdges.size())>1)return;
      Pose before=transform.apply(local);
      transform=new Fit(before.x-Math.cos(angle)*local.x+Math.sin(angle)*local.y,
        z-Math.sin(angle)*x-Math.cos(angle)*y,angle,transform.score);
      headingCalibrated=true;
    }
    // 拟合之后的新采样不参与该拟合，是独立证据：黑点离墨迹超6毫米或白点深入墨迹超3毫米记一次矛盾。
    private void checkDirect(Observation o) {
      double c=Math.cos(local.a),s=Math.sin(local.a);
      for(int i=0;i<5;i++){
        double angle=Math.toRadians(SENSOR_ANGLES[i]),sx=45*Math.cos(angle),sy=45*Math.sin(angle);
        Pose m=transform.apply(new Pose(local.x+c*sx-s*sy,local.y+s*sx+c*sy,0));
        boolean black=o.sensor(i)>=1400;
        directChecks++;
        if(black?locator.map.outside(m.x,m.y)>6:locator.map.inside(m.x,m.y)>3)directBad++;
      }
    }
    // 记录拟合之后每路传感器的黑白跳变（本地坐标、运动方向、跳变后颜色）。只用这段短路程，里程计漂移可忽略。
    private void recordDirectEdges(Observation o) {
      if(previousDirectPose!=null){
        Pose q=previousDirectPose,mid=new Pose((q.x+local.x)/2,(q.y+local.y)/2,q.a+wrap(local.a-q.a)/2);
        for(int i=0;i<5;i++){
          boolean black=o.sensor(i)>=1400;
          if(black==directPreviousBlack[i])continue;
          double angle=Math.toRadians(SENSOR_ANGLES[i]),sx=45*Math.cos(angle),sy=45*Math.sin(angle);
          double[] m=sensorAt(mid,sx,sy),before=sensorAt(q,sx,sy),after=sensorAt(local,sx,sy);
          directEdges.add(new double[]{m[0],m[1],after[0]-before[0],after[1]-before[1],black?1:0});
        }
      }
      previousDirectPose=new Pose(local.x,local.y,local.a);
      for(int i=0;i<5;i++)directPreviousBlack[i]=o.sensor(i)>=1400;
    }
    // 用跳变修正当前拟合；修正量过大说明拟合落在错误的极小值上，返回拒绝原因。
    // 未匹配跳变数只作记录：调参集中正确拟合也常有十几个（线端、角点、擦边），不能区分对错。
    private String correctDirect() {
      if(directEdges.size()<6)return null;
      Fit g=edgeCorrection(transform);Pose before=transform.apply(local),after=g.apply(local);
      if(Math.hypot(after.x-before.x,after.y-before.y)>15||Math.abs(wrap(after.a-before.a))>Math.toRadians(8))return "direct_refit_jump";
      if(edgeMatched>=6)transform=g;
      return null;
    }
    private static double[] solve3(double[][] a,double[] b){
      double det=a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])-a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])+a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]);
      double[] x=new double[3];
      for(int k=0;k<3;k++){
        double[][] m=new double[3][];for(int r=0;r<3;r++){m[r]=a[r].clone();m[r][k]=b[r];}
        x[k]=(m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]))/det;
      }
      return x;
    }
    private static double[] sensorAt(Pose p,double sx,double sy){double c=Math.cos(p.a),s=Math.sin(p.a);return new double[]{p.x+c*sx-s*sy,p.y+s*sx+c*sy};}
    /** 每个跳变匹配到真实墨迹边界（两侧颜色不同且与跳变方向一致，6毫米内），解点到线最小二乘的平移与绕车心转角。
     *  弱约束方向加小正则保持原值。返回修正后的拟合；edgeMatched/edgeUnmatched/edgeRms 记录本次结果。 */
    private Fit edgeCorrection(Fit f) {
      Fit g=f;
      for(int iteration=0;iteration<5;iteration++){
        Pose robot=g.apply(local);double c=Math.cos(g.a),s=Math.sin(g.a);
        double[][] A={{.5,0,0},{0,.5,0},{0,0,.5*45*45}};double[] b=new double[3];
        int matched=0,unmatched=0;double squares=0;
        for(double[] e:directEdges){
          double mx=g.x+c*e[0]-s*e[1],my=g.y+s*e[0]+c*e[1],vx=c*e[2]-s*e[3],vy=s*e[2]+c*e[3],speed=Math.hypot(vx,vy);
          double best=6,nx=0,ny=0,signed=0;boolean found=false;
          if(speed>1e-6)for(Rect r:locator.map.ink)for(int k=0;k<4;k++){
            boolean vertical=k<2;double line=k==0?r.x0:k==1?r.x1:k==2?r.y0:r.y1;
            double along=vertical?my:mx,lo=vertical?r.y0:r.x0,hi=vertical?r.y1:r.x1;
            if(along<lo-3||along>hi+3)continue;
            double residual=(vertical?mx:my)-line,ux=vertical?1:0,uy=vertical?0:1;
            if(Math.abs(residual)>=best)continue;
            double dot=(vx*ux+vy*uy)/speed;if(Math.abs(dot)<.2)continue;
            double side=Math.signum(dot),qx=mx-residual*ux,qy=my-residual*uy;
            boolean ahead=locator.map.black(qx+side*1.5*ux,qy+side*1.5*uy),behind=locator.map.black(qx-side*1.5*ux,qy-side*1.5*uy);
            if(ahead==behind||ahead!=(e[4]>0))continue;
            best=Math.abs(residual);signed=residual;nx=ux;ny=uy;found=true;
          }
          if(!found){unmatched++;continue;}
          double residual=signed;
          double[] j={nx,ny,nx*-(my-robot.y)+ny*(mx-robot.x)};
          for(int u=0;u<3;u++){b[u]-=j[u]*residual;for(int v=0;v<3;v++)A[u][v]+=j[u]*j[v];}
          matched++;squares+=residual*residual;
        }
        edgeMatched=matched;edgeUnmatched=unmatched;edgeRms=matched>0?Math.sqrt(squares/matched):1e9;
        double[] d=solve3(A,b);
        double ca=Math.cos(d[2]),sa=Math.sin(d[2]),tx=g.x-robot.x,ty=g.y-robot.y;
        g=new Fit(robot.x+ca*tx-sa*ty+d[0],robot.y+sa*tx+ca*ty+d[1],wrap(g.a+d[2]),g.score);
        if(Math.abs(d[0])+Math.abs(d[1])<.02&&Math.abs(d[2])<1e-4)break;
      }
      return g;
    }
    private boolean parkSignature(Observation o) {
      // 停车位姿下：中间三路压在最上方横条，两侧45°两路落在白区；倒转180°时±14°两路为白。
      return o.sensor(1)>=1400 && o.sensor(2)>=1400 && o.sensor(3)>=1400 && o.sensor(0)<1400 && o.sensor(4)<1400;
    }
    public Command update(Observation o) {
      Command result=updateBehavior(o);
      if(avoidance==0 && !done() && state!=State.SEARCH)commandHistory.add(result);
      return result;
    }
    private Command updateBehavior(Observation o) {
      if(previousTime>=0 && o.millis<=previousTime){fail("stale_observation");return stop();}
      odometry(o);
      if(done())return stop();
      for(int i=0;i<5;i++)if(!Float.isFinite(o.sensor(i))){fail("invalid_sensor");return stop();}
      // Search runs to first contact; no total-time deadline or timeout score.
      if(state==State.SEARCH && firstBlackMs<0){
        if(compact==null){compact=new CompactSearch(localBoundary);try{compactNetwork=new CompactSearch.Network(Paths.get(System.getProperty("parking.policy",System.getProperty("parking.root",".")+"/models/exploration/policy.bin")));}catch(IOException e){throw new RuntimeException(e);}}
        compact.observe(o);observedCells=compact.unique;edgeEvents=compact.edges;searchPathMm=compact.distance;searchRepeatedMm=compact.repeatedDistance;searchUnique=compact.unique;
        if(compact.invalid){
          if(localBoundary){guardState=compact.guard;fail(compact.guard.equals("LOCAL_DIRECTION_UNCERTAIN_STOP")?"local_direction_uncertain_stop":compact.guard.equals("BOUNDARY_MOTION_STALL")?"boundary_motion_stall":"uncertain_reflectance_stop");}
          else fail("uncertain_reflectance_stop");return stop();
        }
        if(!compact.contact){
          // Match the training adapter's maximum 8-second advance boundary, including
          // a sampled-but-ignored action if clearance recovery spans that boundary.
          if(compact.needsAction(o.millis)){int action=SearchDecision.choose(compact,compactNetwork,policyRandom);compact.select(action,o.millis);}
          Command command=compact.command();guardState=compact.guard;event="compact_subgoal_"+compact.action;return command;
        }
        guardState="CLEAR";
      }
      Command guarded=guard(o);if(guarded!=null)return guarded;
      float[] classified=new float[5];
      for(int i=0;i<5;i++)classified[i]=calibration.kind(i,o.sensor(i))==1?2000:400;
      o=new Observation(o.millis,o.leftCount,o.rightCount,classified);
      for(int i=0;i<5;i++) {
        if(!Float.isFinite(o.sensor(i))){fail("invalid_sensor");return stop();}
        if(firstBlackMs<0 && o.sensor(i)>=1400){firstBlackMs=o.millis;}
      }
      // Search runs to first contact; no total-time deadline or timeout score.
      if(Math.hypot(local.x-lastSampleX,local.y-lastSampleY)>=4 || Math.abs(wrap(local.a-lastSampleA))>=Math.toRadians(4)){
        locator.add(o,local);lastSampleX=local.x;lastSampleY=local.y;lastSampleA=local.a;
        if(direct)checkDirect(o);
      }
      if(firstBlackMs>=0 && tracer==null && state==State.SEARCH){
        tracer=new ParkingFrameProbe(local,o.millis);localScanStart=o.millis;state=State.TRACE_EDGE;
        commandHistory.clear();event="first_ink_begin_edge_trace";
      }
      Pose p=estimatedMapPose();
      if(direct)recordDirectEdges(o);
      switch(state) {
        case SEARCH:
        case TRACE_EDGE:
          if(o.millis-lastFit>=(state==State.TRACE_EDGE?500:4000) && locator.black.size()>=35) {
            lastFit=o.millis;Fit fit=locator.solve();score=locator.bestScore;alternate=locator.alternateScore;
            if(fit!=null){
              transform=fit;attempts++;if(firstFitMs<0)firstFitMs=o.millis;state=State.GO_ENTRY;event="provisional_landmark_fit";
              if(directApproach&&directRejects<2){direct=true;directAttempts++;directBad=directChecks=directRefits=0;lastRefineTravel=travelled;directEdges.clear();previousDirectPose=null;state=State.GO_TARGET;event="provisional_fit_direct_approach";}
              return stop();
            }
          }
          if(state==State.TRACE_EDGE){
            double[] vw=tracer.drive(o,local);cornerEvents=tracer.corners;event=new String[]{"acquire_first_line_direction","follow_first_line","follow_corner_second_line","local_reflectance_recovery"}[tracer.phase];
            if(o.millis-tracer.lastBlackMs>12000){fail("edge_reacquisition_timeout");return stop();}
            return velocity(vw[0],vw[1]);
          }
          return stop();
        case GO_ENTRY:
          if(Math.hypot(p.x,p.y+115*entrySide)<1.8){state=State.ALIGN_ENTRY;event="at_code_approach";return stop();}
          return point(p,0,-115*entrySide,85,false);
        case ALIGN_ENTRY:
          double e=wrap(entrySide*Math.PI/2-p.a);
          if(Math.abs(e)<Math.toRadians(.7)) {
            state=State.VERIFY_CODE;bar=false;codeEvents=0;widths.clear();lastBarCentre=Double.NaN;
            codeEdges.clear();previousCodePose=null;recordEdges(o,p);
            verifyStartTravel=travelled;event="begin_direction_code";return stop();
          }
          return velocity(0,clip(2.5*e,-1,1));
        case VERIFY_CODE:
          recordEdges(o,p);
          boolean detected=o.sensor(1)>=1400 && o.sensor(3)>=1400;
          if(detected && !bar){bar=true;barStart=travelled;lastCodeTravel=p.y;}
          if(!detected && bar){
            bar=false;double width=travelled-barStart;
            double centre=(lastCodeTravel+p.y)/2+entrySide*45*Math.cos(Math.toRadians(14));
            lastBarWidth=width;lastBarY=centre;
            double[] expected={8,16,8}, centres=entrySide>0?new double[]{-50,-15,45}:new double[]{45,-15,-50};
            if(codeEvents>=3 || Math.abs(width-expected[codeEvents])>3 || Math.abs(centre-centres[codeEvents])>8) {
              reject(o.millis,"code_width_or_position");return stop();
            }
            if(codeEvents>0 && Math.abs((centre-lastBarCentre)-(centres[codeEvents]-centres[codeEvents-1]))>8){reject(o.millis,"code_spacing");return stop();}
            widths.add(width);lastBarCentre=centre;codeEvents++;event="direction_bar_"+codeEvents;
            if(codeEvents==3){
              calibrateFromEdges();
              if(!headingCalibrated){reject(o.millis,"heading_edges_unconfirmed");return stop();}
              localized=true;localizationMs=o.millis;skippedLateralScan=lateralEvidenceConfident();state=skippedLateralScan?State.GO_TARGET:State.SCAN_APPROACH;event="direction_code_and_heading_confirmed";return stop();
            }
          }
          // 另一端进入时阈值按条纹位置镜像：+y方向的 -80/-40/15 对应 -y方向的 75/10/-20。
          double[] overdue=entrySide>0?new double[]{-80,-40,15}:new double[]{75,10,-20};
          if(codeEvents<3 && entrySide*p.y>entrySide*overdue[codeEvents]){reject(o.millis,"missing_direction_bar");return stop();}
          if(travelled-verifyStartTravel>210){reject(o.millis,"code_travel_budget");return stop();}
          return velocity(55,clip(2*wrap(entrySide*Math.PI/2-p.a)+entrySide*.045*p.x,-.3,.3));
        case SCAN_APPROACH:
          if(Math.hypot(p.x+70*scanSide,p.y-15)<1.8){state=State.SCAN_ALIGN;return stop();}
          return point(p,-70*scanSide,15,60,false);
        case SCAN_ALIGN:
          double scanError=wrap((scanSide>0?0:Math.PI)-p.a);
          if(Math.abs(scanError)<Math.toRadians(.8)){state=State.SCAN_CROSS;scanBlack=false;scanEnter=null;scanStart=travelled;event="begin_centreline_scan";return stop();}
          return velocity(0,clip(2.5*scanError,-1,1));
        case SCAN_CROSS:
          boolean onGuide=o.sensor(2)>=1400;
          if(onGuide && !scanBlack){scanBlack=true;scanEnter=new Pose(local.x,local.y,local.a);}
          if(!onGuide && scanBlack){
            double width=Math.hypot(local.x-scanEnter.x,local.y-scanEnter.y);
            if(width<11 || width>21){reject(o.millis,"centreline_width");return stop();}
            Pose midpoint=new Pose((local.x+scanEnter.x)/2+45*Math.cos(local.a),(local.y+scanEnter.y)/2+45*Math.sin(local.a),0);
            double measuredX=transform.apply(midpoint).x;
            if(Math.abs(measuredX)>30){reject(o.millis,"centreline_correction_limit");return stop();}
            transform.x-=measuredX;lateralCorrection=-measuredX;lateralCalibrated=true;state=State.GO_TARGET;event="centreline_midpoint_confirmed";return stop();
          }
          if(travelled-scanStart>100 || scanSide*p.x>10){reject(o.millis,"centreline_missing");return stop();}
          return velocity(35,clip(2*wrap((scanSide>0?0:Math.PI)-p.a),-.3,.3));
        case GO_TARGET:
          if(direct){
            if(directBad>=directMismatch){directRejects++;reject(o.millis,"direct_ink_mismatch");return stop();}
            if(travelled-lastRefineTravel>=30||Math.hypot(p.x,p.y)<1.4){
              lastRefineTravel=travelled;
              String why=correctDirect();if(why!=null){directRejects++;reject(o.millis,why);return stop();}
              p=estimatedMapPose();
            }
            if(Math.hypot(p.x,p.y)<1.4&&!lateralEvidenceConfident()){direct=false;directFallbacks++;state=State.GO_ENTRY;event="direct_lateral_unconfirmed_fallback";return stop();}
          }
          if(Math.hypot(p.x,p.y)<1.4){state=State.ALIGN_TARGET;return stop();}
          return point(p,0,0,direct?60:25,true);
        case ALIGN_TARGET:
          if(direct&&directBad>=directMismatch){directRejects++;reject(o.millis,"direct_ink_mismatch");return stop();}
          double error=wrap(Math.PI/2-p.a);
          if(Math.abs(error)<Math.toRadians(.8)){
            if(direct){
              // 原地转正时传感器扫过中线和横条，再修正一次；偏差明显就回去调整，最多6次后改走原验证路线。
              String why=correctDirect();if(why!=null){directRejects++;reject(o.millis,why);return stop();}
              if(edgeMatched<directMinEdges||edgeRms>directMaxRms){direct=false;directFallbacks++;state=State.GO_ENTRY;event="direct_edges_insufficient_fallback";return stop();}
              // 修正后偏差小于3毫米/1.5°就不再调整：静止时这么小的误差对应约1个PWM，电机推不动，会卡在原地（固定64场景7）。
              Pose q=estimatedMapPose();
              if(Math.hypot(q.x,q.y)>=3||Math.abs(wrap(Math.PI/2-q.a))>=Math.toRadians(1.5)){
                if(++directRefits>6){direct=false;directFallbacks++;state=State.GO_ENTRY;event="direct_refit_limit_fallback";return stop();}
                if(Math.hypot(q.x,q.y)>=3)state=State.GO_TARGET;
                event="direct_refit";return stop();
              }
              if(!parkSignature(o)){directRejects++;reject(o.millis,"direct_park_signature");return stop();}
              localized=true;localizationMs=o.millis;
            }
            state=State.SETTLE;settleStart=o.millis;event="zero_pwm_settle";return stop();
          }
          // 直接停车的修正可能让车静止时还差一两度，给转向一个最小角速度（约10 PWM），避免推不动。
          if(direct)return velocity(0,Math.copySign(clip(Math.abs(2.5*error),.3,.6),error));
          return velocity(0,clip(2.5*error,-.6,.6));
        case SETTLE:
          if(o.millis-lastEncoderMotionMs>=1200&&o.millis-settleStart>=1200){state=State.SUCCESS;completionMs=o.millis;event="controller_success";}
          return stop();
        default:return stop();
      }
    }
  }

  /** Truth is available solely to the experiment owner and independent evaluator. */
  public interface Plant {
    Observation observe();
    void step(Command command);
    Pose truth();
    void stop();
    default boolean supportLost(){return false;}
    // 只传用户选择的边界处理模式，不提供场地尺寸、轮廓或墙的位置。
    default boolean usesLocalBoundaryEvidence(){return false;}
  }
  public static final class Trial {
    public final Plant plant;
    public final Controller controller;
    public String id,stratum;
    public boolean success=false,falsePositive=false;
    public double pathLength=0,maxExcursion=0,finalError,headingError,margin, maxPoseError=0;
    public long stationaryMs=0,steps=0;
    private Pose previous;
    public Command command=new Command(0,0);

    public Trial(Plant plant,MapDescription map,String id,String stratum,Calibration calibration) {
      this.plant=plant;controller=new Controller(map,calibration,plant.usesLocalBoundaryEvidence());this.id=id;this.stratum=stratum;previous=plant.truth();
    }
    public void tick() {
      Observation o=plant.observe();
      command=controller.update(o);plant.step(command);Pose truth=plant.truth();
      if(plant.supportLost())controller.fail("wheel_support_lost");
      double move=Math.hypot(truth.x-previous.x,truth.y-previous.y),turn=wrap(truth.a-previous.a);
      pathLength+=move;maxExcursion=Math.max(maxExcursion,Math.max(Math.abs(truth.x),Math.abs(truth.y)));
      if(move/.01<=2 && Math.abs(turn)/.01<=Math.toRadians(1))stationaryMs+=10;else stationaryMs=0;
      // The safety guard terminates a trial; it never guides the controller.
      if(!Double.isFinite(truth.x)||!Double.isFinite(truth.y)||Math.abs(truth.x)>Double.parseDouble(System.getProperty("parking.runawayMm","20000")) || Math.abs(truth.y)>Double.parseDouble(System.getProperty("parking.runawayMm","20000")))controller.fail("numerical_runaway");
      Pose estimate=controller.estimatedMapPose();
      if(controller.localized && estimate!=null)maxPoseError=Math.max(maxPoseError,Math.hypot(truth.x-estimate.x,truth.y-estimate.y));
      steps++;previous=truth;
      if(controller.done()){
        plant.stop();
        finalError=Math.hypot(truth.x,truth.y);headingError=Math.toDegrees(Math.abs(wrap(truth.a-Math.PI/2)));
        margin=Math.min(70-Math.abs(truth.x),85-Math.abs(truth.y))-50;
        success=independentlyParked(truth,stationaryMs,controller.state==Controller.State.SUCCESS);
        falsePositive=controller.state==Controller.State.SUCCESS && !success;
      }
    }
  }
}
