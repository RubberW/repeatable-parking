import java.io.*;import java.nio.file.*;import java.util.*;
/** Cross-runtime numeric parity, rotation/reflection invariance and activation branches. */
public final class MotionChecks {
 static void check(boolean b,String s){if(!b)throw new AssertionError(s);}
 public static void main(String[] args)throws Exception{
  BoundaryMotion.Network n=new BoundaryMotion.Network(Paths.get(args[0]));double max=0;
  List<String> rows=Files.readAllLines(Paths.get(args[1]));for(String row:rows.subList(1,rows.size())){String[] c=row.split(",");double chosen=Double.parseDouble(c[0]),original=Double.parseDouble(c[1]);double[] p=n.predict(chosen,original),r=n.predict(chosen+1,original+1),s=n.predict(-chosen,-original);for(int j=0;j<3;j++){max=Math.max(max,Math.abs(p[j]-Double.parseDouble(c[j+2])));check(Math.abs(p[j]-r[j])<1e-7&&Math.abs(p[j]-s[j])<1e-7,"heading invariance");}check(p[0]>=50&&p[0]<=160&&p[1]>=8&&p[1]<=40&&p[2]>=0&&p[2]<=100,"bounds");}check(max<.0002,"parity "+max);
  MotionCapture.setup();System.setProperty("parking.boundaryMotion","conditional");
  for(double angle:new double[]{10,60,90,170}){CompactSearch m=new CompactSearch();BoundaryMotion b=m.boundaryMotion;b.edge.reliable=true;b.edge.nx=Math.cos(Math.toRadians(angle));b.edge.ny=Math.sin(Math.toRadians(angle));b.edge.alpha=Math.toDegrees(Math.acos(Math.abs(b.edge.nx)));m.goalX=-400;m.lastMs=0;b.beginEstimated(m,0);check(b.active==(angle==10),"conditional branch "+angle);}
  System.out.println("Motion parity max_error="+max+"; branch/rotation/reflection checks passed");
 }
}
