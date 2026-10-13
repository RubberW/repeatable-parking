import java.nio.file.*;import java.util.*;import java.io.*;
/** Independent truth audit also checks exterior decisions the gate might miss. */
public final class HardHeadingAudit {
 public static void main(String[] args)throws Exception{
  System.setProperty("parking.learnBoundary","true");BoundarySearchServer.settings();
  CompactSearch.Network net=new CompactSearch.Network(Paths.get(args[1]));Path output=Paths.get(args[2]);if(Files.exists(output))throw new IllegalStateException("Keep previous audit");
  List<String> rows=Files.readAllLines(Paths.get(args[0]));
  try(PrintWriter out=new PrintWriter(Files.newBufferedWriter(output))){
   out.println("size,id,placement,status,time_s,checked_decisions,outward_decisions,exterior_without_gate,masked_output_violations,unsafe_admitted_candidates");
   for(String row:rows.subList(1,rows.size())){
    String[] c=row.split(",");PanelWorker.Plant p=BoundarySearchServer.scene(c);CompactSearch m=new CompactSearch();Random rng=new Random(Long.parseLong(c[10]));m.observe(p.observe());String status="found";int checked=0,wrong=0,missed=0,maskViolations=0,unsafeAdmitted=0;
    while(!m.contact){
     if(m.invalid){status=m.guard;break;}
     if(m.needsAction(m.lastMs)){
      if(m.decisions>=4096){status="censored_decision_budget";break;}
      float[] f=RewardInputs.observation(m);int a=net.choose(f,rng,false);if(a<0){status="DIRECTION_UNCERTAIN_STOP";break;}
      if(f[CompactSearch.MAP+a*CompactSearch.FEATURES+26]>0)maskViolations++;
      boolean exterior=p.arena.clearance(p.robot.odometry.preciseX,p.robot.odometry.preciseY)<0;
      if(exterior&&!m.headingGate.active)missed++;
      if(exterior||m.edgeLatched||m.headingGate.active){checked++;if(DirectionObjective.wrong(p,m,a))wrong++;
       for(int candidate=0;candidate<32;candidate++)if(f[CompactSearch.MAP+candidate*CompactSearch.FEATURES+26]==0&&DirectionObjective.wrong(p,m,candidate))unsafeAdmitted++;
      }
      m.select(a,m.lastMs);
     }
     p.step(m.command());m.observe(p.observe());
    }
    out.printf(Locale.US,"%s,%s,%s,%s,%.3f,%d,%d,%d,%d,%d%n",c[0],c[1],c[2],status,m.lastMs*.001,checked,wrong,missed,maskViolations,unsafeAdmitted);out.flush();
   }
  }
 }
}
