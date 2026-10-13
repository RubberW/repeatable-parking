import java.io.*;import java.nio.file.*;import java.util.*;
/** Finite evidence collection, never an artificial timeout reward or completion time. */
public final class BoundaryEval {
 public static void main(String[] args)throws Exception{
  System.setProperty("parking.hardHeading",String.valueOf(!args[3].equals("unmasked")));System.setProperty("parking.learnBoundary",String.valueOf(!args[3].equals("baseline")));BoundarySearchServer.settings();
  Path input=Paths.get(args[0]),weights=Paths.get(args[1]),output=Paths.get(args[2]);if(Files.exists(output))throw new IllegalStateException("Preserve evidence");
  CompactSearch.Network net=new CompactSearch.Network(weights);List<String> rows=Files.readAllLines(input);
  try(PrintWriter out=new PrintWriter(Files.newBufferedWriter(output))){out.println("size,id,placement,status,time_s,edges,similar_fraction,distance_mm,policy_decisions,wheel_crossings,max_wheel_excursion_mm,outside_seconds,boundary_cost,partial_seconds,full_wheel_seconds,critical_seconds,critical_events,boundary_decisions,outward_choices,inward_choices,outward_probability_sum");
   int n=0;for(String row:rows.subList(1,rows.size())){
    String[] c=row.split(",");PanelWorker.Plant plant=BoundarySearchServer.scene(c);CompactSearch m=new CompactSearch();Random rng=new Random(Long.parseLong(c[10]));m.observe(plant.observe());String status="found";double outside=0,probabilitySum=0;int boundaryDecisions=0,outwardChoices=0,inwardChoices=0;
    while(!m.contact){
     if(m.invalid){status=m.guard.startsWith("DIRECTION")?m.guard:"invalid_sensor_or_memory_extent";break;}if(m.edgeSafety!=null&&m.edgeSafety.halted){status=m.guard;break;}
     if(m.needsAction(m.lastMs)){
      if(m.decisions>=4096){status="censored_decision_budget";break;}
      int action=SearchDecision.choose(m,net,rng);
      if(action<0){status="DIRECTION_UNCERTAIN_STOP";break;}
      if(DirectionObjective.boundary(m)){
       boundaryDecisions++;double progress=DirectionObjective.progress(plant,m,action);
       if(DirectionObjective.wrong(plant,m,action))outwardChoices++;else inwardChoices++;
       probabilitySum+=DirectionObjective.outwardProbability(plant,m,net.scores(RewardInputs.observation(m)));
      }
      m.select(action,m.lastMs);
     }
     plant.step(m.command());m.observe(plant.observe());if(plant.currentWheel<0)outside+=.01;
    }
    out.printf(Locale.US,"%s,%s,%s,%s,%.3f,%d,%.6f,%.3f,%d,%d,%.4f,%.3f,%.6f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%.8f%n",c[0],c[1],c[2],status,m.lastMs*.001,m.edges,m.memory.similarDistance/Math.max(1,m.distance),m.distance,m.decisions,plant.crossings,Math.max(0,-plant.minWheel),outside,plant.risk.totalCost,plant.risk.partialSeconds,plant.risk.fullWheelSeconds,plant.risk.criticalSeconds,plant.risk.criticalEvents,boundaryDecisions,outwardChoices,inwardChoices,probabilitySum);out.flush();
    if(++n%8==0)System.out.println(args[3]+" "+n+"/"+(rows.size()-1)+" "+status+" "+m.lastMs*.001+"s");
   }
  }
 }
}
