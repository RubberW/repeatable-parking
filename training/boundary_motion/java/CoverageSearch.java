import java.util.*;

/** Online odometry grid: straight rows, recorded visits, BFS to remaining frontiers.
 * Adapted conceptually from PythonRobotics GridBasedSweepCPP and m-explore;
 * no copied source and no known map, global pose, table boundary or simulator input.
 */
public final class CoverageSearch {
  final double x0,y0,a,step;
  final int limit;
  int radius=2;
  final Set<Long> visited=new HashSet<>(),blocked=new HashSet<>();
  final Set<Long> observed=new HashSet<>();
  final Map<String,Integer> traversed=new HashMap<>();
  boolean useRouteMemory=false;
  Set<Long> reachable=new HashSet<>();
  int x=0,y=0,direction=1,lateral=1;
  int[] target;
  boolean exhausted=false;
  CoverageSearch(ParkingCore.Pose origin,double spacing,int cellLimit){
    x0=origin.x;y0=origin.y;a=origin.a;step=spacing;limit=cellLimit;visited.add(key(0,0));
  }
  static long key(int x,int y){return ((long)x<<32)^(y&0xffffffffL);}
  boolean allowed(int xx,int yy){int bound=limit==0?radius:limit;return Math.abs(xx)<=bound && Math.abs(yy)<=bound && !blocked.contains(key(xx,yy));}
  boolean fresh(int xx,int yy){return allowed(xx,yy) && !visited.contains(key(xx,yy));}
  int[][] neighbors(int xx,int yy){return new int[][]{{xx+direction,yy},{xx,yy+lateral},{xx,yy-lateral},{xx-direction,yy}};}
  String edgeKey(int ax,int ay,int bx,int by){long a=key(ax,ay),b=key(bx,by);return Math.min(a,b)+":"+Math.max(a,b);}
  void observe(ParkingCore.Observation o,ParkingCore.Pose p){
    for(int i=0;i<5;i++)if(o.sensor(i)==400 || o.sensor(i)==2000){
      double angle=p.a+Math.toRadians(ParkingCore.SENSOR_ANGLES[i]);
      observed.add(key((int)Math.floor((p.x+45*Math.cos(angle))/20),(int)Math.floor((p.y+45*Math.sin(angle))/20)));
    }
  }
  int[] freshNeighbor(int xx,int yy){
    int[] best=null;double bestCost=Double.POSITIVE_INFINITY;
    for(int[] p:neighbors(xx,yy))if(fresh(p[0],p[1])){
      if(!useRouteMemory)return p;
      double[] w=world(p[0],p[1]);int observedAround=0;
      for(int dx=-1;dx<=1;dx++)for(int dy=-1;dy<=1;dy++)if(observed.contains(key((int)Math.floor(w[0]/20)+dx,(int)Math.floor(w[1]/20)+dy)))observedAround++;
      double cost=observedAround*.15+(p[1]==yy && p[0]-xx==direction?0:.8);
      if(cost<bestCost){bestCost=cost;best=p;}
    }
    return best;
  }
  double[] world(int xx,int yy){return new double[]{x0+step*(xx*Math.cos(a)-yy*Math.sin(a)),y0+step*(xx*Math.sin(a)+yy*Math.cos(a))};}
  boolean arrived(ParkingCore.Pose p){
    if(target==null)return false;
    double[] w=world(target[0],target[1]);
    if(Math.hypot(p.x-w[0],p.y-w[1])>=1.8)return false;
    if(target[1]!=y)direction=-direction;
    else if(target[0]!=x)direction=Integer.signum(target[0]-x);
    String edge=edgeKey(x,y,target[0],target[1]);traversed.put(edge,traversed.getOrDefault(edge,0)+1);
    x=target[0];y=target[1];visited.add(key(x,y));target=null;return true;
  }
  void edgeBlocked(){if(target!=null)blocked.add(key(target[0],target[1]));target=null;}
  void rewind(){x=0;y=0;target=null;}
  double[] next(){
    if(target!=null)return world(target[0],target[1]);
    target=freshNeighbor(x,y);
    if(target==null && useRouteMemory){
      // Penalize repeated road segments and transfer only along actually traversed edges.
      PriorityQueue<double[]> queue=new PriorityQueue<>(Comparator.comparingDouble(n->n[2]));
      Map<Long,Double> distance=new HashMap<>();Map<Long,int[]> firstStep=new HashMap<>();
      queue.add(new double[]{x,y,0});distance.put(key(x,y),0.0);
      while(!queue.isEmpty()){
        double[] q=queue.remove();int qx=(int)q[0],qy=(int)q[1];long qk=key(qx,qy);
        if(q[2]>distance.get(qk)+1e-9)continue;
        if((qx!=x || qy!=y) && freshNeighbor(qx,qy)!=null){target=firstStep.get(qk);break;}
        for(int[] n:neighbors(qx,qy)){
          long nk=key(n[0],n[1]);int count=traversed.getOrDefault(edgeKey(qx,qy,n[0],n[1]),0);
          if(!allowed(n[0],n[1]) || !visited.contains(nk) || count==0)continue;
          double cost=q[2]+1+.5*count;
          if(cost<distance.getOrDefault(nk,Double.POSITIVE_INFINITY)){
            distance.put(nk,cost);firstStep.put(nk,qx==x && qy==y?n:firstStep.get(qk));queue.add(new double[]{n[0],n[1],cost});
          }
        }
      }
      reachable=new HashSet<>(distance.keySet());
    }
    if(target==null && !useRouteMemory){
      // Move only through previously visited cells to the nearest remaining frontier.
      ArrayDeque<int[]> queue=new ArrayDeque<>();Map<Long,int[]> firstStep=new HashMap<>();
      queue.add(new int[]{x,y});firstStep.put(key(x,y),null);
      while(!queue.isEmpty() && target==null){
        int[] p=queue.remove();
        if((p[0]!=x || p[1]!=y) && freshNeighbor(p[0],p[1])!=null){target=firstStep.get(key(p[0],p[1]));break;}
        for(int[] n:neighbors(p[0],p[1])){
          long k=key(n[0],n[1]);
          if(allowed(n[0],n[1]) && visited.contains(k) && !firstStep.containsKey(k)){
            firstStep.put(k,p[0]==x && p[1]==y?n:firstStep.get(key(p[0],p[1])));queue.add(n);
          }
        }
      }
    }
    if(target==null && limit==0){
      // Grow the search area only when its reachable frontiers are exhausted.
      // Keep all visit and edge memory, unlike restarting a fixed circle.
      radius+=2;
      boolean outerFrontier=false;
      for(long k:useRouteMemory?reachable:visited){int xx=(int)(k>>32),yy=(int)k;if(freshNeighbor(xx,yy)!=null){outerFrontier=true;break;}}
      if(outerFrontier)return next();
    }
    if(target==null){exhausted=true;return null;}return world(target[0],target[1]);
  }
}
