import java.util.*;
public final class SearchDecision {
 public static int choose(CompactSearch m,CompactSearch.Network net,Random rng){
  float[] f=RewardInputs.observation(m);
  if(!Boolean.getBoolean("parking.heuristicScore"))return net.choose(f,rng,false);
  double[] s=net.scores(f);double max=-1e30;
  for(int a=0;a<32;a++){s[a]-=3*f[CompactSearch.MAP+a*CompactSearch.FEATURES+25];max=Math.max(max,s[a]);}
  double sum=0;for(int a=0;a<32;a++)sum+=Math.exp(s[a]-max);
  double u=rng.nextDouble()*sum;for(int a=0;a<32;a++){u-=Math.exp(s[a]-max);if(u<=0)return a;}return 31;
 }
}
