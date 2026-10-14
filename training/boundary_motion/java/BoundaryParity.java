import java.io.*;import java.nio.file.*;import java.util.*;
public class BoundaryParity {
 public static void main(String[] args)throws Exception{
  BoundarySearchServer.settings();BoundarySearchServer.Env env=new BoundarySearchServer.Env(8187,0,0);CompactSearch.Network net=new CompactSearch.Network(Paths.get(args[0]));Random r=new Random(17);
  try(DataOutputStream out=new DataOutputStream(Files.newOutputStream(Paths.get(args[1])))){for(int k=0;k<16;k++){float[] obs=RewardInputs.observation(env.m);for(float v:obs)out.writeFloat(v);for(double v:net.scores(obs))out.writeFloat((float)v);env.advance(net.choose(obs,r,false));if(env.terminal)env.reset();}}
 }
}
